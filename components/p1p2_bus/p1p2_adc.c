/*
 * P1P2 ADC — Bus voltage monitoring using ESP32-C6 ADC continuous mode
 *
 * Replaces ATmega ADC ISR (alternating 2-channel, free-running) with
 * ESP32-C6 ADC continuous mode + DMA.
 *
 * Monitors P1 and P2 bus voltages for diagnostics.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "p1p2_bus.h"
#include "p1p2_bus_types.h"
#include "p1p2_bus_config.h"

static const char *TAG = "p1p2_adc";

#define ADC_FRAME_SIZE      256
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_ATTEN           ADC_ATTEN_DB_12  /* 0-3.3V range */

static adc_continuous_handle_t adc_handle = NULL;

/* Results — updated periodically from DMA buffer */
static volatile uint16_t v0_min_val;
static volatile uint16_t v0_max_val;
static volatile uint32_t v0_sum;
static volatile uint16_t v0_count;
static volatile uint16_t v1_min_val;
static volatile uint16_t v1_max_val;
static volatile uint32_t v1_sum;
static volatile uint16_t v1_count;

/* Averaged results (latched for reading) */
static volatile uint32_t v0_avg_result;
static volatile uint32_t v1_avg_result;
static volatile uint16_t v0_min_result = 0xFFF;
static volatile uint16_t v0_max_result;
static volatile uint16_t v1_min_result = 0xFFF;
static volatile uint16_t v1_max_result;

static bool adc_initialized = false;

/*
 * Process ADC DMA results.
 * Called periodically from housekeeping task (not ISR).
 */
static void process_adc_data(uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&data[i];
        uint16_t val = p->type2.data;
        int chan = p->type2.channel;

        if (chan == 0) {
            v0_sum += val;
            v0_count++;
            if (val < v0_min_val) v0_min_val = val;
            if (val > v0_max_val) v0_max_val = val;
            if (v0_count >= (1 << (16 - P1P2_ADC_CNT_SHIFT))) {
                v0_avg_result = v0_sum;
                v0_min_result = v0_min_val;
                v0_max_result = v0_max_val;
                v0_sum = 0;
                v0_count = 0;
                v0_min_val = 0xFFF;
                v0_max_val = 0;
            }
        } else if (chan == 1) {
            v1_sum += val;
            v1_count++;
            if (val < v1_min_val) v1_min_val = val;
            if (val > v1_max_val) v1_max_val = val;
            if (v1_count >= (1 << (16 - P1P2_ADC_CNT_SHIFT))) {
                v1_avg_result = v1_sum;
                v1_min_result = v1_min_val;
                v1_max_result = v1_max_val;
                v1_sum = 0;
                v1_count = 0;
                v1_min_val = 0xFFF;
                v1_max_val = 0;
            }
        }
    }
}

/*
 * ADC polling task — reads DMA buffer periodically.
 * Runs at low priority as ADC data is non-critical.
 */
static void adc_task(void *pvParameters)
{
    uint8_t result[ADC_FRAME_SIZE];
    uint32_t ret_num = 0;

    while (1) {
        if (adc_handle) {
            esp_err_t ret = adc_continuous_read(adc_handle, result,
                                                ADC_FRAME_SIZE, &ret_num, 100);
            if (ret == ESP_OK && ret_num > 0) {
                process_adc_data(result, ret_num);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); /* ~100 Hz polling */
    }
}

esp_err_t p1p2_adc_init(int gpio_adc0, int gpio_adc1)
{
    esp_err_t ret;

    /* Reset state */
    v0_min_val = 0xFFF; v0_max_val = 0; v0_sum = 0; v0_count = 0;
    v1_min_val = 0xFFF; v1_max_val = 0; v1_sum = 0; v1_count = 0;
    v0_avg_result = 0; v1_avg_result = 0;
    v0_min_result = 0xFFF; v0_max_result = 0;
    v1_min_result = 0xFFF; v1_max_result = 0;

    /* Configure ADC continuous mode */
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_FRAME_SIZE,
    };
    ret = adc_continuous_new_handle(&adc_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC handle: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure two channels alternating */
    adc_digi_pattern_config_t adc_pattern[2] = {
        {
            .atten = ADC_ATTEN,
            .channel = 0, /* ADC1_CHANNEL_0 */
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        },
        {
            .atten = ADC_ATTEN,
            .channel = 1, /* ADC1_CHANNEL_1 */
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        },
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 8000, /* ~4kSPS per channel (alternating) */
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
        .pattern_num = 2,
        .adc_pattern = adc_pattern,
    };
    ret = adc_continuous_config(adc_handle, &dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start ADC polling task at low priority */
    xTaskCreate(adc_task, "adc_poll", 2048, NULL, 1, NULL);

    adc_initialized = true;
    ESP_LOGI(TAG, "ADC initialized: 2 channels, 8kSPS total");
    return ESP_OK;
}

void p1p2_adc_deinit(void)
{
    if (adc_handle) {
        adc_continuous_stop(adc_handle);
        adc_continuous_deinit(adc_handle);
        adc_handle = NULL;
    }
    adc_initialized = false;
}

void p1p2_adc_get_results(p1p2_adc_results_t *results)
{
    if (!adc_initialized) {
        memset(results, 0, sizeof(*results));
        return;
    }

    results->v0_min = v0_min_result;
    results->v0_max = v0_max_result;
    results->v0_avg = v0_avg_result;
    results->v1_min = v1_min_result;
    results->v1_max = v1_max_result;
    results->v1_avg = v1_avg_result;

    /* Reset min/max on read (like ATmega behavior) */
    v0_min_result = 0xFFF;
    v0_max_result = 0;
    v1_min_result = 0xFFF;
    v1_max_result = 0;
}
