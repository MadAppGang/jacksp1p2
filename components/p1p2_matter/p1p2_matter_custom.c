/*
 * P1P2 Matter Custom Cluster — VRV-specific diagnostics
 *
 * Manufacturer-specific cluster (0xFFF1xxxx) for data not covered
 * by standard Matter clusters:
 *   - Compressor frequency
 *   - Flow rates
 *   - Error codes
 *   - Operation hours / compressor starts
 *   - Bus voltage monitoring
 *   - Packet statistics
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"
#include "p1p2_bus.h"

static const char *TAG = "matter_vrv";

static uint16_t prev_compressor_freq = 0xFFFF;
static uint16_t prev_flow_rate = 0xFFFF;
static uint16_t prev_error_code = 0xFFFF;
static uint32_t prev_op_hours = 0xFFFFFFFF;
static uint32_t prev_comp_starts = 0xFFFFFFFF;
static uint32_t prev_packet_count = 0xFFFFFFFF;

void p1p2_matter_custom_update(const p1p2_hvac_state_t *state)
{
    /* Compressor frequency */
    if ((state->changed & CHANGED_COMPRESSOR) &&
        state->compressor_freq != prev_compressor_freq) {
        ESP_LOGD(TAG, "Compressor freq: %d Hz", state->compressor_freq);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_COMPRESSOR_FREQ, &state->compressor_freq);
         */
        prev_compressor_freq = state->compressor_freq;
    }

    /* Flow rate */
    if ((state->changed & CHANGED_FLOW_RATE) &&
        state->flow_rate != prev_flow_rate) {
        ESP_LOGD(TAG, "Flow rate: %d (%.1f L/min)", state->flow_rate, state->flow_rate / 10.0);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_FLOW_RATE, &state->flow_rate);
         */
        prev_flow_rate = state->flow_rate;
    }

    /* Error code */
    if ((state->changed & CHANGED_ERROR_CODE) &&
        state->error_code != prev_error_code) {
        if (state->error_code) {
            ESP_LOGW(TAG, "Error code: 0x%04X", state->error_code);
        } else {
            ESP_LOGI(TAG, "Error cleared");
        }
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_ERROR_CODE, &state->error_code);
         */
        prev_error_code = state->error_code;
    }

    /* Operation hours */
    if ((state->changed & CHANGED_OP_HOURS) &&
        state->operation_hours != prev_op_hours) {
        ESP_LOGD(TAG, "Operation hours: %lu", (unsigned long)state->operation_hours);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_OPERATION_HOURS, &state->operation_hours);
         */
        prev_op_hours = state->operation_hours;
    }

    /* Compressor starts */
    if ((state->changed & CHANGED_COMP_STARTS) &&
        state->compressor_starts != prev_comp_starts) {
        ESP_LOGD(TAG, "Compressor starts: %lu", (unsigned long)state->compressor_starts);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_COMPRESSOR_STARTS, &state->compressor_starts);
         */
        prev_comp_starts = state->compressor_starts;
    }

    /* Packet count — update unconditionally on change */
    if (state->packet_count != prev_packet_count) {
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_PACKET_COUNT, &state->packet_count);
         */
        prev_packet_count = state->packet_count;
    }

    /* Bus voltage monitoring — read ADC periodically */
    static int update_counter = 0;
    if (++update_counter >= 30) { /* every ~60s at 2s update rate */
        update_counter = 0;
        p1p2_adc_results_t adc;
        p1p2_bus_get_adc(&adc);
        ESP_LOGD(TAG, "Bus ADC: V0 avg=%lu V1 avg=%lu",
                 (unsigned long)adc.v0_avg, (unsigned long)adc.v1_avg);
        /*
         * TODO: Convert ADC values to mV and update attributes:
         * uint16_t v_p1 = (uint16_t)(adc.v0_avg * ADC_TO_MV_FACTOR);
         * uint16_t v_p2 = (uint16_t)(adc.v1_avg * ADC_TO_MV_FACTOR);
         * esp_matter::attribute::update(EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_BUS_VOLTAGE_P1, &v_p1);
         * esp_matter::attribute::update(EP_CUSTOM_VRV, CLUSTER_CUSTOM_VRV,
         *     ATTR_VRV_BUS_VOLTAGE_P2, &v_p2);
         */
    }
}
