/*
 * P1P2MQTT ESP32-C6 — Main application entry point
 *
 * Single-chip P1/P2 bus interface for Daikin VRV (F-Series) over Matter/Thread.
 *
 * Replaces the dual-MCU architecture (ATmega328P + ESP8266) with a single
 * ESP32-C6 using MCPWM for bus I/O and IEEE 802.15.4 for Thread/Matter.
 *
 * FreeRTOS Task Architecture:
 *   Priority  Task              Purpose
 *   --------  ----              -------
 *   ISR       MCPWM/GPTimer     Bit-level bus I/O (capture, sample, toggle)
 *   22        bus_io_task        Packet assembly from ISR ring buffer
 *   15        protocol_task      F-series decode, control response generation
 *   10        matter_task        Matter attribute updates, command callbacks
 *   8         thread_task        OpenThread stack (managed by esp-matter)
 *   3         cli_task           Serial USB console commands
 *   1         adc_task           ADC bus voltage monitoring (housekeeping)
 *
 * Inter-task communication:
 *   ISR → ring buffer → bus_io_task → packet queue → protocol_task
 *       → Matter attributes → matter_task → Thread radio → Border Router → HA
 *
 * ESP32-C6 port: 2026
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "p1p2_bus.h"
#include "p1p2_protocol.h"
#include "p1p2_matter.h"
#include "p1p2_network.h"

/* CLI init (defined in p1p2_cli component) */
extern esp_err_t p1p2_cli_init(void);

static const char *TAG = "p1p2_main";

/*
 * Housekeeping task — LED heartbeat, stats logging, NVS periodic save.
 * Runs at priority 1 (lowest).
 */
static void housekeeping_task(void *pvParameters)
{
    int counter = 0;
    bool led_state = false;

    while (1) {
        /* LED heartbeat: blink power LED every 2s */
        led_state = !led_state;
        p1p2_led_power(led_state);

        /* Periodic stats logging (every 60s) */
        if (++counter >= 30) {
            counter = 0;

            p1p2_bus_stats_t stats;
            p1p2_bus_get_stats(&stats);

            int64_t uptime_s = stats.uptime_us / 1000000LL;
            ESP_LOGI(TAG, "Uptime: %llds | RX: %lu pkts | TX: %lu pkts | "
                          "CRC err: %lu | Parity err: %lu | Collisions: %lu",
                     uptime_s,
                     (unsigned long)stats.packets_received,
                     (unsigned long)stats.packets_sent,
                     (unsigned long)stats.crc_errors,
                     (unsigned long)stats.parity_errors,
                     (unsigned long)stats.collision_errors);

            const p1p2_hvac_state_t *state = p1p2_protocol_get_state();
            if (state->data_valid) {
                ESP_LOGI(TAG, "HVAC: power=%d mode=%d cool=%.1fC heat=%.1fC "
                              "room=%.1fC outdoor=%.1fC",
                         state->power, state->mode,
                         state->target_temp_cool / 10.0,
                         state->target_temp_heat / 10.0,
                         state->room_temp / 10.0,
                         state->outdoor_temp / 10.0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "P1P2MQTT ESP32-C6 — Daikin VRV F-Series");
    ESP_LOGI(TAG, "Matter over Thread, single-chip design");
    ESP_LOGI(TAG, "========================================");

    /* ---- Phase 1: NVS / Config Store ---- */
    ESP_LOGI(TAG, "Initializing config store...");
    esp_err_t ret = p1p2_config_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config init failed: %s", esp_err_to_name(ret));
        /* Continue anyway — NVS failures are non-fatal */
    }

    /* ---- Phase 2: Bus I/O (MCPWM + GPTimer + ADC) ---- */
    ESP_LOGI(TAG, "Initializing P1/P2 bus I/O...");
    p1p2_bus_config_t bus_config = P1P2_BUS_CONFIG_DEFAULT();
    ret = p1p2_bus_init(&bus_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bus I/O init failed: %s — cannot continue", esp_err_to_name(ret));
        return;
    }

    /* ---- Phase 3: Protocol Engine (F-series decode + control) ---- */
    ESP_LOGI(TAG, "Initializing protocol engine...");
    ret = p1p2_protocol_init(p1p2_bus_get_rx_queue(), p1p2_bus_get_tx_queue());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Protocol init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Load saved control level from NVS */
    uint8_t saved_level;
    if (p1p2_config_get_u8("ctrl_level", &saved_level) == ESP_OK) {
        p1p2_protocol_set_control_level(saved_level);
        ESP_LOGI(TAG, "Restored control level: %d", saved_level);
    }

    /* ---- Phase 4: Thread / Matter ---- */
    ESP_LOGI(TAG, "Initializing Thread stack...");
    ret = p1p2_thread_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Thread init failed (non-fatal): %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Initializing Matter device...");
    ret = p1p2_matter_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Matter init failed (non-fatal): %s", esp_err_to_name(ret));
    } else {
        ret = p1p2_matter_start();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Matter start failed: %s", esp_err_to_name(ret));
        }
    }

    /* ---- Phase 5: OTA ---- */
    ESP_LOGI(TAG, "Initializing OTA support...");
    p1p2_ota_init();

    /* ---- Phase 6: CLI ---- */
    ESP_LOGI(TAG, "Initializing CLI...");
    p1p2_cli_init();

    /* ---- Phase 7: Housekeeping ---- */
    xTaskCreate(housekeeping_task, "housekeep", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initialization complete!");
    ESP_LOGI(TAG, "  Bus I/O:  RX=GPIO%d TX=GPIO%d",
             bus_config.gpio_rx, bus_config.gpio_tx);
    ESP_LOGI(TAG, "  Model:    %d", CONFIG_P1P2_F_MODEL_ID);
    ESP_LOGI(TAG, "  Control:  level %d", p1p2_protocol_get_control_level());
    ESP_LOGI(TAG, "  Thread:   %s", p1p2_thread_is_attached() ? "attached" : "not attached");
    ESP_LOGI(TAG, "  Matter:   %s", p1p2_matter_is_commissioned() ? "commissioned" : "not commissioned");
    ESP_LOGI(TAG, "========================================");

    /* app_main returns; all work is done in FreeRTOS tasks */
}
