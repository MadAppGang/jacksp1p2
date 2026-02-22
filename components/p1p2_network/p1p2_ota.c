/*
 * P1P2 OTA — Firmware update support
 *
 * Supports two update methods:
 * 1. Matter OTA cluster — standard Matter OTA provider
 * 2. USB serial fallback — esptool.py over USB-C
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "p1p2_network.h"

static const char *TAG = "p1p2_ota";

esp_err_t p1p2_ota_init(void)
{
    ESP_LOGI(TAG, "OTA support initialized");

    /* Log current firmware info */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "Running firmware: %s v%s (built %s %s)",
             app_desc->project_name, app_desc->version,
             app_desc->date, app_desc->time);

    /* Log partition info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running from partition: %s (offset 0x%lx)",
             running->label, (unsigned long)running->address);

    /*
     * TODO: When esp-matter SDK is integrated:
     * - Matter OTA Requestor cluster is set up automatically
     * - OTA Provider address comes from the Matter controller
     * - Firmware images are downloaded over Thread via the border router
     *
     * For USB serial fallback, the standard ESP-IDF esptool.py
     * works via USB-C JTAG on ESP32-C6.
     */

    return ESP_OK;
}

esp_err_t p1p2_ota_check(void)
{
    /* TODO: Implement Matter OTA check */
    ESP_LOGD(TAG, "OTA check (not yet implemented)");
    return ESP_OK;
}
