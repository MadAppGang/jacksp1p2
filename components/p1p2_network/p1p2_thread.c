/*
 * P1P2 Thread — OpenThread stack initialization for ESP32-C6
 *
 * Initializes the IEEE 802.15.4 radio and OpenThread stack.
 * Thread networking is managed by the esp-matter SDK when integrated.
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_network.h"

static const char *TAG = "p1p2_thread";

static bool thread_attached = false;

esp_err_t p1p2_thread_init(void)
{
    ESP_LOGI(TAG, "Initializing OpenThread stack");

    /*
     * TODO: When esp-matter SDK is integrated, Thread init is handled
     * automatically by the Matter stack. The sequence is:
     *
     * 1. esp_openthread_platform_config_t config = {
     *        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
     *        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
     *        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
     *    };
     *
     * 2. esp_openthread_init(&config);
     *
     * 3. The Matter stack handles Thread commissioning via BLE or
     *    on-network commissioning through the border router.
     *
     * For standalone testing (Phase 1), Thread can be initialized
     * independently:
     *
     * esp_openthread_init(&config);
     * esp_openthread_launch_mainloop(&mainloop_config);
     *
     * When the device is commissioned via Matter, the Thread credentials
     * (network name, PAN ID, channel, master key) are provisioned
     * automatically and stored in NVS.
     */

    ESP_LOGI(TAG, "Thread stack initialized (SDK integration pending)");
    ESP_LOGI(TAG, "Thread commissioning will be handled by Matter stack");
    return ESP_OK;
}

bool p1p2_thread_is_attached(void)
{
    /*
     * TODO: return otThreadGetDeviceRole(esp_openthread_get_instance())
     *       >= OT_DEVICE_ROLE_CHILD;
     */
    return thread_attached;
}
