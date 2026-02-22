/*
 * P1P2 Thread — OpenThread stack initialization for ESP32-C6
 *
 * When the esp-matter SDK is present, Thread is managed by the Matter stack
 * automatically (commissioning provisions Thread credentials). This file
 * provides the init/status API; actual Thread setup is in the bridge.
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_network.h"

#ifdef P1P2_MATTER_SDK_AVAILABLE
#include <openthread/thread.h>
#include "esp_openthread.h"
#endif

static const char *TAG = "p1p2_thread";

esp_err_t p1p2_thread_init(void)
{
    ESP_LOGI(TAG, "Initializing OpenThread stack");

#ifdef P1P2_MATTER_SDK_AVAILABLE
    /*
     * When esp-matter SDK is active, Thread initialization is handled
     * by the Matter stack via set_openthread_platform_config() in the bridge.
     * No separate init needed here.
     */
    ESP_LOGI(TAG, "Thread managed by Matter stack");
#else
    ESP_LOGI(TAG, "Thread stack initialized (stub mode — no SDK)");
    ESP_LOGI(TAG, "Thread commissioning will be handled by Matter stack");
#endif

    return ESP_OK;
}

bool p1p2_thread_is_attached(void)
{
#ifdef P1P2_MATTER_SDK_AVAILABLE
    otInstance *instance = esp_openthread_get_instance();
    if (instance) {
        return otThreadGetDeviceRole(instance) >= OT_DEVICE_ROLE_CHILD;
    }
#endif
    return false;
}
