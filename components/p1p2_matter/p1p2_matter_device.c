/*
 * P1P2 Matter Device — Matter node setup, endpoint registration
 *
 * Creates the Matter device as a Thermostat with multiple endpoints.
 * When the esp-matter SDK is available, this uses the real Matter API.
 * For now, it provides the structure and callbacks that will be linked
 * against the SDK.
 *
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"

static const char *TAG = "p1p2_matter";

/*
 * NOTE: The actual Matter SDK integration uses esp_matter APIs.
 * This file provides the framework; actual SDK calls are conditional
 * on ESP_MATTER_PATH being set during build.
 *
 * The structure follows Espressif's thermostat example:
 * https://github.com/espressif/esp-matter/tree/main/examples/thermostat
 */

/* Matter device state */
static bool matter_initialized = false;
static bool matter_commissioned = false;

/* External thermostat/fan/sensor attribute update functions */
extern void p1p2_matter_thermostat_update(const p1p2_hvac_state_t *state);
extern void p1p2_matter_fan_update(const p1p2_hvac_state_t *state);
extern void p1p2_matter_sensors_update(const p1p2_hvac_state_t *state);
extern void p1p2_matter_custom_update(const p1p2_hvac_state_t *state);

/*
 * Matter attribute change callback.
 * Called when Home Assistant (or other Matter controller) writes an attribute.
 * Routes to the appropriate handler based on endpoint and cluster.
 */
static void matter_attribute_update_cb(uint16_t endpoint_id, uint32_t cluster_id,
                                        uint32_t attribute_id, void *val,
                                        uint16_t val_size)
{
    ESP_LOGI(TAG, "Attribute update: ep=%d cluster=0x%04lX attr=0x%04lX",
             endpoint_id, (unsigned long)cluster_id, (unsigned long)attribute_id);

    switch (endpoint_id) {
    case EP_THERMOSTAT:
        if (cluster_id == CLUSTER_THERMOSTAT) {
            switch (attribute_id) {
            case ATTR_SYSTEM_MODE: {
                uint8_t mode = *(uint8_t *)val;
                p1p2_system_mode_t p1p2_mode;
                switch (mode) {
                case THERMOSTAT_MODE_OFF:      p1p2_mode = P1P2_MODE_OFF;  break;
                case THERMOSTAT_MODE_HEAT:     p1p2_mode = P1P2_MODE_HEAT; break;
                case THERMOSTAT_MODE_COOL:     p1p2_mode = P1P2_MODE_COOL; break;
                case THERMOSTAT_MODE_AUTO:     p1p2_mode = P1P2_MODE_AUTO; break;
                case THERMOSTAT_MODE_FAN_ONLY: p1p2_mode = P1P2_MODE_FAN;  break;
                default:                       p1p2_mode = P1P2_MODE_OFF;  break;
                }
                if (p1p2_mode == P1P2_MODE_OFF) {
                    p1p2_protocol_send_cmd(P1P2_CMD_SET_POWER, 0);
                } else {
                    p1p2_protocol_send_cmd(P1P2_CMD_SET_POWER, 1);
                    p1p2_protocol_send_cmd(P1P2_CMD_SET_MODE, p1p2_mode);
                }
                break;
            }
            case ATTR_OCCUPIED_COOLING_SETPOINT: {
                /* Matter sends °C × 100, we need °C × 10 */
                int16_t temp_100 = *(int16_t *)val;
                p1p2_protocol_send_cmd(P1P2_CMD_SET_TEMP_COOL, temp_100 / 10);
                break;
            }
            case ATTR_OCCUPIED_HEATING_SETPOINT: {
                int16_t temp_100 = *(int16_t *)val;
                p1p2_protocol_send_cmd(P1P2_CMD_SET_TEMP_HEAT, temp_100 / 10);
                break;
            }
            }
        }
        break;

    case EP_FAN:
        if (cluster_id == CLUSTER_FAN_CONTROL && attribute_id == ATTR_FAN_MODE) {
            uint8_t fan_mode = *(uint8_t *)val;
            p1p2_fan_mode_t p1p2_fan;
            switch (fan_mode) {
            case FAN_MODE_LOW:  p1p2_fan = P1P2_FAN_LOW;  break;
            case FAN_MODE_MED:  p1p2_fan = P1P2_FAN_MED;  break;
            case FAN_MODE_HIGH: p1p2_fan = P1P2_FAN_HIGH;  break;
            case FAN_MODE_AUTO: p1p2_fan = P1P2_FAN_AUTO;  break;
            default:            p1p2_fan = P1P2_FAN_AUTO;  break;
            }
            /* Set both cool and heat fan speeds */
            p1p2_protocol_send_cmd(P1P2_CMD_SET_FAN_COOL, p1p2_fan);
            p1p2_protocol_send_cmd(P1P2_CMD_SET_FAN_HEAT, p1p2_fan);
        }
        break;

    case EP_DHW_ONOFF:
        if (cluster_id == CLUSTER_ON_OFF && attribute_id == ATTR_ON_OFF) {
            bool on = *(bool *)val;
            p1p2_protocol_send_cmd(P1P2_CMD_SET_DHW_POWER, on ? 1 : 0);
        }
        break;

    default:
        ESP_LOGD(TAG, "Unhandled attribute update on endpoint %d", endpoint_id);
        break;
    }
}

/*
 * Matter task — periodically reads HVAC state and updates Matter attributes.
 * Priority 10 (below protocol at 15).
 */
static void matter_task(void *pvParameters)
{
    p1p2_hvac_state_t state;
    int64_t last_update = 0;

    ESP_LOGI(TAG, "Matter task started");

    while (1) {
        /* Get latest HVAC state */
        p1p2_protocol_get_state_copy(&state);

        /* Only update if state has changed since last push */
        if (state.last_update_us > last_update && state.data_valid) {
            p1p2_matter_thermostat_update(&state);
            p1p2_matter_fan_update(&state);
            p1p2_matter_sensors_update(&state);
            p1p2_matter_custom_update(&state);
            last_update = state.last_update_us;
        }

        /* Update every 2 seconds (bus cycle is 0.8-2s) */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t p1p2_matter_init(void)
{
    ESP_LOGI(TAG, "Initializing Matter device");

    /*
     * TODO: When esp-matter SDK is integrated, replace this with:
     *
     * esp_matter::node_t *node = esp_matter::node::create(...);
     *
     * // Endpoint 1: Thermostat
     * esp_matter::endpoint_t *thermostat_ep =
     *     esp_matter::thermostat::create(node, ...);
     *
     * // Endpoint 2: Fan Control
     * esp_matter::endpoint_t *fan_ep =
     *     esp_matter::fan::create(node, ...);
     *
     * // Endpoint 3: Temperature Sensor
     * esp_matter::endpoint_t *temp_ep =
     *     esp_matter::temperature_sensor::create(node, ...);
     *
     * // Endpoint 4: Custom VRV cluster
     * // Use esp_matter::cluster::create() with manufacturer-specific ID
     *
     * // Endpoint 5: On/Off (DHW)
     * esp_matter::endpoint_t *dhw_ep =
     *     esp_matter::on_off_light::create(node, ...);
     *
     * // Register attribute callback
     * esp_matter::attribute::set_callback(matter_attribute_update_cb);
     */

    matter_initialized = true;
    ESP_LOGI(TAG, "Matter device initialized (SDK integration pending)");
    return ESP_OK;
}

esp_err_t p1p2_matter_start(void)
{
    if (!matter_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * TODO: When esp-matter SDK is integrated:
     * esp_matter::start(matter_attribute_update_cb);
     */

    /* Start matter attribute update task */
    BaseType_t ret = xTaskCreate(matter_task, "matter", 4096, NULL, 10, NULL);
    if (ret != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Matter stack started");
    return ESP_OK;
}

void p1p2_matter_update_attributes(const p1p2_hvac_state_t *state)
{
    p1p2_matter_thermostat_update(state);
    p1p2_matter_fan_update(state);
    p1p2_matter_sensors_update(state);
    p1p2_matter_custom_update(state);
}

bool p1p2_matter_is_commissioned(void)
{
    return matter_commissioned;
}

void p1p2_matter_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset requested");
    matter_commissioned = false;
    /* TODO: esp_matter::factory_reset(); */
}
