/*
 * P1P2 Matter Sensors — Temperature Measurement cluster management
 *
 * Maps temperature readings to Matter Temperature Measurement cluster (0x0402).
 * Multiple sub-endpoints for different temperature sources:
 *   - Outdoor temperature
 *   - Room/return temperature (also used by Thermostat LocalTemperature)
 *   - Leaving water temperature
 *   - Return water temperature
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"

static const char *TAG = "matter_sens";

static int16_t prev_outdoor = 0x7FFF;
static int16_t prev_room = 0x7FFF;
static int16_t prev_leaving_water = 0x7FFF;
static int16_t prev_return_water = 0x7FFF;

void p1p2_matter_sensors_update(const p1p2_hvac_state_t *state)
{
    /* Matter temperatures: °C × 100 */

    if (state->changed & CHANGED_OUTDOOR_TEMP) {
        int16_t outdoor = state->outdoor_temp * 10;  /* ×10 → ×100 */
        if (outdoor != prev_outdoor) {
            ESP_LOGD(TAG, "Outdoor temp: %d (%.1f°C)", outdoor, outdoor / 100.0);
            /*
             * TODO: esp_matter::attribute::update(
             *     EP_TEMP_SENSORS, CLUSTER_TEMP_MEASUREMENT,
             *     ATTR_MEASURED_VALUE, &outdoor);
             * Sub-endpoint: outdoor temperature sensor
             */
            prev_outdoor = outdoor;
        }
    }

    if (state->changed & CHANGED_ROOM_TEMP) {
        int16_t room = state->room_temp * 10;
        if (room != prev_room) {
            ESP_LOGD(TAG, "Room temp: %d (%.1f°C)", room, room / 100.0);
            /*
             * TODO: esp_matter::attribute::update(
             *     EP_TEMP_SENSORS, CLUSTER_TEMP_MEASUREMENT,
             *     ATTR_MEASURED_VALUE, &room);
             * Sub-endpoint: room temperature sensor
             */
            prev_room = room;
        }
    }

    if (state->changed & CHANGED_WATER_TEMPS) {
        int16_t leaving = state->leaving_water_temp * 10;
        if (leaving != prev_leaving_water) {
            ESP_LOGD(TAG, "Leaving water temp: %d (%.1f°C)", leaving, leaving / 100.0);
            /*
             * TODO: esp_matter::attribute::update(
             *     EP_TEMP_SENSORS, CLUSTER_TEMP_MEASUREMENT,
             *     ATTR_MEASURED_VALUE, &leaving);
             * Sub-endpoint: leaving water temperature sensor
             */
            prev_leaving_water = leaving;
        }

        int16_t ret_water = state->return_water_temp * 10;
        if (ret_water != prev_return_water) {
            ESP_LOGD(TAG, "Return water temp: %d (%.1f°C)", ret_water, ret_water / 100.0);
            /*
             * TODO: esp_matter::attribute::update(
             *     EP_TEMP_SENSORS, CLUSTER_TEMP_MEASUREMENT,
             *     ATTR_MEASURED_VALUE, &ret_water);
             * Sub-endpoint: return water temperature sensor
             */
            prev_return_water = ret_water;
        }
    }
}
