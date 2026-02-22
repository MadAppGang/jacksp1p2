/*
 * P1P2 Matter Sensors — Temperature Measurement cluster management
 *
 * Maps temperature readings to Matter Temperature Measurement cluster (0x0402).
 * Each temperature source has its own endpoint:
 *   EP3: Outdoor temperature
 *   EP6: Room temperature
 *   EP7: Leaving water temperature
 *   EP8: Return water temperature
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"

#ifdef P1P2_MATTER_SDK_AVAILABLE
#include "p1p2_matter_bridge.h"
#endif

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
#ifdef P1P2_MATTER_SDK_AVAILABLE
            p1p2_matter_bridge_update_i16(EP_TEMP_OUTDOOR, CLUSTER_TEMP_MEASUREMENT,
                                           ATTR_MEASURED_VALUE, outdoor);
#endif
            prev_outdoor = outdoor;
        }
    }

    if (state->changed & CHANGED_ROOM_TEMP) {
        int16_t room = state->room_temp * 10;
        if (room != prev_room) {
            ESP_LOGD(TAG, "Room temp: %d (%.1f°C)", room, room / 100.0);
#ifdef P1P2_MATTER_SDK_AVAILABLE
            p1p2_matter_bridge_update_i16(EP_TEMP_ROOM, CLUSTER_TEMP_MEASUREMENT,
                                           ATTR_MEASURED_VALUE, room);
#endif
            prev_room = room;
        }
    }

    if (state->changed & CHANGED_WATER_TEMPS) {
        int16_t leaving = state->leaving_water_temp * 10;
        if (leaving != prev_leaving_water) {
            ESP_LOGD(TAG, "Leaving water temp: %d (%.1f°C)", leaving, leaving / 100.0);
#ifdef P1P2_MATTER_SDK_AVAILABLE
            p1p2_matter_bridge_update_i16(EP_TEMP_LEAVING, CLUSTER_TEMP_MEASUREMENT,
                                           ATTR_MEASURED_VALUE, leaving);
#endif
            prev_leaving_water = leaving;
        }

        int16_t ret_water = state->return_water_temp * 10;
        if (ret_water != prev_return_water) {
            ESP_LOGD(TAG, "Return water temp: %d (%.1f°C)", ret_water, ret_water / 100.0);
#ifdef P1P2_MATTER_SDK_AVAILABLE
            p1p2_matter_bridge_update_i16(EP_TEMP_RETURN, CLUSTER_TEMP_MEASUREMENT,
                                           ATTR_MEASURED_VALUE, ret_water);
#endif
            prev_return_water = ret_water;
        }
    }
}
