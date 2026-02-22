/*
 * P1P2 Matter Fan Control — Fan cluster attribute management
 *
 * Maps HVAC fan state to Matter Fan Control cluster (0x0202).
 * Uses changed bitmask to avoid unnecessary attribute updates.
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"

static const char *TAG = "matter_fan";

static uint8_t prev_fan_mode = 0xFF;

/*
 * Convert P1P2 fan mode to Matter FanMode.
 */
static uint8_t fan_to_matter(p1p2_fan_mode_t fan)
{
    switch (fan) {
    case P1P2_FAN_LOW:  return FAN_MODE_LOW;
    case P1P2_FAN_MED:  return FAN_MODE_MED;
    case P1P2_FAN_HIGH: return FAN_MODE_HIGH;
    case P1P2_FAN_AUTO: return FAN_MODE_AUTO;
    default:            return FAN_MODE_AUTO;
    }
}

void p1p2_matter_fan_update(const p1p2_hvac_state_t *state)
{
    /* Only update if relevant fields changed */
    if (!(state->changed & (CHANGED_FAN_COOL | CHANGED_FAN_HEAT | CHANGED_MODE | CHANGED_POWER))) {
        return;
    }

    /* Use the fan mode for the current operating mode */
    p1p2_fan_mode_t active_fan;
    if (state->mode == P1P2_MODE_HEAT) {
        active_fan = state->fan_mode_heat;
    } else {
        active_fan = state->fan_mode_cool;
    }

    uint8_t fan_mode = fan_to_matter(active_fan);

    if (!state->power) {
        fan_mode = FAN_MODE_OFF;
    }

    if (fan_mode != prev_fan_mode) {
        ESP_LOGI(TAG, "FanMode: %d", fan_mode);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_FAN, CLUSTER_FAN_CONTROL,
         *     ATTR_FAN_MODE, &fan_mode);
         */
        prev_fan_mode = fan_mode;
    }
}
