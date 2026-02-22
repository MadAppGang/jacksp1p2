/*
 * P1P2 Matter Thermostat — Thermostat cluster attribute management
 *
 * Maps HVAC state to Matter Thermostat cluster (0x0201) attributes:
 *   LocalTemperature, OccupiedCoolingSetpoint, OccupiedHeatingSetpoint,
 *   SystemMode, ThermostatRunningState
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "p1p2_matter.h"
#include "p1p2_matter_clusters.h"
#include "p1p2_protocol.h"

static const char *TAG = "matter_therm";

/* Cached previous values for change detection */
static int16_t prev_local_temp = 0x7FFF;
static int16_t prev_cool_sp = 0x7FFF;
static int16_t prev_heat_sp = 0x7FFF;
static uint8_t prev_mode = 0xFF;
static uint16_t prev_running = 0xFFFF;

/*
 * Convert p1p2_system_mode_t to Matter Thermostat SystemMode.
 */
static uint8_t mode_to_matter(p1p2_system_mode_t mode, bool power)
{
    if (!power) return THERMOSTAT_MODE_OFF;

    switch (mode) {
    case P1P2_MODE_HEAT: return THERMOSTAT_MODE_HEAT;
    case P1P2_MODE_COOL: return THERMOSTAT_MODE_COOL;
    case P1P2_MODE_AUTO: return THERMOSTAT_MODE_AUTO;
    case P1P2_MODE_FAN:  return THERMOSTAT_MODE_FAN_ONLY;
    case P1P2_MODE_DRY:  return THERMOSTAT_MODE_COOL; /* Map dry to cool */
    default:             return THERMOSTAT_MODE_OFF;
    }
}

/*
 * Convert running state to Matter RunningState bitmap.
 */
static uint16_t running_to_matter(p1p2_running_state_t running)
{
    switch (running) {
    case P1P2_RUNNING_HEATING: return RUNNING_STATE_HEAT_ON | RUNNING_STATE_FAN_ON;
    case P1P2_RUNNING_COOLING: return RUNNING_STATE_COOL_ON | RUNNING_STATE_FAN_ON;
    case P1P2_RUNNING_IDLE:
    default:                   return 0;
    }
}

/*
 * Update thermostat attributes from HVAC state.
 * Only pushes updates when values actually change.
 */
void p1p2_matter_thermostat_update(const p1p2_hvac_state_t *state)
{
    /* Matter temperatures are °C × 100 */
    int16_t local_temp = state->room_temp * 10;   /* state has ×10, Matter needs ×100 */
    int16_t cool_sp = state->target_temp_cool * 10;
    int16_t heat_sp = state->target_temp_heat * 10;
    uint8_t mode = mode_to_matter(state->mode, state->power);
    uint16_t running = running_to_matter(state->running);

    if (local_temp != prev_local_temp) {
        ESP_LOGD(TAG, "LocalTemperature: %d (%.1f°C)", local_temp, local_temp / 100.0);
        /*
         * TODO: esp_matter::attribute::update(
         *     EP_THERMOSTAT, CLUSTER_THERMOSTAT,
         *     ATTR_LOCAL_TEMPERATURE, &local_temp);
         */
        prev_local_temp = local_temp;
    }

    if (cool_sp != prev_cool_sp) {
        ESP_LOGD(TAG, "OccupiedCoolingSetpoint: %d (%.1f°C)", cool_sp, cool_sp / 100.0);
        prev_cool_sp = cool_sp;
    }

    if (heat_sp != prev_heat_sp) {
        ESP_LOGD(TAG, "OccupiedHeatingSetpoint: %d (%.1f°C)", heat_sp, heat_sp / 100.0);
        prev_heat_sp = heat_sp;
    }

    if (mode != prev_mode) {
        ESP_LOGI(TAG, "SystemMode: %d", mode);
        prev_mode = mode;
    }

    if (running != prev_running) {
        ESP_LOGD(TAG, "RunningState: 0x%04X", running);
        prev_running = running;
    }
}
