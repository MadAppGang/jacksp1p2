/*
 * P1P2 Matter — Public API for Matter device with HVAC clusters
 *
 * Creates a Matter Thermostat device with multiple endpoints:
 *   Endpoint 1: Thermostat (cluster 0x0201)
 *   Endpoint 2: Fan Control (cluster 0x0202)
 *   Endpoint 3: Temperature Sensors (cluster 0x0402)
 *   Endpoint 4: Custom VRV Cluster (manufacturer-specific 0xFFF1xxxx)
 *   Endpoint 5: On/Off for DHW (cluster 0x0006)
 *
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "p1p2_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize the Matter stack and create all endpoints/clusters.
 * Must be called after p1p2_protocol_init() so the protocol state is available.
 */
esp_err_t p1p2_matter_init(void);

/*
 * Start the Matter stack (commissioning, Thread networking, etc.).
 * Call after p1p2_matter_init().
 */
esp_err_t p1p2_matter_start(void);

/*
 * Update all Matter attributes from current HVAC state.
 * Called periodically by the matter_task after reading protocol state.
 */
void p1p2_matter_update_attributes(const p1p2_hvac_state_t *state);

/*
 * Check if the Matter device is commissioned (joined Thread network).
 */
bool p1p2_matter_is_commissioned(void);

/*
 * Factory reset the Matter device (remove commissioning data).
 */
void p1p2_matter_factory_reset(void);

#ifdef __cplusplus
}
#endif
