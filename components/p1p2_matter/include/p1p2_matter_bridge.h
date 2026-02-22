/*
 * P1P2 Matter Bridge — C-callable wrapper for esp-matter C++ SDK
 *
 * The esp-matter SDK is C++ only. This bridge provides C-callable functions
 * for creating Matter endpoints and updating attributes, used by the
 * existing C source files.
 *
 * When CONFIG_ESP_MATTER_ENABLE is not defined, these functions are not
 * available and the .c files fall back to logging-only stubs.
 *
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Attribute write callback signature (from C side).
 * Called when a Matter controller writes an attribute.
 */
typedef void (*p1p2_matter_write_cb_t)(uint16_t endpoint_id, uint32_t cluster_id,
                                        uint32_t attribute_id, void *val,
                                        uint16_t val_size);

/*
 * Create the Matter node with all endpoints:
 *   EP1: Thermostat (heat+cool+auto)
 *   EP2: Fan Control
 *   EP3: Temperature Sensor (outdoor)
 *   EP4: Custom VRV cluster
 *   EP5: On/Off (DHW)
 *   EP6: Temperature Sensor (room)
 *   EP7: Temperature Sensor (leaving water)
 *   EP8: Temperature Sensor (return water)
 *
 * write_cb: called when controller writes an attribute
 */
esp_err_t p1p2_matter_bridge_create_device(p1p2_matter_write_cb_t write_cb);

/*
 * Start the Matter stack (commissioning, Thread, etc.).
 */
esp_err_t p1p2_matter_bridge_start(void);

/*
 * Factory reset the Matter device.
 */
void p1p2_matter_bridge_factory_reset(void);

/*
 * Update a single attribute value. Type is inferred from val_size:
 *   1 byte  → uint8/bool
 *   2 bytes → int16/uint16
 *   4 bytes → uint32/int32
 */
esp_err_t p1p2_matter_bridge_update_attr(uint16_t endpoint_id,
                                          uint32_t cluster_id,
                                          uint32_t attribute_id,
                                          const void *val,
                                          uint16_t val_size);

/* Convenience typed wrappers */
esp_err_t p1p2_matter_bridge_update_i16(uint16_t ep, uint32_t cluster,
                                         uint32_t attr, int16_t val);
esp_err_t p1p2_matter_bridge_update_u8(uint16_t ep, uint32_t cluster,
                                        uint32_t attr, uint8_t val);
esp_err_t p1p2_matter_bridge_update_u16(uint16_t ep, uint32_t cluster,
                                         uint32_t attr, uint16_t val);
esp_err_t p1p2_matter_bridge_update_u32(uint16_t ep, uint32_t cluster,
                                         uint32_t attr, uint32_t val);
esp_err_t p1p2_matter_bridge_update_bool(uint16_t ep, uint32_t cluster,
                                          uint32_t attr, bool val);

#ifdef __cplusplus
}
#endif
