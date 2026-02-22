/*
 * P1P2 Network — Thread, OTA, and NVS config store
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
 * Initialize the OpenThread stack on ESP32-C6.
 * Configures the IEEE 802.15.4 radio and joins/creates Thread network.
 */
esp_err_t p1p2_thread_init(void);

/*
 * Check if Thread network is attached.
 */
bool p1p2_thread_is_attached(void);

/*
 * Initialize OTA update support.
 * Supports Matter OTA cluster and USB serial fallback.
 */
esp_err_t p1p2_ota_init(void);

/*
 * Check for and apply OTA update.
 */
esp_err_t p1p2_ota_check(void);

/*
 * NVS config store — replaces ATmega EEPROM.
 * Key-value storage for persistent configuration.
 */
esp_err_t p1p2_config_init(void);

/*
 * Read a uint8_t config value.
 */
esp_err_t p1p2_config_get_u8(const char *key, uint8_t *value);

/*
 * Write a uint8_t config value.
 */
esp_err_t p1p2_config_set_u8(const char *key, uint8_t value);

/*
 * Read a uint16_t config value.
 */
esp_err_t p1p2_config_get_u16(const char *key, uint16_t *value);

/*
 * Write a uint16_t config value.
 */
esp_err_t p1p2_config_set_u16(const char *key, uint16_t value);

/*
 * Read a string config value.
 */
esp_err_t p1p2_config_get_str(const char *key, char *buf, size_t buf_len);

/*
 * Write a string config value.
 */
esp_err_t p1p2_config_set_str(const char *key, const char *value);

/*
 * Erase all config (factory reset).
 */
esp_err_t p1p2_config_erase_all(void);

#ifdef __cplusplus
}
#endif
