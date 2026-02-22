/*
 * P1P2 Config Store — NVS-based persistent configuration
 *
 * Replaces the ATmega EEPROM key-value store with ESP32-C6 NVS.
 * Stores: control level, model selection, delay timeout, GPIO assignments,
 * and any user-configurable parameters.
 *
 * ESP32-C6 port: 2026
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "p1p2_network.h"

static const char *TAG = "p1p2_config";
static const char *NVS_NAMESPACE = "p1p2";

static nvs_handle_t config_nvs_handle;
static bool config_initialized = false;

esp_err_t p1p2_config_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, reformatting...");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) return ret;
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &config_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    config_initialized = true;
    ESP_LOGI(TAG, "Config store initialized (NVS namespace: %s)", NVS_NAMESPACE);
    return ESP_OK;
}

esp_err_t p1p2_config_get_u8(const char *key, uint8_t *value)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    return nvs_get_u8(config_nvs_handle, key, value);
}

esp_err_t p1p2_config_set_u8(const char *key, uint8_t value)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = nvs_set_u8(config_nvs_handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(config_nvs_handle);
    return ret;
}

esp_err_t p1p2_config_get_u16(const char *key, uint16_t *value)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    return nvs_get_u16(config_nvs_handle, key, value);
}

esp_err_t p1p2_config_set_u16(const char *key, uint16_t value)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = nvs_set_u16(config_nvs_handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(config_nvs_handle);
    return ret;
}

esp_err_t p1p2_config_get_str(const char *key, char *buf, size_t buf_len)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    return nvs_get_str(config_nvs_handle, key, buf, &buf_len);
}

esp_err_t p1p2_config_set_str(const char *key, const char *value)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = nvs_set_str(config_nvs_handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(config_nvs_handle);
    return ret;
}

esp_err_t p1p2_config_erase_all(void)
{
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = nvs_erase_all(config_nvs_handle);
    if (ret == ESP_OK) ret = nvs_commit(config_nvs_handle);
    ESP_LOGW(TAG, "All config erased");
    return ret;
}
