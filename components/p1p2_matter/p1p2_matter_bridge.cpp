/*
 * P1P2 Matter Bridge — esp-matter SDK integration (C++ side)
 *
 * This file is only compiled when ESP_MATTER_PATH is set and the
 * esp_matter component is available. It provides the C-callable bridge
 * functions declared in p1p2_matter_bridge.h.
 *
 * ESP32-C6 port: 2026
 */

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <platform/ESP32/OpenthreadLauncher.h>

#include "p1p2_matter_bridge.h"
#include "p1p2_matter_clusters.h"

static const char *TAG = "matter_bridge";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

/* Stored write callback from C side */
static p1p2_matter_write_cb_t s_write_cb = NULL;

/* ---- Attribute callback (Matter → C bridge) ---- */

static esp_err_t bridge_attribute_cb(attribute::callback_type_t type,
                                      uint16_t endpoint_id,
                                      uint32_t cluster_id,
                                      uint32_t attribute_id,
                                      esp_matter_attr_val_t *val,
                                      void *priv_data)
{
    if (type != PRE_UPDATE || !s_write_cb) {
        return ESP_OK;
    }

    /* Route to C callback with raw value */
    void *raw_val = NULL;
    uint16_t val_size = 0;

    switch (val->type) {
    case ESP_MATTER_VAL_TYPE_BOOLEAN:
        raw_val = &val->val.b;
        val_size = sizeof(bool);
        break;
    case ESP_MATTER_VAL_TYPE_INTEGER:
        raw_val = &val->val.i;
        val_size = sizeof(int);
        break;
    case ESP_MATTER_VAL_TYPE_UINT8:
    case ESP_MATTER_VAL_TYPE_ENUM8:
    case ESP_MATTER_VAL_TYPE_BITMAP8:
        raw_val = &val->val.u8;
        val_size = sizeof(uint8_t);
        break;
    case ESP_MATTER_VAL_TYPE_UINT16:
    case ESP_MATTER_VAL_TYPE_BITMAP16:
        raw_val = &val->val.u16;
        val_size = sizeof(uint16_t);
        break;
    case ESP_MATTER_VAL_TYPE_UINT32:
    case ESP_MATTER_VAL_TYPE_BITMAP32:
        raw_val = &val->val.u32;
        val_size = sizeof(uint32_t);
        break;
    case ESP_MATTER_VAL_TYPE_INT8:
        raw_val = &val->val.i8;
        val_size = sizeof(int8_t);
        break;
    case ESP_MATTER_VAL_TYPE_INT16:
    case ESP_MATTER_VAL_TYPE_NULLABLE_INT16:
        raw_val = &val->val.i16;
        val_size = sizeof(int16_t);
        break;
    default:
        ESP_LOGW(TAG, "Unhandled val type %d for ep=%d cluster=0x%04lX attr=0x%04lX",
                 val->type, endpoint_id, (unsigned long)cluster_id,
                 (unsigned long)attribute_id);
        return ESP_OK;
    }

    s_write_cb(endpoint_id, cluster_id, attribute_id, raw_val, val_size);
    return ESP_OK;
}

static esp_err_t bridge_identify_cb(identification::callback_type_t type,
                                     uint16_t endpoint_id,
                                     uint8_t effect_id,
                                     uint8_t effect_variant,
                                     void *priv_data)
{
    ESP_LOGI(TAG, "Identify: ep=%d effect=%d variant=%d", endpoint_id,
             effect_id, effect_variant);
    return ESP_OK;
}

/* ---- Device creation ---- */

esp_err_t p1p2_matter_bridge_create_device(p1p2_matter_write_cb_t write_cb)
{
    s_write_cb = write_cb;

    /* Create Matter node (root endpoint 0 is automatic) */
    node::config_t node_config;
    node_t *node = node::create(&node_config, bridge_attribute_cb,
                                 bridge_identify_cb);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return ESP_FAIL;
    }

    /* EP1: Thermostat (heating + cooling + auto) */
    {
        thermostat::config_t cfg;
        cfg.thermostat.local_temperature = 2200;  /* 22.00°C */
        cfg.thermostat.control_sequence_of_operation = 4; /* CoolingAndHeating */
        cfg.thermostat.system_mode = 1; /* Auto */
        endpoint_t *ep = thermostat::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create thermostat endpoint");
            return ESP_FAIL;
        }
        uint16_t ep_id = endpoint::get_id(ep);
        ESP_LOGI(TAG, "Thermostat endpoint created: %d (expected %d)", ep_id, EP_THERMOSTAT);

        /* Add heating + cooling + auto features */
        cluster_t *therm_cluster = cluster::get(ep, Thermostat::Id);
        if (therm_cluster) {
            cluster::thermostat::feature::heating::config_t heat_cfg;
            heat_cfg.occupied_heating_setpoint = 2000; /* 20.00°C */
            cluster::thermostat::feature::heating::add(therm_cluster, &heat_cfg);

            cluster::thermostat::feature::cooling::config_t cool_cfg;
            cool_cfg.occupied_cooling_setpoint = 2600; /* 26.00°C */
            cluster::thermostat::feature::cooling::add(therm_cluster, &cool_cfg);

            cluster::thermostat::feature::auto_mode::config_t auto_cfg;
            auto_cfg.min_setpoint_dead_band = 2;
            cluster::thermostat::feature::auto_mode::add(therm_cluster, &auto_cfg);
        }
    }

    /* EP2: Fan Control */
    {
        fan::config_t cfg;
        cfg.fan_control.fan_mode = 0;
        cfg.fan_control.fan_mode_sequence = 2; /* OffLowMedHighAuto */
        endpoint_t *ep = fan::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create fan endpoint");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Fan endpoint created: %d", endpoint::get_id(ep));
    }

    /* EP3: Temperature Sensor — outdoor */
    {
        temperature_sensor::config_t cfg;
        endpoint_t *ep = temperature_sensor::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create outdoor temp sensor endpoint");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Outdoor temp sensor endpoint created: %d", endpoint::get_id(ep));
    }

    /* EP4: Custom VRV (generic endpoint + custom cluster) */
    {
        endpoint_t *ep = endpoint::create(node, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create custom VRV endpoint");
            return ESP_FAIL;
        }

        /* Add manufacturer-specific cluster */
        cluster_t *custom_cluster = cluster::create(ep, CLUSTER_CUSTOM_VRV,
                                                      CLUSTER_FLAG_SERVER);
        if (custom_cluster) {
            /* Compressor freq (u16) */
            attribute::create(custom_cluster, ATTR_VRV_COMPRESSOR_FREQ,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
            /* Flow rate (u16) */
            attribute::create(custom_cluster, ATTR_VRV_FLOW_RATE,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
            /* Error code (u16) */
            attribute::create(custom_cluster, ATTR_VRV_ERROR_CODE,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
            /* Operation hours (u32) */
            attribute::create(custom_cluster, ATTR_VRV_OPERATION_HOURS,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint32(0));
            /* Compressor starts (u32) */
            attribute::create(custom_cluster, ATTR_VRV_COMPRESSOR_STARTS,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint32(0));
            /* Bus voltage P1 (u16) */
            attribute::create(custom_cluster, ATTR_VRV_BUS_VOLTAGE_P1,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
            /* Bus voltage P2 (u16) */
            attribute::create(custom_cluster, ATTR_VRV_BUS_VOLTAGE_P2,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
            /* Packet count (u32) */
            attribute::create(custom_cluster, ATTR_VRV_PACKET_COUNT,
                            ATTRIBUTE_FLAG_NONE, esp_matter_uint32(0));
        }
        ESP_LOGI(TAG, "Custom VRV endpoint created: %d", endpoint::get_id(ep));
    }

    /* EP5: On/Off — DHW (domestic hot water) */
    {
        on_off_light::config_t cfg;
        cfg.on_off.on_off = false;
        endpoint_t *ep = on_off_light::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create DHW on/off endpoint");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "DHW on/off endpoint created: %d", endpoint::get_id(ep));
    }

    /* EP6: Temperature Sensor — room */
    {
        temperature_sensor::config_t cfg;
        endpoint_t *ep = temperature_sensor::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create room temp sensor endpoint");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Room temp sensor endpoint created: %d", endpoint::get_id(ep));
    }

    /* EP7: Temperature Sensor — leaving water */
    {
        temperature_sensor::config_t cfg;
        endpoint_t *ep = temperature_sensor::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create leaving water temp sensor endpoint");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Leaving water temp sensor endpoint created: %d", endpoint::get_id(ep));
    }

    /* EP8: Temperature Sensor — return water */
    {
        temperature_sensor::config_t cfg;
        endpoint_t *ep = temperature_sensor::create(node, &cfg, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create return water temp sensor endpoint");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Return water temp sensor endpoint created: %d", endpoint::get_id(ep));
    }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Configure OpenThread platform */
    esp_openthread_platform_config_t ot_config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_config);
#endif

    ESP_LOGI(TAG, "Matter device created with 8 endpoints");
    return ESP_OK;
}

/* ---- Event callback ---- */

static void bridge_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;
    default:
        break;
    }
}

/* ---- Start ---- */

esp_err_t p1p2_matter_bridge_start(void)
{
    esp_err_t err = esp_matter::start(bridge_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* ---- Factory reset ---- */

void p1p2_matter_bridge_factory_reset(void)
{
    esp_matter::factory_reset();
}

/* ---- Attribute update helpers ---- */

esp_err_t p1p2_matter_bridge_update_i16(uint16_t ep, uint32_t cluster_id,
                                         uint32_t attr, int16_t val)
{
    esp_matter_attr_val_t matter_val = esp_matter_nullable_int16(val);
    return attribute::update(ep, cluster_id, attr, &matter_val);
}

esp_err_t p1p2_matter_bridge_update_u8(uint16_t ep, uint32_t cluster_id,
                                        uint32_t attr, uint8_t val)
{
    esp_matter_attr_val_t matter_val = esp_matter_enum8(val);
    return attribute::update(ep, cluster_id, attr, &matter_val);
}

esp_err_t p1p2_matter_bridge_update_u16(uint16_t ep, uint32_t cluster_id,
                                         uint32_t attr, uint16_t val)
{
    esp_matter_attr_val_t matter_val = esp_matter_uint16(val);
    return attribute::update(ep, cluster_id, attr, &matter_val);
}

esp_err_t p1p2_matter_bridge_update_u32(uint16_t ep, uint32_t cluster_id,
                                         uint32_t attr, uint32_t val)
{
    esp_matter_attr_val_t matter_val = esp_matter_uint32(val);
    return attribute::update(ep, cluster_id, attr, &matter_val);
}

esp_err_t p1p2_matter_bridge_update_bool(uint16_t ep, uint32_t cluster_id,
                                          uint32_t attr, bool val)
{
    esp_matter_attr_val_t matter_val = esp_matter_bool(val);
    return attribute::update(ep, cluster_id, attr, &matter_val);
}

esp_err_t p1p2_matter_bridge_update_attr(uint16_t ep, uint32_t cluster_id,
                                          uint32_t attr, const void *val,
                                          uint16_t val_size)
{
    switch (val_size) {
    case 1:
        return p1p2_matter_bridge_update_u8(ep, cluster_id, attr, *(const uint8_t *)val);
    case 2:
        return p1p2_matter_bridge_update_i16(ep, cluster_id, attr, *(const int16_t *)val);
    case 4:
        return p1p2_matter_bridge_update_u32(ep, cluster_id, attr, *(const uint32_t *)val);
    default:
        ESP_LOGW(TAG, "Unsupported val_size %d", val_size);
        return ESP_ERR_INVALID_ARG;
    }
}
