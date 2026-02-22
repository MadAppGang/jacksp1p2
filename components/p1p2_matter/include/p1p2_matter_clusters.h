/*
 * P1P2 Matter Clusters — Endpoint and cluster ID definitions
 *
 * Maps P1P2 HVAC data to standard Matter clusters where possible,
 * and uses manufacturer-specific clusters for VRV-specific data.
 *
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Endpoint IDs ---- */
#define EP_ROOT             0   /* Root node (mandatory) */
#define EP_THERMOSTAT       1   /* Thermostat cluster */
#define EP_FAN              2   /* Fan Control cluster */
#define EP_TEMP_SENSORS     3   /* Temperature Measurement clusters */
#define EP_CUSTOM_VRV       4   /* Manufacturer-specific VRV data */
#define EP_DHW_ONOFF        5   /* On/Off for DHW control */

/* ---- Standard Matter Cluster IDs ---- */
#define CLUSTER_ON_OFF              0x0006
#define CLUSTER_THERMOSTAT          0x0201
#define CLUSTER_FAN_CONTROL         0x0202
#define CLUSTER_TEMP_MEASUREMENT    0x0402

/* ---- Thermostat Cluster Attributes (0x0201) ---- */
#define ATTR_LOCAL_TEMPERATURE          0x0000  /* int16_t, °C × 100 */
#define ATTR_OCCUPIED_COOLING_SETPOINT  0x0011  /* int16_t, °C × 100 */
#define ATTR_OCCUPIED_HEATING_SETPOINT  0x0012  /* int16_t, °C × 100 */
#define ATTR_SYSTEM_MODE                0x001C  /* enum8 */
#define ATTR_THERMOSTAT_RUNNING_STATE   0x0029  /* bitmap16 */
#define ATTR_CONTROL_SEQUENCE           0x001B  /* enum8 */

/* Thermostat SystemMode values */
#define THERMOSTAT_MODE_OFF         0x00
#define THERMOSTAT_MODE_AUTO        0x01
#define THERMOSTAT_MODE_COOL        0x03
#define THERMOSTAT_MODE_HEAT        0x04
#define THERMOSTAT_MODE_FAN_ONLY    0x07

/* Thermostat RunningState bit positions */
#define RUNNING_STATE_HEAT_ON       (1 << 0)
#define RUNNING_STATE_COOL_ON       (1 << 1)
#define RUNNING_STATE_FAN_ON        (1 << 2)

/* ---- Fan Control Cluster Attributes (0x0202) ---- */
#define ATTR_FAN_MODE               0x0000  /* enum8 */
#define ATTR_FAN_MODE_SEQUENCE      0x0001  /* enum8 */

/* FanMode values */
#define FAN_MODE_OFF    0x00
#define FAN_MODE_LOW    0x01
#define FAN_MODE_MED    0x02
#define FAN_MODE_HIGH   0x03
#define FAN_MODE_AUTO   0x05

/* ---- Temperature Measurement Cluster Attributes (0x0402) ---- */
#define ATTR_MEASURED_VALUE          0x0000  /* int16_t, °C × 100 */
#define ATTR_MIN_MEASURED_VALUE      0x0001  /* int16_t */
#define ATTR_MAX_MEASURED_VALUE      0x0002  /* int16_t */

/* ---- Manufacturer-Specific Custom Cluster (VRV Data) ---- */
#define CLUSTER_CUSTOM_VRV          0xFFF10001  /* Vendor-specific */

/* Custom VRV attributes */
#define ATTR_VRV_COMPRESSOR_FREQ    0x0000  /* uint16_t, Hz */
#define ATTR_VRV_FLOW_RATE          0x0001  /* uint16_t, L/min × 10 */
#define ATTR_VRV_ERROR_CODE         0x0002  /* uint16_t */
#define ATTR_VRV_OPERATION_HOURS    0x0003  /* uint32_t */
#define ATTR_VRV_COMPRESSOR_STARTS  0x0004  /* uint32_t */
#define ATTR_VRV_BUS_VOLTAGE_P1     0x0005  /* uint16_t, mV */
#define ATTR_VRV_BUS_VOLTAGE_P2     0x0006  /* uint16_t, mV */
#define ATTR_VRV_PACKET_COUNT       0x0007  /* uint32_t */

/* ---- On/Off Cluster Attributes (0x0006) ---- */
#define ATTR_ON_OFF                 0x0000  /* bool */

#ifdef __cplusplus
}
#endif
