/*
 * P1P2 Parameter Tables — Packet field definitions for F-series decode
 *
 * These tables define the byte offset, length, and meaning of each
 * field within F-series P1/P2 packets. They replace the large switch
 * statements in P1P2_ParameterConversion.h with data-driven lookup.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Parameter value types
 */
typedef enum {
    PARAM_TYPE_U8,        /* unsigned 8-bit */
    PARAM_TYPE_S8,        /* signed 8-bit */
    PARAM_TYPE_U16,       /* unsigned 16-bit big-endian */
    PARAM_TYPE_S16,       /* signed 16-bit big-endian */
    PARAM_TYPE_U16_LE,    /* unsigned 16-bit little-endian */
    PARAM_TYPE_U32,       /* unsigned 32-bit big-endian */
    PARAM_TYPE_FLAG8,     /* 8-bit flags (individual bits) */
    PARAM_TYPE_TEMP8,     /* 8-bit temperature (raw °C) */
    PARAM_TYPE_TEMP16,    /* 16-bit temperature (°C × 10, signed) */
    PARAM_TYPE_FAN,       /* fan speed encoded value */
    PARAM_TYPE_MODE,      /* operating mode encoded value */
} p1p2_param_type_t;

/*
 * Parameter category (maps to Matter cluster)
 */
typedef enum {
    PARAM_CAT_THERMOSTAT,
    PARAM_CAT_FAN,
    PARAM_CAT_TEMP_SENSOR,
    PARAM_CAT_DIAGNOSTIC,
    PARAM_CAT_COUNTER,
    PARAM_CAT_SETTING,
} p1p2_param_cat_t;

/*
 * A single parameter field definition.
 */
typedef struct {
    uint8_t          packet_type;   /* P1/P2 packet type (0x10, 0x38, etc.) */
    uint8_t          payload_offset;/* byte offset within payload */
    uint8_t          byte_length;   /* 1, 2, or 4 bytes */
    p1p2_param_type_t value_type;
    p1p2_param_cat_t  category;
    const char       *name;         /* human-readable name */
    float             scale;        /* multiply raw value by this to get real value */
    float             offset;       /* add this after scaling */
} p1p2_param_def_t;

/*
 * F-series parameter table.
 * This is a subset of the most important fields; the full P1P2_ParameterConversion.h
 * has thousands of parameters across all models. We start with the ones needed
 * for Matter cluster attributes.
 */
static const p1p2_param_def_t f_series_params[] = {
    /* ---- Packet 0x10: Main status ---- */
    { 0x10, 0, 1, PARAM_TYPE_FLAG8,  PARAM_CAT_THERMOSTAT, "power",              1.0, 0 },
    { 0x10, 2, 1, PARAM_TYPE_MODE,   PARAM_CAT_THERMOSTAT, "operating_mode",     1.0, 0 },
    { 0x10, 4, 1, PARAM_TYPE_TEMP8,  PARAM_CAT_THERMOSTAT, "target_temp_cool",   1.0, 0 },
    { 0x10, 6, 1, PARAM_TYPE_FAN,    PARAM_CAT_FAN,        "fan_speed_cool",     1.0, 0 },
    { 0x10, 8, 1, PARAM_TYPE_TEMP8,  PARAM_CAT_THERMOSTAT, "target_temp_heat",   1.0, 0 },
    { 0x10, 10, 1, PARAM_TYPE_FAN,   PARAM_CAT_FAN,        "fan_speed_heat",     1.0, 0 },

    /* ---- Packet 0x11: Temperature readings ---- */
    { 0x11, 0, 1, PARAM_TYPE_TEMP8,  PARAM_CAT_TEMP_SENSOR, "room_temp",         1.0, 0 },
    { 0x11, 2, 1, PARAM_TYPE_S8,     PARAM_CAT_TEMP_SENSOR, "outdoor_temp",      1.0, 0 },

    /* ---- Packet 0x14: Compressor/flow data ---- */
    { 0x14, 0, 2, PARAM_TYPE_U16,    PARAM_CAT_DIAGNOSTIC,  "compressor_freq",   1.0, 0 },
    { 0x14, 2, 2, PARAM_TYPE_U16,    PARAM_CAT_DIAGNOSTIC,  "flow_rate",         0.1, 0 },

    /* ---- Packet 0xA3: Counter data ---- */
    { 0xA3, 0, 4, PARAM_TYPE_U32,    PARAM_CAT_COUNTER,     "operation_hours",   1.0, 0 },
    { 0xA3, 4, 4, PARAM_TYPE_U32,    PARAM_CAT_COUNTER,     "compressor_starts", 1.0, 0 },

    /* Sentinel */
    { 0, 0, 0, 0, 0, NULL, 0, 0 },
};

#define F_SERIES_PARAM_COUNT  (sizeof(f_series_params) / sizeof(f_series_params[0]) - 1)

#ifdef __cplusplus
}
#endif
