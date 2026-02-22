/*
 * P1P2 Protocol — Public API for F-series packet decode and control
 *
 * Decodes P1/P2 bus packets into structured HVAC data and generates
 * control responses for auxiliary controller operation.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "p1p2_bus_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HVAC system mode — matches Matter Thermostat SystemMode attribute
 */
typedef enum {
    P1P2_MODE_OFF   = 0,
    P1P2_MODE_HEAT  = 1,
    P1P2_MODE_COOL  = 2,
    P1P2_MODE_AUTO  = 3,
    P1P2_MODE_FAN   = 4,
    P1P2_MODE_DRY   = 5,
} p1p2_system_mode_t;

/*
 * Fan speed — matches Matter FanControl FanMode attribute
 */
typedef enum {
    P1P2_FAN_AUTO = 0,
    P1P2_FAN_LOW  = 1,
    P1P2_FAN_MED  = 2,
    P1P2_FAN_HIGH = 3,
} p1p2_fan_mode_t;

/*
 * Running state — matches Matter Thermostat RunningState
 */
typedef enum {
    P1P2_RUNNING_IDLE    = 0,
    P1P2_RUNNING_HEATING = 1,
    P1P2_RUNNING_COOLING = 2,
} p1p2_running_state_t;

/*
 * Changed bitmask — tracks which HVAC state fields were modified
 * by the most recent decode cycle. The Matter layer checks this to
 * avoid pushing unchanged attributes.
 */
#define CHANGED_POWER          (1 << 0)
#define CHANGED_MODE           (1 << 1)
#define CHANGED_TEMP_COOL      (1 << 2)
#define CHANGED_TEMP_HEAT      (1 << 3)
#define CHANGED_ROOM_TEMP      (1 << 4)
#define CHANGED_OUTDOOR_TEMP   (1 << 5)
#define CHANGED_FAN_COOL       (1 << 6)
#define CHANGED_FAN_HEAT       (1 << 7)
#define CHANGED_COMPRESSOR     (1 << 8)
#define CHANGED_FLOW_RATE      (1 << 9)
#define CHANGED_ERROR_CODE     (1 << 10)
#define CHANGED_OP_HOURS       (1 << 11)
#define CHANGED_COMP_STARTS    (1 << 12)
#define CHANGED_DHW            (1 << 13)
#define CHANGED_WATER_TEMPS    (1 << 14)
#define CHANGED_ZONES          (1 << 15)

/*
 * Decoded HVAC state — the "model" of the VRV system.
 * Updated by the protocol decoder from bus packets.
 * Read by the Matter layer to update cluster attributes.
 */
typedef struct {
    /* Change tracking */
    uint32_t changed;              /* bitmask of fields modified since last read */

    /* Thermostat */
    bool     power;                /* on/off */
    p1p2_system_mode_t mode;       /* current operating mode */
    p1p2_running_state_t running;  /* currently heating/cooling/idle */
    int16_t  target_temp_cool;     /* target cooling temp × 10 (e.g., 240 = 24.0°C) */
    int16_t  target_temp_heat;     /* target heating temp × 10 */
    int16_t  room_temp;            /* room/return temperature × 10 */

    /* Fan */
    p1p2_fan_mode_t fan_mode_cool;
    p1p2_fan_mode_t fan_mode_heat;

    /* Temperatures (× 10) */
    int16_t  outdoor_temp;
    int16_t  leaving_water_temp;
    int16_t  return_water_temp;

    /* VRV-specific diagnostics */
    uint16_t compressor_freq;      /* Hz */
    uint16_t flow_rate;            /* L/min × 10 */
    uint16_t error_code;           /* active error code, 0 = none */
    uint32_t operation_hours;
    uint32_t compressor_starts;

    /* DHW (domestic hot water) */
    bool     dhw_active;
    int16_t  dhw_temp;             /* × 10 */
    int16_t  dhw_target;           /* × 10 */

    /* HVAC zones (for FDYQ model M) */
    uint8_t  active_zones;         /* bitmask of active zones */

    /* Bus diagnostics */
    float    bus_voltage_p1;
    float    bus_voltage_p2;

    /* Timestamps (for change detection) */
    int64_t  last_update_us;       /* esp_timer_get_time() of last decode */
    uint32_t packet_count;         /* total packets decoded */
    bool     data_valid;           /* true after at least one full cycle decoded */
} p1p2_hvac_state_t;

/*
 * Control command — sent from Matter command callbacks to protocol task
 */
typedef enum {
    P1P2_CMD_SET_POWER,
    P1P2_CMD_SET_MODE,
    P1P2_CMD_SET_TEMP_COOL,
    P1P2_CMD_SET_TEMP_HEAT,
    P1P2_CMD_SET_FAN_COOL,
    P1P2_CMD_SET_FAN_HEAT,
    P1P2_CMD_SET_DHW_POWER,
    P1P2_CMD_SET_DHW_TEMP,
    P1P2_CMD_SET_ZONES,
} p1p2_cmd_type_t;

typedef struct {
    p1p2_cmd_type_t type;
    int32_t         value;
} p1p2_control_cmd_t;

/* Control level */
#define P1P2_CONTROL_OFF         0  /* No control, read-only */
#define P1P2_CONTROL_AUX         1  /* Auxiliary controller active */
#define P1P2_CONTROL_MONITOR     5  /* Monitor only (no responses) */

/*
 * Initialize the protocol engine.
 * rx_queue: receives packets from bus I/O
 * tx_queue: sends write requests to bus I/O
 */
esp_err_t p1p2_protocol_init(QueueHandle_t rx_queue, QueueHandle_t tx_queue);

/*
 * Get pointer to the current HVAC state (read-only from other tasks).
 * The protocol task updates this structure; other tasks may read it.
 * For thread safety, reading is done via copy or with the protocol mutex.
 */
const p1p2_hvac_state_t *p1p2_protocol_get_state(void);

/*
 * Copy current HVAC state (thread-safe snapshot).
 */
void p1p2_protocol_get_state_copy(p1p2_hvac_state_t *out);

/*
 * Get the command queue handle for sending control commands to protocol task.
 */
QueueHandle_t p1p2_protocol_get_cmd_queue(void);

/*
 * Send a control command (convenience wrapper).
 */
esp_err_t p1p2_protocol_send_cmd(p1p2_cmd_type_t type, int32_t value);

/*
 * Set control level.
 */
void p1p2_protocol_set_control_level(uint8_t level);

/*
 * Get control level.
 */
uint8_t p1p2_protocol_get_control_level(void);

/*
 * Clear the changed bitmask (call after Matter has consumed state).
 */
void p1p2_protocol_clear_changed(void);

/*
 * Log a raw packet in hex format for debugging.
 */
void p1p2_log_packet(const p1p2_packet_t *pkt, const char *prefix);

#ifdef __cplusplus
}
#endif
