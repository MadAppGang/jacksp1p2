/*
 * P1P2 Bus Types — Packet, error, timing, and state types for ESP32-C6
 *
 * Ported from P1P2MQTT.h (ATmega328P).
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "p1p2_bus_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Error and signal flags — identical to original P1P2MQTT.h values.
 * These are stored per-byte in the error buffer alongside received data.
 */
#define P1P2_ERROR_SB       0x01  /* Start bit error during write */
#define P1P2_ERROR_BE       0x02  /* Data read-back error (bus collision) */
#define P1P2_ERROR_PE       0x04  /* Parity error */
#define P1P2_ERROR_OR       0x08  /* Read buffer overrun */
#define P1P2_ERROR_CRC_CS   0x10  /* CRC or checksum error */
#define P1P2_ERROR_BC       0x20  /* High bit half read-back error (bus collision) */
#define P1P2_SIGNAL_EOP     0x80  /* End of packet signal (not an error) */

/* Error mask: all real errors, excluding EOP signal */
#define P1P2_ERROR_MASK     0x7F

typedef uint8_t p1p2_error_t;

/*
 * RX state machine states — mirrors ATmega ISR states.
 * State 0: idle, waiting for start bit
 * State 1: waiting for next start bit (or EOP timeout)
 * State 2-9: data bits (falling edge = '0')
 * State 10: parity bit
 * State 11: stop bit
 */
typedef enum {
    RX_STATE_IDLE        = 0,
    RX_STATE_WAIT_START  = 1,
    RX_STATE_DATA_BIT_0  = 2,
    RX_STATE_DATA_BIT_1  = 3,
    RX_STATE_DATA_BIT_2  = 4,
    RX_STATE_DATA_BIT_3  = 5,
    RX_STATE_DATA_BIT_4  = 6,
    RX_STATE_DATA_BIT_5  = 7,
    RX_STATE_DATA_BIT_6  = 8,
    RX_STATE_DATA_BIT_7  = 9,
    RX_STATE_PARITY      = 10,
    RX_STATE_STOP        = 11,
} p1p2_rx_state_t;

/*
 * TX state machine states — mirrors ATmega ISR 20-state half-bit machine.
 * States 1-20: half-bit states during byte transmission.
 *   Odd states: first half of bit (low for '0', high for '1')
 *   Even states: second half of bit (always high)
 *   1-2: start bit
 *   3-18: data bits (LSB first)
 *   19-20: parity bit
 *   After 20: stop bit / schedule next byte or finish
 * State 99: waiting for scheduled delay to start transmission.
 * State 0: idle, not transmitting.
 */
typedef enum {
    TX_STATE_IDLE      = 0,
    TX_STATE_START_1   = 1,
    TX_STATE_START_2   = 2,
    TX_STATE_D0_1      = 3,
    TX_STATE_D0_2      = 4,
    TX_STATE_D1_1      = 5,
    TX_STATE_D1_2      = 6,
    TX_STATE_D2_1      = 7,
    TX_STATE_D2_2      = 8,
    TX_STATE_D3_1      = 9,
    TX_STATE_D3_2      = 10,
    TX_STATE_D4_1      = 11,
    TX_STATE_D4_2      = 12,
    TX_STATE_D5_1      = 13,
    TX_STATE_D5_2      = 14,
    TX_STATE_D6_1      = 15,
    TX_STATE_D6_2      = 16,
    TX_STATE_D7_1      = 17,
    TX_STATE_D7_2      = 18,
    TX_STATE_PARITY_1  = 19,
    TX_STATE_PARITY_2  = 20,
    TX_STATE_SCHEDULED = 99,
} p1p2_tx_state_t;

/*
 * Assembled packet — passed from bus I/O task to protocol task via queue.
 */
typedef struct {
    uint8_t  data[P1P2_MAX_PACKET_SIZE];
    p1p2_error_t errors[P1P2_MAX_PACKET_SIZE];
    uint16_t delta;             /* ms since previous byte/packet */
    uint8_t  length;            /* number of bytes in packet */
    bool     has_error;         /* true if any byte has a non-zero error flag */
} p1p2_packet_t;

/*
 * Write request — passed from protocol/control task to bus I/O task.
 */
typedef struct {
    uint8_t  data[P1P2_MAX_PACKET_SIZE];
    uint8_t  length;
    uint16_t delay_ms;          /* ms to wait after last bus activity before writing */
    uint8_t  crc_gen;           /* CRC generator polynomial, 0 to disable */
    uint8_t  crc_feed;          /* CRC initial value */
} p1p2_write_request_t;

/*
 * ADC measurement results — bus voltage monitoring.
 */
typedef struct {
    uint16_t v0_min;
    uint16_t v0_max;
    uint32_t v0_avg;
    uint16_t v1_min;
    uint16_t v1_max;
    uint32_t v1_avg;
} p1p2_adc_results_t;

/*
 * Bus statistics — error counters and uptime.
 */
typedef struct {
    uint32_t packets_received;
    uint32_t packets_sent;
    uint32_t crc_errors;
    uint32_t parity_errors;
    uint32_t collision_errors;
    uint32_t overrun_errors;
    int64_t  uptime_us;         /* from esp_timer_get_time() */
} p1p2_bus_stats_t;

#ifdef __cplusplus
}
#endif
