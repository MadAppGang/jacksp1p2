/*
 * P1P2 Bus Configuration — Timing constants and buffer sizes for ESP32-C6
 *
 * Ported from P1P2MQTT.h/.cpp (ATmega328P) to ESP32-C6 MCPWM/GPTimer peripherals.
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Timer resolution: 8 MHz to match ATmega tick counts exactly.
 * MCPWM capture timer and GPTimer both configured at 8 MHz.
 */
#define P1P2_TIMER_FREQ_HZ         8000000  /* 8 MHz — matches ATmega F_CPU */
#define P1P2_BAUD_RATE             9600

/* Tick counts at 8 MHz / 9600 baud — identical to ATmega values */
#define TICKS_PER_BIT              833      /* 104.17 µs */
#define TICKS_PER_SEMIBIT          416      /* 52.08 µs  */
#define TICKS_PER_BIT_AND_SEMIBIT  1249     /* 156.25 µs */

/* Suppression zone: ignore edges within 3/4 of a semibit after previous edge */
#define TICKS_SUPPRESSION          (TICKS_PER_SEMIBIT + TICKS_PER_SEMIBIT / 4)

/* Schedule delay for TX: must be >= 1.5 bits to safely start next byte */
#define TICKS_SCHEDULE_DELAY       TICKS_PER_BIT_AND_SEMIBIT

/*
 * Buffer sizes — F-Series defaults.
 * These match the original P1P2MQTT.h values for the default (non-H, non-MHI) case.
 */
#ifdef CONFIG_P1P2_F_SERIES
#define P1P2_TX_BUFFER_SIZE        25
#define P1P2_RX_BUFFER_SIZE        25
#else
#define P1P2_TX_BUFFER_SIZE        25
#define P1P2_RX_BUFFER_SIZE        25
#endif

/* Maximum packet size (bytes) for F-series */
#define P1P2_MAX_PACKET_SIZE       24

/* Allow pause between bytes (in bit times) before signaling end-of-packet */
#define P1P2_ALLOW_PAUSE_BETWEEN_BYTES  9

/* Ring buffer for assembled packets passed to protocol task */
#define P1P2_PACKET_QUEUE_SIZE     8

/* NO_HEAD2 sentinel — indicates no pending byte in rx_buffer_head2 */
#define P1P2_NO_HEAD2              0xFF

/* ADC configuration */
#define P1P2_ADC_AVG_SHIFT         4   /* average 16 samples before min/max */
#define P1P2_ADC_CNT_SHIFT         4   /* average 4096 samples for Vavg (~1s at ~4kSPS) */

/* GPIO pin defaults (overridden by Kconfig) */
#ifndef CONFIG_P1P2_GPIO_RX
#define CONFIG_P1P2_GPIO_RX        2
#endif
#ifndef CONFIG_P1P2_GPIO_TX
#define CONFIG_P1P2_GPIO_TX        3
#endif
#ifndef CONFIG_P1P2_GPIO_ADC0
#define CONFIG_P1P2_GPIO_ADC0      0
#endif
#ifndef CONFIG_P1P2_GPIO_ADC1
#define CONFIG_P1P2_GPIO_ADC1      1
#endif
#ifndef CONFIG_P1P2_GPIO_LED_POWER
#define CONFIG_P1P2_GPIO_LED_POWER 4
#endif
#ifndef CONFIG_P1P2_GPIO_LED_READ
#define CONFIG_P1P2_GPIO_LED_READ  5
#endif
#ifndef CONFIG_P1P2_GPIO_LED_WRITE
#define CONFIG_P1P2_GPIO_LED_WRITE 6
#endif
#ifndef CONFIG_P1P2_GPIO_LED_ERROR
#define CONFIG_P1P2_GPIO_LED_ERROR 7
#endif

#ifdef __cplusplus
}
#endif
