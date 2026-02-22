/*
 * P1P2 Bus I/O — Public API for ESP32-C6 MCPWM-based P1/P2 bus communication
 *
 * This component replaces the ATmega328P Timer1-based bit-banging with
 * ESP32-C6 MCPWM capture (RX) + MCPWM generator (TX) + GPTimer (mid-bit sampling).
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
#include "p1p2_bus_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bus I/O configuration passed to p1p2_bus_init().
 */
typedef struct {
    int gpio_rx;            /* MCPWM capture input pin */
    int gpio_tx;            /* MCPWM generator output pin */
    int gpio_adc0;          /* ADC channel 0 (bus voltage) */
    int gpio_adc1;          /* ADC channel 1 (bus voltage) */
    int gpio_led_power;
    int gpio_led_read;
    int gpio_led_write;
    int gpio_led_error;
    bool enable_adc;        /* enable bus voltage monitoring */
    bool echo_writes;       /* read-back written bytes for verification (default true) */
    uint8_t allow_pause;    /* max inter-byte pause in bit-times before EOP (default 9) */
} p1p2_bus_config_t;

/* Default configuration macro */
#define P1P2_BUS_CONFIG_DEFAULT() { \
    .gpio_rx        = CONFIG_P1P2_GPIO_RX, \
    .gpio_tx        = CONFIG_P1P2_GPIO_TX, \
    .gpio_adc0      = CONFIG_P1P2_GPIO_ADC0, \
    .gpio_adc1      = CONFIG_P1P2_GPIO_ADC1, \
    .gpio_led_power = CONFIG_P1P2_GPIO_LED_POWER, \
    .gpio_led_read  = CONFIG_P1P2_GPIO_LED_READ, \
    .gpio_led_write = CONFIG_P1P2_GPIO_LED_WRITE, \
    .gpio_led_error = CONFIG_P1P2_GPIO_LED_ERROR, \
    .enable_adc     = true, \
    .echo_writes    = true, \
    .allow_pause    = P1P2_ALLOW_PAUSE_BETWEEN_BYTES, \
}

/*
 * Initialize bus I/O hardware (MCPWM, GPTimer, GPIO, ADC).
 * Must be called before any other p1p2_bus function.
 * Returns ESP_OK on success.
 */
esp_err_t p1p2_bus_init(const p1p2_bus_config_t *config);

/*
 * Deinitialize bus I/O hardware and free resources.
 */
void p1p2_bus_deinit(void);

/*
 * Get the queue handle for received packets.
 * The bus_io_task assembles bytes from the ISR ring buffer into packets
 * and posts them to this queue. The protocol task reads from it.
 */
QueueHandle_t p1p2_bus_get_rx_queue(void);

/*
 * Get the queue handle for write requests.
 * The protocol/control task posts write requests here.
 * The bus_io_task picks them up and schedules transmission.
 */
QueueHandle_t p1p2_bus_get_tx_queue(void);

/*
 * Read a complete packet (blocking).
 * Convenience wrapper: blocks until a packet is available or timeout.
 * Returns number of bytes in packet, or 0 on timeout.
 */
uint8_t p1p2_bus_read_packet(p1p2_packet_t *pkt, uint32_t timeout_ms);

/*
 * Write a packet to the bus.
 * Queues a write request with specified delay after last bus activity.
 * CRC is appended automatically if crc_gen != 0.
 * Returns ESP_OK if successfully queued.
 */
esp_err_t p1p2_bus_write_packet(const uint8_t *data, uint8_t length,
                                 uint16_t delay_ms,
                                 uint8_t crc_gen, uint8_t crc_feed);

/*
 * Check if a packet is available in the RX queue (non-blocking).
 */
bool p1p2_bus_packet_available(void);

/*
 * Check if the transmitter is idle (no pending writes).
 */
bool p1p2_bus_write_ready(void);

/*
 * Set echo mode: if true, transmitted bytes are read back for verification.
 */
void p1p2_bus_set_echo(bool echo);

/*
 * Set the max inter-byte pause (in bit times) before end-of-packet detection.
 */
void p1p2_bus_set_allow_pause(uint8_t bit_times);

/*
 * Get current ADC results and reset min/max.
 */
void p1p2_bus_get_adc(p1p2_adc_results_t *results);

/*
 * Get bus statistics.
 */
void p1p2_bus_get_stats(p1p2_bus_stats_t *stats);

/*
 * LED control.
 */
void p1p2_led_power(bool on);
void p1p2_led_read(bool on);
void p1p2_led_write(bool on);
void p1p2_led_error(bool on);

#ifdef __cplusplus
}
#endif
