/*
 * P1P2 Bus — Packet assembly, CRC, ring buffer management, public API
 *
 * Ties together p1p2_mcpwm_rx.c and p1p2_mcpwm_tx.c with FreeRTOS queues
 * for inter-task communication.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "p1p2_bus.h"
#include "p1p2_bus_types.h"
#include "p1p2_bus_config.h"

static const char *TAG = "p1p2_bus";

/* ---- Shared ring buffers (written by ISR, read by bus_io_task) ---- */

volatile uint8_t     rx_buffer[P1P2_RX_BUFFER_SIZE];
volatile p1p2_error_t error_buffer[P1P2_RX_BUFFER_SIZE];
volatile uint16_t    delta_buffer[P1P2_RX_BUFFER_SIZE];
volatile uint8_t     rx_buffer_head  = 0;
volatile uint8_t     rx_buffer_head2 = P1P2_NO_HEAD2;
volatile uint8_t     rx_buffer_tail  = 0;

/* Shared time_msec counter (ISR ms timer increments, TX checks) */
volatile uint16_t    time_msec = 0;

/* Configuration shared with ISRs */
volatile uint8_t     echo_enabled = 1;
volatile uint8_t     allow_pause  = P1P2_ALLOW_PAUSE_BETWEEN_BYTES;

/* LED GPIO numbers */
int gpio_led_power;
int gpio_led_read;
int gpio_led_write;
int gpio_led_error;

/* FreeRTOS queues */
static QueueHandle_t rx_packet_queue = NULL;
static QueueHandle_t tx_request_queue = NULL;

/* Bus statistics */
static p1p2_bus_stats_t bus_stats;

/* External init functions from rx/tx modules */
extern esp_err_t p1p2_rx_init(int gpio_rx);
extern void      p1p2_rx_deinit(void);
extern esp_err_t p1p2_tx_init(int gpio_tx, int gpio_rx);
extern void      p1p2_tx_deinit(void);
extern void      p1p2_tx_write_byte(uint8_t b, uint16_t delay);
extern bool      p1p2_tx_is_idle(void);
extern bool      p1p2_tx_write_ready(void);
extern void      p1p2_tx_set_delay_timeout(uint16_t timeout_ms);

/* External ADC init */
extern esp_err_t p1p2_adc_init(int gpio_adc0, int gpio_adc1);
extern void      p1p2_adc_deinit(void);
extern void      p1p2_adc_get_results(p1p2_adc_results_t *results);

/*
 * CRC calculation — matches ATmega implementation.
 * Polynomial: crc_gen (typically 0xD9 for Daikin)
 * Feed: crc_feed (typically 0x00)
 */
static uint8_t calc_crc(const uint8_t *data, uint8_t length,
                         uint8_t crc_gen, uint8_t crc_feed)
{
    uint8_t crc = crc_feed;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t c = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = ((crc ^ c) & 0x01) ? ((crc >> 1) ^ crc_gen) : (crc >> 1);
            c >>= 1;
        }
    }
    return crc;
}

/*
 * Read next byte from ISR ring buffer (non-blocking).
 * Returns true if a byte was available.
 */
static bool ring_buffer_read(uint8_t *byte_out, p1p2_error_t *error_out,
                              uint16_t *delta_out)
{
    uint8_t head = rx_buffer_head;
    uint8_t tail = rx_buffer_tail;
    if (head == tail) return false;

    if (++tail >= P1P2_RX_BUFFER_SIZE) tail = 0;

    *byte_out  = rx_buffer[tail];
    *error_out = error_buffer[tail];
    *delta_out = delta_buffer[tail];
    rx_buffer_tail = tail;
    return true;
}

/*
 * Check if there's data with EOP in the ring buffer.
 */
static bool ring_buffer_has_packet(void)
{
    uint8_t head = rx_buffer_head;
    uint8_t tail = rx_buffer_tail;
    while (head != tail) {
        if (++tail >= P1P2_RX_BUFFER_SIZE) tail = 0;
        if (error_buffer[tail] & P1P2_SIGNAL_EOP) return true;
    }
    return false;
}

/*
 * ============================================================
 * Bus I/O Task — Assembles packets from ring buffer
 * ============================================================
 * Runs at priority 22 (highest non-ISR).
 * Reads bytes from the ISR ring buffer, assembles them into packets,
 * and posts complete packets to the RX queue for the protocol task.
 * Also picks up write requests from the TX queue and transmits them.
 */
static void bus_io_task(void *pvParameters)
{
    p1p2_packet_t pkt;
    p1p2_write_request_t wr_req;
    uint8_t byte_val;
    p1p2_error_t err;
    uint16_t delta;
    bool assembling = false;

    memset(&pkt, 0, sizeof(pkt));

    while (1) {
        /* Check for write requests (non-blocking) */
        if (xQueueReceive(tx_request_queue, &wr_req, 0) == pdTRUE) {
            /* Compute CRC if requested */
            uint8_t total_len = wr_req.length;
            if (wr_req.crc_gen) {
                uint8_t crc = calc_crc(wr_req.data, wr_req.length,
                                       wr_req.crc_gen, wr_req.crc_feed);
                wr_req.data[total_len++] = crc;
            }

            /* Queue bytes for transmission */
            for (uint8_t i = 0; i < total_len; i++) {
                uint16_t d = (i == 0) ? wr_req.delay_ms : 0;
                p1p2_tx_write_byte(wr_req.data[i], d);
            }
            bus_stats.packets_sent++;
        }

        /* Read bytes from ISR ring buffer */
        while (ring_buffer_read(&byte_val, &err, &delta)) {
            if (!assembling) {
                memset(&pkt, 0, sizeof(pkt));
                pkt.delta = delta;
                assembling = true;
            }

            if (pkt.length < P1P2_MAX_PACKET_SIZE) {
                pkt.data[pkt.length] = byte_val;
                pkt.errors[pkt.length] = err & ~P1P2_SIGNAL_EOP;
                if (err & P1P2_ERROR_MASK & ~P1P2_SIGNAL_EOP) {
                    pkt.has_error = true;
                }
                pkt.length++;
            }

            if (err & P1P2_SIGNAL_EOP) {
                /* Packet complete — post to queue */
                assembling = false;
                bus_stats.packets_received++;

                /* Update error counters */
                for (uint8_t i = 0; i < pkt.length; i++) {
                    if (pkt.errors[i] & P1P2_ERROR_CRC_CS) bus_stats.crc_errors++;
                    if (pkt.errors[i] & P1P2_ERROR_PE)     bus_stats.parity_errors++;
                    if (pkt.errors[i] & (P1P2_ERROR_BE | P1P2_ERROR_BC))
                        bus_stats.collision_errors++;
                    if (pkt.errors[i] & P1P2_ERROR_OR)     bus_stats.overrun_errors++;
                }

                if (rx_packet_queue) {
                    /* Non-blocking post — drop packet if queue full */
                    xQueueSend(rx_packet_queue, &pkt, 0);
                }
            }
        }

        /* Short sleep to yield CPU — bus_io_task is high priority */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*
 * ============================================================
 * Public API
 * ============================================================
 */

esp_err_t p1p2_bus_init(const p1p2_bus_config_t *config)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing P1P2 bus I/O");

    /* Store LED pin numbers for ISR use */
    gpio_led_power = config->gpio_led_power;
    gpio_led_read  = config->gpio_led_read;
    gpio_led_write = config->gpio_led_write;
    gpio_led_error = config->gpio_led_error;
    echo_enabled   = config->echo_writes ? 1 : 0;
    allow_pause    = config->allow_pause;

    /* Configure LED GPIOs */
    uint64_t led_mask = (1ULL << config->gpio_led_power) |
                        (1ULL << config->gpio_led_read) |
                        (1ULL << config->gpio_led_write) |
                        (1ULL << config->gpio_led_error);
    gpio_config_t led_cfg = {
        .pin_bit_mask = led_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);

    /* All LEDs on during init (like ATmega behavior) */
    gpio_set_level(config->gpio_led_power, 1);
    gpio_set_level(config->gpio_led_read, 1);
    gpio_set_level(config->gpio_led_write, 1);
    gpio_set_level(config->gpio_led_error, 1);

    /* Reset ring buffers */
    rx_buffer_head  = 0;
    rx_buffer_head2 = P1P2_NO_HEAD2;
    rx_buffer_tail  = 0;
    time_msec = 0;
    memset(&bus_stats, 0, sizeof(bus_stats));

    /* Create FreeRTOS queues */
    rx_packet_queue  = xQueueCreate(P1P2_PACKET_QUEUE_SIZE, sizeof(p1p2_packet_t));
    tx_request_queue = xQueueCreate(P1P2_PACKET_QUEUE_SIZE, sizeof(p1p2_write_request_t));
    if (!rx_packet_queue || !tx_request_queue) {
        ESP_LOGE(TAG, "Failed to create packet queues");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize RX (MCPWM capture + GPTimers) */
    ret = p1p2_rx_init(config->gpio_rx);
    if (ret != ESP_OK) return ret;

    /* Initialize TX (MCPWM operator/comparator/generator) */
    ret = p1p2_tx_init(config->gpio_tx, config->gpio_rx);
    if (ret != ESP_OK) return ret;

    /* Initialize ADC if enabled */
    if (config->enable_adc) {
        ret = p1p2_adc_init(config->gpio_adc0, config->gpio_adc1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC init failed (non-fatal): %s", esp_err_to_name(ret));
        }
    }

    /* Create bus I/O task at high priority */
    BaseType_t xret = xTaskCreate(bus_io_task, "bus_io", 4096, NULL, 22, NULL);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bus_io_task");
        return ESP_ERR_NO_MEM;
    }

    /* Turn off non-power LEDs after init */
    gpio_set_level(config->gpio_led_read, 0);
    gpio_set_level(config->gpio_led_write, 0);
    gpio_set_level(config->gpio_led_error, 0);

    ESP_LOGI(TAG, "P1P2 bus I/O initialized successfully");
    return ESP_OK;
}

void p1p2_bus_deinit(void)
{
    p1p2_rx_deinit();
    p1p2_tx_deinit();
    p1p2_adc_deinit();

    if (rx_packet_queue)  { vQueueDelete(rx_packet_queue);  rx_packet_queue = NULL; }
    if (tx_request_queue) { vQueueDelete(tx_request_queue); tx_request_queue = NULL; }
}

QueueHandle_t p1p2_bus_get_rx_queue(void)
{
    return rx_packet_queue;
}

QueueHandle_t p1p2_bus_get_tx_queue(void)
{
    return tx_request_queue;
}

uint8_t p1p2_bus_read_packet(p1p2_packet_t *pkt, uint32_t timeout_ms)
{
    if (xQueueReceive(rx_packet_queue, pkt, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return pkt->length;
    }
    return 0;
}

esp_err_t p1p2_bus_write_packet(const uint8_t *data, uint8_t length,
                                 uint16_t delay_ms,
                                 uint8_t crc_gen, uint8_t crc_feed)
{
    if (length > P1P2_MAX_PACKET_SIZE - 1) return ESP_ERR_INVALID_SIZE;

    p1p2_write_request_t req;
    memcpy(req.data, data, length);
    req.length   = length;
    req.delay_ms = delay_ms;
    req.crc_gen  = crc_gen;
    req.crc_feed = crc_feed;

    if (xQueueSend(tx_request_queue, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool p1p2_bus_packet_available(void)
{
    return (uxQueueMessagesWaiting(rx_packet_queue) > 0);
}

bool p1p2_bus_write_ready(void)
{
    return p1p2_tx_write_ready();
}

void p1p2_bus_set_echo(bool echo)
{
    echo_enabled = echo ? 1 : 0;
}

void p1p2_bus_set_allow_pause(uint8_t bit_times)
{
    allow_pause = bit_times;
}

void p1p2_bus_get_adc(p1p2_adc_results_t *results)
{
    p1p2_adc_get_results(results);
}

void p1p2_bus_get_stats(p1p2_bus_stats_t *stats)
{
    *stats = bus_stats;
    stats->uptime_us = esp_timer_get_time();
}

void p1p2_led_power(bool on) { gpio_set_level(gpio_led_power, on); }
void p1p2_led_read(bool on)  { gpio_set_level(gpio_led_read, on); }
void p1p2_led_write(bool on) { gpio_set_level(gpio_led_write, on); }
void p1p2_led_error(bool on) { gpio_set_level(gpio_led_error, on); }
