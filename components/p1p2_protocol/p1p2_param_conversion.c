/*
 * P1P2 Protocol Task — Packet processing, decode, and control response
 *
 * This is the main protocol engine that:
 * 1. Receives packets from the bus I/O queue
 * 2. Decodes them to update HVAC state
 * 3. Generates control responses (when acting as auxiliary controller)
 * 4. Processes commands from the Matter layer
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "p1p2_protocol.h"
#include "p1p2_fseries.h"
#include "p1p2_bus.h"
#include "p1p2_bus_types.h"

static const char *TAG = "p1p2_proto";

/* HVAC state — updated by decoder, read by Matter layer */
static p1p2_hvac_state_t hvac_state;
static SemaphoreHandle_t state_mutex;

/* Queues */
static QueueHandle_t rx_queue;    /* from bus I/O */
static QueueHandle_t tx_queue;    /* to bus I/O */
static QueueHandle_t cmd_queue;   /* from Matter/CLI */

/* Control level */
static volatile uint8_t control_level;

/* External functions from decode/control modules */
extern void p1p2_fseries_decode_packet(const p1p2_packet_t *pkt, p1p2_hvac_state_t *state);
extern void p1p2_fseries_control_init(int model);
extern uint8_t p1p2_fseries_build_response_38(const uint8_t *rb, uint8_t rb_len, uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3b(const uint8_t *rb, uint8_t rb_len, uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_39(const uint8_t *rb, uint8_t rb_len, uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3a(const uint8_t *rb, uint8_t rb_len, uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3c(const uint8_t *rb, uint8_t rb_len, uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_empty(const uint8_t *rb, uint8_t rb_len, uint8_t *wb, uint8_t wb_max);
extern esp_err_t p1p2_fseries_apply_command(const p1p2_control_cmd_t *cmd);

/*
 * Determine if a received packet requires an auxiliary controller response.
 * Returns the appropriate delay (in ms) before responding.
 * Returns 0 if no response is needed.
 *
 * F-series: packets addressed to 0x40 (aux controller) in the 0x30-0x3F range
 * need responses.
 */
static uint16_t packet_needs_response(const p1p2_packet_t *pkt)
{
    if (pkt->length < 3) return 0;

    uint8_t dst  = pkt->data[1];
    uint8_t type = pkt->data[2];

    /* Only respond to packets addressed to the auxiliary controller */
    if (dst != P1P2_ADDR_AUX_CTRL) return 0;

    /* Only respond to 0x3x packet types */
    if (type < 0x30 || type > 0x3F) return 0;

    /* Standard response delay: 25ms after the request packet */
    return 25;
}

/*
 * Build and send an auxiliary controller response.
 */
static void send_control_response(const p1p2_packet_t *pkt, uint16_t delay_ms)
{
    uint8_t wb[P1P2_MAX_PACKET_SIZE];
    uint8_t nwrite = 0;
    uint8_t type = pkt->data[2];

    switch (type) {
    case PKT_TYPE_CTRL_35:
    case PKT_TYPE_CTRL_36:
    case PKT_TYPE_CTRL_37:
        nwrite = p1p2_fseries_build_response_empty(pkt->data, pkt->length,
                                                    wb, sizeof(wb));
        break;

    case PKT_TYPE_CTRL_38:
        nwrite = p1p2_fseries_build_response_38(pkt->data, pkt->length,
                                                 wb, sizeof(wb));
        break;

    case PKT_TYPE_CTRL_39:
        nwrite = p1p2_fseries_build_response_39(pkt->data, pkt->length,
                                                 wb, sizeof(wb));
        break;

    case PKT_TYPE_CTRL_3A:
        nwrite = p1p2_fseries_build_response_3a(pkt->data, pkt->length,
                                                 wb, sizeof(wb));
        break;

    case PKT_TYPE_CTRL_3B:
        nwrite = p1p2_fseries_build_response_3b(pkt->data, pkt->length,
                                                 wb, sizeof(wb));
        break;

    case PKT_TYPE_CTRL_3C:
        nwrite = p1p2_fseries_build_response_3c(pkt->data, pkt->length,
                                                 wb, sizeof(wb));
        break;

    default:
        ESP_LOGD(TAG, "No response handler for packet type 0x%02X", type);
        return;
    }

    if (nwrite > 0) {
        esp_err_t ret = p1p2_bus_write_packet(wb, nwrite, delay_ms,
                                               F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send response for 0x%02X: %s",
                     type, esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "Sent response for 0x%02X (%d bytes, %dms delay)",
                     type, nwrite, delay_ms);
        }
    }
}

/*
 * Protocol task — main processing loop.
 * Priority 15 (below bus_io at 22, above matter at 10).
 */
static void protocol_task(void *pvParameters)
{
    p1p2_packet_t pkt;
    p1p2_control_cmd_t cmd;

    ESP_LOGI(TAG, "Protocol task started (control_level=%d)", control_level);

    while (1) {
        /* Process any pending control commands from Matter */
        while (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Processing command: type=%d value=%ld", cmd.type, (long)cmd.value);
            p1p2_fseries_apply_command(&cmd);
        }

        /* Wait for next packet from bus */
        if (xQueueReceive(rx_queue, &pkt, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Decode packet to update HVAC state */
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                p1p2_fseries_decode_packet(&pkt, &hvac_state);
                xSemaphoreGive(state_mutex);
            }

            /* If acting as auxiliary controller, send response if needed */
            if (control_level == P1P2_CONTROL_AUX) {
                uint16_t delay = packet_needs_response(&pkt);
                if (delay > 0) {
                    send_control_response(&pkt, delay);
                }
            }

            /* Log packet at debug level */
            if (pkt.length > 0) {
                ESP_LOGD(TAG, "Pkt: src=0x%02X dst=0x%02X type=0x%02X len=%d err=%s",
                         pkt.data[0], pkt.data[1], pkt.data[2], pkt.length,
                         pkt.has_error ? "YES" : "no");
            }
        }
    }
}

/*
 * ============================================================
 * Public API
 * ============================================================
 */

esp_err_t p1p2_protocol_init(QueueHandle_t bus_rx_queue, QueueHandle_t bus_tx_queue)
{
    rx_queue = bus_rx_queue;
    tx_queue = bus_tx_queue;

    /* Create command queue */
    cmd_queue = xQueueCreate(16, sizeof(p1p2_control_cmd_t));
    if (!cmd_queue) return ESP_ERR_NO_MEM;

    /* Create state mutex */
    state_mutex = xSemaphoreCreateMutex();
    if (!state_mutex) return ESP_ERR_NO_MEM;

    /* Initialize HVAC state */
    memset(&hvac_state, 0, sizeof(hvac_state));

    /* Set control level from Kconfig */
#ifdef CONFIG_P1P2_CONTROL_LEVEL
    control_level = CONFIG_P1P2_CONTROL_LEVEL;
#else
    control_level = P1P2_CONTROL_OFF;
#endif

    /* Initialize F-series control engine */
#ifdef CONFIG_P1P2_F_MODEL_ID
    p1p2_fseries_control_init(CONFIG_P1P2_F_MODEL_ID);
#else
    p1p2_fseries_control_init(F_MODEL_BCL);
#endif

    /* Start protocol task at priority 15 */
    BaseType_t ret = xTaskCreate(protocol_task, "protocol", 8192, NULL, 15, NULL);
    if (ret != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Protocol engine initialized");
    return ESP_OK;
}

const p1p2_hvac_state_t *p1p2_protocol_get_state(void)
{
    return &hvac_state;
}

void p1p2_protocol_get_state_copy(p1p2_hvac_state_t *out)
{
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(out, &hvac_state, sizeof(hvac_state));
        /* Clear changed flags after Matter has consumed the snapshot */
        hvac_state.changed = 0;
        xSemaphoreGive(state_mutex);
    }
}

void p1p2_protocol_clear_changed(void)
{
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        hvac_state.changed = 0;
        xSemaphoreGive(state_mutex);
    }
}

QueueHandle_t p1p2_protocol_get_cmd_queue(void)
{
    return cmd_queue;
}

esp_err_t p1p2_protocol_send_cmd(p1p2_cmd_type_t type, int32_t value)
{
    p1p2_control_cmd_t cmd = { .type = type, .value = value };
    if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void p1p2_protocol_set_control_level(uint8_t level)
{
    control_level = level;
    ESP_LOGI(TAG, "Control level set to %d", level);
}

uint8_t p1p2_protocol_get_control_level(void)
{
    return control_level;
}
