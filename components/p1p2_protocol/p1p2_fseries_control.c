/*
 * P1P2 F-Series Control — Auxiliary controller response construction
 *
 * Port of 0x38/0x3B response logic from P1P2Monitor.ino lines 2415-2556.
 *
 * When acting as an auxiliary controller, the ESP32-C6 must respond to
 * control packets (0x38 for model BCL/P, 0x3B for model M) by echoing
 * back the current state with any requested changes applied.
 *
 * The response also applies any pending write commands (temperature change,
 * mode change, etc.) from the Matter layer.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_log.h"
#include "p1p2_protocol.h"
#include "p1p2_fseries.h"
#include "p1p2_bus.h"
#include "p1p2_bus_types.h"

static const char *TAG = "p1p2_ctrl";

/* Pending write commands — set by Matter command callbacks */
#define MAX_PENDING_WRITES 8

typedef struct {
    uint8_t  packet_type;    /* which packet type this write targets */
    uint8_t  payload_offset; /* byte offset within response payload */
    uint8_t  value;          /* value to write */
    uint8_t  mask;           /* mask for preserving existing bits (0x00 = replace all) */
    uint8_t  count;          /* remaining write attempts (0 = inactive) */
} pending_write_t;

static pending_write_t pending_writes[MAX_PENDING_WRITES];
static int model_id;

void p1p2_fseries_control_init(int model)
{
    model_id = model;
    memset(pending_writes, 0, sizeof(pending_writes));
    ESP_LOGI(TAG, "F-series control initialized for model %d", model);
}

/*
 * Queue a pending parameter write.
 * This will be applied to the next matching control response.
 */
esp_err_t p1p2_fseries_queue_write(uint8_t packet_type, uint8_t payload_offset,
                                    uint8_t value, uint8_t mask, uint8_t count)
{
    for (int i = 0; i < MAX_PENDING_WRITES; i++) {
        if (pending_writes[i].count == 0) {
            pending_writes[i].packet_type = packet_type;
            pending_writes[i].payload_offset = payload_offset;
            pending_writes[i].value = value;
            pending_writes[i].mask = mask;
            pending_writes[i].count = count;
            ESP_LOGI(TAG, "Queued write: pkt=0x%02X off=%d val=0x%02X mask=0x%02X cnt=%d",
                     packet_type, payload_offset, value, mask, count);
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "Pending write buffer full");
    return ESP_ERR_NO_MEM;
}

/*
 * Apply pending writes to a response buffer.
 */
static void apply_pending_writes(uint8_t packet_type, uint8_t *wb, uint8_t wb_len)
{
    for (int i = 0; i < MAX_PENDING_WRITES; i++) {
        if (pending_writes[i].count && pending_writes[i].packet_type == packet_type) {
            uint8_t off = pending_writes[i].payload_offset;
            if (off < wb_len) {
                wb[off] = pending_writes[i].value |
                          (pending_writes[i].mask & wb[off]);
                pending_writes[i].count |= 0x80; /* mark as applied this cycle */
            }
        }
    }

    /* Decrement counts for applied writes */
    for (int i = 0; i < MAX_PENDING_WRITES; i++) {
        if (pending_writes[i].count & 0x80) {
            pending_writes[i].count--;
            pending_writes[i].count &= 0x7F;
            ESP_LOGI(TAG, "Write applied: pkt=0x%02X off=%d remaining=%d",
                     pending_writes[i].packet_type,
                     pending_writes[i].payload_offset,
                     pending_writes[i].count);
        }
    }
}

/*
 * Encode fan speed to F-series format.
 */
static uint8_t encode_fan_speed(p1p2_fan_mode_t mode)
{
    switch (mode) {
    case P1P2_FAN_LOW:  return F_FAN_LOW;
    case P1P2_FAN_MED:  return F_FAN_MED;
    case P1P2_FAN_HIGH: return F_FAN_HIGH;
    default:            return F_FAN_LOW;
    }
}

/*
 * Encode operating mode to F-series format.
 */
static uint8_t encode_mode(p1p2_system_mode_t mode)
{
    switch (mode) {
    case P1P2_MODE_HEAT: return F_MODE_HEAT | F_MODE_ACTIVE_MASK;
    case P1P2_MODE_COOL: return F_MODE_COOL | F_MODE_ACTIVE_MASK;
    case P1P2_MODE_AUTO: return F_MODE_AUTO | F_MODE_ACTIVE_MASK;
    case P1P2_MODE_FAN:  return F_MODE_FAN  | F_MODE_ACTIVE_MASK;
    case P1P2_MODE_DRY:  return F_MODE_DRY  | F_MODE_ACTIVE_MASK;
    default:             return F_MODE_COOL | F_MODE_ACTIVE_MASK;
    }
}

/*
 * Build auxiliary controller response to a 0x38 control packet.
 *
 * Model BCL (FDY — models A/B/C/L/LA):
 *   18 bytes total (header + 15-byte payload + CRC)
 *   Port of P1P2Monitor.ino lines 2416-2439
 *
 * Model P (FXMQ — models P/PA):
 *   20 bytes total (header + 17-byte payload + CRC)
 *   Port of P1P2Monitor.ino lines 2441-2469
 *
 * Returns response length (excluding CRC, which bus layer adds), or 0 if no response needed.
 */
uint8_t p1p2_fseries_build_response_38(const uint8_t *rb, uint8_t rb_len,
                                        uint8_t *wb, uint8_t wb_max)
{
    if (rb_len < 18) return 0; /* minimum packet size for 0x38 */

    uint8_t nwrite;

    /* Response header: src=aux controller, dst=indoor, type=same */
    wb[0] = P1P2_ADDR_AUX_CTRL;
    wb[1] = rb[0]; /* respond to sender */
    wb[2] = rb[2]; /* same packet type */

    if (model_id == F_MODEL_BCL) {
        /* ---- Model BCL (FDY): 15-byte payload ---- */
        nwrite = 18; /* 3 header + 15 payload (CRC added by bus layer) */
        if (wb_max < nwrite) return 0;

        /* Clear bytes 13-15 */
        wb[13] = 0x00;
        wb[14] = 0x00;
        wb[15] = 0x00;

        wb[3]  = rb[3] & 0x01;           /* Target status (power) */
        wb[4]  = (rb[5] & 0x07) | 0x60;  /* Target operating mode */
        wb[5]  = rb[7];                   /* Target temperature cooling */
        wb[6]  = 0x00;                    /* Clear change flag */
        wb[7]  = (rb[9] & 0x60) | 0x11;  /* Target fan speed cooling */
        wb[8]  = 0x00;                    /* Clear change flag */
        wb[9]  = rb[11];                  /* Target temperature heating */
        wb[10] = 0x00;                    /* Clear change flag */
        wb[11] = (rb[13] & 0x60) | 0x11; /* Target fan speed heating */
        wb[12] = rb[14] & 0x7F;          /* Clear change flag */
        wb[16] = rb[18];                  /* Target fan mode */
        wb[17] = 0x00;                    /* Change flag */

        /* Apply any pending writes from Matter commands */
        apply_pending_writes(PKT_TYPE_CTRL_38, &wb[3], nwrite - 3);

    } else if (model_id == F_MODEL_P) {
        /* ---- Model P (FXMQ): 17-byte payload ---- */
        nwrite = 20;
        if (wb_max < nwrite) return 0;

        wb[3]  = rb[3] & 0x01;
        wb[4]  = rb[5];
        wb[5]  = rb[7];
        wb[6]  = 0x00;
        wb[7]  = rb[9];
        wb[8]  = 0x00;
        wb[9]  = rb[11];
        wb[10] = 0x00;
        wb[11] = rb[13];
        wb[12] = rb[14];
        wb[13] = 0x00;
        wb[14] = 0x00;
        wb[15] = 0x00;
        wb[16] = rb[18];
        wb[17] = 0x00;
        wb[18] = 0;                       /* puzzle: initially 0, then 2, then 1 */
        wb[19] = 0x00;

        /* FXMQ special: if turning on, set bit in wb[16] */
        apply_pending_writes(PKT_TYPE_CTRL_38, &wb[3], nwrite - 3);

        /* FXMQ: if power is being turned on via pending write, set mode flag */
        for (int i = 0; i < MAX_PENDING_WRITES; i++) {
            if (pending_writes[i].count && pending_writes[i].packet_type == PKT_TYPE_CTRL_38) {
                if ((pending_writes[i].payload_offset == 0) &&
                    (wb[3] == 0x00) && (pending_writes[i].value)) {
                    wb[16] |= 0x20;
                }
            }
        }
    } else {
        return 0; /* model not supported for 0x38 */
    }

    return nwrite;
}

/*
 * Build auxiliary controller response to a 0x3B control packet.
 *
 * Model M (FDYQ):
 *   22 bytes total (header + 19-byte payload + CRC)
 *   Port of P1P2Monitor.ino lines 2514-2540
 */
uint8_t p1p2_fseries_build_response_3b(const uint8_t *rb, uint8_t rb_len,
                                        uint8_t *wb, uint8_t wb_max)
{
    if (model_id != F_MODEL_M) return 0;
    if (rb_len < 22) return 0;

    uint8_t nwrite = 22;
    if (wb_max < nwrite) return 0;

    wb[0] = P1P2_ADDR_AUX_CTRL;
    wb[1] = rb[0];
    wb[2] = rb[2];

    /* Clear bytes 13-18 */
    for (int w = 13; w <= 18; w++) wb[w] = 0x00;

    wb[3]  = rb[3] & 0x01;           /* Target status */
    wb[4]  = (rb[5] & 0x07) | 0x60;  /* Target operating mode */
    wb[5]  = rb[7];                   /* Target temperature cooling */
    wb[6]  = 0x00;                    /* Clear change flag */
    wb[7]  = (rb[9] & 0x60) | 0x11;  /* Target fan speed cooling */
    wb[8]  = 0x00;                    /* Clear change flag */
    wb[9]  = rb[11];                  /* Target temperature heating */
    wb[10] = 0x00;                    /* Clear change flag */
    wb[11] = (rb[13] & 0x60) | 0x11; /* Target fan speed heating */
    wb[12] = rb[14] & 0x7F;          /* Clear change flag */
    wb[19] = rb[20];                  /* Active HVAC zones */
    wb[20] = rb[21] & 0x03;          /* Target fan mode */
    wb[21] = 0x00;                    /* Change flag */

    /* Apply pending writes */
    apply_pending_writes(PKT_TYPE_CTRL_3B, &wb[3], nwrite - 3);

    return nwrite;
}

/*
 * Build response to 0x39 packet (filter warning / status).
 */
uint8_t p1p2_fseries_build_response_39(const uint8_t *rb, uint8_t rb_len,
                                        uint8_t *wb, uint8_t wb_max)
{
    wb[0] = P1P2_ADDR_AUX_CTRL;
    wb[1] = rb[0];
    wb[2] = rb[2];

    if (model_id == F_MODEL_BCL) {
        /* 4-byte payload */
        uint8_t nwrite = 7;
        if (wb_max < nwrite) return 0;
        wb[3] = 0x00;
        wb[4] = 0x00;
        wb[5] = rb[11];
        wb[6] = rb[12];
        return nwrite;
    } else if (model_id == F_MODEL_P) {
        /* 5-byte all-zero payload */
        uint8_t nwrite = 8;
        if (wb_max < nwrite) return 0;
        memset(&wb[3], 0, 5);
        return nwrite;
    }

    return 0;
}

/*
 * Build response to 0x3A packet (FXMQ only).
 */
uint8_t p1p2_fseries_build_response_3a(const uint8_t *rb, uint8_t rb_len,
                                        uint8_t *wb, uint8_t wb_max)
{
    if (model_id != F_MODEL_P) return 0;

    uint8_t nwrite = 11; /* 8-byte all-zero payload */
    if (wb_max < nwrite) return 0;

    wb[0] = P1P2_ADDR_AUX_CTRL;
    wb[1] = rb[0];
    wb[2] = rb[2];
    memset(&wb[3], 0, 8);

    return nwrite;
}

/*
 * Build response to 0x3C packet (FDYQ filter, model M only).
 */
uint8_t p1p2_fseries_build_response_3c(const uint8_t *rb, uint8_t rb_len,
                                        uint8_t *wb, uint8_t wb_max)
{
    if (model_id != F_MODEL_M) return 0;

    uint8_t nwrite = 5; /* 2-byte zero payload */
    if (wb_max < nwrite) return 0;

    wb[0] = P1P2_ADDR_AUX_CTRL;
    wb[1] = rb[0];
    wb[2] = rb[2];
    wb[3] = 0x00;
    wb[4] = 0x00;

    return nwrite;
}

/*
 * Build response for empty-payload reply (0x35, 0x36, 0x37).
 */
uint8_t p1p2_fseries_build_response_empty(const uint8_t *rb, uint8_t rb_len,
                                           uint8_t *wb, uint8_t wb_max)
{
    uint8_t nwrite = 3; /* header only, CRC added by bus layer */
    if (wb_max < nwrite) return 0;

    wb[0] = P1P2_ADDR_AUX_CTRL;
    wb[1] = rb[0];
    wb[2] = rb[2];

    return nwrite;
}

/*
 * Translate a Matter control command into a pending F-series write.
 */
esp_err_t p1p2_fseries_apply_command(const p1p2_control_cmd_t *cmd)
{
    uint8_t pkt_type;

    /* Determine which packet type to target based on model */
    if (model_id == F_MODEL_M) {
        pkt_type = PKT_TYPE_CTRL_3B;
    } else {
        pkt_type = PKT_TYPE_CTRL_38;
    }

    switch (cmd->type) {
    case P1P2_CMD_SET_POWER:
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_STATUS,
                                        cmd->value ? 0x01 : 0x00, 0x00, 3);

    case P1P2_CMD_SET_MODE:
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_MODE,
                                        encode_mode((p1p2_system_mode_t)cmd->value),
                                        0x00, 3);

    case P1P2_CMD_SET_TEMP_COOL:
        /* value is temperature × 10, F-series uses integer °C */
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_COOL_TEMP,
                                        (uint8_t)(cmd->value / 10), 0x00, 3);

    case P1P2_CMD_SET_TEMP_HEAT:
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_HEAT_TEMP,
                                        (uint8_t)(cmd->value / 10), 0x00, 3);

    case P1P2_CMD_SET_FAN_COOL:
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_COOL_FAN,
                                        encode_fan_speed((p1p2_fan_mode_t)cmd->value),
                                        0x00, 3);

    case P1P2_CMD_SET_FAN_HEAT:
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_HEAT_FAN,
                                        encode_fan_speed((p1p2_fan_mode_t)cmd->value),
                                        0x00, 3);

    case P1P2_CMD_SET_DHW_POWER:
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_DHW_POWER,
                                        cmd->value ? 0x01 : 0x00, 0x00, 3);

    case P1P2_CMD_SET_DHW_TEMP:
        /* value is temperature × 10, F-series uses integer °C */
        return p1p2_fseries_queue_write(pkt_type, F38_RSP_DHW_TEMP,
                                        (uint8_t)(cmd->value / 10), 0x00, 3);

    case P1P2_CMD_SET_ZONES:
        if (model_id == F_MODEL_M) {
            /* Zones: offset 16 in 0x3B response payload */
            return p1p2_fseries_queue_write(PKT_TYPE_CTRL_3B, 16,
                                            (uint8_t)cmd->value, 0x00, 3);
        }
        return ESP_ERR_NOT_SUPPORTED;

    default:
        ESP_LOGW(TAG, "Unsupported command type %d", cmd->type);
        return ESP_ERR_NOT_SUPPORTED;
    }
}
