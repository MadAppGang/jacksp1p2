/*
 * P1P2 F-Series Decode — Extract HVAC data from P1/P2 bus packets
 *
 * Port of bytesbits2keyvalue() / bytes2keyvalue() from P1P2_ParameterConversion.h
 * for F-series VRV systems.
 *
 * Instead of converting to MQTT topic/value strings, this decoder updates
 * a p1p2_hvac_state_t structure that is read by the Matter layer.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "p1p2_protocol.h"
#include "p1p2_fseries.h"
#include "p1p2_param_tables.h"

static const char *TAG = "p1p2_decode";

/*
 * Decode F-series fan speed byte to p1p2_fan_mode_t.
 * Fan encoding: bits 6-5 select speed.
 *   0x11 (00) = Low
 *   0x31 (01) = Medium
 *   0x51 (10) = High
 */
static p1p2_fan_mode_t decode_fan_speed(uint8_t raw)
{
    uint8_t speed_bits = (raw >> 5) & 0x03;
    switch (speed_bits) {
    case 0: return P1P2_FAN_LOW;
    case 1: return P1P2_FAN_MED;
    case 2: return P1P2_FAN_HIGH;
    default: return P1P2_FAN_AUTO;
    }
}

/*
 * Decode F-series operating mode byte to p1p2_system_mode_t.
 * Mode: low 3 bits.
 */
static p1p2_system_mode_t decode_mode(uint8_t raw)
{
    uint8_t mode_bits = raw & 0x07;
    switch (mode_bits) {
    case F_MODE_HEAT: return P1P2_MODE_HEAT;
    case F_MODE_COOL: return P1P2_MODE_COOL;
    case F_MODE_AUTO: return P1P2_MODE_AUTO;
    case F_MODE_FAN:  return P1P2_MODE_FAN;
    case F_MODE_DRY:  return P1P2_MODE_DRY;
    default:          return P1P2_MODE_OFF;
    }
}

/*
 * Log raw packet bytes in hex format for debugging on real hardware.
 */
void p1p2_log_packet(const p1p2_packet_t *pkt, const char *prefix)
{
    if (pkt->length == 0) return;

    char buf[P1P2_MAX_PACKET_SIZE * 3 + 16];
    int pos = 0;

    for (int i = 0; i < pkt->length && pos < (int)sizeof(buf) - 4; i++) {
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "%02X ", pkt->data[i]);
    }

    ESP_LOGI(TAG, "%s: %s[%s]", prefix, buf, pkt->has_error ? "ERR" : "OK");
}

/*
 * Decode a single F-series packet and update HVAC state.
 * Sets bits in state->changed for any field that actually changes value.
 *
 * Packet layout:
 *   data[0] = source address
 *   data[1] = destination address
 *   data[2] = packet type
 *   data[3..length-2] = payload
 *   data[length-1] = CRC
 */
void p1p2_fseries_decode_packet(const p1p2_packet_t *pkt, p1p2_hvac_state_t *state)
{
    if (pkt->length < 4) return; /* minimum: src + dst + type + CRC */

    uint8_t src  = pkt->data[0];
    uint8_t dst  = pkt->data[1];
    uint8_t type = pkt->data[2];
    const uint8_t *payload = &pkt->data[3];
    uint8_t payload_len = pkt->length - 4; /* exclude src, dst, type, CRC */

    (void)src;
    (void)dst;

    switch (type) {
    case PKT_TYPE_STATUS_10:
        /*
         * Status packet 0x10: Power, mode, target temperatures, fan speeds.
         * This is the primary status packet from the indoor unit.
         *
         * Payload layout (varies by model, typical for BCL/M):
         *   [0] status flags (bit 0 = power on)
         *   [2] operating mode
         *   [4] target cooling temperature
         *   [6] fan speed cooling
         *   [8] target heating temperature
         *   [10] fan speed heating
         */
        if (payload_len >= 1) {
            bool pwr = (payload[0] & 0x01);
            if (pwr != state->power) state->changed |= CHANGED_POWER;
            state->power = pwr;
        }
        if (payload_len >= 3) {
            p1p2_system_mode_t m = decode_mode(payload[2]);
            if (m != state->mode) state->changed |= CHANGED_MODE;
            state->mode = m;
        }
        if (payload_len >= 5) {
            int16_t tc = payload[4] * 10;
            if (tc != state->target_temp_cool) state->changed |= CHANGED_TEMP_COOL;
            state->target_temp_cool = tc;
        }
        if (payload_len >= 7) {
            p1p2_fan_mode_t fc = decode_fan_speed(payload[6]);
            if (fc != state->fan_mode_cool) state->changed |= CHANGED_FAN_COOL;
            state->fan_mode_cool = fc;
        }
        if (payload_len >= 9) {
            int16_t th = payload[8] * 10;
            if (th != state->target_temp_heat) state->changed |= CHANGED_TEMP_HEAT;
            state->target_temp_heat = th;
        }
        if (payload_len >= 11) {
            p1p2_fan_mode_t fh = decode_fan_speed(payload[10]);
            if (fh != state->fan_mode_heat) state->changed |= CHANGED_FAN_HEAT;
            state->fan_mode_heat = fh;
        }

        /* Determine running state from mode and power */
        if (!state->power) {
            state->running = P1P2_RUNNING_IDLE;
        } else if (state->mode == P1P2_MODE_HEAT) {
            state->running = P1P2_RUNNING_HEATING;
        } else if (state->mode == P1P2_MODE_COOL) {
            state->running = P1P2_RUNNING_COOLING;
        }

        state->data_valid = true;
        break;

    case PKT_TYPE_STATUS_11:
        /*
         * Temperature readings packet 0x11.
         *   [0] room/return temperature
         *   [2] outdoor temperature (signed)
         */
        if (payload_len >= 1) {
            int16_t rt = payload[0] * 10;
            if (rt != state->room_temp) state->changed |= CHANGED_ROOM_TEMP;
            state->room_temp = rt;
        }
        if (payload_len >= 3) {
            int16_t ot = (int8_t)payload[2] * 10;
            if (ot != state->outdoor_temp) state->changed |= CHANGED_OUTDOOR_TEMP;
            state->outdoor_temp = ot;
        }
        break;

    case PKT_TYPE_DATETIME_12:
        /* Date/time packet — we don't need this for Matter but could use for diagnostics */
        break;

    case PKT_TYPE_STATUS_13:
        /*
         * Extended status packet 0x13.
         *   [0] Error sub-code or status flags
         *   [1-2] Extended mode/status bits
         * We extract the error sub-code into error_code if non-zero.
         */
        if (payload_len >= 1) {
            uint16_t ec = payload[0];
            if (payload_len >= 3) {
                ec = (payload[1] << 8) | payload[2];
            }
            if (ec != state->error_code) state->changed |= CHANGED_ERROR_CODE;
            state->error_code = ec;
        }
        break;

    case PKT_TYPE_STATUS_14:
        /*
         * Extended status / compressor data.
         *   [0-1] compressor frequency (16-bit)
         *   [2-3] flow rate (16-bit, L/min × 10)
         */
        if (payload_len >= 2) {
            uint16_t cf = (payload[0] << 8) | payload[1];
            if (cf != state->compressor_freq) state->changed |= CHANGED_COMPRESSOR;
            state->compressor_freq = cf;
        }
        if (payload_len >= 4) {
            uint16_t fr = (payload[2] << 8) | payload[3];
            if (fr != state->flow_rate) state->changed |= CHANGED_FLOW_RATE;
            state->flow_rate = fr;
        }
        break;

    case PKT_TYPE_STATUS_15:
        /*
         * DHW and water temperatures packet 0x15.
         *   [0] DHW active flag
         *   [1] DHW target temperature
         *   [2] DHW actual temperature
         *   [3-4] Leaving water temperature (signed, big-endian)
         *   [5-6] Return water temperature (signed, big-endian)
         */
        if (payload_len >= 1) {
            bool dhw = (payload[0] & 0x01);
            if (dhw != state->dhw_active) state->changed |= CHANGED_DHW;
            state->dhw_active = dhw;
        }
        if (payload_len >= 2) {
            int16_t dt = payload[1] * 10;
            if (dt != state->dhw_target) state->changed |= CHANGED_DHW;
            state->dhw_target = dt;
        }
        if (payload_len >= 3) {
            int16_t da = payload[2] * 10;
            if (da != state->dhw_temp) state->changed |= CHANGED_DHW;
            state->dhw_temp = da;
        }
        if (payload_len >= 5) {
            int16_t lwt = (int16_t)((payload[3] << 8) | payload[4]);
            if (lwt != state->leaving_water_temp) state->changed |= CHANGED_WATER_TEMPS;
            state->leaving_water_temp = lwt;
        }
        if (payload_len >= 7) {
            int16_t rwt = (int16_t)((payload[5] << 8) | payload[6]);
            if (rwt != state->return_water_temp) state->changed |= CHANGED_WATER_TEMPS;
            state->return_water_temp = rwt;
        }
        break;

    case PKT_TYPE_STATUS_16:
        /*
         * Additional status packet 0x16.
         *   [0-1] Error code (16-bit)
         */
        if (payload_len >= 2) {
            uint16_t ec = (payload[0] << 8) | payload[1];
            if (ec != state->error_code) state->changed |= CHANGED_ERROR_CODE;
            state->error_code = ec;
        }
        break;

    case PKT_TYPE_CTRL_38:
        /*
         * 0x38 control exchange (request from indoor unit).
         * This packet is what we must respond to as auxiliary controller.
         * We also decode it to update our state with the "current request" values.
         */
        if (payload_len >= 1) {
            bool pwr = (payload[0] & 0x01);
            if (pwr != state->power) state->changed |= CHANGED_POWER;
            state->power = pwr;
        }
        if (payload_len >= 3) {
            p1p2_system_mode_t m = decode_mode(payload[2]);
            if (m != state->mode) state->changed |= CHANGED_MODE;
            state->mode = m;
        }
        if (payload_len >= 5) {
            int16_t tc = payload[4] * 10;
            if (tc != state->target_temp_cool) state->changed |= CHANGED_TEMP_COOL;
            state->target_temp_cool = tc;
        }
        if (payload_len >= 7) {
            p1p2_fan_mode_t fc = decode_fan_speed(payload[6]);
            if (fc != state->fan_mode_cool) state->changed |= CHANGED_FAN_COOL;
            state->fan_mode_cool = fc;
        }
        if (payload_len >= 9) {
            int16_t th = payload[8] * 10;
            if (th != state->target_temp_heat) state->changed |= CHANGED_TEMP_HEAT;
            state->target_temp_heat = th;
        }
        if (payload_len >= 11) {
            p1p2_fan_mode_t fh = decode_fan_speed(payload[10]);
            if (fh != state->fan_mode_heat) state->changed |= CHANGED_FAN_HEAT;
            state->fan_mode_heat = fh;
        }
        state->data_valid = true;
        break;

    case PKT_TYPE_CTRL_3B:
        /*
         * 0x3B control exchange (FDYQ model M variant).
         * Similar to 0x38 but with zone support.
         */
        if (payload_len >= 1) {
            bool pwr = (payload[0] & 0x01);
            if (pwr != state->power) state->changed |= CHANGED_POWER;
            state->power = pwr;
        }
        if (payload_len >= 3) {
            p1p2_system_mode_t m = decode_mode(payload[2]);
            if (m != state->mode) state->changed |= CHANGED_MODE;
            state->mode = m;
        }
        if (payload_len >= 5) {
            int16_t tc = payload[4] * 10;
            if (tc != state->target_temp_cool) state->changed |= CHANGED_TEMP_COOL;
            state->target_temp_cool = tc;
        }
        if (payload_len >= 7) {
            p1p2_fan_mode_t fc = decode_fan_speed(payload[6]);
            if (fc != state->fan_mode_cool) state->changed |= CHANGED_FAN_COOL;
            state->fan_mode_cool = fc;
        }
        if (payload_len >= 9) {
            int16_t th = payload[8] * 10;
            if (th != state->target_temp_heat) state->changed |= CHANGED_TEMP_HEAT;
            state->target_temp_heat = th;
        }
        if (payload_len >= 11) {
            p1p2_fan_mode_t fh = decode_fan_speed(payload[10]);
            if (fh != state->fan_mode_heat) state->changed |= CHANGED_FAN_HEAT;
            state->fan_mode_heat = fh;
        }
        if (payload_len >= 18) {
            uint8_t z = payload[17];
            if (z != state->active_zones) state->changed |= CHANGED_ZONES;
            state->active_zones = z;
        }
        state->data_valid = true;
        break;

    case PKT_TYPE_COUNTER_A3:
        /*
         * Counter data (operation hours, compressor starts, etc.)
         */
        if (payload_len >= 8) {
            uint32_t oh = ((uint32_t)payload[0] << 24) |
                          ((uint32_t)payload[1] << 16) |
                          ((uint32_t)payload[2] << 8) |
                          payload[3];
            if (oh != state->operation_hours) state->changed |= CHANGED_OP_HOURS;
            state->operation_hours = oh;

            uint32_t cs = ((uint32_t)payload[4] << 24) |
                          ((uint32_t)payload[5] << 16) |
                          ((uint32_t)payload[6] << 8) |
                          payload[7];
            if (cs != state->compressor_starts) state->changed |= CHANGED_COMP_STARTS;
            state->compressor_starts = cs;
        }
        break;

    default:
        /* Other packet types — log at debug level */
        ESP_LOGD(TAG, "Unhandled packet type 0x%02X (len=%d)", type, pkt->length);
        break;
    }

    state->last_update_us = esp_timer_get_time();
    state->packet_count++;
}
