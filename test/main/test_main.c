/*
 * P1P2 Protocol Unit Tests
 *
 * Tests the F-series decode and control response logic
 * using Unity framework. Runs on ESP32-C6 target.
 *
 * Build: cd test && idf.py set-target esp32c6 && idf.py build
 * Flash: idf.py -p PORT flash monitor
 */

#include <string.h>
#include <stdio.h>
#include "unity.h"
#include "p1p2_protocol.h"
#include "p1p2_fseries.h"
#include "p1p2_bus_types.h"

/* External decode function from p1p2_fseries_decode.c */
extern void p1p2_fseries_decode_packet(const p1p2_packet_t *pkt, p1p2_hvac_state_t *state);
extern void p1p2_log_packet(const p1p2_packet_t *pkt, const char *prefix);

/* External control functions from p1p2_fseries_control.c */
extern void    p1p2_fseries_control_init(int model);
extern uint8_t p1p2_fseries_build_response_38(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3b(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_39(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3a(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_3c(const uint8_t *rb, uint8_t rb_len,
                                               uint8_t *wb, uint8_t wb_max);
extern uint8_t p1p2_fseries_build_response_empty(const uint8_t *rb, uint8_t rb_len,
                                                  uint8_t *wb, uint8_t wb_max);
extern esp_err_t p1p2_fseries_queue_write(uint8_t packet_type, uint8_t payload_offset,
                                           uint8_t value, uint8_t mask, uint8_t count);
extern esp_err_t p1p2_fseries_apply_command(const p1p2_control_cmd_t *cmd);

/* ================================================================
 * Helper: build a test packet
 * ================================================================ */
static p1p2_packet_t make_packet(const uint8_t *data, uint8_t len)
{
    p1p2_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.data, data, len);
    pkt.length = len;
    pkt.has_error = false;
    return pkt;
}

/* ================================================================
 * DECODE TESTS — Original
 * ================================================================ */

TEST_CASE("decode: packet too short is ignored", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {0x00, 0x00, 0x10}; /* only 3 bytes, need at least 4 */
    p1p2_packet_t pkt = make_packet(raw, 3);

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_FALSE(state.data_valid);
}

TEST_CASE("decode: 0x10 status packet — power on, cool mode, 24C", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* src=0x00, dst=0x80, type=0x10, payload..., CRC */
    uint8_t raw[] = {
        0x00, 0x80, 0x10,       /* header */
        0x01,                    /* [0] power ON */
        0x00,                    /* [1] */
        F_MODE_COOL,             /* [2] mode = cool */
        0x00,                    /* [3] */
        24,                      /* [4] target cool temp = 24C */
        0x00,                    /* [5] */
        F_FAN_MED,               /* [6] fan cool = medium (0x31) */
        0x00,                    /* [7] */
        22,                      /* [8] target heat temp = 22C */
        0x00,                    /* [9] */
        F_FAN_LOW,               /* [10] fan heat = low (0x11) */
        0xAA,                    /* CRC (dummy) */
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_TRUE(state.data_valid);
    TEST_ASSERT_TRUE(state.power);
    TEST_ASSERT_EQUAL(P1P2_MODE_COOL, state.mode);
    TEST_ASSERT_EQUAL(240, state.target_temp_cool);  /* 24 * 10 */
    TEST_ASSERT_EQUAL(220, state.target_temp_heat);  /* 22 * 10 */
    TEST_ASSERT_EQUAL(P1P2_FAN_MED, state.fan_mode_cool);
    TEST_ASSERT_EQUAL(P1P2_FAN_LOW, state.fan_mode_heat);
    TEST_ASSERT_EQUAL(P1P2_RUNNING_COOLING, state.running);
}

TEST_CASE("decode: 0x10 status — power off → idle", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x00,                    /* power OFF */
        0x00, F_MODE_HEAT, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_FALSE(state.power);
    TEST_ASSERT_EQUAL(P1P2_RUNNING_IDLE, state.running);
}

TEST_CASE("decode: 0x10 heat mode → running heating", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01,                    /* power ON */
        0x00, F_MODE_HEAT, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_TRUE(state.power);
    TEST_ASSERT_EQUAL(P1P2_MODE_HEAT, state.mode);
    TEST_ASSERT_EQUAL(P1P2_RUNNING_HEATING, state.running);
}

TEST_CASE("decode: 0x11 temperature packet", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x11,
        23,                      /* [0] room temp = 23C */
        0x00,                    /* [1] */
        (uint8_t)(-5),           /* [2] outdoor temp = -5C (signed) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(230, state.room_temp);     /* 23 * 10 */
    TEST_ASSERT_EQUAL(-50, state.outdoor_temp);  /* -5 * 10 */
}

TEST_CASE("decode: 0x14 compressor frequency", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x14,
        0x00, 0x3C,             /* compressor freq = 60 Hz */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(60, state.compressor_freq);
}

TEST_CASE("decode: 0xA3 counter packet", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0xA3,
        0x00, 0x00, 0x10, 0x00, /* operation hours = 4096 */
        0x00, 0x00, 0x00, 0x64, /* compressor starts = 100 */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(4096, state.operation_hours);
    TEST_ASSERT_EQUAL(100, state.compressor_starts);
}

TEST_CASE("decode: fan speed encoding", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* Test high fan speed (0x51 = bits 6:5 = 10) */
    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_COOL, 0x00,
        24, 0x00, F_FAN_HIGH, 0x00,
        22, 0x00, F_FAN_HIGH,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(P1P2_FAN_HIGH, state.fan_mode_cool);
    TEST_ASSERT_EQUAL(P1P2_FAN_HIGH, state.fan_mode_heat);
}

TEST_CASE("decode: all mode values", "[decode]")
{
    p1p2_hvac_state_t state;
    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, 0x00, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };

    /* Test each mode */
    struct { uint8_t raw; p1p2_system_mode_t expected; } modes[] = {
        {F_MODE_FAN,  P1P2_MODE_FAN},
        {F_MODE_HEAT, P1P2_MODE_HEAT},
        {F_MODE_COOL, P1P2_MODE_COOL},
        {F_MODE_AUTO, P1P2_MODE_AUTO},
        {F_MODE_DRY,  P1P2_MODE_DRY},
    };

    for (int i = 0; i < sizeof(modes)/sizeof(modes[0]); i++) {
        memset(&state, 0, sizeof(state));
        raw[5] = modes[i].raw;
        p1p2_packet_t pkt = make_packet(raw, sizeof(raw));
        p1p2_fseries_decode_packet(&pkt, &state);
        TEST_ASSERT_EQUAL_MESSAGE(modes[i].expected, state.mode,
                                  "Mode mismatch");
    }
}

TEST_CASE("decode: packet count increments", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {0x00, 0x80, 0x10, 0x01, 0xAA};
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_EQUAL(1, state.packet_count);

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_EQUAL(2, state.packet_count);
}

/* ================================================================
 * NEW DECODE TESTS — 0x13, 0x14 flow, 0x15, 0x16
 * ================================================================ */

TEST_CASE("decode: 0x13 extended status — error code", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x13,
        0x42,                    /* [0] status flags */
        0x00, 0xA5,              /* [1-2] error sub-code = 0x00A5 */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(0x00A5, state.error_code);
}

TEST_CASE("decode: 0x14 compressor frequency and flow rate", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x14,
        0x00, 0x4B,             /* compressor freq = 75 Hz */
        0x00, 0x96,             /* flow rate = 150 (15.0 L/min) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(75, state.compressor_freq);
    TEST_ASSERT_EQUAL(150, state.flow_rate);
}

TEST_CASE("decode: 0x15 DHW active and temperatures", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x15,
        0x01,                    /* [0] DHW active */
        55,                      /* [1] DHW target = 55C */
        48,                      /* [2] DHW actual = 48C */
        0x01, 0xF4,              /* [3-4] leaving water = 500 (50.0C) */
        0x01, 0x90,              /* [5-6] return water = 400 (40.0C) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_TRUE(state.dhw_active);
    TEST_ASSERT_EQUAL(550, state.dhw_target);    /* 55 * 10 */
    TEST_ASSERT_EQUAL(480, state.dhw_temp);      /* 48 * 10 */
    TEST_ASSERT_EQUAL(500, state.leaving_water_temp);
    TEST_ASSERT_EQUAL(400, state.return_water_temp);
}

TEST_CASE("decode: 0x15 DHW inactive", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x15,
        0x00,                    /* [0] DHW inactive */
        40,                      /* [1] DHW target */
        35,                      /* [2] DHW actual */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_FALSE(state.dhw_active);
    TEST_ASSERT_EQUAL(400, state.dhw_target);
    TEST_ASSERT_EQUAL(350, state.dhw_temp);
}

TEST_CASE("decode: 0x15 negative water temperatures", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x15,
        0x00,                    /* [0] DHW inactive */
        0x00,                    /* [1] DHW target */
        0x00,                    /* [2] DHW actual */
        0xFF, 0xCE,              /* [3-4] leaving water = -50 (-5.0C signed) */
        0xFF, 0x9C,              /* [5-6] return water = -100 (-10.0C signed) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(-50, state.leaving_water_temp);
    TEST_ASSERT_EQUAL(-100, state.return_water_temp);
}

TEST_CASE("decode: 0x16 error code extraction", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x16,
        0x0E, 0x03,             /* [0-1] error code = 0x0E03 */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(0x0E03, state.error_code);
}

TEST_CASE("decode: 0x16 error code zero clears error", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));
    state.error_code = 0x1234; /* pre-existing error */

    uint8_t raw[] = {
        0x00, 0x80, 0x16,
        0x00, 0x00,             /* error cleared */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(0, state.error_code);
}

TEST_CASE("decode: negative outdoor temperature", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* Extreme cold: -20C */
    uint8_t raw[] = {
        0x00, 0x80, 0x11,
        20,                      /* [0] room temp = 20C */
        0x00,
        (uint8_t)(-20),          /* [2] outdoor = -20C (0xEC) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(200, state.room_temp);
    TEST_ASSERT_EQUAL(-200, state.outdoor_temp); /* -20 * 10 */
}

TEST_CASE("decode: temperature boundary — max 50C", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_COOL, 0x00,
        50,                      /* max cool temp = 50C */
        0x00, F_FAN_LOW, 0x00,
        50,                      /* max heat temp = 50C */
        0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(500, state.target_temp_cool);
    TEST_ASSERT_EQUAL(500, state.target_temp_heat);
}

TEST_CASE("decode: temperature boundary — min 16C", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_HEAT, 0x00,
        16,                      /* min cool temp = 16C */
        0x00, F_FAN_LOW, 0x00,
        16,                      /* min heat temp = 16C */
        0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_EQUAL(160, state.target_temp_cool);
    TEST_ASSERT_EQUAL(160, state.target_temp_heat);
}

TEST_CASE("decode: oversized packet handled safely", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* Build a large packet — still decodes the fields we know about */
    uint8_t raw[P1P2_MAX_PACKET_SIZE];
    memset(raw, 0, sizeof(raw));
    raw[0] = 0x00; raw[1] = 0x80; raw[2] = 0x10;
    raw[3] = 0x01; /* power on */
    raw[5] = F_MODE_COOL;
    raw[7] = 25;
    raw[sizeof(raw) - 1] = 0xAA;

    p1p2_packet_t pkt = make_packet(raw, P1P2_MAX_PACKET_SIZE);

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_TRUE(state.power);
    TEST_ASSERT_EQUAL(P1P2_MODE_COOL, state.mode);
    TEST_ASSERT_EQUAL(250, state.target_temp_cool);
}

TEST_CASE("decode: zero-length payload per type", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));
    state.compressor_freq = 42; /* pre-set value */

    /* Minimum valid packet: src + dst + type + CRC, but 0 payload bytes */
    uint8_t raw[] = {0x00, 0x80, 0x14, 0xAA};
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    /* compressor_freq should NOT be modified (payload too short) */
    TEST_ASSERT_EQUAL(42, state.compressor_freq);
    /* But packet_count should still increment */
    TEST_ASSERT_EQUAL(1, state.packet_count);
}

TEST_CASE("decode: unhandled packet type is safe", "[decode]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {0x00, 0x80, 0xFF, 0x01, 0x02, 0xAA};
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_FALSE(state.data_valid);
    TEST_ASSERT_EQUAL(1, state.packet_count);
}

/* ================================================================
 * CONTROL RESPONSE TESTS — Original
 * ================================================================ */

TEST_CASE("control: BCL 0x38 response — echo back state", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Simulated 0x38 request from indoor unit (18+ bytes) */
    uint8_t rb[24] = {0};
    rb[0]  = 0x00;  /* src = main controller */
    rb[1]  = 0x40;  /* dst = aux controller */
    rb[2]  = 0x38;  /* type */
    rb[3]  = 0x01;  /* power ON */
    rb[5]  = F_MODE_COOL | F_MODE_ACTIVE_MASK;  /* mode = cool */
    rb[7]  = 24;    /* target cool temp */
    rb[9]  = F_FAN_MED; /* fan cool = medium */
    rb[11] = 22;    /* target heat temp */
    rb[13] = F_FAN_LOW; /* fan heat = low */
    rb[14] = 0x00;
    rb[18] = 0x03;  /* fan mode */

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(18, len);
    TEST_ASSERT_EQUAL(P1P2_ADDR_AUX_CTRL, wb[0]);  /* src = aux */
    TEST_ASSERT_EQUAL(0x00, wb[1]);                  /* dst = sender */
    TEST_ASSERT_EQUAL(0x38, wb[2]);                  /* same type */
    TEST_ASSERT_EQUAL(0x01, wb[3]);                  /* power echoed */
    TEST_ASSERT_EQUAL(24, wb[5]);                    /* temp echoed */
    TEST_ASSERT_EQUAL(22, wb[9]);                    /* heat temp echoed */
}

TEST_CASE("control: P model 0x38 response is 20 bytes", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_P);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; rb[9] = F_FAN_LOW;
    rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(20, len);
    TEST_ASSERT_EQUAL(P1P2_ADDR_AUX_CTRL, wb[0]);
}

TEST_CASE("control: M model uses 0x3B, 22-byte response", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_M);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x3B;
    rb[3] = 0x01; rb[5] = F_MODE_HEAT;
    rb[7] = 24; rb[9] = F_FAN_LOW;
    rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[20] = 0x07; /* zones */
    rb[21] = 0x01; /* fan mode */

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_3b(rb, 22, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(22, len);
    TEST_ASSERT_EQUAL(0x07, wb[19]); /* zones echoed */
    TEST_ASSERT_EQUAL(0x01, wb[20]); /* fan mode echoed */
}

TEST_CASE("control: M model rejects 0x38", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_M);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0, len); /* model M doesn't use 0x38 */
}

TEST_CASE("control: BCL model rejects 0x3B", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x3B;
    rb[3] = 0x01;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_3b(rb, 22, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0, len);
}

TEST_CASE("control: empty response (0x35/0x36/0x37)", "[control]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    uint8_t rb[] = {0x00, 0x40, 0x35, 0xAA};
    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_empty(rb, sizeof(rb), wb, sizeof(wb));

    TEST_ASSERT_EQUAL(3, len);
    TEST_ASSERT_EQUAL(P1P2_ADDR_AUX_CTRL, wb[0]);
    TEST_ASSERT_EQUAL(0x35, wb[2]);
}

/* ================================================================
 * PENDING WRITE TESTS — Original
 * ================================================================ */

TEST_CASE("control: pending write applies to response", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Queue a temperature change: cool temp = 26C */
    p1p2_control_cmd_t cmd = {
        .type = P1P2_CMD_SET_TEMP_COOL,
        .value = 260, /* 26.0C × 10 */
    };
    esp_err_t ret = p1p2_fseries_apply_command(&cmd);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Build response */
    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; /* original temp 24C */
    rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    uint8_t len = p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(18, len);
    TEST_ASSERT_EQUAL(26, wb[5]); /* temperature overridden to 26C */
}

TEST_CASE("control: apply power command", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    p1p2_control_cmd_t cmd = {
        .type = P1P2_CMD_SET_POWER,
        .value = 0, /* power OFF */
    };
    esp_err_t ret = p1p2_fseries_apply_command(&cmd);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; /* currently ON */
    rb[5] = F_MODE_COOL; rb[7] = 24;
    rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0x00, wb[3]); /* power overridden to OFF */
}

/* ================================================================
 * NEW CONTROL TESTS — DHW, buffer overflow, retry exhaustion
 * ================================================================ */

TEST_CASE("control: DHW power command", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    p1p2_control_cmd_t cmd = {
        .type = P1P2_CMD_SET_DHW_POWER,
        .value = 1,
    };
    esp_err_t ret = p1p2_fseries_apply_command(&cmd);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Build response and verify DHW power offset is written */
    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    /* DHW power is at payload offset F38_RSP_DHW_POWER (10), so wb[3+10] = wb[13] */
    TEST_ASSERT_EQUAL(0x01, wb[3 + F38_RSP_DHW_POWER]);
}

TEST_CASE("control: DHW temperature command", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    p1p2_control_cmd_t cmd = {
        .type = P1P2_CMD_SET_DHW_TEMP,
        .value = 550, /* 55.0C × 10 */
    };
    esp_err_t ret = p1p2_fseries_apply_command(&cmd);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    /* DHW temp at payload offset F38_RSP_DHW_TEMP (11), so wb[3+11] = wb[14] */
    TEST_ASSERT_EQUAL(55, wb[3 + F38_RSP_DHW_TEMP]);
}

TEST_CASE("control: pending write buffer full returns ESP_ERR_NO_MEM", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Fill all 8 pending write slots */
    for (int i = 0; i < 8; i++) {
        esp_err_t ret = p1p2_fseries_queue_write(PKT_TYPE_CTRL_38, i, 0x42, 0x00, 3);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
    }

    /* 9th write should fail */
    esp_err_t ret = p1p2_fseries_queue_write(PKT_TYPE_CTRL_38, 0, 0x99, 0x00, 3);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, ret);
}

TEST_CASE("control: pending write retry count exhaustion", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Queue a write with count=1 (single attempt) */
    esp_err_t ret = p1p2_fseries_queue_write(PKT_TYPE_CTRL_38, F38_RSP_COOL_TEMP, 28, 0x00, 1);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};

    /* First response: write should apply */
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));
    TEST_ASSERT_EQUAL(28, wb[5]); /* temp overridden */

    /* Second response: write should be exhausted, original value echoed */
    memset(wb, 0, sizeof(wb));
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));
    TEST_ASSERT_EQUAL(24, wb[5]); /* back to original */
}

TEST_CASE("control: multiple simultaneous pending writes", "[control][write]")
{
    p1p2_fseries_control_init(F_MODEL_BCL);

    /* Queue power OFF + temp change + fan change simultaneously */
    p1p2_control_cmd_t cmd_power = { .type = P1P2_CMD_SET_POWER, .value = 0 };
    p1p2_control_cmd_t cmd_temp  = { .type = P1P2_CMD_SET_TEMP_COOL, .value = 280 };
    p1p2_control_cmd_t cmd_fan   = { .type = P1P2_CMD_SET_FAN_COOL, .value = P1P2_FAN_HIGH };

    TEST_ASSERT_EQUAL(ESP_OK, p1p2_fseries_apply_command(&cmd_power));
    TEST_ASSERT_EQUAL(ESP_OK, p1p2_fseries_apply_command(&cmd_temp));
    TEST_ASSERT_EQUAL(ESP_OK, p1p2_fseries_apply_command(&cmd_fan));

    uint8_t rb[24] = {0};
    rb[0] = 0x00; rb[1] = 0x40; rb[2] = 0x38;
    rb[3] = 0x01; rb[5] = F_MODE_COOL;
    rb[7] = 24; rb[9] = F_FAN_LOW; rb[11] = 22; rb[13] = F_FAN_LOW;
    rb[14] = 0x00; rb[18] = 0x00;

    uint8_t wb[24] = {0};
    p1p2_fseries_build_response_38(rb, 20, wb, sizeof(wb));

    TEST_ASSERT_EQUAL(0x00, wb[3]);       /* power OFF */
    TEST_ASSERT_EQUAL(28, wb[5]);         /* temp = 28C */
    TEST_ASSERT_EQUAL(F_FAN_HIGH, wb[7]); /* fan = high */
}

/* ================================================================
 * CHANGE DETECTION TESTS
 * ================================================================ */

TEST_CASE("change: bitmask set when value differs", "[change]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* First decode: power=0 → 1, should set CHANGED_POWER */
    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_COOL, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_BITS(CHANGED_POWER, CHANGED_POWER, state.changed);
    TEST_ASSERT_BITS(CHANGED_MODE, CHANGED_MODE, state.changed);
    TEST_ASSERT_BITS(CHANGED_TEMP_COOL, CHANGED_TEMP_COOL, state.changed);
    TEST_ASSERT_BITS(CHANGED_TEMP_HEAT, CHANGED_TEMP_HEAT, state.changed);
}

TEST_CASE("change: bitmask NOT set when value is same", "[change]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* Pre-set state to match the packet */
    state.power = true;
    state.mode = P1P2_MODE_COOL;
    state.target_temp_cool = 240;
    state.target_temp_heat = 220;
    state.fan_mode_cool = P1P2_FAN_LOW;
    state.fan_mode_heat = P1P2_FAN_LOW;

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_COOL, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    /* No fields should be marked as changed */
    TEST_ASSERT_EQUAL(0, state.changed);
}

TEST_CASE("change: multiple fields changed in one packet", "[change]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    /* Pre-set some fields with different values */
    state.compressor_freq = 50;
    state.flow_rate = 100;

    uint8_t raw[] = {
        0x00, 0x80, 0x14,
        0x00, 0x4B,             /* compressor = 75 (was 50) */
        0x00, 0xC8,             /* flow_rate = 200 (was 100) */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_BITS(CHANGED_COMPRESSOR, CHANGED_COMPRESSOR, state.changed);
    TEST_ASSERT_BITS(CHANGED_FLOW_RATE, CHANGED_FLOW_RATE, state.changed);
}

TEST_CASE("change: bitmask cleared after read", "[change]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0x10,
        0x01, 0x00, F_MODE_COOL, 0x00,
        24, 0x00, F_FAN_LOW, 0x00,
        22, 0x00, F_FAN_LOW,
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_NOT_EQUAL(0, state.changed);

    /* Simulate Matter consuming state: clear changed flags */
    state.changed = 0;
    TEST_ASSERT_EQUAL(0, state.changed);

    /* Decode same packet again — no changes → bitmask stays 0 */
    p1p2_fseries_decode_packet(&pkt, &state);
    TEST_ASSERT_EQUAL(0, state.changed);
}

TEST_CASE("change: counter fields set changed bits", "[change]")
{
    p1p2_hvac_state_t state;
    memset(&state, 0, sizeof(state));

    uint8_t raw[] = {
        0x00, 0x80, 0xA3,
        0x00, 0x00, 0x10, 0x00, /* operation hours = 4096 */
        0x00, 0x00, 0x00, 0x64, /* compressor starts = 100 */
        0xAA,
    };
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    p1p2_fseries_decode_packet(&pkt, &state);

    TEST_ASSERT_BITS(CHANGED_OP_HOURS, CHANGED_OP_HOURS, state.changed);
    TEST_ASSERT_BITS(CHANGED_COMP_STARTS, CHANGED_COMP_STARTS, state.changed);
}

/* ================================================================
 * PACKET LOGGING TEST
 * ================================================================ */

TEST_CASE("log: hex dump does not crash", "[log]")
{
    uint8_t raw[] = {0x00, 0x80, 0x10, 0x01, 0xAA};
    p1p2_packet_t pkt = make_packet(raw, sizeof(raw));

    /* Just verify it doesn't crash */
    p1p2_log_packet(&pkt, "TEST");

    /* Zero-length packet */
    p1p2_packet_t empty;
    memset(&empty, 0, sizeof(empty));
    p1p2_log_packet(&empty, "EMPTY");

    TEST_ASSERT_TRUE(true);
}

/* ================================================================
 * CRC TESTS — Original
 * ================================================================ */

/* CRC calculation — duplicated here for testing (same as p1p2_bus.c) */
static uint8_t test_calc_crc(const uint8_t *data, uint8_t length,
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

TEST_CASE("CRC: Daikin F-series polynomial 0xD9", "[crc]")
{
    /* Known test vector: a simple status packet header */
    uint8_t data[] = {0x00, 0x00, 0x10};
    uint8_t crc = test_calc_crc(data, 3, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);

    /* CRC should be non-zero and deterministic */
    TEST_ASSERT_NOT_EQUAL(0, crc);

    /* Same input should always produce same CRC */
    uint8_t crc2 = test_calc_crc(data, 3, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
    TEST_ASSERT_EQUAL(crc, crc2);
}

TEST_CASE("CRC: full packet CRC should verify to 0", "[crc]")
{
    /* Build a packet with valid CRC appended */
    uint8_t data[] = {0x00, 0x00, 0x10, 0x01, 0x00};
    uint8_t crc = test_calc_crc(data, 4, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
    data[4] = crc;

    /* CRC over entire packet (including CRC byte) should be 0 for Daikin */
    uint8_t verify = test_calc_crc(data, 5, F_SERIES_CRC_GEN, F_SERIES_CRC_FEED);
    TEST_ASSERT_EQUAL(0, verify);
}

/* ================================================================
 * MAIN
 * ================================================================ */

void app_main(void)
{
    printf("\n\n");
    printf("========================================\n");
    printf("  P1P2MQTT Protocol Unit Tests\n");
    printf("========================================\n\n");

    UNITY_BEGIN();

    /* Decode tests — original */
    unity_run_test_by_name("decode: packet too short is ignored");
    unity_run_test_by_name("decode: 0x10 status packet — power on, cool mode, 24C");
    unity_run_test_by_name("decode: 0x10 status — power off → idle");
    unity_run_test_by_name("decode: 0x10 heat mode → running heating");
    unity_run_test_by_name("decode: 0x11 temperature packet");
    unity_run_test_by_name("decode: 0x14 compressor frequency");
    unity_run_test_by_name("decode: 0xA3 counter packet");
    unity_run_test_by_name("decode: fan speed encoding");
    unity_run_test_by_name("decode: all mode values");
    unity_run_test_by_name("decode: packet count increments");

    /* Decode tests — new */
    unity_run_test_by_name("decode: 0x13 extended status — error code");
    unity_run_test_by_name("decode: 0x14 compressor frequency and flow rate");
    unity_run_test_by_name("decode: 0x15 DHW active and temperatures");
    unity_run_test_by_name("decode: 0x15 DHW inactive");
    unity_run_test_by_name("decode: 0x15 negative water temperatures");
    unity_run_test_by_name("decode: 0x16 error code extraction");
    unity_run_test_by_name("decode: 0x16 error code zero clears error");
    unity_run_test_by_name("decode: negative outdoor temperature");
    unity_run_test_by_name("decode: temperature boundary — max 50C");
    unity_run_test_by_name("decode: temperature boundary — min 16C");
    unity_run_test_by_name("decode: oversized packet handled safely");
    unity_run_test_by_name("decode: zero-length payload per type");
    unity_run_test_by_name("decode: unhandled packet type is safe");

    /* Control response tests — original */
    unity_run_test_by_name("control: BCL 0x38 response — echo back state");
    unity_run_test_by_name("control: P model 0x38 response is 20 bytes");
    unity_run_test_by_name("control: M model uses 0x3B, 22-byte response");
    unity_run_test_by_name("control: M model rejects 0x38");
    unity_run_test_by_name("control: BCL model rejects 0x3B");
    unity_run_test_by_name("control: empty response (0x35/0x36/0x37)");

    /* Pending write tests — original */
    unity_run_test_by_name("control: pending write applies to response");
    unity_run_test_by_name("control: apply power command");

    /* Control tests — new */
    unity_run_test_by_name("control: DHW power command");
    unity_run_test_by_name("control: DHW temperature command");
    unity_run_test_by_name("control: pending write buffer full returns ESP_ERR_NO_MEM");
    unity_run_test_by_name("control: pending write retry count exhaustion");
    unity_run_test_by_name("control: multiple simultaneous pending writes");

    /* Change detection tests */
    unity_run_test_by_name("change: bitmask set when value differs");
    unity_run_test_by_name("change: bitmask NOT set when value is same");
    unity_run_test_by_name("change: multiple fields changed in one packet");
    unity_run_test_by_name("change: bitmask cleared after read");
    unity_run_test_by_name("change: counter fields set changed bits");

    /* Packet logging test */
    unity_run_test_by_name("log: hex dump does not crash");

    /* CRC tests */
    unity_run_test_by_name("CRC: Daikin F-series polynomial 0xD9");
    unity_run_test_by_name("CRC: full packet CRC should verify to 0");

    UNITY_END();

    printf("\n========================================\n");
    printf("  Tests complete\n");
    printf("========================================\n");
}
