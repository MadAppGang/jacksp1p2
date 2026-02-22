/*
 * P1P2 F-Series Protocol — Packet type definitions and decode tables
 *
 * F-Series models: A/B/C/L/LA (FDY), P/PA (FXMQ), M (FDYQ)
 *
 * Packet structure:
 *   Byte 0: Source address (0x00 = main controller, 0x40 = aux controller,
 *                           0x80 = outdoor unit, etc.)
 *   Byte 1: Destination address
 *   Byte 2: Packet type
 *   Byte 3+: Payload
 *   Last byte: CRC
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* F-series model IDs */
#define F_MODEL_BCL   10   /* FDY/FBQ — models A, B, C, L, LA */
#define F_MODEL_P     11   /* FXMQ — models P, PA */
#define F_MODEL_M     12   /* FDYQ — model M */

/* Packet source/destination addresses */
#define P1P2_ADDR_MAIN_CTRL     0x00
#define P1P2_ADDR_OUTDOOR       0x00  /* in some contexts */
#define P1P2_ADDR_AUX_CTRL      0x40
#define P1P2_ADDR_INDOOR        0x80

/* CRC parameters for F-series */
#define F_SERIES_CRC_GEN        0xD9
#define F_SERIES_CRC_FEED       0x00

/*
 * F-series packet types relevant for decode and control.
 * Packet types 0x10-0x16: status/temperature data (main → indoor)
 * Packet types 0x30-0x3F: auxiliary controller exchange
 */
#define PKT_TYPE_STATUS_10      0x10
#define PKT_TYPE_STATUS_11      0x11
#define PKT_TYPE_DATETIME_12    0x12
#define PKT_TYPE_STATUS_13      0x13
#define PKT_TYPE_STATUS_14      0x14
#define PKT_TYPE_STATUS_15      0x15
#define PKT_TYPE_STATUS_16      0x16

#define PKT_TYPE_CTRL_30        0x30
#define PKT_TYPE_CTRL_31        0x31
#define PKT_TYPE_CTRL_32        0x32
#define PKT_TYPE_CTRL_33        0x33
#define PKT_TYPE_CTRL_34        0x34
#define PKT_TYPE_CTRL_35        0x35
#define PKT_TYPE_CTRL_36        0x36
#define PKT_TYPE_CTRL_37        0x37
#define PKT_TYPE_CTRL_38        0x38  /* Primary control packet (model BCL, P) */
#define PKT_TYPE_CTRL_39        0x39  /* Filter/status (model BCL, P) */
#define PKT_TYPE_CTRL_3A        0x3A  /* Status (model P) */
#define PKT_TYPE_CTRL_3B        0x3B  /* Primary control packet (model M) */
#define PKT_TYPE_CTRL_3C        0x3C  /* Filter (model M) */

#define PKT_TYPE_COUNTER_A3     0xA3  /* Counter request/response */

/*
 * Control packet (0x38/0x3B) payload byte offsets.
 * These are relative to payload start (byte 3 of packet).
 *
 * For 0x38 request (from indoor unit):
 *   Byte  0: status flags (bit 0 = power)
 *   Byte  1: ???
 *   Byte  2: operating mode (low 3 bits: cool/heat/auto/fan/dry)
 *   Byte  3: ???
 *   Byte  4: target cool temperature
 *   Byte  5: cool temp change flag (bit 7)
 *   Byte  6: fan speed cooling (bits 6-5: speed, rest: flags)
 *   Byte  7: cool fan change flag
 *   Byte  8: target heat temperature
 *   Byte  9: heat temp change flag
 *   Byte 10: fan speed heating
 *   Byte 11: heat fan change flag
 */

/* 0x38 request payload indices (in packet bytes, starting from byte[3]) */
#define F38_REQ_STATUS          0   /* RB[3] */
#define F38_REQ_MODE            2   /* RB[5] */
#define F38_REQ_COOL_TEMP       4   /* RB[7] */
#define F38_REQ_COOL_TEMP_CHG   5   /* RB[8] */
#define F38_REQ_COOL_FAN        6   /* RB[9] */
#define F38_REQ_COOL_FAN_CHG    7   /* RB[10] */
#define F38_REQ_HEAT_TEMP       8   /* RB[11] */
#define F38_REQ_HEAT_TEMP_CHG   9   /* RB[12] */
#define F38_REQ_HEAT_FAN        10  /* RB[13] */
#define F38_REQ_HEAT_FAN_CHG    11  /* RB[14] */
#define F38_REQ_FAN_MODE        15  /* RB[18] */

/*
 * 0x38 response payload indices (in WB bytes, WB[3+n]).
 * Model BCL: 15-byte payload (nwrite=18 total with header+CRC)
 * Model P: 17-byte payload (nwrite=20)
 */
#define F38_RSP_STATUS          0   /* WB[3]: target status */
#define F38_RSP_MODE            1   /* WB[4]: target operating mode */
#define F38_RSP_COOL_TEMP       2   /* WB[5]: target cool temp */
#define F38_RSP_COOL_TEMP_CHG   3   /* WB[6]: clear change flag */
#define F38_RSP_COOL_FAN        4   /* WB[7]: target cool fan speed */
#define F38_RSP_COOL_FAN_CHG    5   /* WB[8]: clear change flag */
#define F38_RSP_HEAT_TEMP       6   /* WB[9]: target heat temp */
#define F38_RSP_HEAT_TEMP_CHG   7   /* WB[10]: clear change flag */
#define F38_RSP_HEAT_FAN        8   /* WB[11]: target heat fan speed */
#define F38_RSP_HEAT_FAN_CHG    9   /* WB[12]: heat fan change flag */
#define F38_RSP_FAN_MODE        13  /* WB[16]: target fan mode */
#define F38_RSP_FAN_MODE_CHG    14  /* WB[17]: fan mode change flag */

/*
 * Fan speed encoding in F-series:
 *   0x11 = Low
 *   0x31 = Medium
 *   0x51 = High
 *   Bits 6-5: speed level (00=low, 01=med, 10=high)
 */
#define F_FAN_LOW               0x11
#define F_FAN_MED               0x31
#define F_FAN_HIGH              0x51

/*
 * Mode encoding in F-series (low 3 bits of mode byte):
 *   0 = Fan only
 *   1 = Heat
 *   2 = Cool
 *   3 = Auto
 *   7 = Dry
 * Upper bits: 0x60 is typical for "active" mode byte in response
 */
#define F_MODE_FAN              0
#define F_MODE_HEAT             1
#define F_MODE_COOL             2
#define F_MODE_AUTO             3
#define F_MODE_DRY              7
#define F_MODE_ACTIVE_MASK      0x60

/*
 * DHW control offsets in 0x38/0x3B response payload.
 * DHW power and temperature are controlled via the response packet.
 */
#define F38_RSP_DHW_POWER       10  /* DHW on/off in response payload */
#define F38_RSP_DHW_TEMP        11  /* DHW target temperature in response payload */

/*
 * Temperature decode: F-series uses raw byte values directly as °C
 * (integer precision for VRV systems, unlike E-series which uses ×10).
 */

#ifdef __cplusplus
}
#endif
