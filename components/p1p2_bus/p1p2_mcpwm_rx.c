/*
 * P1P2 MCPWM Receive — ESP32-C6 MCPWM capture + GPTimer mid-bit sampling
 *
 * Replaces ATmega TIMER1_CAPT_vect (falling edge capture) and
 * TIMER1_COMPB_vect (mid-bit sampling) with:
 *   - MCPWM capture channel callback → falling edge detection (hardware timestamped)
 *   - GPTimer alarm callback → mid-bit sampling (marks '1' bits, handles stop/EOP)
 *
 * All callbacks are IRAM_ATTR for minimum latency.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "p1p2_bus.h"
#include "p1p2_bus_types.h"
#include "p1p2_bus_config.h"

static const char *TAG = "p1p2_rx";

/* ---- Internal state shared with p1p2_bus.c and p1p2_mcpwm_tx.c ---- */

/* Ring buffer: ISR writes bytes here, bus_io_task reads them out */
extern volatile uint8_t  rx_buffer[P1P2_RX_BUFFER_SIZE];
extern volatile p1p2_error_t error_buffer[P1P2_RX_BUFFER_SIZE];
extern volatile uint16_t delta_buffer[P1P2_RX_BUFFER_SIZE];
extern volatile uint8_t  rx_buffer_head;
extern volatile uint8_t  rx_buffer_head2;
extern volatile uint8_t  rx_buffer_tail;

/* Time tracking: millisecond counter since last start bit */
extern volatile uint16_t time_msec;

/* LED GPIO pins (set during init) */
extern int gpio_led_read;
extern int gpio_led_error;

/* Echo and config */
extern volatile uint8_t  echo_enabled;
extern volatile uint8_t  allow_pause;

/* ---- RX-private state ---- */

static volatile uint8_t  rx_state;
static volatile uint8_t  rx_byte;
static volatile uint8_t  rx_paritycheck;
static volatile uint32_t rx_target;      /* target timestamp for next mid-bit sample */
static volatile uint32_t prev_edge_capture;
static volatile uint16_t startbit_delta; /* time_msec at start of current byte */

/* MCPWM capture handle */
static mcpwm_cap_channel_handle_t cap_channel = NULL;
static mcpwm_cap_timer_handle_t   cap_timer   = NULL;

/* GPTimer for mid-bit sampling and ms counter */
static gptimer_handle_t gptimer_midbit = NULL;
static gptimer_handle_t gptimer_ms     = NULL;

/* Forward declarations */
static bool IRAM_ATTR capture_callback(mcpwm_cap_channel_handle_t cap_ch,
                                        const mcpwm_capture_event_data_t *edata,
                                        void *user_ctx);
static bool IRAM_ATTR midbit_alarm_callback(gptimer_handle_t timer,
                                             const gptimer_alarm_event_data_t *edata,
                                             void *user_ctx);
static bool IRAM_ATTR ms_timer_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_ctx);

/* GPIO number for RX pin — needed to read pin level in ISR */
static int rx_gpio_num;

/*
 * Read the RX pin level in ISR context.
 * Uses direct register access for speed.
 */
static inline bool IRAM_ATTR rx_pin_level(void)
{
    return gpio_get_level(rx_gpio_num);
}

/*
 * Schedule the GPTimer mid-bit alarm at an absolute target time.
 * The GPTimer runs free at 8 MHz, matching the MCPWM capture timer.
 */
static inline void IRAM_ATTR schedule_midbit_alarm(uint32_t target_count)
{
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = target_count,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(gptimer_midbit, &alarm_cfg);
}

static inline void IRAM_ATTR disable_midbit_alarm(void)
{
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(gptimer_midbit, &alarm_cfg);
}

/* Store a received byte into the ring buffer */
static inline void IRAM_ATTR store_rx_byte(uint8_t byte_val, uint16_t delta,
                                            p1p2_error_t error_flags)
{
    uint8_t head = rx_buffer_head + 1;
    if (head >= P1P2_RX_BUFFER_SIZE) head = 0;
    if (head != rx_buffer_tail) {
        rx_buffer[head] = byte_val;
        delta_buffer[head] = delta;
        error_buffer[head] = error_flags;
        rx_buffer_head2 = head;
    } else {
        /* Buffer overrun — flag on previous byte */
        error_buffer[rx_buffer_head] |= P1P2_ERROR_OR;
        gpio_set_level(gpio_led_error, 1);
        rx_buffer_head2 = rx_buffer_head;
    }
}

/*
 * ============================================================
 * MCPWM Capture Callback — Falling Edge Detected
 * ============================================================
 * Replaces ATmega ISR(TIMER1_CAPT_vect) / ISR(CAPTURE_INTERRUPT)
 *
 * Called on each falling edge of the bus RX pin.
 * Hardware timestamps the edge — survives ISR entry delay.
 *
 * State machine:
 *   0: First falling edge = start bit of first byte after idle
 *   1: Start bit of next byte (store previous byte without EOP)
 *   2-9: Data bit falling edge → mark bit as '0'
 *   10: Parity bit falling edge
 *   11: Should not normally get falling edge in stop bit
 */
static bool IRAM_ATTR capture_callback(mcpwm_cap_channel_handle_t cap_ch,
                                        const mcpwm_capture_event_data_t *edata,
                                        void *user_ctx)
{
    uint32_t capture = edata->cap_value;
    uint8_t state = rx_state;

    /* Suppress oscillations/spikes: ignore edges too close to previous */
    if (state && (capture - prev_edge_capture < TICKS_SUPPRESSION)) {
        return false;
    }

    switch (state) {
    case 0: /* Idle → first start bit */
    case 1: /* Inter-byte → next start bit (confirm previous byte) */
        if (rx_buffer_head2 != P1P2_NO_HEAD2) {
            rx_buffer_head = rx_buffer_head2;
            rx_buffer_head2 = P1P2_NO_HEAD2;
        }

        if (state == 0) {
            gpio_set_level(gpio_led_read, 1);
            gpio_set_level(gpio_led_error, 0);
        }

        startbit_delta = time_msec;
        time_msec = 0;

        /* Schedule mid-bit sample at 1.5 bit times after start bit edge */
        rx_target = capture + TICKS_PER_BIT_AND_SEMIBIT;
        rx_state = 2;
        rx_paritycheck = 0;
        schedule_midbit_alarm(rx_target);
        break;

    case 2: case 3: case 4: case 5:
    case 6: case 7: case 8: case 9:
        /* Data bit falling edge → this bit is '0' (no need to set bit, already 0 from shift) */
        rx_byte >>= 1;
        /* Parity: '0' bit doesn't change parity */
        rx_target += TICKS_PER_BIT;
        rx_state = state + 1;
        schedule_midbit_alarm(rx_target);
        break;

    case 10: /* Parity bit falling edge → parity bit is '0' */
        /* No change to rx_paritycheck (XOR with 0) */
        rx_target += TICKS_PER_BIT;
        rx_state = 11;
        schedule_midbit_alarm(rx_target);
        break;

    case 11: /* Falling edge during stop bit — should not happen for Daikin F-series */
        break;

    default:
        break;
    }

    prev_edge_capture = capture;
    return false; /* no high-priority task woken */
}

/*
 * ============================================================
 * GPTimer Mid-Bit Alarm Callback
 * ============================================================
 * Replaces ATmega ISR(TIMER1_COMPB_vect) / ISR(COMPARE_R_INTERRUPT)
 *
 * Called at mid-bit point when no falling edge occurred → bit is '1'.
 * Also handles stop bit completion and EOP timeout detection.
 *
 * State machine:
 *   1: No start bit within expected window → EOP
 *   2-9: Data bit mid-sample → bit is '1'
 *   10: Parity bit is '1'
 *   11: Stop bit → store byte, schedule EOP timeout
 */
static bool IRAM_ATTR midbit_alarm_callback(gptimer_handle_t timer,
                                             const gptimer_alarm_event_data_t *edata,
                                             void *user_ctx)
{
    uint8_t state = rx_state;

    switch (state) {
    case 1: /* EOP timeout: no new start bit detected */
        rx_state = 0;
        if (rx_buffer_head2 != P1P2_NO_HEAD2) {
            rx_buffer_head = rx_buffer_head2;
            error_buffer[rx_buffer_head] |= P1P2_SIGNAL_EOP;
            rx_buffer_head2 = P1P2_NO_HEAD2;
        }
        gpio_set_level(gpio_led_read, 0);
        return false;

    case 2: /* First data bit is '1' */
        rx_byte = (rx_byte >> 1) | 0x80;
        rx_paritycheck ^= 0x80;
        rx_state = 3;
        rx_target += TICKS_PER_BIT;
        schedule_midbit_alarm(rx_target);
        break;

    case 3: case 4: case 5: case 6:
    case 7: case 8: case 9:
        /* Data bits 1-7: mid-sample → bit is '1' */
        rx_byte = (rx_byte >> 1) | 0x80;
        rx_paritycheck ^= 0x80;
        rx_state = state + 1;
        rx_target += TICKS_PER_BIT;
        schedule_midbit_alarm(rx_target);
        break;

    case 10: /* Parity bit is '1' */
        rx_paritycheck ^= 0x80;
        rx_state = 11;
        rx_target += TICKS_PER_BIT;
        schedule_midbit_alarm(rx_target);
        break;

    case 11: { /* Stop bit — byte complete, store it */
        p1p2_error_t err = 0;
        if (rx_paritycheck) {
            err |= P1P2_ERROR_PE;
        }

        store_rx_byte(rx_byte, startbit_delta, err);

        /* Schedule EOP timeout: if no start bit within (1 + allow_pause) bit times */
        rx_state = 1;
        rx_target += TICKS_PER_BIT * (1 + allow_pause);
        schedule_midbit_alarm(rx_target);

        /* Re-enable ms timer for TX scheduling */
        time_msec = 1; /* preset to 1ms since we're at the parity/stop bit boundary */
        break;
    }

    case 0:
    default:
        break;
    }

    return false;
}

/*
 * ============================================================
 * Millisecond Timer Callback
 * ============================================================
 * Replaces ATmega ISR(TIMER2_COMPA_vect) / ISR(MS_TIMER_COMP_vect)
 *
 * Called every 1 ms. Increments time_msec counter used for:
 * - TX delay scheduling (waiting for bus silence before writing)
 * - Delta timing between bytes/packets
 */
static bool IRAM_ATTR ms_timer_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_ctx)
{
    if (time_msec < 0xFFFF) {
        time_msec++;
    }

    /* TX scheduling: handled by p1p2_mcpwm_tx.c via shared time_msec */
    extern void p1p2_tx_check_schedule(void);
    p1p2_tx_check_schedule();

    return false;
}

/*
 * ============================================================
 * Initialization
 * ============================================================
 */
esp_err_t p1p2_rx_init(int gpio_rx)
{
    esp_err_t ret;
    rx_gpio_num = gpio_rx;

    /* Reset state */
    rx_state = 0;
    rx_byte = 0;
    rx_paritycheck = 0;
    rx_target = 0;
    prev_edge_capture = 0;
    startbit_delta = 0;

    /* ---- MCPWM Capture Timer (8 MHz free-running) ---- */
    mcpwm_capture_timer_config_t cap_timer_cfg = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = P1P2_TIMER_FREQ_HZ,
    };
    ret = mcpwm_new_capture_timer(&cap_timer_cfg, &cap_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create capture timer: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- MCPWM Capture Channel (falling edge on RX pin) ---- */
    mcpwm_capture_channel_config_t cap_ch_cfg = {
        .gpio_num = gpio_rx,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = false,
        .flags.pull_up = true,
    };
    ret = mcpwm_new_capture_channel(cap_timer, &cap_ch_cfg, &cap_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create capture channel: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register capture callback */
    mcpwm_capture_event_callbacks_t cap_cbs = {
        .on_cap = capture_callback,
    };
    ret = mcpwm_capture_channel_register_event_callbacks(cap_channel, &cap_cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register capture callback: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_capture_channel_enable(cap_channel);
    if (ret != ESP_OK) return ret;

    ret = mcpwm_capture_timer_enable(cap_timer);
    if (ret != ESP_OK) return ret;

    ret = mcpwm_capture_timer_start(cap_timer);
    if (ret != ESP_OK) return ret;

    /* ---- GPTimer for mid-bit sampling (8 MHz, one-shot alarms) ---- */
    gptimer_config_t midbit_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = P1P2_TIMER_FREQ_HZ,
    };
    ret = gptimer_new_timer(&midbit_cfg, &gptimer_midbit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mid-bit timer: %s", esp_err_to_name(ret));
        return ret;
    }

    gptimer_event_callbacks_t midbit_cbs = {
        .on_alarm = midbit_alarm_callback,
    };
    ret = gptimer_register_event_callbacks(gptimer_midbit, &midbit_cbs, NULL);
    if (ret != ESP_OK) return ret;

    ret = gptimer_enable(gptimer_midbit);
    if (ret != ESP_OK) return ret;

    ret = gptimer_start(gptimer_midbit);
    if (ret != ESP_OK) return ret;

    /* ---- GPTimer for millisecond counter (1 kHz periodic) ---- */
    gptimer_config_t ms_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, /* 1 MHz for 1ms resolution */
    };
    ret = gptimer_new_timer(&ms_cfg, &gptimer_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ms timer: %s", esp_err_to_name(ret));
        return ret;
    }

    gptimer_event_callbacks_t ms_cbs = {
        .on_alarm = ms_timer_callback,
    };
    ret = gptimer_register_event_callbacks(gptimer_ms, &ms_cbs, NULL);
    if (ret != ESP_OK) return ret;

    gptimer_alarm_config_t ms_alarm = {
        .alarm_count = 1000, /* 1ms at 1 MHz */
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ret = gptimer_set_alarm_action(gptimer_ms, &ms_alarm);
    if (ret != ESP_OK) return ret;

    ret = gptimer_enable(gptimer_ms);
    if (ret != ESP_OK) return ret;

    ret = gptimer_start(gptimer_ms);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "RX initialized: GPIO%d, MCPWM capture @ %d Hz",
             gpio_rx, P1P2_TIMER_FREQ_HZ);
    return ESP_OK;
}

void p1p2_rx_deinit(void)
{
    if (cap_channel) {
        mcpwm_capture_channel_disable(cap_channel);
        mcpwm_del_capture_channel(cap_channel);
        cap_channel = NULL;
    }
    if (cap_timer) {
        mcpwm_capture_timer_stop(cap_timer);
        mcpwm_capture_timer_disable(cap_timer);
        mcpwm_del_capture_timer(cap_timer);
        cap_timer = NULL;
    }
    if (gptimer_midbit) {
        gptimer_stop(gptimer_midbit);
        gptimer_disable(gptimer_midbit);
        gptimer_del_timer(gptimer_midbit);
        gptimer_midbit = NULL;
    }
    if (gptimer_ms) {
        gptimer_stop(gptimer_ms);
        gptimer_disable(gptimer_ms);
        gptimer_del_timer(gptimer_ms);
        gptimer_ms = NULL;
    }
}
