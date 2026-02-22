/*
 * P1P2 MCPWM Transmit — ESP32-C6 MCPWM operator/comparator/generator TX
 *
 * Replaces ATmega TIMER1_COMPA_vect (output compare with hardware pin toggle)
 * with MCPWM generator actions on comparator events.
 *
 * The 20-state half-bit state machine is a direct port:
 *   States 1-2: start bit (low, then high)
 *   States 3-18: 8 data bits × 2 half-bits each (LSB first)
 *   States 19-20: parity bit
 *   After state 20: stop bit / schedule next byte or finish
 *   State 99: scheduled — waiting for ms timer to trigger start
 *
 * Hardware pin toggle: MCPWM generator actions on comparator match give
 * zero-jitter output, equivalent to ATmega OC1A pin toggle on compare match.
 *
 * Original: Copyright (c) 2019-2024 Arnold Niessen — CC BY-NC-ND 4.0
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "p1p2_bus.h"
#include "p1p2_bus_types.h"
#include "p1p2_bus_config.h"

static const char *TAG = "p1p2_tx";

/* ---- Shared state with p1p2_bus.c and p1p2_mcpwm_rx.c ---- */
extern volatile uint8_t  rx_buffer[P1P2_RX_BUFFER_SIZE];
extern volatile p1p2_error_t error_buffer[P1P2_RX_BUFFER_SIZE];
extern volatile uint16_t delta_buffer[P1P2_RX_BUFFER_SIZE];
extern volatile uint8_t  rx_buffer_head;
extern volatile uint8_t  rx_buffer_head2;
extern volatile uint8_t  rx_buffer_tail;
extern volatile uint16_t time_msec;
extern int gpio_led_read;
extern int gpio_led_write;
extern int gpio_led_error;
extern volatile uint8_t echo_enabled;

/* ---- TX-private state ---- */
static volatile uint32_t tx_next_compare; /* tracks the next comparator value */
static volatile uint8_t  tx_state;
static volatile uint8_t  tx_byte;
static volatile uint8_t  tx_bit;
static volatile uint8_t  tx_byte_verify;
static volatile uint8_t  tx_paritybit;
static volatile uint8_t  tx_rx_readbackerror;
static volatile uint16_t tx_wait;
static volatile uint16_t startbit_delta_tx;

/* TX ring buffer */
static volatile uint8_t  tx_buffer_head;
static volatile uint8_t  tx_buffer_tail;
static volatile uint8_t  tx_buffer[P1P2_TX_BUFFER_SIZE];
static volatile uint16_t tx_buffer_delay[P1P2_TX_BUFFER_SIZE];
static volatile uint16_t tx_setdelay;
static volatile uint16_t tx_setdelaytimeout = 2500;

/* MCPWM handles */
static mcpwm_timer_handle_t     mcpwm_tx_timer    = NULL;
static mcpwm_oper_handle_t      mcpwm_tx_oper     = NULL;
static mcpwm_cmpr_handle_t      mcpwm_tx_cmpr     = NULL;
static mcpwm_gen_handle_t       mcpwm_tx_gen      = NULL;

/* GPIO number for RX pin (for read-back verification) and TX pin */
static int tx_gpio_num;
static int rx_gpio_num_tx; /* RX pin read during TX for collision detection */

/* Forward declaration */
static bool IRAM_ATTR tx_compare_callback(mcpwm_cmpr_handle_t cmpr,
                                           const mcpwm_compare_event_data_t *edata,
                                           void *user_ctx);

/*
 * Read the bus RX pin for read-back verification during TX.
 */
static inline bool IRAM_ATTR tx_read_bus(void)
{
    return gpio_get_level(rx_gpio_num_tx);
}

/*
 * Set the TX output pin level directly via MCPWM generator force action.
 */
static inline void IRAM_ATTR tx_set_high(void)
{
    mcpwm_generator_set_force_level(mcpwm_tx_gen, 1, true);
}

static inline void IRAM_ATTR tx_set_low(void)
{
    mcpwm_generator_set_force_level(mcpwm_tx_gen, 0, true);
}

/*
 * Schedule next comparator event (relative from last compare point).
 * Since we can't read the raw timer count, we track the last compare
 * value and add ticks to it. This avoids any accumulated jitter.
 */
static inline void IRAM_ATTR tx_schedule_next(uint32_t ticks)
{
    tx_next_compare = (tx_next_compare + ticks) & 0xFFFF;
    mcpwm_comparator_set_compare_value(mcpwm_tx_cmpr, tx_next_compare);
}

/*
 * ============================================================
 * TX Check Schedule — Called from ms timer ISR
 * ============================================================
 * Replaces the tx_state==99 check in ATmega MS_TIMER_COMP_vect.
 * When tx_state is 99 (scheduled), checks if enough silence has passed
 * to begin writing.
 */
void IRAM_ATTR p1p2_tx_check_schedule(void)
{
    if (tx_state != TX_STATE_SCHEDULED) return;

    if ((time_msec == tx_wait) ||
        ((time_msec >= tx_wait) && (time_msec >= tx_setdelaytimeout))) {
        /* Start writing: schedule falling edge of start bit */
        tx_state = TX_STATE_START_1;
        startbit_delta_tx = time_msec;
        time_msec = 0;

        gpio_set_level(gpio_led_write, 1);

        /* Drive TX pin low (start bit) after schedule delay */
        tx_set_low();
        /* Initialize compare tracking from 0 for first schedule */
        tx_next_compare = 0;
        tx_schedule_next(TICKS_PER_SEMIBIT);
        tx_rx_readbackerror = 0;
    }
}

/*
 * ============================================================
 * MCPWM Comparator Callback — TX Half-Bit State Machine
 * ============================================================
 * Replaces ATmega ISR(TIMER1_COMPA_vect) / ISR(COMPARE_W_INTERRUPT)
 *
 * Direct port of the 20-state half-bit machine.
 * Each comparator match fires every half-bit (semibit) time.
 */
static bool IRAM_ATTR tx_compare_callback(mcpwm_cmpr_handle_t cmpr,
                                           const mcpwm_compare_event_data_t *edata,
                                           void *user_ctx)
{
    uint8_t state = tx_state;
    uint8_t bit_input;
    uint8_t head = 0, tail;
    uint16_t delay;

    if (state == 0 || state == TX_STATE_SCHEDULED) return false;

    gpio_set_level(gpio_led_error, 0);

    if (state < 20) {
        /* Schedule next semibit */
        tx_schedule_next(TICKS_PER_SEMIBIT);

        bit_input = tx_read_bus();

        if (state & 1) {
            /* Odd state (1,3,5,...,19): first half of bit — pin is LOW for data/start */
            /* Set pin HIGH for second half */
            tx_set_high();

            if (state == 1) {
                /* Start bit: first half should read 0 */
                startbit_delta_tx = time_msec;
                time_msec = 0;
                tx_rx_readbackerror = 0;
                if (bit_input) {
                    tx_rx_readbackerror = P1P2_ERROR_SB;
                }
            } else if (state == 19) {
                /* Parity bit first half: restart ms timer */
                time_msec = 1;
            } else {
                /* Data bit first half: verify read-back */
                if (bit_input != tx_bit) {
                    tx_rx_readbackerror |= P1P2_ERROR_BE;
                }
            }
        } else {
            /* Even state (2,4,...,18): second half of bit — pin is HIGH */
            uint8_t bit;
            if (state < 18) {
                /* Next semibit will be data bit part 1, LSB first */
                bit = (tx_byte & 1);
                if (!bit) {
                    tx_set_low(); /* Drive low for '0' data bit */
                }
                /* else pin stays high for '1' */
                tx_paritybit ^= bit;
                tx_byte >>= 1;
            } else {
                /* state==18: next semibit is parity bit part 1 */
                bit = tx_paritybit;
                if (!bit) {
                    tx_set_low();
                }
            }
            /* Verify second half should be HIGH (bus collision detection) */
            if (!bit_input) {
                tx_rx_readbackerror |= P1P2_ERROR_BC;
            }
            tx_bit = bit;
        }

        tx_state = state + 1;
        return false;
    }

    /* ---- State 20: end of parity bit, entering stop bit ---- */
    /* Stop bit is HIGH — pin should already be high */
    tx_set_high();

    if (tx_rx_readbackerror) {
        gpio_set_level(gpio_led_error, 1);
        /* Bus collision suspected — flush write buffer to reduce further risk */
        tx_buffer_tail = tx_buffer_head;
    }

    /* Store transmitted byte as if received (if echo enabled) */
    if (echo_enabled) {
        head = rx_buffer_head + 1;
        if (head >= P1P2_RX_BUFFER_SIZE) head = 0;
        if (head != rx_buffer_tail) {
            rx_buffer[head] = tx_byte_verify;
            delta_buffer[head] = startbit_delta_tx;
            error_buffer[head] = tx_rx_readbackerror;
            rx_buffer_head = head;
        } else {
            error_buffer[rx_buffer_head] |= P1P2_ERROR_OR;
            gpio_set_level(gpio_led_error, 1);
        }
    }

    /* More data to write? */
    uint8_t errorhead = head;
    head = tx_buffer_head;
    tail = tx_buffer_tail;

    if (head != tail) {
        /* More bytes to send */
        if (++tail >= P1P2_TX_BUFFER_SIZE) tail = 0;
        tx_buffer_tail = tail;
        tx_byte = tx_buffer[tail];
        tx_byte_verify = tx_byte;
        delay = tx_buffer_delay[tail];
        tx_paritybit = 0;

        if (delay < 2) {
            /* No inter-byte delay: continue immediately after stop bit */
            tx_schedule_next(TICKS_PER_BIT_AND_SEMIBIT);
            tx_set_low(); /* will go low at schedule time */
            tx_state = TX_STATE_START_1;
            return false;
        } else {
            /* Inter-byte delay requested: go to scheduled wait */
            tx_state = TX_STATE_SCHEDULED;
            tx_wait = delay;
        }
    } else {
        /* TX buffer empty — done writing */
        tx_state = TX_STATE_IDLE;
    }

    /* Mark end-of-packet on the echo'd bytes */
    if (echo_enabled) {
        error_buffer[errorhead] |= P1P2_SIGNAL_EOP;
    }
    gpio_set_level(gpio_led_write, 0);

    return false;
}

/*
 * ============================================================
 * Public: Queue a byte for transmission
 * ============================================================
 */
void p1p2_tx_write_byte(uint8_t b, uint16_t delay)
{
    uint8_t head = tx_buffer_head + 1;
    if (head >= P1P2_TX_BUFFER_SIZE) head = 0;

    /* Spin-wait if buffer full (shouldn't happen with proper scheduling) */
    while (tx_buffer_tail == head) {
        /* tight loop — should be very brief */
    }

    if (tx_state) {
        /* Already writing — add byte to buffer */
        tx_buffer[head] = b;
        tx_buffer_delay[head] = delay;
        tx_buffer_head = head;
    } else {
        /* Not writing — schedule transmission */
        tx_byte = b;
        tx_byte_verify = b;
        tx_paritybit = 0;
        tx_rx_readbackerror = 0;

        tx_state = TX_STATE_SCHEDULED;
        tx_wait = (delay < 2) ? 2 : delay;
    }
}

bool p1p2_tx_is_idle(void)
{
    return (tx_state == TX_STATE_IDLE);
}

bool p1p2_tx_write_ready(void)
{
    return (tx_buffer_tail == tx_buffer_head);
}

void p1p2_tx_set_delay_timeout(uint16_t timeout_ms)
{
    tx_setdelaytimeout = timeout_ms;
}

/*
 * ============================================================
 * Initialization
 * ============================================================
 */
esp_err_t p1p2_tx_init(int gpio_tx, int gpio_rx)
{
    esp_err_t ret;
    tx_gpio_num = gpio_tx;
    rx_gpio_num_tx = gpio_rx;

    /* Reset state */
    tx_state = TX_STATE_IDLE;
    tx_byte = 0;
    tx_bit = 0;
    tx_byte_verify = 0;
    tx_paritybit = 0;
    tx_rx_readbackerror = 0;
    tx_wait = 0;
    tx_buffer_head = 0;
    tx_buffer_tail = 0;
    tx_setdelay = 0;
    startbit_delta_tx = 0;

    /* Set TX pin HIGH initially (idle bus state) */
    gpio_config_t tx_pin_cfg = {
        .pin_bit_mask = (1ULL << gpio_tx),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&tx_pin_cfg);
    gpio_set_level(gpio_tx, 1);

    /* ---- MCPWM Timer for TX (8 MHz, count-up) ---- */
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = P1P2_TIMER_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 0xFFFF, /* free-running */
    };
    ret = mcpwm_new_timer(&timer_cfg, &mcpwm_tx_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TX timer: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- MCPWM Operator ---- */
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0,
    };
    ret = mcpwm_new_operator(&oper_cfg, &mcpwm_tx_oper);
    if (ret != ESP_OK) return ret;

    ret = mcpwm_operator_connect_timer(mcpwm_tx_oper, mcpwm_tx_timer);
    if (ret != ESP_OK) return ret;

    /* ---- MCPWM Comparator ---- */
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = false,
        .flags.update_cmp_on_tep = false,
        .flags.update_cmp_on_sync = false,
    };
    ret = mcpwm_new_comparator(mcpwm_tx_oper, &cmpr_cfg, &mcpwm_tx_cmpr);
    if (ret != ESP_OK) return ret;

    /* Register compare event callback */
    mcpwm_comparator_event_callbacks_t cmpr_cbs = {
        .on_reach = tx_compare_callback,
    };
    ret = mcpwm_comparator_register_event_callbacks(mcpwm_tx_cmpr, &cmpr_cbs, NULL);
    if (ret != ESP_OK) return ret;

    /* ---- MCPWM Generator (drives TX pin) ---- */
    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = gpio_tx,
    };
    ret = mcpwm_new_generator(mcpwm_tx_oper, &gen_cfg, &mcpwm_tx_gen);
    if (ret != ESP_OK) return ret;

    /* Set initial level HIGH (idle) */
    mcpwm_generator_set_force_level(mcpwm_tx_gen, 1, true);

    /* Enable and start */
    ret = mcpwm_timer_enable(mcpwm_tx_timer);
    if (ret != ESP_OK) return ret;

    ret = mcpwm_timer_start_stop(mcpwm_tx_timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "TX initialized: GPIO%d, MCPWM operator @ %d Hz",
             gpio_tx, P1P2_TIMER_FREQ_HZ);
    return ESP_OK;
}

void p1p2_tx_deinit(void)
{
    if (mcpwm_tx_gen) {
        mcpwm_del_generator(mcpwm_tx_gen);
        mcpwm_tx_gen = NULL;
    }
    if (mcpwm_tx_cmpr) {
        mcpwm_del_comparator(mcpwm_tx_cmpr);
        mcpwm_tx_cmpr = NULL;
    }
    if (mcpwm_tx_oper) {
        mcpwm_del_operator(mcpwm_tx_oper);
        mcpwm_tx_oper = NULL;
    }
    if (mcpwm_tx_timer) {
        mcpwm_timer_start_stop(mcpwm_tx_timer, MCPWM_TIMER_STOP_FULL);
        mcpwm_timer_disable(mcpwm_tx_timer);
        mcpwm_del_timer(mcpwm_tx_timer);
        mcpwm_tx_timer = NULL;
    }
}
