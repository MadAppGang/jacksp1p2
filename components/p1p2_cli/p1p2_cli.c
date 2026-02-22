/*
 * P1P2 CLI — Serial console command parser for debugging
 *
 * Provides a serial USB command interface for:
 * - Bus monitoring (hex packet dump)
 * - Control level changes (L0/L1/L5)
 * - Manual parameter writes (F command)
 * - Status display
 * - Factory reset
 *
 * Command format matches the original P1P2Monitor serial interface
 * where practical, for backward compatibility with existing tooling.
 *
 * ESP32-C6 port: 2026
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_console.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "p1p2_bus.h"
#include "p1p2_protocol.h"
#include "p1p2_network.h"

static const char *TAG = "p1p2_cli";

/* CLI task handle */
static TaskHandle_t cli_task_handle = NULL;

/*
 * Command: L — Set control level
 *   L0 = control off (read-only)
 *   L1 = auxiliary controller active
 *   L5 = monitor only
 */
static int cmd_control_level(int argc, char **argv)
{
    if (argc < 2) {
        printf("Control level: %d\n", p1p2_protocol_get_control_level());
        return 0;
    }
    int level = atoi(argv[1]);
    p1p2_protocol_set_control_level(level);
    printf("Control level set to %d\n", level);
    return 0;
}

/*
 * Command: S — Show status
 */
static int cmd_status(int argc, char **argv)
{
    p1p2_hvac_state_t state;
    p1p2_protocol_get_state_copy(&state);

    printf("=== P1P2MQTT ESP32-C6 Status ===\n");
    printf("Data valid:   %s\n", state.data_valid ? "YES" : "NO");
    printf("Power:        %s\n", state.power ? "ON" : "OFF");
    printf("Mode:         %d\n", state.mode);
    printf("Running:      %d\n", state.running);
    printf("Cool target:  %.1f C\n", state.target_temp_cool / 10.0);
    printf("Heat target:  %.1f C\n", state.target_temp_heat / 10.0);
    printf("Room temp:    %.1f C\n", state.room_temp / 10.0);
    printf("Outdoor temp: %.1f C\n", state.outdoor_temp / 10.0);
    printf("Fan cool:     %d\n", state.fan_mode_cool);
    printf("Fan heat:     %d\n", state.fan_mode_heat);
    printf("Comp freq:    %d Hz\n", state.compressor_freq);
    printf("Error code:   0x%04X\n", state.error_code);
    printf("Op hours:     %lu\n", (unsigned long)state.operation_hours);
    printf("Packets:      %lu\n", (unsigned long)state.packet_count);

    p1p2_bus_stats_t bus_stats;
    p1p2_bus_get_stats(&bus_stats);
    printf("\n=== Bus Statistics ===\n");
    printf("RX packets:   %lu\n", (unsigned long)bus_stats.packets_received);
    printf("TX packets:   %lu\n", (unsigned long)bus_stats.packets_sent);
    printf("CRC errors:   %lu\n", (unsigned long)bus_stats.crc_errors);
    printf("Parity err:   %lu\n", (unsigned long)bus_stats.parity_errors);
    printf("Collisions:   %lu\n", (unsigned long)bus_stats.collision_errors);
    printf("Overruns:     %lu\n", (unsigned long)bus_stats.overrun_errors);
    printf("Uptime:       %lld s\n", bus_stats.uptime_us / 1000000LL);

    printf("\nControl level: %d\n", p1p2_protocol_get_control_level());
    printf("Matter: %s\n", "pending SDK integration");

    return 0;
}

/*
 * Command: V — Show bus voltage
 */
static int cmd_voltage(int argc, char **argv)
{
    p1p2_adc_results_t adc;
    p1p2_bus_get_adc(&adc);
    printf("Bus voltage CH0: min=%u max=%u avg=%lu\n",
           adc.v0_min, adc.v0_max, (unsigned long)adc.v0_avg);
    printf("Bus voltage CH1: min=%u max=%u avg=%lu\n",
           adc.v1_min, adc.v1_max, (unsigned long)adc.v1_avg);
    return 0;
}

/*
 * Command: R — Factory reset
 */
static int cmd_reset(int argc, char **argv)
{
    printf("Factory reset...\n");
    p1p2_config_erase_all();
    printf("Config erased. Restarting...\n");
    esp_restart();
    return 0; /* unreachable */
}

/*
 * Register CLI commands with esp_console.
 */
static void register_commands(void)
{
    esp_console_cmd_t cmds[] = {
        {
            .command = "L",
            .help = "Set control level (L0=off, L1=aux, L5=monitor)",
            .hint = "<level>",
            .func = cmd_control_level,
        },
        {
            .command = "S",
            .help = "Show system status",
            .hint = NULL,
            .func = cmd_status,
        },
        {
            .command = "V",
            .help = "Show bus voltage",
            .hint = NULL,
            .func = cmd_voltage,
        },
        {
            .command = "R",
            .help = "Factory reset (erases all config)",
            .hint = NULL,
            .func = cmd_reset,
        },
    };

    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        esp_console_cmd_register(&cmds[i]);
    }
}

/*
 * CLI task — runs the console REPL.
 * Priority 3 (lowest of the active tasks).
 */
static void cli_task(void *pvParameters)
{
    ESP_LOGI(TAG, "CLI task started");

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "p1p2> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_usb_serial_jtag_config_t hw_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();

    esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);

    register_commands();

    esp_console_start_repl(repl);

    /* REPL runs in its own task; this task can exit */
    vTaskDelete(NULL);
}

esp_err_t p1p2_cli_init(void)
{
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
    };
    esp_err_t ret = esp_console_init(&console_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Console init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t xret = xTaskCreate(cli_task, "cli", 4096, NULL, 3, &cli_task_handle);
    if (xret != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "CLI initialized");
    return ESP_OK;
}
