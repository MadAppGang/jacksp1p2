# P1P2MQTT ESP32-C6

Single-chip replacement for the dual-MCU P1P2MQTT system, targeting **Daikin VRV F-Series** heat pumps over **Matter/Thread**.

Replaces the ATmega328P (bus I/O) + ESP8266 (WiFi/MQTT) architecture with a single ESP32-C6 using MCPWM hardware peripherals for P1/P2 bus bit-banging and IEEE 802.15.4 for Thread networking.

**Status**: Firmware skeleton complete, protocol engine tested (20/20 unit tests passing), ready for hardware validation.

---

## Table of Contents

- [Why This Exists](#why-this-exists)
- [Architecture Decision: Single Chip](#architecture-decision-single-chip)
- [ESP32 Chip Selection Research](#esp32-chip-selection-research)
- [Why MCPWM Is Essential](#why-mcpwm-is-essential)
- [Why Matter over Thread (Not WiFi)](#why-matter-over-thread-not-wifi)
- [Hardware Design](#hardware-design)
- [Software Architecture](#software-architecture)
- [Bus I/O Design](#bus-io-design)
- [Matter Device Model](#matter-device-model)
- [F-Series Protocol Support](#f-series-protocol-support)
- [Testing & Simulation](#testing--simulation)
- [Build Instructions](#build-instructions)
- [Implementation Roadmap](#implementation-roadmap)
- [References](#references)

---

## Why This Exists

The original [P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT) project uses a **dual-MCU architecture**:

| MCU | Role | Why |
|---|---|---|
| **ATmega328P** (8 MHz) | Real-time P1/P2 bus bit-banging | Has hardware Timer1 input capture + output compare with pin toggle |
| **ESP8266** | WiFi, MQTT, OTA, Home Assistant integration | Has WiFi but lacks timer peripherals for bus I/O |

This split exists because the ESP8266 **cannot** do the bus I/O — it lacks the hardware timer peripherals (input capture, output compare with hardware pin toggle) needed for the 9600-baud HBS protocol. The ATmega provides these via Timer1 ICP1 (edge capture) and OC1A (hardware pin toggle on compare match).

**The problem**: Two MCUs means two firmware images, a serial bridge between them, doubled power consumption, and increased complexity.

**The solution**: The ESP32-C6 has **MCPWM** (Motor Control PWM) — a peripheral with hardware capture and hardware pin-toggle-on-compare. This enables identical bus I/O on a single chip, while also providing a native IEEE 802.15.4 radio for Thread/Matter.

---

## Architecture Decision: Single Chip

### Before (Dual-MCU)

```
Daikin VRV                                            Home Assistant
    │                                                       │
    │ P1/P2 bus (2-wire, 9600 baud HBS)                    │ WiFi/MQTT
    │                                                       │
┌───┴───────────┐   250kBaud UART   ┌───────────────────┐  │
│  ATmega328P   │ ←──────────────→  │    ESP8266         │──┘
│  8 MHz AVR    │                   │    80 MHz          │
│               │                   │                    │
│  Timer1 ICP1  │                   │  WiFi stack        │
│  Timer1 OC1A  │                   │  AsyncMqttClient   │
│  ISR bit-bang │                   │  HA auto-discovery │
│  CRC, ADC     │                   │  OTA updates       │
└───────────────┘                   └────────────────────┘
```

### After (Single ESP32-C6)

```
Daikin VRV                                     Home Assistant
    │                                               │
    │ P1/P2 bus (2-wire, 9600 baud HBS)            │ Thread (802.15.4)
    │                                               │ via Border Router
┌───┴───────────────────────────────────────┐       │
│              ESP32-C6                      │───────┘
│              160 MHz RISC-V               │
│                                           │
│  MCPWM capture → replaces Timer1 ICP1    │
│  MCPWM generator → replaces Timer1 OC1A  │
│  GPTimer → replaces Timer1 OCR1B         │
│  ADC continuous mode with DMA            │
│  NVS → replaces EEPROM                  │
│  802.15.4 radio → Thread/Matter          │
│  FreeRTOS tasks → structured concurrency │
│  USB-C serial → debug console            │
└───────────────────────────────────────────┘
```

**What's eliminated**: Serial bridge, ESP8266, WiFi stack, MQTT client, second firmware image.

---

## ESP32 Chip Selection Research

We evaluated all ESP32 variants for this application. The critical requirement is **MCPWM** — the only peripheral that provides hardware edge capture + hardware pin toggle on compare, both needed for the P1/P2 HBS protocol.

| Chip | MCPWM | 802.15.4 (Thread) | WiFi | Cores | Clock | Status |
|---|---|---|---|---|---|---|
| ESP32 | 2 units | No | Yes | 2 (Xtensa) | 240 MHz | No Thread radio |
| ESP32-S2 | No | No | Yes | 1 (Xtensa) | 240 MHz | No MCPWM |
| ESP32-S3 | 2 units | No | Yes | 2 (Xtensa) | 240 MHz | No Thread radio |
| ESP32-C2 | No | No | Yes (BLE) | 1 (RISC-V) | 120 MHz | No MCPWM |
| ESP32-C3 | No | No | Yes | 1 (RISC-V) | 160 MHz | No MCPWM |
| **ESP32-C6** | **1 unit** | **Yes** | **Yes** | **1 (RISC-V)** | **160 MHz** | **Best fit** |
| ESP32-C5 | 1 unit | Yes | Yes (WiFi 6) | 1 (RISC-V) | 240 MHz | Future upgrade |
| ESP32-H2 | 1 unit | Yes | No | 1 (RISC-V) | 96 MHz | Too slow |

### Why ESP32-C6 wins

- **Has MCPWM** — hardware capture + hardware pin toggle (essential for bus I/O)
- **Has IEEE 802.15.4** — native Thread/Zigbee support
- **160 MHz RISC-V** — more than enough for 9600 baud bus + protocol decode
- **Mature SDK** — ESP-IDF v5.4+ has stable MCPWM and OpenThread support
- **Available and cheap** — $5-8 for dev boards (XIAO ESP32-C6, DevKitC-1)

### Why not ESP32-C3?

The C3 was initially considered because it's common and inexpensive. However, it **lacks MCPWM entirely**. Without MCPWM, the alternatives are:

| Alternative | Problem |
|---|---|
| UART | 9600/8E1 is supported, but UART cannot detect bus collisions during write, and cannot handle the variable inter-byte gaps in HBS packets |
| RMT | Records waveforms but is designed for IR/LED protocols; awkward for byte-oriented half-duplex with parity and CRC |
| Software bit-bang with GPIO ISR | No hardware timestamp on edges — ISR latency (especially with WiFi/Thread) causes bit errors |
| LEDC/PCNT | Read-only (capture edges), cannot drive TX with hardware pin toggle |

None of these can replicate the ATmega's Timer1 ICP1 + OC1A behavior. MCPWM is the only ESP32 peripheral that can.

### ESP32-C5 as future upgrade

The ESP32-C5 has everything the C6 has plus:
- 240 MHz clock (vs 160 MHz)
- WiFi 6 dual-band (2.4 + 5 GHz)
- Larger SRAM

The C5 SDK is mature as of ESP-IDF v5.5.2, but the C6 has a longer track record. The firmware is designed to be a drop-in recompile for C5 (`idf.py set-target esp32c5`).

---

## Why MCPWM Is Essential

The P1/P2 bus uses the **HBS (Home Bus System)** protocol — a 2-wire, half-duplex, 9600-baud serial protocol with even parity and CRC. It looks simple, but has critical timing requirements:

### Receive: Why not just use UART?

1. **Collision detection**: During write, we must simultaneously read back what's on the bus to detect collisions. UART doesn't support simultaneous independent TX/RX on the same logical channel.
2. **Variable inter-byte gaps**: HBS packets have variable pauses between bytes. UART framing doesn't handle this — it expects continuous byte streams.
3. **End-of-packet detection**: Packets end with a timeout (no more bytes for ~9 bit times). UART has no built-in timeout mechanism that signals EOP.

### Transmit: Why not just toggle a GPIO?

1. **Jitter**: Software GPIO toggle from an ISR has variable latency (1-10 us), especially when WiFi/Thread interrupts are active. At 9600 baud, each half-bit is 52 us — even 5 us jitter is ~10% of a half-bit.
2. **Hardware pin toggle**: MCPWM's generator can set/clear a GPIO pin **in hardware** on a comparator match — zero jitter, identical to ATmega's OC1A behavior.
3. **Collision detection during TX**: The 20-state half-bit state machine reads back the bus state during each half-bit to detect collisions. This requires precise timing that software GPIO cannot guarantee.

### What MCPWM provides

| ATmega Feature | MCPWM Equivalent | Purpose |
|---|---|---|
| Timer1 Input Capture (ICP1) | MCPWM Capture Channel | Hardware-timestamp falling edges on RX pin |
| Timer1 OC1A (pin toggle on match) | MCPWM Generator action on compare event | Zero-jitter TX pin toggle |
| Timer1 OCR1B (mid-bit sample) | GPTimer one-shot alarm | Sample bit value at mid-point |

The MCPWM capture hardware **latches the edge timestamp** even if the ISR is delayed by a few microseconds. This is the key reliability feature — the timestamp is accurate regardless of ISR jitter.

### Timer resolution: 8 MHz

We configure the MCPWM capture timer at 8 MHz to match the ATmega's F_CPU exactly:
- `TICKS_PER_BIT = 833` (104.17 us at 9600 baud)
- `TICKS_PER_SEMIBIT = 416` (52.08 us)
- `TICKS_PER_BIT_AND_SEMIBIT = 1249` (156.25 us)

This means all timing constants from the original ATmega code are used unchanged.

---

## Why Matter over Thread (Not WiFi)

| Aspect | WiFi + MQTT (old) | Matter over Thread (new) |
|---|---|---|
| **Bus I/O reliability** | HIGH RISK: WiFi ISRs block CPU 5-10 ms | LOW RISK: Thread radio has ~100 us bursts |
| **Power consumption** | ~120 mA continuous | ~15-30 mA (Thread sleepy end device possible) |
| **HA integration** | Custom MQTT topics + auto-discovery | Native Matter integration (standard) |
| **OTA updates** | HTTP/HTTPS over WiFi | Matter OTA cluster or USB serial |
| **Protocol standard** | Custom MQTT topics | Industry-standard Matter clusters |
| **Network resilience** | Single point of failure | Thread mesh self-heals |
| **Future-proofing** | MQTT will persist | Matter is the smart home standard |

### The key technical win: ISR coexistence

On a single-core MCU, the bus I/O ISRs (MCPWM capture + GPTimer) must run without being blocked by the radio stack. This is the single biggest risk.

**WiFi** is terrible for this: the ESP32 WiFi stack routinely disables interrupts for 5-10 ms during TX/RX and beacon processing. At 9600 baud, a single byte takes 1.04 ms — a 5 ms WiFi ISR block would corrupt 5 bytes.

**Thread (802.15.4)** is much better: the radio uses short ~100 us bursts, and the OpenThread stack is cooperative. The MCPWM capture hardware latches timestamps even during these brief delays, so edge timing is preserved.

### Infrastructure: Sonoff iHost as Thread Border Router

A Thread device needs a border router to connect to the IP network. The Sonoff iHost has:
- Quad-core ARM Cortex-A35 + 2 GB RAM
- Built-in EFR32MG21 radio (supports Zigbee + Thread via MultiPAN)
- Can run Home Assistant OS (HAOS) with OpenThread Border Router add-on

This eliminates the need for a separate Thread border router dongle.

---

## Hardware Design

### Components (~$12-20 total)

| Component | Part | Purpose |
|---|---|---|
| ESP32-C6 dev board | Seeed XIAO ESP32-C6 or DevKitC-1 | MCU + Thread radio |
| P1/P2 bus transceiver | XL1192 or MM1192 | HBS bus interface (bidirectional) |
| 3.3V voltage regulator | AMS1117-3.3 | Power from USB 5V |
| Resistors | 3x 150R, 4x 330R | Bus impedance + LED current limiting |
| Capacitors | 2x 100nF, 1x 10uF, 1x 22-100uF film | Decoupling + DC blocking |
| Protection diodes | 2x 1N4148 | Bus overvoltage protection |
| LEDs | 4x (white, green, blue, red) | Status: power, read, write, error |

### Connection Diagram

```
                                        ESP32-C6 Dev Board
                                       +------------------+
P1/P2 Bus     XL1192/MM1192            |                  |
  P1 --+     +----------+              | GPIO2 <-- RX     | (MCPWM Capture)
       +-->--| RXD  VCC |---- 3.3V ---|                  |
       |     |          |              | GPIO3 --> TX     | (MCPWM Generator)
  P2 --+     | TXD  GND |---- GND ----|                  |
  (2-wire    +----------+              | GPIO0 <-- V_bus  | (ADC ch0)
   bus)     150R series resistors      | GPIO1 <-- V_bus  | (ADC ch1)
            + DC-blocking caps         |                  |
            between bus and IC         | GPIO4 --> LED_W  | (power)
                                       | GPIO5 --> LED_G  | (read)
                                       | GPIO6 --> LED_B  | (write)
                                       | GPIO7 --> LED_R  | (error)
                                       |                  |
                                       | 802.15.4 antenna | ))) Thread
                                       | USB-C <-- power  |
                                       +------------------+
```

### Alternative: MAX22088 transceiver
- Can extract power directly from the P1/P2 bus (no external USB power needed)
- TQFN-24 package with 0.5 mm pitch — requires reflow soldering

---

## Software Architecture

### Project Structure

```
p1p2mqtt-esp32c6/
+-- CMakeLists.txt
+-- sdkconfig.defaults            # Thread on, WiFi off, MCPWM on
+-- partitions.csv                # Dual OTA partition layout
+-- main/
|   +-- main.c                    # app_main(), task creation
|   +-- Kconfig.projbuild         # Model/pin/control-level config
+-- components/
|   +-- p1p2_bus/                 # Bus I/O HAL (MCPWM + GPTimer)
|   |   +-- p1p2_mcpwm_rx.c      # RX: MCPWM capture + GPTimer sampling
|   |   +-- p1p2_mcpwm_tx.c      # TX: MCPWM generator + 20-state machine
|   |   +-- p1p2_bus.c            # Packet assembly, CRC, ring buffer
|   |   +-- p1p2_adc.c            # Bus voltage monitoring (ADC continuous)
|   +-- p1p2_protocol/            # F-series decode + control
|   |   +-- p1p2_fseries_decode.c # Packet decode (0x10-0x16, 0xA3)
|   |   +-- p1p2_fseries_control.c# 0x38/0x3B response construction
|   |   +-- p1p2_param_conversion.c
|   +-- p1p2_matter/              # Matter device + cluster definitions
|   |   +-- p1p2_matter_device.c  # Node setup, endpoint registration
|   |   +-- p1p2_matter_thermostat.c
|   |   +-- p1p2_matter_fan.c
|   |   +-- p1p2_matter_sensors.c
|   |   +-- p1p2_matter_custom.c  # VRV-specific custom cluster
|   +-- p1p2_network/             # Thread, OTA, NVS config
|   |   +-- p1p2_thread.c         # OpenThread init, commissioning
|   |   +-- p1p2_ota.c            # Matter OTA + USB serial fallback
|   |   +-- p1p2_config_store.c   # NVS (replaces ATmega EEPROM)
|   +-- p1p2_cli/                 # Serial USB debug console
|       +-- p1p2_cli.c
+-- test/                         # Unity tests (run in Wokwi simulator)
    +-- main/test_main.c
```

### FreeRTOS Task Architecture

```
Priority  Task              Core    Stack   Purpose
--------  ----              ----    -----   -------
ISR       MCPWM/GPTimer     -       IRAM    Bit-level bus I/O (capture, sample, toggle)
22        bus_io_task        0       4096    Packet assembly from ISR ring buffer
15        protocol_task      0       8192    F-series decode, control response
10        matter_task        0       8192    Matter attribute updates, command callbacks
8         thread_task        0       -       OpenThread stack (managed by esp-matter)
3         cli_task           0       4096    Serial console commands
1         housekeeping       0       4096    ADC stats, LEDs, NVS save, uptime
```

### Data Flow

```
Daikin VRV P1/P2 Bus
       |
       v
MCPWM Capture ISR (IRAM_ATTR) -- hardware-timestamps falling edges
       |
       v
GPTimer Alarm ISR (IRAM_ATTR) -- samples bit value at mid-point
       |
       v
ISR Ring Buffer (byte + error per slot)
       |
       v
bus_io_task (priority 22) -- assembles bytes into packets, detects EOP
       |
       v
FreeRTOS Packet Queue (8 slots)
       |
       v
protocol_task (priority 15) -- F-series decode, state update, control responses
       |
       v
Matter Attribute Updates
       |
       v
matter_task (priority 10) -- pushes to Thread radio
       |
       v
Thread Border Router (Sonoff iHost)
       |
       v
Home Assistant
```

### ATmega to ESP32-C6 Peripheral Mapping

| ATmega Feature | ESP32-C6 Replacement |
|---|---|
| Timer1 Input Capture (ICP1) | MCPWM Capture Channel |
| Timer1 Output Compare A (OC1A) | MCPWM Generator action on comparator |
| Timer1 Output Compare B (OCR1B) | GPTimer one-shot alarm |
| Timer2 (1 kHz ms counter) | `esp_timer` periodic |
| Timer0 (uptime) | `esp_timer_get_time()` (64-bit us) |
| ADC (ISR-driven, 2-channel) | ADC Continuous Mode with DMA |
| EEPROM | NVS (Non-Volatile Storage) |
| Direct port GPIO (LEDs) | `gpio_set_level()` |
| UART to ESP8266 (250 kBaud) | Eliminated (single chip) |
| WiFi (ESP8266) | Thread 802.15.4 (native) |
| AsyncMqttClient | esp-matter SDK (Matter protocol) |

---

## Bus I/O Design

This is the most critical part of the port. The P1/P2 HBS protocol requires precise bit-level timing with collision detection.

### Receive Path (`p1p2_mcpwm_rx.c`)

Two cooperating ISRs, directly mirroring the ATmega `TIMER1_CAPT_vect` and `TIMER1_COMPB_vect`:

**1. MCPWM Capture Callback** (`IRAM_ATTR`)
- Triggers on **falling edge** of bus RX pin
- Hardware-timestamps the edge (latch survives ISR delay up to ~10 us)
- State machine: detect start bit, mark data bits as '0'
- Schedules GPTimer alarm for mid-bit sampling

**2. GPTimer Alarm Callback** (`IRAM_ATTR`)
- Fires at **mid-bit point** (416 ticks = 52 us after falling edge)
- If no falling edge occurred since last alarm → bit is '1'
- On stop bit: stores byte + error flags in ring buffer
- On EOP timeout (9+ bit times without activity): signals end-of-packet

### RX State Machine (12 states)

```
IDLE → (falling edge) → WAIT_START → DATA_BIT_0 ... DATA_BIT_7 → PARITY → STOP → WAIT_START
                                                                                → (timeout) → IDLE + EOP
```

### Transmit Path (`p1p2_mcpwm_tx.c`)

MCPWM Operator + Comparator + Generator, replacing `TIMER1_COMPA_vect`:

- **Hardware pin toggle**: `mcpwm_generator_set_action_on_compare_event()` drives the TX pin high or low on comparator match — zero jitter
- Comparator event callback implements the **20-state half-bit state machine**
- Each half-bit: reads GPIO input pin to detect collisions

### TX State Machine (20 half-bit states per byte)

```
State  1: Start bit, first half  (drive LOW)
State  2: Start bit, second half (drive LOW)
State  3: D0, first half  (drive LOW if bit=0, HIGH if bit=1)
State  4: D0, second half (drive HIGH)
...
State 17: D7, first half
State 18: D7, second half
State 19: Parity, first half
State 20: Parity, second half
→ Stop bit (release bus HIGH), schedule next byte or finish
```

### Collision Detection

During each half-bit ISR, the TX state machine reads the GPIO input pin:
- If driving LOW but reading HIGH → bus conflict (another device won)
- If driving HIGH but reading LOW → bus conflict (another device pulling low)
- On collision: abort transmission, set `P1P2_ERROR_BE` flag

### CRC

F-series uses CRC polynomial **0xD9** with initial value **0x00**:

```c
uint8_t crc = 0x00;
for (each byte) {
    for (each bit, LSB first) {
        crc = ((crc ^ byte) & 0x01) ? ((crc >> 1) ^ 0xD9) : (crc >> 1);
        byte >>= 1;
    }
}
```

CRC over the full packet (including CRC byte) verifies to zero.

---

## Matter Device Model

The device appears as a **Matter Thermostat** with multiple endpoints:

### Endpoint 1: Thermostat (cluster 0x0201)
| Attribute | ID | Type | Maps to |
|---|---|---|---|
| LocalTemperature | 0x0000 | int16 (C x 100) | Room/return temperature |
| OccupiedCoolingSetpoint | 0x0011 | int16 (C x 100) | Target cooling temp |
| OccupiedHeatingSetpoint | 0x0012 | int16 (C x 100) | Target heating temp |
| SystemMode | 0x001C | enum8 | Off/Heat/Cool/Auto |
| ThermostatRunningState | 0x0029 | bitmap16 | Idle/Heating/Cooling |

### Endpoint 2: Fan Control (cluster 0x0202)
| Attribute | ID | Maps to |
|---|---|---|
| FanMode | 0x0000 | Low/Medium/High/Auto |
| FanModeSequence | 0x0001 | Available modes |

### Endpoint 3: Temperature Sensors (cluster 0x0402)
Multiple instances for outdoor, leaving water, and return water temperatures.

### Endpoint 4: Custom VRV Cluster (0xFFF10001)
Manufacturer-specific attributes for VRV data not covered by standard clusters:
| Attribute | ID | Description |
|---|---|---|
| CompressorFreq | 0x0000 | Hz |
| FlowRate | 0x0001 | L/min x 10 |
| ErrorCode | 0x0002 | Daikin error code |
| OperationHours | 0x0003 | Total hours |
| CompressorStarts | 0x0004 | Total starts |
| BusVoltageP1 | 0x0005 | mV |
| BusVoltageP2 | 0x0006 | mV |
| PacketCount | 0x0007 | Total RX packets |

### Endpoint 5: On/Off (cluster 0x0006)
DHW (domestic hot water) on/off control.

Home Assistant automatically discovers and creates entities for all standard Matter clusters.

---

## F-Series Protocol Support

### Supported Models

Configured via `idf.py menuconfig` → P1P2MQTT Configuration:

| Kconfig Choice | Model ID | Daikin Units | Control Packet | Response Size |
|---|---|---|---|---|
| Model A/B/C/L/LA | 10 (BCL) | FDY, FBQ | 0x38 | 18 bytes |
| Model P/PA | 11 (P) | FXMQ | 0x38 | 20 bytes |
| Model M | 12 (M) | FDYQ | 0x3B | 22 bytes |

### Packet Types Decoded

| Type | Direction | Content |
|---|---|---|
| 0x10 | Main → Indoor | Power, mode, target temps, fan speeds |
| 0x11 | Main → Indoor | Room temp, outdoor temp |
| 0x12 | Main → Indoor | Date/time |
| 0x14 | Main → Indoor | Compressor frequency |
| 0xA3 | Counter request | Operation hours, compressor starts |
| 0x38 | Main ↔ Aux | Control exchange (BCL/P models) |
| 0x3B | Main ↔ Aux | Control exchange (M model) |
| 0x39/0x3A | Main ↔ Aux | Filter/status (BCL/P models) |
| 0x3C | Main ↔ Aux | Filter (M model) |

### Auxiliary Controller Operation

The ESP32-C6 acts as an **auxiliary controller** on the bus (address 0x40). When the indoor unit sends a control request packet (0x38 or 0x3B), the firmware:

1. Receives the request packet
2. Builds a response that echoes back the current state
3. Applies any pending control commands (temperature changes, mode changes, power on/off)
4. Appends CRC and transmits the response

Control levels (set via NVS or CLI):
- **Level 0**: Disabled (read-only monitoring)
- **Level 1**: Active auxiliary controller (responds to 0x38/0x3B)
- **Level 5**: Monitor only (listens but does not respond)

---

## Testing & Simulation

### Unit Tests

20 Unity test cases validated in [Wokwi simulator](https://wokwi.com/) on ESP32-C6 target:

| Category | Tests | What's Covered |
|---|---|---|
| Decode | 10 | Packet parsing for 0x10 (status, power, modes, fan), 0x11 (temps), 0x14 (compressor), 0xA3 (counters) |
| Control | 6 | 0x38 response for BCL/P models, 0x3B for M model, model cross-rejection, empty responses |
| Pending writes | 2 | Temperature override, power command application |
| CRC | 2 | Polynomial 0xD9, verify-to-zero property |

### Running Tests

```bash
# Build test firmware
cd test
source ~/esp/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build

# Run in Wokwi simulator
source ../.env  # loads WOKWI_CLI_TOKEN
wokwi-cli --timeout 15000 .
```

### Simulation Options Evaluated

| Tool | ESP32-C6 Support | MCPWM | Verdict |
|---|---|---|---|
| **Wokwi** | Yes | No | Best for protocol/logic testing |
| QEMU | No (ESP32 only) | No | Not usable |
| Renode | No | No | Not usable |
| ESP-IDF Linux/POSIX | Partial | No | Protocol-only testing |
| Unity + CMock | N/A (host) | N/A | Pure logic testing |

**Wokwi** is the best option: it runs real ESP-IDF firmware on a simulated ESP32-C6 with FreeRTOS, NVS, GPTimer, and ADC support. MCPWM is not simulated, but that only affects bus I/O — all protocol logic, control responses, and CRC calculations run correctly.

Bus I/O validation requires **real hardware** + oscilloscope.

---

## Build Instructions

### Prerequisites

- ESP-IDF v5.4+ (`~/esp/esp-idf`)
- Python 3.8+
- cmake, ninja

### Install ESP-IDF

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
```

### Build Firmware

```bash
cd p1p2mqtt-esp32c6
source ~/esp/esp-idf/export.sh
idf.py set-target esp32c6
idf.py menuconfig    # Optional: set model, pins, control level
idf.py build
```

Or use the helper script:

```bash
./build.sh build
```

### Flash to Hardware

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### Configuration

All configuration is via `idf.py menuconfig` under "P1P2MQTT Configuration":

| Setting | Default | Options |
|---|---|---|
| Daikin series | F-Series | F-Series, E-Series |
| F-Series model | BCL (A/B/C/L/LA) | BCL, P, M |
| Bus RX GPIO | 2 | Any valid GPIO |
| Bus TX GPIO | 3 | Any valid GPIO |
| ADC channel 0 | GPIO 0 | Any ADC-capable GPIO |
| ADC channel 1 | GPIO 1 | Any ADC-capable GPIO |
| LED GPIOs | 4, 5, 6, 7 | Any valid GPIO |
| Control level | 0 (disabled) | 0-5 |

---

## Implementation Roadmap

### Phase 0: Infrastructure (complete)
- [x] ESP-IDF v5.4.1 installed with ESP32-C6 toolchain
- [x] Project skeleton with all components
- [x] Wokwi simulator configured
- [x] 20 unit tests passing
- [x] GitHub repo created

### Phase 1: Bus I/O Validation
- [ ] MCPWM capture-based receive — test with signal generator
- [ ] MCPWM generator-based transmit — verify with oscilloscope
- [ ] TX/RX loopback with collision detection
- [ ] Connect to real VRV bus (read-only) — dump packets
- [ ] **Gate**: Zero CRC errors over 24-hour passive monitoring

### Phase 2: Protocol Engine
- [ ] Port full F-series data structures from P1P2_ParameterConversion.h
- [ ] Port complete `bytesbits2keyvalue()` decode (0x10-0x3F, 0xA3)
- [ ] Port 0x38/0x3B auxiliary controller response logic
- [ ] Pseudo-packet generation
- [ ] **Gate**: Decoded values match existing dual-MCU system

### Phase 3: Matter/Thread Integration
- [ ] OpenThread + Matter on ESP32-C6
- [ ] Matter Thermostat endpoint
- [ ] Fan Control and Temperature Sensor endpoints
- [ ] Manufacturer-specific VRV custom cluster
- [ ] Commission to Thread network via iHost
- [ ] **Gate**: All entities visible and controllable in Home Assistant

### Phase 4: Polish
- [ ] Matter OTA + USB serial fallback
- [ ] NVS config store (~80 key-value pairs from EEPROM)
- [ ] Serial CLI command parser
- [ ] LED status indicators
- [ ] 7-day stability test on real VRV hardware

### Risks and Mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| Thread ISR interferes with bus I/O | LOW | Thread has minimal ISR overhead; MCPWM capture survives delays |
| MCPWM TX jitter | LOW | Hardware pin toggle; validate with scope; RMT fallback available |
| F-series code port (6K+ lines) | MEDIUM | Logic is platform-independent; port incrementally |
| Matter clusters incomplete for VRV | MEDIUM | Standard clusters + custom cluster for VRV-specific data |
| Single-core task starvation | LOW | Priority preemption; Thread lighter than WiFi |

---

## Partition Table

Dual OTA layout on 4 MB flash:

| Name | Type | Offset | Size | Purpose |
|---|---|---|---|---|
| nvs | data | 0x9000 | 24 KB | Config key-value store |
| otadata | data | 0xF000 | 8 KB | OTA state tracking |
| phy_init | data | 0x11000 | 4 KB | RF calibration |
| ota_0 | app | 0x20000 | 1.75 MB | Firmware slot A |
| ota_1 | app | 0x1E0000 | 1.75 MB | Firmware slot B |
| nvs_key | data | 0x3A0000 | 4 KB | NVS encryption keys |
| fctry | data | 0x3A1000 | 24 KB | Factory NVS data |

---

## References

- [P1P2MQTT (original dual-MCU project)](https://github.com/Arnold-n/P1P2MQTT)
- [ESP32-C6 MCPWM API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/mcpwm.html)
- [ESP-Matter SDK](https://docs.espressif.com/projects/esp-matter/en/latest/esp32c6/index.html)
- [Home Assistant Matter Integration](https://www.home-assistant.io/integrations/matter/)
- [Home Assistant Thread Integration](https://www.home-assistant.io/integrations/thread/)
- [Wokwi ESP32-C6 Simulator](https://wokwi.com/)
- [Seeed XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_zigbee/)

---

## License

Based on [P1P2MQTT](https://github.com/Arnold-n/P1P2MQTT) by Arnold Niessen (CC BY-NC-ND 4.0).
ESP32-C6 port: 2026.
