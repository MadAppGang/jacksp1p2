# P1P2MQTT ESP32-C6 — Hardware Setup & Bring-Up Guide

Complete instructions for building, wiring, and commissioning a P1/P2 bus interface for Daikin VRV (F-Series) systems using an ESP32-C6 with Matter over Thread.

## 1. Required Hardware

| Component | Description | Notes |
|-----------|-------------|-------|
| **ESP32-C6 dev board** | ESP32-C6-DevKitC-1 or equivalent | Must have IEEE 802.15.4 (Thread) radio |
| **P1/P2 bus interface IC** | MM1192 or XL1192 | Bus transceiver for Daikin P1/P2 protocol |
| **Resistors** | 10kΩ pull-up, voltage divider (see schematic) | For bus signal conditioning |
| **LEDs** (optional) | 4× LEDs + 330Ω resistors | Power, Read, Write, Error indicators |
| **Daikin VRV unit** | F-series (FDY, FDYQ, FXMQ, FBQ) | With accessible P1/P2 bus terminals |
| **Thread Border Router** | Apple HomePod Mini, Google Nest Hub (2nd gen), or OTBR | Required for Matter commissioning |
| **USB-C cable** | For programming and serial monitor | Connects to ESP32-C6 dev board |

## 2. GPIO Pin Assignment

Default pins (configurable via `idf.py menuconfig` → P1P2MQTT Configuration → GPIO Pin Assignment):

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 2 | Bus RX (MCPWM capture) | Input | From MM1192/XL1192 RXD output |
| 3 | Bus TX (MCPWM generator) | Output | To MM1192/XL1192 TXD input |
| 0 | ADC channel 0 | Input | Bus voltage monitor P1 |
| 1 | ADC channel 1 | Input | Bus voltage monitor P2 |
| 4 | LED Power (white) | Output | Heartbeat blink every 2s |
| 5 | LED Read (green) | Output | Blinks on RX packet |
| 6 | LED Write (blue) | Output | Blinks on TX packet |
| 7 | LED Error (red) | Output | Lights on CRC/parity error |

## 3. Circuit Schematic

### Using MM1192/XL1192 Bus Transceiver (Recommended)

The MM1192 (or pin-compatible XL1192) handles P1/P2 bus voltage levels (15-18V differential) and converts to 3.3V logic.

```
 Daikin VRV                MM1192 / XL1192              ESP32-C6
 ┌────────┐               ┌──────────────┐            ┌──────────┐
 │        │               │              │            │          │
 │  P1  ──┼───────────────┤ P1     RXD ──┼────────────┤ GPIO2 RX │
 │        │               │              │            │          │
 │  P2  ──┼───────────────┤ P2     TXD ──┼────────────┤ GPIO3 TX │
 │        │               │              │            │          │
 │  GND ──┼───────────────┤ GND    VCC ──┼── 3.3V ───┤ 3V3      │
 │        │               │              │            │          │
 └────────┘               └──────────────┘            └──────────┘
```

### Bus Voltage Monitoring (Optional)

For voltage monitoring via ADC, add a resistive voltage divider from the P1/P2 lines:

```
  P1 ──── 100kΩ ──┬── 10kΩ ── GND
                   │
                   └──────────── GPIO0 (ADC0)

  P2 ──── 100kΩ ──┬── 10kΩ ── GND
                   │
                   └──────────── GPIO1 (ADC1)
```

This gives approximately 10:1 voltage division (18V bus → 1.6V at ADC).

### LED Indicators (Optional)

```
  GPIO4 ── 330Ω ──┤>│── GND   (Power/heartbeat, white)
  GPIO5 ── 330Ω ──┤>│── GND   (Read activity, green)
  GPIO6 ── 330Ω ──┤>│── GND   (Write activity, blue)
  GPIO7 ── 330Ω ──┤>│── GND   (Error, red)
```

## 4. Build & Flash

### Prerequisites

Install ESP-IDF v5.4:

```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git -b v5.4.1
cd esp-idf
./install.sh esp32c6
```

### Stub Mode (Without Matter SDK)

For development or monitoring-only use without Matter:

```bash
cd ~/smarthome/jacksp1p2
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Full Mode (With Matter SDK)

For full Matter/Thread support:

```bash
# Install esp-matter SDK (one-time)
cd ~/esp
git clone --recursive https://github.com/espressif/esp-matter.git
cd esp-matter
./install.sh
source export.sh

# Build with Matter
export ESP_MATTER_PATH=~/esp/esp-matter
cd ~/smarthome/jacksp1p2
source ~/esp/esp-idf/export.sh
source $ESP_MATTER_PATH/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Configuration

Before building, optionally configure your model variant:

```bash
idf.py menuconfig
```

Navigate to **P1P2MQTT Configuration**:
- **Daikin series protocol**: F-Series (default)
- **F-Series model variant**: Select your unit type
  - Model A/B/C/L/LA (FDY/FBQ) — most common
  - Model P/PA (FXMQ)
  - Model M (FDYQ)
- **GPIO Pin Assignment**: Adjust if not using default pins
- **Default control level**: 0 (read-only, safest for first boot)

## 5. First Boot Verification

After flashing, open the serial monitor (`idf.py monitor` or screen/minicom at 115200 baud). You should see:

```
========================================
P1P2MQTT ESP32-C6 — Daikin VRV F-Series
Matter over Thread, single-chip design
========================================
Initializing config store...
Initializing P1/P2 bus I/O...
Initializing protocol engine...
Initializing Thread stack...
Initializing Matter device...
Matter stack started
========================================
Initialization complete!
  Bus I/O:  RX=GPIO2 TX=GPIO3
  Model:    10
  Control:  level 0
  Thread:   not attached
  Matter:   not commissioned
========================================
```

Within 1-2 seconds, you should see decoded packets:

```
I p1p2_decode: [00→80] Type=0x10 Len=18 — Status
I p1p2_decode: TEST: 00 80 10 01 AA 18 00 ...
I matter_therm: SystemMode: 3
I matter_therm: LocalTemperature: 2150 (21.5°C)
```

### What to check:

- **Packets appearing**: Bus I/O is working. You should see a new packet every 0.8-2 seconds.
- **Decoded values**: Room temperature, mode, and setpoints should match your VRV's display.
- **Periodic stats**: Every 60 seconds you'll see uptime and packet counters.

## 6. Monitor-Only Mode (Safe Start)

The default control level is 0 (disabled). In this mode the device **only reads** from the P1/P2 bus — it never transmits, so it cannot affect your HVAC system.

This is the safest way to verify everything works:

1. Connect the hardware and flash the firmware
2. Open serial monitor
3. Verify packets are being decoded correctly
4. Compare decoded temperatures/modes with the indoor unit display
5. Let it run for several hours to check stability

To explicitly set monitor-only mode via the CLI:

```
p1p2> ctrl 0
Control level set to 0 (disabled)
```

## 7. Matter Commissioning

Once you're confident the bus interface works, commission the device via Matter.

### Prerequisites

- Thread Border Router on your network (HomePod Mini, Nest Hub 2nd gen, or standalone OTBR)
- Matter controller app (Apple Home, Google Home, Samsung SmartThings, or Home Assistant with Matter integration)

### Commissioning Steps

1. **Power on** the ESP32-C6. It enters commissioning mode automatically on first boot.
2. **Open your controller app** and add a new Matter device.
3. **Scan the QR code** or enter the manual pairing code from the serial monitor output.
4. The controller provisions Thread network credentials and pairs the device.
5. After commissioning, the serial monitor shows:
   ```
   I p1p2_matter: Commissioning complete
   I p1p2_thread: Thread: attached
   ```

### Verifying in Home Assistant

After commissioning, Home Assistant should show:
- **Climate entity**: Thermostat with heat/cool/auto modes and temperature setpoints
- **Fan entity**: Fan speed control (Low/Med/High/Auto)
- **Temperature sensors**: Outdoor, room, leaving water, return water
- **Switch**: DHW (domestic hot water) on/off
- **Custom sensors**: Compressor frequency, flow rate, error codes (via custom VRV cluster)

## 8. Auxiliary Controller Mode

Once monitoring is verified, enable auxiliary controller mode to allow control from your smart home system.

```
p1p2> ctrl 1
Control level set to 1 (auxiliary controller)
```

In this mode, the ESP32-C6 responds to the VRV's polling packets (0x38 or 0x3B depending on model), allowing it to:
- Change operating mode (heat/cool/auto/fan/dry)
- Adjust temperature setpoints
- Control fan speed
- Toggle DHW (domestic hot water)

### Verify control works:

1. Change the thermostat setpoint in Home Assistant
2. Observe the serial monitor for the write command:
   ```
   I p1p2_ctrl: Queued write: pkt=0x38 off=2 val=0x1C cnt=3
   I p1p2_ctrl: Write applied: pkt=0x38 off=2 remaining=2
   ```
3. Verify the VRV unit responds (check indoor unit display)

### Save control level to NVS:

The control level persists across reboots (saved to NVS automatically).

## 9. Troubleshooting

### No packets received

- **Check wiring**: Verify P1 and P2 are connected to the correct MM1192 pins
- **Check bus voltage**: Use ADC monitoring (`p1p2> adc`) or a multimeter. P1-P2 should show 15-18V DC
- **Check GPIO config**: Verify RX pin matches your wiring (`idf.py menuconfig` → GPIO Pin Assignment)
- **Bus not active**: Some VRV units only activate the P1/P2 bus when an indoor unit or controller is connected

### CRC errors

- **Bus noise**: Keep wires short, use twisted pair, avoid routing near power cables
- **Wrong model**: Check that the F-Series model variant matches your unit
- **Parity errors**: May indicate marginal signal quality — check connections and consider shorter cable runs

### No Thread network

- **Border Router**: Ensure your Thread Border Router is powered on and on the same network
- **Commissioning**: The device must be commissioned via Matter before it joins Thread
- **Range**: ESP32-C6's 802.15.4 radio range is approximately 10-30m indoors. Move closer to the border router if needed
- **Reset**: If commissioning fails, factory reset via CLI (`p1p2> factory_reset`) or by erasing flash (`idf.py erase-flash`)

### Matter device not appearing

- **Check commissioning window**: It's open for 5 minutes after boot. Reboot if expired
- **Controller compatibility**: Ensure your Matter controller supports Thread devices
- **Logs**: Check serial monitor for Matter stack errors

### Decoded values don't match

- **Wrong model variant**: Different F-Series models have different packet formats. Try switching between Model A/B/C/L, Model P, and Model M in menuconfig
- **Multi-indoor setup**: If multiple indoor units are on the bus, you may see packets from different units. The ESP32-C6 monitors address 0x80 (first indoor unit) by default

### High packet error rate

- **Cable quality**: Use shielded twisted pair for runs over 5m
- **Grounding**: Ensure common ground between ESP32-C6 and VRV unit
- **Power supply**: Use a stable 3.3V supply. USB power from a computer is usually fine for development
