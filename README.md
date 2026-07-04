# LightTrack — ESP32 Tracker Firmware

## Project

Firmware for the ESP32 that drives the **LightTrack** dual-axis solar tracker — the
field/hardware layer of a four-part system (ESP32 firmware, Raspberry Pi gateway,
cloud backend, web dashboard). It reads four light sensors, aims the panel with two
servos to follow the sun, measures the panel and battery with two INA219 sensors, shows
status on an OLED, and exchanges telemetry and commands with the system over MQTT.

## Deliverables

- **Repository (full source code, no compiled binaries):** `<https://gitlab.upt.ro/...>`
- Source files: the Arduino sketch (`LightTrack.ino`) and a `secrets.h` that you create
  from the template below (credentials are never committed).
- No build artifacts (`.bin`, `.elf`, `build/`) are included — you compile and flash it
  yourself with the steps below.

## Hardware

- ESP32 development board
- 2× servo — azimuth on GPIO 18, elevation on GPIO 19
- 4× LDR light sensors on ADC pins 32 / 35 / 34 / 33 (top-left, top-right, bottom-left, bottom-right)
- 2× INA219 on the I²C bus (SDA 21, SCL 22): panel sensor at `0x40`, battery sensor at `0x41`
- SSD1306 128×64 OLED display on the same I²C bus

## Dependencies / prerequisites

**Toolchain** (either one):
- Arduino IDE 2.x, or
- arduino-cli

**ESP32 board support** — "esp32 by Espressif Systems", **version 3.x** (the firmware uses
the ESP-IDF 5 task-watchdog API, so core 2.x will not compile). In Arduino IDE add this
Boards Manager URL, then install **esp32**:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

**Libraries** (install from the Library Manager):
- PubSubClient — Nick O'Leary
- ArduinoJson — Benoit Blanchon (v6 API)
- ESP32Servo
- Adafruit INA219 (also installs Adafruit BusIO)
- U8g2 — oliver
- SunSet — Peter Buelow

`WiFi`, `WiFiMulti`, `Wire`, `Preferences` and the task watchdog come with the ESP32 core —
no separate install.

## Configuration — `secrets.h`

Create `secrets.h` in the same folder as the sketch. It is not committed. Template:

```cpp
#pragma once

// Wi-Fi (primary network)
#define SECRET_WIFI_SSID       "your-wifi"
#define SECRET_WIFI_PASSWORD   "your-wifi-password"

// Wi-Fi (optional second network)
// #define SECRET_WIFI_SSID_2     "backup-wifi"
// #define SECRET_WIFI_PASSWORD_2 "backup-password"

// MQTT broker
#define SECRET_MQTT_HOST       "192.168.1.10"   // Raspberry Pi / broker IP
// #define SECRET_MQTT_HOST_2  "second-broker"  // optional failover broker
#define SECRET_MQTT_PORT       1883
#define SECRET_MQTT_CLIENT_ID  "lighttrack-esp32"
#define SECRET_MQTT_USERNAME   "mqtt-user"
#define SECRET_MQTT_PASSWORD   "mqtt-password"
```

## Build / compilation steps

**Arduino IDE**
1. Open the sketch (`LightTrack.ino`); keep `secrets.h` in the same folder.
2. Install the ESP32 board package (v3.x) and the libraries listed above.
3. Tools → Board → **ESP32 Dev Module** (or your exact board).
4. Sketch → **Verify** to compile.

**arduino-cli** (alternative)
```bash
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

## Installation steps (flashing)

**Arduino IDE**
1. Connect the ESP32 over USB.
2. Tools → Port → select the board's serial port.
3. Sketch → **Upload**.

**arduino-cli**
```bash
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 .
```
Replace the port with yours (Windows uses a `COM` port).

## Running / launch

The firmware starts automatically on power-up or reset. To watch it, open the **Serial
Monitor at 115200 baud**.

On boot it centers the servos, connects to Wi-Fi and the MQTT broker, syncs time over NTP,
and starts tracking the sun. If it cannot reach the broker it keeps working offline and
falls back to automatic tracking after ~60 s. It publishes telemetry to `solar/telemetry`
and accepts commands (move, set mode, reset, start/stop tracking) on `solar/commands`.