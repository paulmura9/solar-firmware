# solar-firmware

ESP32 firmware for LightTrack, a dual-axis solar tracking system developed as a bachelor's thesis project. The firmware controls two servo motors that orient a small solar panel toward the sun using light sensor feedback, monitors panel and battery power, displays local status on an OLED, and communicates with a Raspberry Pi gateway over MQTT.

## Hardware

- ESP32 DevKit (WROOM-32)
- 2x SG90 servos: azimuth on GPIO 18, elevation on GPIO 19
- 4x LDR sensors on ADC1 pins: top-left GPIO 32, top-right GPIO 35, bottom-left GPIO 34, bottom-right GPIO 33
- 2x INA219 power monitors on I2C: solar panel at 0x40, battery at 0x41
- SSD1306 0.96" OLED (dual-color) at 0x3C on the same I2C bus
- I2C: SDA on GPIO 21, SCL on GPIO 22

## Behavior

### Tracking
In AUTO mode the four LDRs are read as two differential pairs (left/right and top/bottom). The error on each axis drives the corresponding servo with a proportional step, saturated at a maximum of 2 degrees per tick, with a deadband to prevent jitter. All movement is clamped to safe mechanical limits (5 to 175 degrees on both axes). When AUTO is entered from another mode, a coarse sweep of both axes finds the brightest direction before fine tracking takes over.

### Night and day handling
The firmware syncs time over NTP and computes astronomical sunrise and sunset for the configured location using the SunSet library. At night the panel parks at the home position (90, 90) and the previous mode is saved. In the morning that mode is restored; if it was AUTO, the panel pre-aims at the computed sunrise azimuth so fine tracking picks up the sun immediately.

### Telemetry
Every second the firmware publishes a JSON snapshot over MQTT: servo angles, tracking mode, raw LDR values and differentials, solar voltage/current/power, daily energy, and battery voltage, estimated charge percent, and charging status. The battery INA219 measures net battery current, so charging figures represent net energy into the battery.

### Persistence
Daily energy counters (solar generation and net charged energy) are accumulated in Wh and saved to NVS every 5 minutes, so values survive resets. Counters reset automatically at the first sample of a new day.

### Commands
The firmware accepts JSON commands (MOVE_PANEL, SET_MODE, START_TRACKING, STOP_TRACKING, RESET_POSITION, REQUEST_STATUS) and acknowledges each with a status message. Out-of-range movement requests are rejected and reported.

## Communication

MQTT broker: the local Mosquitto instance running on the Raspberry Pi gateway (authenticated).

Topics:
- `solar/telemetry` - published by the firmware, sensor and state snapshot
- `solar/commands` - subscribed by the firmware, incoming commands
- `solar/commands/ack` - published by the firmware, command acknowledgements

Data flow: this firmware -> MQTT -> Raspberry Pi gateway -> Express backend -> web dashboard.

## Build and flash

Arduino IDE with the ESP32 board package. Board: "ESP32 Dev Module".

Libraries used:
- ESP32Servo
- PubSubClient
- ArduinoJson (7.x)
- Adafruit INA219
- U8g2
- SunSet (buelowp)
- Preferences, WiFi, Wire (bundled with the ESP32 core)

WiFi and MQTT credentials are defined as constants at the top of the sketch and must be edited before flashing.

## Configuration

All hardware and tuning constants are defined in the configuration block at the top of the sketch: pin assignments, servo angle limits and home position, LDR thresholds and tracking gains, I2C addresses, battery discharge curve, location coordinates, and timezone.
