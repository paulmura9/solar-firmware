/*
 * LightTrack - ESP32 two-servo controller + 2x INA219 + OLED (MQTT)
 *
 * Telemetry units are SI: solar_voltage [V], solar_current [A], solar_power [W],
 * solar_energy_today_wh [Wh]. The OLED shows mA/mW locally for readability.
 *
 * Two INA219 sensors share the I2C bus:
 *   inaPanel   @ 0x40 -> solar panel side (default address)
 *   inaBattery @ BATTERY_INA_ADDRESS -> battery side (MUST differ from 0x40)
 *
 * The battery INA219 measures NET battery current (charge minus load on the
 * shared rail). charging_* and charged_energy_today_wh therefore represent NET
 * energy gained by the battery, not gross MPPT-delivered energy.
 *
 * AUTO tracking uses a proportional step: the correction applied per tick is
 * scaled by the LDR error magnitude and saturated at MAX_TRACK_STEP. This gives
 * fast convergence on large errors and fine motion near the optimum, while the
 * deadband prevents jitter. No integral/derivative term is used because the
 * servos already provide internal position control.
 *
 * Sky sweep policy: a full coarse scan runs whenever the panel TRANSITIONS
 * INTO AUTO from a different mode (boot/IDLE -> AUTO, MANUAL -> AUTO), and as
 * a fallback if AUTO sees darkness for too long during the DAY ("lost sun").
 * Lost-sun fallback is suppressed during astronomical night - there is no sun
 * to find below the horizon, so scanning would just burn through servos.
 * Re-entering AUTO when already in AUTO does NOT trigger a redundant scan.
 *
 * Night policy:
 *   - ENTRY: panel moves to HOME (90,90) and switches to NIGHT mode. The
 *     mode that was active before night is saved in modeBeforeNight.
 *   - EXIT (morning): mode is RESTORED to whatever was active before night.
 *       AUTO -> stays AUTO, panel pre-aims at today's sunrise azimuth so
 *               the LDR fine-tune immediately picks up the rising sun.
 *       MANUAL -> stays MANUAL, panel remains parked at HOME until the
 *               user sends a new MOVE_PANEL command.
 *       IDLE   -> stays IDLE, panel remains parked at HOME doing nothing.
 *     This preserves user intent across the night - if you chose MANUAL or
 *     IDLE in the afternoon, you keep that mode in the morning.
 *   - TELEMETRY: while in NIGHT the telemetry cadence drops from the 1s
 *     daytime rate to TELEMETRY_INTERVAL_NIGHT_MS (30s), since nothing is
 *     tracking and the values barely change - this cuts WiFi/MQTT traffic and
 *     radio power overnight. A REQUEST_STATUS command still forces an immediate
 *     publish, and any manual command exits NIGHT, restoring the 1s cadence for
 *     the duration of the manual-override window.
 *
 * Error policy:
 *   - ERROR is entered only on a catastrophic I2C bus failure at boot (no INA219
 *     and no OLED answering at all). Tracking itself uses ADC (LDRs) + PWM
 *     (servos) and does not need I2C, but with the whole sensing/display bus
 *     dead the system cannot monitor anything, so it refuses to run unattended.
 *   - A SINGLE missing I2C device is NOT an error: it is handled by graceful
 *     degradation (its fields are omitted from telemetry).
 *   - ERROR is sticky: recovery is by reboot after fixing the wiring.
 *
 * OLED: dual-color SSD1306 0.96" (yellow top 16 rows + white bottom 48 rows,
 * colors fixed by the panel hardware). The normal screen uses the yellow band
 * for the "-- PANEL --" header and the white band for V/I/P. When the panel
 * reaches a mechanical limit on either axis (AUTO, MANUAL, or rejected
 * out-of-range commands), the screen switches to a "!! WARNING !!" banner in
 * yellow with the axis/limit detail in white, for LIMIT_WARNING_MS, then
 * returns to normal. The warning fires once per ENTRY into the limit
 * (anti-spam) and re-arms when the panel leaves the limit.
 *
 * Cardinal azimuth convention (for documentation, not exposed to firmware):
 *   panel_azimuth = horizontal_angle + 90
 *   0°=E / 90°=S / 180°=W (i.e. servo 90° aims at true South).
 *   Sunrise azimuth math here uses 0=N, 90=E, 180=S, 270=W (compass),
 *   converted to servo via servo = cardinal - 90.
 *
 * Dependencies (Arduino IDE):
 *   ESP32 board package, ESP32Servo, PubSubClient, ArduinoJson 7.x,
 *   Adafruit INA219, U8g2, SunSet (buelowp) -> #include <sunset.h>, Preferences.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>
#include <sunset.h>
#include "esp_task_wdt.h"
#include "secrets.h"   // WiFi + MQTT credentials (NOT tracked in git)

// =========================== Configuration ==================================

static const char*    WIFI_SSID     = SECRET_WIFI_SSID;
static const char*    WIFI_PASSWORD = SECRET_WIFI_PASSWORD;

static const char*    MQTT_HOST      = SECRET_MQTT_HOST;
static const uint16_t MQTT_PORT      = SECRET_MQTT_PORT;
static const char*    MQTT_CLIENT_ID = SECRET_MQTT_CLIENT_ID;
static const char*    MQTT_USERNAME  = SECRET_MQTT_USERNAME;
static const char*    MQTT_PASSWORD  = SECRET_MQTT_PASSWORD;

static const char* TOPIC_TELEMETRY = "solar/telemetry";
static const char* TOPIC_COMMANDS  = "solar/commands";
static const char* TOPIC_ACK       = "solar/commands/ack";

static const uint8_t PIN_SERVO_AZIMUTH   = 18;
static const uint8_t PIN_SERVO_ELEVATION = 19;

static const uint8_t PIN_LDR_TL = 32;
static const uint8_t PIN_LDR_TR = 35;
static const uint8_t PIN_LDR_BL = 34;
static const uint8_t PIN_LDR_BR = 33;

static const uint8_t PIN_I2C_SDA = 21;
static const uint8_t PIN_I2C_SCL = 22;

static const uint8_t H_MIN = 5;
static const uint8_t H_MAX = 175;
static const uint8_t V_MIN = 5;
static const uint8_t V_MAX = 175;

static const uint8_t H_HOME = 90;
static const uint8_t V_HOME = 90;

// Sunrise pre-aim elevation: panel low, close to where the sun is at horizon.
// The LDR fine-tune will lift it as the sun climbs.
static const uint8_t V_SUNRISE = 30;

static const uint16_t MIN_STEP_DELAY_MS     = 35;   // smoother manual sweep

// Telemetry cadence: 1s while active (AUTO/MANUAL/IDLE during the day), and a
// reduced 30s rate while in NIGHT mode (nothing is tracking, values are static,
// so we save WiFi/MQTT traffic and radio power overnight). REQUEST_STATUS still
// forces an immediate publish regardless of cadence.
static const uint16_t TELEMETRY_INTERVAL_MS       = 1000;
static const uint32_t TELEMETRY_INTERVAL_NIGHT_MS = 30000;

static const uint8_t LDR_SAMPLES = 8;

// ----- Tracking tuning (gentler still - peak ~20°/s in AUTO) -----
static const int      LDR_DEADBAND      = 350;
static const int      DARK_THRESHOLD    = 450;   // avg LDR below this -> no tracking/scan
static const uint8_t  MAX_TRACK_STEP    = 2;     // max degrees per tracking tick
static const int      PROP_STEP_DIVISOR = 1000;  // larger -> gentler ramp to max
static const uint16_t TRACK_INTERVAL_MS = 100;
static const uint16_t SAMPLE_INTERVAL_MS = 200;

// ----- Coarse scan -----
// 5° step + 90 ms settle: a smooth, near-continuous sweep (the 8°/200 ms
// variant moved in big jerky steps and strained the servos). The scan takes
// ~6-7 s but pets the watchdog itself (see performScan), so its blocking
// duration does not trip the WDT. Fine tracking (trackStep) corrects the small
// scan coarseness within a few ticks.
static const uint8_t  SCAN_STEP        = 5;
static const uint16_t SCAN_SETTLE_MS   = 90;
static const uint8_t  SCAN_LDR_SAMPLES = 4;

// "Lost sun" fallback: after this many consecutive AUTO ticks below
// DARK_THRESHOLD (~10s at TRACK_INTERVAL_MS=100), request a fresh sweep.
// updateLowLight() owns the longer night/idle transitions; this only
// catches "panel pointed completely wrong after a manual move".
// Suppressed at astronomical night - see trackStep().
static const uint16_t LOST_SUN_TICKS = 100;

// ----- Low-light handling -----
static const int      LOW_LIGHT_THRESHOLD  = 650;
static const uint32_t LOW_LIGHT_CONFIRM_MS = 5000;
static const uint32_t DARK_TO_IDLE_MS      = 1800000;
static const uint32_t MANUAL_OVERRIDE_MS   = 600000;
static const uint32_t LIGHT_RESTORE_MS     = 5000;

// ----- OLED limit warning -----
static const uint32_t LIMIT_WARNING_MS = 5000;

// ----- Hardware watchdog -----
// If loop() doesn't pet the WDT within this window, the chip reboots. The
// system is unattended outdoor, so silent hangs (I2C stuck, library deadlock,
// etc.) must self-recover. 30s is comfortably above the coarse scan (~6-7s,
// which also pets the WDT itself) and below any user's patience for a frozen
// dashboard. WiFi/MQTT reconnect loops pet the WDT themselves so a missing
// router doesn't cause a reboot storm.
static const uint32_t WATCHDOG_TIMEOUT_MS = 30000;

// ----- Energy accumulation (Wh) -----
static const uint32_t ENERGY_SAVE_INTERVAL_MS = 300000;
static const char*    NVS_NAMESPACE           = "lighttrack";
static const char*    NVS_KEY_ENERGY_WH       = "energyWh";
static const char*    NVS_KEY_ENERGY_DAY      = "energyDay";
static const char*    NVS_KEY_CHARGED_WH      = "chargedWh";

// ----- Battery monitoring (second INA219) -----
static const uint8_t BATTERY_INA_ADDRESS              = 0x41;
static const uint8_t BATTERY_CELLS                    = 1;
static const float   BATTERY_CURRENT_DEADBAND_MA      = 30.0f;
static const bool    BATTERY_CHARGE_IS_POSITIVE_CURRENT = true;
static const uint8_t BATTERY_FULL_PERCENT             = 98;

struct BatteryPoint { float cellVoltage; uint8_t percent; };
static const BatteryPoint BATTERY_CURVE[] = {
  {4.20f, 100}, {4.06f, 90}, {3.98f, 80}, {3.92f, 70}, {3.87f, 60},
  {3.82f,  50}, {3.79f, 40}, {3.77f, 30}, {3.74f, 20}, {3.68f, 10},
  {3.45f,   5}, {3.00f,  0}
};
static const size_t BATTERY_CURVE_LEN = sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]);

// ----- Location + time (for sunrise/sunset) -----
static const double LOCATION_LAT = 45.75;   // Timisoara
static const double LOCATION_LON = 21.23;
static const char*  TZ_INFO = "EET-2EEST,M3.5.0/3,M10.5.0/4";
static const char*  NTP_SERVER = "pool.ntp.org";

// Local PI - some toolchains do not expose M_PI by default.
static constexpr double PI_D = 3.14159265358979323846;

// =========================== Tracking mode ==================================
enum class TrackingMode : uint8_t { AUTO, MANUAL, IDLE, NIGHT, ERROR };

static const char* trackingModeStr(TrackingMode m) {
  switch (m) {
    case TrackingMode::AUTO:   return "AUTO";
    case TrackingMode::MANUAL: return "MANUAL";
    case TrackingMode::IDLE:   return "IDLE";
    case TrackingMode::NIGHT:  return "NIGHT";
    case TrackingMode::ERROR:  return "ERROR";
  }
  return "ERROR";
}

static bool parseTrackingMode(const char* s, TrackingMode& out) {
  if (!s) return false;
  if (strcmp(s, "AUTO")   == 0) { out = TrackingMode::AUTO;   return true; }
  if (strcmp(s, "MANUAL") == 0) { out = TrackingMode::MANUAL; return true; }
  if (strcmp(s, "IDLE")   == 0) { out = TrackingMode::IDLE;   return true; }
  if (strcmp(s, "NIGHT")  == 0) { out = TrackingMode::NIGHT;  return true; }
  return false;
}

// =========================== Globals ========================================

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

Servo servoAzimuth;
Servo servoElevation;

Adafruit_INA219 inaPanel(0x40);
Adafruit_INA219 inaBattery(BATTERY_INA_ADDRESS);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE);

SunSet sun;

Preferences prefs;

static bool inaOk        = false;
static bool inaBatteryOk = false;
static bool oledOk       = false;
static bool timeValid    = false;

struct PanelState {
  uint8_t      horizontalAngle;
  uint8_t      verticalAngle;
  bool         isMoving;
  TrackingMode mode;
};
static PanelState state = { H_HOME, V_HOME, false, TrackingMode::IDLE };

struct SensorSnapshot {
  int   ldrTL, ldrTR, ldrBL, ldrBR;
  int   hDiff, vDiff;
  float solarV;
  float solarMa;
  float solarMw;
};
static SensorSnapshot sensors = {0,0,0,0,0,0,0,0,0};

struct BatteryReading {
  float       voltage;
  float       current;
  float       power;
  int         percent;
  const char* status;
};
static BatteryReading battery = {0.0f, 0.0f, 0.0f, 0, "UNKNOWN"};

static uint32_t lastTelemetryMs = 0;
static uint32_t lastSampleMs    = 0;
static uint32_t lastTrackMs     = 0;
static bool     telemetryNowRequested = false;

static bool     scanRequested        = false;
static uint16_t lostSunTicks         = 0;

// Anti-spam edge detectors for the OLED limit warning.
static bool     wasAtHLimit          = false;
static bool     wasAtVLimit          = false;

static TrackingMode modeBeforeNight = TrackingMode::IDLE;
static bool         isNight         = false;
static uint32_t     lowLightSinceMs = 0;
static uint32_t     lightOkSinceMs  = 0;
static uint32_t     manualOverrideUntilMs = 0;

static float    energyTodayWh    = 0.0f;
static float    chargedTodayWh   = 0.0f;
static uint32_t lastEnergyMs     = 0;
static int      energyDay        = -1;
static uint32_t lastEnergySaveMs = 0;

// OLED limit warning state
static uint32_t limitWarningUntilMs = 0;
static char     limitWarningText[24] = "";

// =========================== Forward declarations ===========================
static void moveTo(uint8_t targetH, uint8_t targetV);
static void preAimAtSunrise();
static bool isAstronomicalNight();

// =========================== Helpers ========================================

static uint8_t clampAngle(int value, uint8_t lo, uint8_t hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return static_cast<uint8_t>(value);
}

static uint8_t proportionalStep(int diff) {
  const int magnitude = abs(diff);
  if (magnitude <= LDR_DEADBAND) return 0;
  int step = 1 + (magnitude - LDR_DEADBAND) / PROP_STEP_DIVISOR;
  if (step > MAX_TRACK_STEP) step = MAX_TRACK_STEP;
  return static_cast<uint8_t>(step);
}

static int readLdrAveraged(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < LDR_SAMPLES; i++) sum += analogRead(pin);
  return static_cast<int>(sum / LDR_SAMPLES);
}

static void triggerLimitWarning(const char* text) {
  strncpy(limitWarningText, text, sizeof(limitWarningText) - 1);
  limitWarningText[sizeof(limitWarningText) - 1] = '\0';
  limitWarningUntilMs = millis() + LIMIT_WARNING_MS;
}

static void checkAxisLimits() {
  const bool atHLimit = (state.horizontalAngle == H_MIN || state.horizontalAngle == H_MAX);
  if (atHLimit && !wasAtHLimit) {
    triggerLimitWarning(state.horizontalAngle == H_MAX ? "H MAX (175)" : "H MIN (5)");
  }
  wasAtHLimit = atHLimit;

  const bool atVLimit = (state.verticalAngle == V_MIN || state.verticalAngle == V_MAX);
  if (atVLimit && !wasAtVLimit) {
    triggerLimitWarning(state.verticalAngle == V_MAX ? "V MAX (175)" : "V MIN (5)");
  }
  wasAtVLimit = atVLimit;
}

// ----- Battery -----

static int batteryPercentFromVoltage(float packVoltage) {
  const float cellV = packVoltage / BATTERY_CELLS;
  if (cellV >= BATTERY_CURVE[0].cellVoltage) return 100;
  if (cellV <= BATTERY_CURVE[BATTERY_CURVE_LEN - 1].cellVoltage) return 0;
  for (size_t i = 1; i < BATTERY_CURVE_LEN; i++) {
    if (cellV >= BATTERY_CURVE[i].cellVoltage) {
      const BatteryPoint& hi = BATTERY_CURVE[i - 1];
      const BatteryPoint& lo = BATTERY_CURVE[i];
      const float t = (cellV - lo.cellVoltage) / (hi.cellVoltage - lo.cellVoltage);
      return static_cast<int>(lo.percent + t * (hi.percent - lo.percent) + 0.5f);
    }
  }
  return 0;
}

static const char* batteryStatusFrom(float chargeCurrentMa, int percent) {
  if (chargeCurrentMa >  BATTERY_CURRENT_DEADBAND_MA) return "CHARGING";
  if (chargeCurrentMa < -BATTERY_CURRENT_DEADBAND_MA) return "DISCHARGING";
  if (percent >= BATTERY_FULL_PERCENT) return "FULL";
  return "IDLE";
}

static void readBattery() {
  if (!inaBatteryOk) {
    battery.voltage = 0.0f;
    battery.current = 0.0f;
    battery.power   = 0.0f;
    battery.percent = 0;
    battery.status  = "UNKNOWN";
    return;
  }

  const float busV   = inaBattery.getBusVoltage_V();
  const float shuntV = inaBattery.getShuntVoltage_mV() / 1000.0f;
  float currentMa    = inaBattery.getCurrent_mA();
  if (!BATTERY_CHARGE_IS_POSITIVE_CURRENT) currentMa = -currentMa;

  battery.voltage = busV + shuntV;
  battery.current = currentMa / 1000.0f;
  battery.power   = battery.voltage * battery.current;
  battery.percent = batteryPercentFromVoltage(battery.voltage);
  battery.status  = batteryStatusFrom(currentMa, battery.percent);
}

static void sampleSensors() {
  sensors.ldrTL = readLdrAveraged(PIN_LDR_TL);
  sensors.ldrTR = readLdrAveraged(PIN_LDR_TR);
  sensors.ldrBL = readLdrAveraged(PIN_LDR_BL);
  sensors.ldrBR = readLdrAveraged(PIN_LDR_BR);

  sensors.hDiff = (sensors.ldrTL + sensors.ldrBL) - (sensors.ldrTR + sensors.ldrBR);
  sensors.vDiff = (sensors.ldrTL + sensors.ldrTR) - (sensors.ldrBL + sensors.ldrBR);

  if (inaOk) {
    sensors.solarV  = inaPanel.getBusVoltage_V() +
                      (inaPanel.getShuntVoltage_mV() / 1000.0f);
    sensors.solarMa = inaPanel.getCurrent_mA();
    sensors.solarMw = inaPanel.getPower_mW();
  } else {
    sensors.solarV = sensors.solarMa = sensors.solarMw = 0.0f;
  }

  readBattery();
}

static int averageLight() {
  return (sensors.ldrTL + sensors.ldrTR + sensors.ldrBL + sensors.ldrBR) / 4;
}

static bool isLowLightMajority() {
  uint8_t belowCount = 0;
  if (sensors.ldrTL < LOW_LIGHT_THRESHOLD) belowCount++;
  if (sensors.ldrTR < LOW_LIGHT_THRESHOLD) belowCount++;
  if (sensors.ldrBL < LOW_LIGHT_THRESHOLD) belowCount++;
  if (sensors.ldrBR < LOW_LIGHT_THRESHOLD) belowCount++;
  return belowCount >= 3;
}

static void markManualActivity() {
  manualOverrideUntilMs = millis() + MANUAL_OVERRIDE_MS;
}

static bool isAstronomicalNight() {
  if (!timeValid) return false;

  time_t nowSec = time(nullptr);
  struct tm lt;
  localtime_r(&nowSec, &lt);

  sun.setTZOffset(lt.tm_isdst > 0 ? 3 : 2);
  sun.setCurrentDate(lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);

  const double sunriseMin = sun.calcSunrise();
  const double sunsetMin  = sun.calcSunset();

  const double nowMin = lt.tm_hour * 60.0 + lt.tm_min + lt.tm_sec / 60.0;
  return (nowMin < sunriseMin) || (nowMin > sunsetMin);
}

// =========================== Sunrise azimuth math ===========================

static double sunriseAzimuthDeg() {
  if (!timeValid) return 90.0;

  time_t nowSec = time(nullptr);
  struct tm lt;
  localtime_r(&nowSec, &lt);
  const int doy = lt.tm_yday + 1;  // tm_yday is 0..365; formula uses 1..365

  const double decl = 0.4093 * sin(2.0 * PI_D * (doy - 81) / 365.0);
  const double latRad = LOCATION_LAT * PI_D / 180.0;

  double cosA = sin(decl) / cos(latRad);
  if (cosA >  1.0) cosA =  1.0;
  if (cosA < -1.0) cosA = -1.0;

  return acos(cosA) * 180.0 / PI_D;  // 0..180, measured from North
}

static uint8_t cardinalToServoH(double cardinalDeg) {
  const int servo = static_cast<int>(cardinalDeg + 0.5) - 90;
  return clampAngle(servo, H_MIN, H_MAX);
}

static void preAimAtSunrise() {
  const double azNorth = sunriseAzimuthDeg();
  const uint8_t targetH = cardinalToServoH(azNorth);
  const uint8_t targetV = V_SUNRISE;
  Serial.printf("[sunrise] pre-aim H=%u V=%u (azimuth %.1f° from N)\n",
                targetH, targetV, azNorth);
  moveTo(targetH, targetV);
}

// =========================== Energy accumulation ============================

static void loadEnergyFromNvs() {
  prefs.begin(NVS_NAMESPACE, false);
  energyTodayWh  = prefs.getFloat(NVS_KEY_ENERGY_WH, 0.0f);
  chargedTodayWh = prefs.getFloat(NVS_KEY_CHARGED_WH, 0.0f);
  energyDay      = prefs.getInt(NVS_KEY_ENERGY_DAY, -1);
  prefs.end();
  Serial.printf("[energy] loaded gen=%.3f charged=%.3f Wh (day=%d)\n",
                energyTodayWh, chargedTodayWh, energyDay);
}

static void saveEnergyToNvs() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putFloat(NVS_KEY_ENERGY_WH, energyTodayWh);
  prefs.putFloat(NVS_KEY_CHARGED_WH, chargedTodayWh);
  prefs.putInt(NVS_KEY_ENERGY_DAY, energyDay);
  prefs.end();
}

static void updateEnergy() {
  const uint32_t now = millis();

  if (timeValid) {
    time_t nowSec = time(nullptr);
    struct tm lt; localtime_r(&nowSec, &lt);
    if (energyDay == -1) {
      energyDay = lt.tm_mday;
      saveEnergyToNvs();
    } else if (lt.tm_mday != energyDay) {
      energyTodayWh  = 0.0f;
      chargedTodayWh = 0.0f;
      energyDay = lt.tm_mday;
      saveEnergyToNvs();
    }
  }

  if (lastEnergyMs != 0) {
    const float elapsedHours = (now - lastEnergyMs) / 3600000.0f;
    if (inaOk) {
      const float solarW = sensors.solarMw / 1000.0f;
      if (solarW > 0.0f) energyTodayWh += solarW * elapsedHours;
    }
    if (inaBatteryOk && battery.power > 0.0f) {
      chargedTodayWh += battery.power * elapsedHours;
    }
  }
  lastEnergyMs = now;

  if (now - lastEnergySaveMs >= ENERGY_SAVE_INTERVAL_MS) {
    lastEnergySaveMs = now;
    saveEnergyToNvs();
  }
}

// =========================== Coarse scan ====================================

static long readTotalLight() {
  long total = 0;
  for (uint8_t i = 0; i < SCAN_LDR_SAMPLES; i++) {
    total += analogRead(PIN_LDR_TL);
    total += analogRead(PIN_LDR_TR);
    total += analogRead(PIN_LDR_BL);
    total += analogRead(PIN_LDR_BR);
  }
  return total / SCAN_LDR_SAMPLES;
}

static void performScan() {
  Serial.println("[scan] start");

  uint8_t bestH = state.horizontalAngle;
  long    bestHLight = -1;
  for (int h = H_MIN; h <= H_MAX; h += SCAN_STEP) {
    servoAzimuth.write(h);
    delay(SCAN_SETTLE_MS);
    esp_task_wdt_reset();              // pet WDT during the blocking sweep
    const long light = readTotalLight();
    if (light > bestHLight) { bestHLight = light; bestH = h; }
  }
  servoAzimuth.write(bestH);
  state.horizontalAngle = bestH;
  delay(SCAN_SETTLE_MS);
  esp_task_wdt_reset();

  uint8_t bestV = state.verticalAngle;
  long    bestVLight = -1;
  for (int v = V_MIN; v <= V_MAX; v += SCAN_STEP) {
    servoElevation.write(v);
    delay(SCAN_SETTLE_MS);
    esp_task_wdt_reset();              // pet WDT during the blocking sweep
    const long light = readTotalLight();
    if (light > bestVLight) { bestVLight = light; bestV = v; }
  }
  servoElevation.write(bestV);
  state.verticalAngle = bestV;
  delay(SCAN_SETTLE_MS);
  esp_task_wdt_reset();

  Serial.print("[scan] best H="); Serial.print(bestH);
  Serial.print(" V="); Serial.println(bestV);
}

// =========================== Low-light state machine ========================

static void updateLowLight() {
  if (state.mode == TrackingMode::ERROR) return;   // [ERROR] sticky until reboot

  const uint32_t now = millis();
  const bool lowLight = isLowLightMajority();
  const bool overrideActive = (now < manualOverrideUntilMs);

  if (isNight) {
    if (!lowLight) {
      if (lightOkSinceMs == 0) lightOkSinceMs = now;
      if (now - lightOkSinceMs >= LIGHT_RESTORE_MS) {
        // Morning wake: restore the mode that was active before night.
        isNight = false;
        state.mode = modeBeforeNight;
        scanRequested = false;
        lostSunTicks = 0;
        lightOkSinceMs = 0;
        lowLightSinceMs = 0;
        if (modeBeforeNight == TrackingMode::AUTO) {
          preAimAtSunrise();
        }
      }
    } else {
      lightOkSinceMs = 0;
    }
    if (overrideActive) isNight = false;
    return;
  }

  if (!lowLight) {
    lowLightSinceMs = 0;
    return;
  }
  if (lowLightSinceMs == 0) lowLightSinceMs = now;

  if (now - lowLightSinceMs < LOW_LIGHT_CONFIRM_MS) return;

  if (isAstronomicalNight()) {
    if (!overrideActive) {
      modeBeforeNight = state.mode;
      isNight = true;
      state.mode = TrackingMode::NIGHT;
      moveTo(H_HOME, V_HOME);
      lightOkSinceMs = 0;
    }
  } else {
    if (!overrideActive &&
        state.mode != TrackingMode::IDLE &&
        (now - lowLightSinceMs >= DARK_TO_IDLE_MS)) {
      state.mode = TrackingMode::IDLE;
    }
  }
}

// =========================== AUTO tracking ==================================

static void trackStep() {
  if (state.mode != TrackingMode::AUTO) return;

  if (scanRequested) {
    if (averageLight() >= DARK_THRESHOLD) {
      scanRequested = false;
      state.isMoving = true;
      performScan();
      state.isMoving = false;
      lastTrackMs = millis();
      lostSunTicks = 0;
      checkAxisLimits();
    }
    return;
  }

  const uint32_t now = millis();
  if (now - lastTrackMs < TRACK_INTERVAL_MS) return;
  lastTrackMs = now;

  if (averageLight() < DARK_THRESHOLD) {
    if (!isAstronomicalNight() && ++lostSunTicks >= LOST_SUN_TICKS) {
      lostSunTicks = 0;
      scanRequested = true;
    }
    state.isMoving = false;
    return;
  }
  lostSunTicks = 0;

  uint8_t newH = state.horizontalAngle;
  uint8_t newV = state.verticalAngle;

  const uint8_t stepH = proportionalStep(sensors.hDiff);
  if (stepH > 0) {
    const int wantH = (sensors.hDiff > 0)
      ? static_cast<int>(state.horizontalAngle) + stepH
      : static_cast<int>(state.horizontalAngle) - stepH;
    newH = clampAngle(wantH, H_MIN, H_MAX);
  }

  const uint8_t stepV = proportionalStep(sensors.vDiff);
  if (stepV > 0) {
    const int wantV = (sensors.vDiff > 0)
      ? static_cast<int>(state.verticalAngle) + stepV
      : static_cast<int>(state.verticalAngle) - stepV;
    newV = clampAngle(wantV, V_MIN, V_MAX);
  }

  const bool moved = (newH != state.horizontalAngle) || (newV != state.verticalAngle);
  if (newH != state.horizontalAngle) { state.horizontalAngle = newH; servoAzimuth.write(newH); }
  if (newV != state.verticalAngle)   { state.verticalAngle   = newV; servoElevation.write(newV); }
  state.isMoving = moved;

  checkAxisLimits();
}

// =========================== OLED ===========================================

static void oledShow() {
  if (!oledOk) return;
  oled.clearBuffer();

  if (millis() < limitWarningUntilMs) {
    oled.setFont(u8g2_font_ncenB10_tr);
    oled.drawStr(8, 13, "!! WARNING !!");

    oled.setFont(u8g2_font_ncenB12_tr);
    oled.drawStr(4, 40, limitWarningText);
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(4, 60, "exceeded limit");

    oled.sendBuffer();
    return;
  }

  oled.setFont(u8g2_font_ncenB12_tr);
  oled.drawStr(22, 13, "-- PANEL --");

  oled.setFont(u8g2_font_ncenB10_tr);
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "V: %.2f V",  sensors.solarV);
  oled.drawStr(0, 32, buffer);
  snprintf(buffer, sizeof(buffer), "I: %.1f mA", sensors.solarMa);
  oled.drawStr(0, 48, buffer);
  snprintf(buffer, sizeof(buffer), "P: %.1f mW", sensors.solarMw);
  oled.drawStr(0, 62, buffer);

  oled.sendBuffer();
}

// =========================== Movement (command-driven) ======================

static void moveTo(uint8_t targetH, uint8_t targetV) {
  targetH = clampAngle(targetH, H_MIN, H_MAX);
  targetV = clampAngle(targetV, V_MIN, V_MAX);

  if (targetH == state.horizontalAngle && targetV == state.verticalAngle) {
    state.isMoving = false;
    checkAxisLimits();
    return;
  }

  state.isMoving = true;

  while (state.horizontalAngle != targetH || state.verticalAngle != targetV) {
    if (state.horizontalAngle != targetH) {
      state.horizontalAngle += (targetH > state.horizontalAngle) ? 1 : -1;
      servoAzimuth.write(state.horizontalAngle);
    }
    if (state.verticalAngle != targetV) {
      state.verticalAngle += (targetV > state.verticalAngle) ? 1 : -1;
      servoElevation.write(state.verticalAngle);
    }
    delay(MIN_STEP_DELAY_MS);
    esp_task_wdt_reset();   // pet WDT during long manual sweeps
  }

  state.isMoving = false;
  checkAxisLimits();
}

// =========================== ACK publishing =================================

static void publishAck(const char* commandId,
                       const char* status,
                       const char* message = nullptr) {
  StaticJsonDocument<256> doc;
  doc["status"]    = status;
  if (message) doc["message"] = message;
  doc["current_h_angle"]  = state.horizontalAngle;
  doc["current_v_angle"]  = state.verticalAngle;
  doc["tracking_mode"]    = trackingModeStr(state.mode);
  doc["commandId"] = commandId;
  doc["device_id"] = MQTT_CLIENT_ID;

  char buf[256];
  const size_t len = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_ACK, reinterpret_cast<uint8_t*>(buf), len, false);
}

// =========================== Command dispatch ===============================

static bool handleMovePanel(const char* commandId, JsonVariantConst payload) {
  if (!payload["h_angle"].is<int>() || !payload["v_angle"].is<int>()) {
    publishAck(commandId, "FAILED", "missing h_angle or v_angle");
    return false;
  }
  const int h = payload["h_angle"].as<int>();
  const int v = payload["v_angle"].as<int>();

  if (h < H_MIN || h > H_MAX || v < V_MIN || v > V_MAX) {
    if (h < H_MIN || h > H_MAX) {
      triggerLimitWarning("H OUT OF RANGE");
    } else {
      triggerLimitWarning("V OUT OF RANGE");
    }
    publishAck(commandId, "FAILED", "angle out of safe range");
    return false;
  }

  markManualActivity();
  state.mode = TrackingMode::MANUAL;
  isNight = false;
  moveTo(static_cast<uint8_t>(h), static_cast<uint8_t>(v));
  publishAck(commandId, "ACKNOWLEDGED");
  return true;
}

static bool handleSetMode(const char* commandId, JsonVariantConst payload) {
  const char* modeStr = payload["mode"] | static_cast<const char*>(nullptr);
  TrackingMode mode;
  if (!parseTrackingMode(modeStr, mode)) {
    publishAck(commandId, "FAILED", "invalid mode");
    return false;
  }
  if (mode == TrackingMode::NIGHT) {
    publishAck(commandId, "FAILED", "NIGHT is firmware-only");
    return false;
  }
  const TrackingMode prevMode = state.mode;
  markManualActivity();
  state.mode = mode;
  isNight = false;
  if (mode == TrackingMode::AUTO && prevMode != TrackingMode::AUTO) {
    scanRequested = true;
  }
  publishAck(commandId, "ACKNOWLEDGED");
  return true;
}

static bool handleResetPosition(const char* commandId) {
  markManualActivity();
  state.mode = TrackingMode::IDLE;
  isNight = false;
  moveTo(H_HOME, V_HOME);
  publishAck(commandId, "ACKNOWLEDGED");
  return true;
}

static bool handleRequestStatus(const char* commandId) {
  telemetryNowRequested = true;
  publishAck(commandId, "ACKNOWLEDGED");
  return true;
}

static bool handleStartTracking(const char* commandId) {
  const TrackingMode prevMode = state.mode;
  markManualActivity();
  state.mode = TrackingMode::AUTO;
  isNight = false;
  if (prevMode != TrackingMode::AUTO) {
    scanRequested = true;
  }
  publishAck(commandId, "ACKNOWLEDGED");
  return true;
}

static bool handleStopTracking(const char* commandId) {
  markManualActivity();
  state.mode = TrackingMode::IDLE;
  isNight = false;
  publishAck(commandId, "ACKNOWLEDGED");
  return true;
}

static void handleCommand(const JsonDocument& doc) {
  const char* commandId   = doc["commandId"]   | "";
  const char* commandType = doc["command_type"] | "";

  if (commandId[0] == '\0') {
    Serial.println("[cmd] missing commandId, dropping");
    return;
  }
  if (commandType[0] == '\0') {
    publishAck(commandId, "FAILED", "missing command_type");
    return;
  }

  // [ERROR] In ERROR the device refuses everything except a status query.
  if (state.mode == TrackingMode::ERROR &&
      strcmp(commandType, "REQUEST_STATUS") != 0) {
    publishAck(commandId, "FAILED", "device in ERROR (I2C unavailable)");
    return;
  }

  JsonVariantConst payload = doc["payload"];

  if (strcmp(commandType, "MOVE_PANEL") == 0) {
    handleMovePanel(commandId, payload);
  } else if (strcmp(commandType, "SET_MODE") == 0) {
    handleSetMode(commandId, payload);
  } else if (strcmp(commandType, "RESET_POSITION") == 0) {
    handleResetPosition(commandId);
  } else if (strcmp(commandType, "REQUEST_STATUS") == 0) {
    handleRequestStatus(commandId);
  } else if (strcmp(commandType, "START_TRACKING") == 0) {
    handleStartTracking(commandId);
  } else if (strcmp(commandType, "STOP_TRACKING") == 0) {
    handleStopTracking(commandId);
  } else {
    publishAck(commandId, "FAILED", "unsupported command_type");
  }
}

// =========================== MQTT callback ==================================

static void onMqttMessage(char* topic, uint8_t* payload, unsigned int len) {
  if (strcmp(topic, TOPIC_COMMANDS) != 0) return;

  StaticJsonDocument<384> doc;
  const DeserializationError err = deserializeJson(doc, payload, len);
  if (err) {
    Serial.print("[mqtt] invalid JSON: ");
    Serial.println(err.c_str());
    return;
  }
  handleCommand(doc);
}

// =========================== Telemetry ======================================

static void publishTelemetry() {
  StaticJsonDocument<512> doc;
  doc["horizontal_angle"] = state.horizontalAngle;
  doc["vertical_angle"]   = state.verticalAngle;
  doc["tracking_mode"]    = trackingModeStr(state.mode);
  doc["is_moving"]        = state.isMoving;

  doc["ldr_top_left"]     = sensors.ldrTL;
  doc["ldr_top_right"]    = sensors.ldrTR;
  doc["ldr_bottom_left"]  = sensors.ldrBL;
  doc["ldr_bottom_right"] = sensors.ldrBR;
  doc["horizontal_light_difference"] = sensors.hDiff;
  doc["vertical_light_difference"]   = sensors.vDiff;

  doc["device_id"] = MQTT_CLIENT_ID;

  if (inaOk) {
    doc["solar_voltage"]         = sensors.solarV;
    doc["solar_current"]         = sensors.solarMa / 1000.0f;
    doc["solar_power"]           = sensors.solarMw / 1000.0f;
    doc["solar_energy_today_wh"] = energyTodayWh;
  }

  if (inaBatteryOk) {
    doc["battery_voltage"]         = battery.voltage;
    doc["battery_percent"]         = battery.percent;
    doc["battery_status"]          = battery.status;
    doc["charged_energy_today_wh"] = chargedTodayWh;
    if (battery.power > 0.0f) {
      doc["charging_voltage"] = battery.voltage;
      doc["charging_current"] = battery.current;
      doc["charging_power"]   = battery.power;
    }
  }

  char buf[512];
  const size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_TELEMETRY, reinterpret_cast<uint8_t*>(buf), n, false);
}

// =========================== Connection / time ==============================

static void connectWifi() {
  Serial.print("[wifi] connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    esp_task_wdt_reset();  // avoid WDT trip on slow router boot
    delay(500);
    Serial.print(".");
  }
  Serial.print("\n[wifi] connected, ip=");
  Serial.println(WiFi.localIP());
}

static void startTimeSync() {
  configTzTime(TZ_INFO, NTP_SERVER);
}

static void refreshTimeValid() {
  if (timeValid) return;
  time_t nowSec = time(nullptr);
  if (nowSec > 1600000000) {
    timeValid = true;
    struct tm lt; localtime_r(&nowSec, &lt);
    Serial.printf("[time] synced: %04d-%02d-%02d %02d:%02d (dst=%d)\n",
                  lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                  lt.tm_hour, lt.tm_min, lt.tm_isdst);
  }
}

static void ensureMqtt() {
  while (!mqtt.connected()) {
    esp_task_wdt_reset();  // avoid WDT trip while broker is offline
    Serial.print("[mqtt] connecting to ");
    Serial.print(MQTT_HOST); Serial.print(":"); Serial.println(MQTT_PORT);
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("[mqtt] connected");
      mqtt.subscribe(TOPIC_COMMANDS, 1);
    } else {
      Serial.print("[mqtt] failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(", retrying in 2s");
      delay(2000);
    }
  }
}

// =========================== Arduino entry points ===========================

void setup() {
  Serial.begin(115200);
  delay(300);

  // Report why we last reset (BROWNOUT vs WATCHDOG vs PANIC) - useful for
  // field debugging power vs firmware issues.
  esp_reset_reason_t rr = esp_reset_reason();
  Serial.printf("[boot] reset reason = %s\n",
    rr == ESP_RST_BROWNOUT ? "BROWNOUT" :
    (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT) ? "WATCHDOG" :
    rr == ESP_RST_PANIC ? "PANIC/CRASH" : "other");

  // Hardware watchdog: the Arduino core already starts a Task WDT on loopTask,
  // so esp_task_wdt_init() would fail with "already initialized". We reconfigure
  // it to our timeout instead. Long blocking ops (performScan, moveTo, WiFi/MQTT
  // reconnect) pet the WDT themselves so they don't trip it.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = WATCHDOG_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic  = true,
  };
  esp_task_wdt_reconfigure(&wdt_config);  // apply our 30s window to the core's TWDT
  esp_task_wdt_add(NULL);                 // ensure loopTask is watched (no-op if already)

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  for (uint8_t i = 0; i < 3 && !oledOk; i++) {
    if (oled.begin()) { oledOk = true; break; }
    delay(200);
  }
  if (oledOk) {
    oled.setContrast(255);
    oled.clearBuffer();
    oled.setFont(u8g2_font_ncenB10_tr);
    oled.drawStr(0, 24, "LightTrack");
    oled.drawStr(0, 48, "Starting...");
    oled.sendBuffer();
  } else {
    Serial.println("[oled] init failed");
  }

  if (inaPanel.begin()) {
    // Panel is small (~1-5W, max ~300mA). 16V/400mA gives ~10 uA per LSB ->
    // ~10x cleaner readings at low currents than the default 32V/2A, while
    // still covering the panel's full Voc and Isc.
    inaPanel.setCalibration_16V_400mA();
    inaOk = true;
    Serial.println("[ina] panel OK (16V/400mA)");
  } else {
    Serial.println("[ina] panel FAIL");
  }

  if (inaBattery.begin()) {
    // Battery rail sees servo stall currents, so keep the default 32V/2A
    // calibration to avoid clipping during transients.
    inaBatteryOk = true;
    Serial.printf("[ina] battery OK (0x%02X, 32V/2A)\n", BATTERY_INA_ADDRESS);
  } else {
    Serial.printf("[ina] battery FAIL (no device at 0x%02X)\n", BATTERY_INA_ADDRESS);
  }

  // [ERROR] Catastrophic I2C bus failure: if NOT A SINGLE I2C device answered
  // (both INA219 and the OLED), the bus is dead/unwired - nothing to measure,
  // nothing to show. Enter the reserved ERROR state and refuse to run
  // unattended. A SINGLE missing device is handled by graceful degradation
  // (its fields are simply omitted from telemetry), not by ERROR.
  if (!inaOk && !inaBatteryOk && !oledOk) {
    state.mode = TrackingMode::ERROR;
    Serial.println("[boot] I2C bus dead (no devices) -> ERROR");
  }

  loadEnergyFromNvs();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoAzimuth.setPeriodHertz(50);
  servoElevation.setPeriodHertz(50);
  servoAzimuth.attach(PIN_SERVO_AZIMUTH,     500, 2400);
  servoElevation.attach(PIN_SERVO_ELEVATION, 500, 2400);

  servoAzimuth.write(H_HOME);
  servoElevation.write(V_HOME);
  delay(500);

  sun.setPosition(LOCATION_LAT, LOCATION_LON, 2);

  connectWifi();
  startTimeSync();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(768);
  mqtt.setCallback(onMqttMessage);
  ensureMqtt();
}

void loop() {
  esp_task_wdt_reset();  // pet the watchdog: "still alive"

  if (WiFi.status() != WL_CONNECTED) connectWifi();
  if (!mqtt.connected())            ensureMqtt();
  mqtt.loop();

  refreshTimeValid();

  const uint32_t now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    sampleSensors();
    updateEnergy();
    updateLowLight();
    oledShow();
  }

  trackStep();

  const uint32_t telemetryInterval = isNight ? TELEMETRY_INTERVAL_NIGHT_MS
                                             : TELEMETRY_INTERVAL_MS;
  const bool intervalElapsed = (now - lastTelemetryMs) >= telemetryInterval;
  if (intervalElapsed || telemetryNowRequested) {
    lastTelemetryMs = now;
    telemetryNowRequested = false;
    publishTelemetry();
  }
}