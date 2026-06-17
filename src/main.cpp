/*
  ┌─────────────────────────────────────────────────────────────┐
  │  CAT TRACKER TX (ESP32S3 + FreeRTOS)                       │
  │  SX1262 LoRa + TinyGPSPlus + BLE "Home" beacon             │
  │  Behaviour: wake → GPS/ BLE → build JSON → LoRa TX → sleep │
  │  Compact JSON: src/dst/seq/name/status/GPS Unix time       │
  └─────────────────────────────────────────────────────────────┘

  High‑level RTOS design
  ───────────────────────
  • TaskGPS  : reads UART1, feeds TinyGPSPlus, publishes latest fix.
  • TaskBLE  : short active scan window; sets Event bit when beacon "Home" seen.
  • TaskLoRa : sole owner of RadioLib; sends payloads queued by the Power task; handles RX later.
  • TaskPower: orchestrates one acquisition/decision cycle then enters sleep.

  Sleep policy
  ────────────
  • Uses deep sleep with profile timer or GPIO21 user-button wake.
  • All state that must persist across deep sleep uses RTC_DATA_ATTR.

  NOTE: Keep RadioLib access in a single task (TaskLoRa). Do not use RadioLib
  from ISRs. If you later need LoRa RX commands, push them into a queue.
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <driver/rtc_io.h>
#include "config.h" // Operating modes and shared configuration

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "unknown"
#endif
#ifndef BUILD_TIMESTAMP
#define BUILD_TIMESTAMP "unknown"
#endif

// ─────────────────────────────────────────────
// Device Identity (per-node configuration)
// ─────────────────────────────────────────────
// Device ID is derived from the ESP32's factory-burned MAC address on boot,
// producing a stable 3-digit number (100–998) unique to each physical chip.
// No manual programming needed — flash the same firmware on any collar and
// it gets its own ID automatically.
// The human-friendly NAME ("Podge", "Macy", etc.) is a runtime value
// (g_senderName) stored in NVS and changed at any time via an OTAP command
// from the receiver. Default if NVS is empty: "Device-<id>".
//
// OTAP reserved device IDs (plain integers on the wire, shared with the base):
#define BASE_ID 1        // the base station / receiver
#define BROADCAST_ID 999 // a command addressed here is acted on by every collar
static uint16_t DEVICE_ID_INT = 0;

static uint8_t g_mac[6];

static void initDeviceId()
{
  esp_efuse_mac_get_default(g_mac);
  uint32_t hash = g_mac[0];
  for (int i = 1; i < 6; i++)
    hash = hash * 31 + g_mac[i];
  // 100–998: 999 is reserved for broadcast, so the hash is mod 899 (never 999).
  DEVICE_ID_INT = 100 + (hash % 899);
}

// SENDER_NAME_MAX_LEN is defined in name_store.h. The friendly name is still
// part of telemetry; it just can no longer be changed over the air (V3.8.0
// removed all remote commands — the collar is transmit-only).
#include "name_store.h"
static char g_senderName[SENDER_NAME_MAX_LEN + 1] = {0};

// V3.6.9: MASTER CONFIG persistence (the simple, reliable model).
//
// Every persisted collar setting (currently just the friendly name) is a field
// of ONE JSON blob — the master config — written to NVS as a single string
// under a FRESH namespace/key. Why this shape, after renames kept failing to
// stick in the field:
//
//   * msg_id (an integer key) survived deep sleep fine, but the name (a string
//     under key "name" in the old "cattracker" namespace) never did. That
//     one-key-works / other-key-fails split is the classic signature of a
//     stale, WRONG-TYPE "name" entry: once an NVS key exists with a non-string
//     type, nvs_set_str refuses it forever and putString silently returns 0.
//   * A BRAND-NEW namespace + key that has never been written sidesteps that
//     poisoned entry completely, and a single JSON blob means there is only
//     ONE key, of ONE type, to ever manage.
//   * Every write is read back and verified, so a genuine failure is reported
//     instead of vanishing.
//
// name_store talks to the generic INvs get/put-string interface; on the device
// each "key" maps to a field of the master-config blob.
#define CFG_NS "bpcfg"     // fresh NVS namespace — never written by old firmware
#define CFG_KEY "config"   // single master-config JSON blob lives here
struct PrefsNvs : INvs
{
  // Read the whole master-config blob into `doc`. False if absent/unparseable.
  static bool readBlob(JsonDocument &doc)
  {
    Preferences p;
    String s;
    if (p.begin(CFG_NS, true)) // read-only
    {
      s = p.getString(CFG_KEY, "");
      p.end();
    }
    if (s.length() == 0)
      return false;
    return deserializeJson(doc, s) == DeserializationError::Ok;
  }

  // Serialize `doc` and persist it as the single blob, then read it back to
  // confirm it genuinely stuck. Returns true only on a verified round-trip.
  static bool writeBlobVerified(JsonDocument &doc)
  {
    String json;
    serializeJson(doc, json);
    Preferences p;
    if (!p.begin(CFG_NS, false))
    {
      Serial.println("[CFG] begin(rw) FAILED");
      return false;
    }
    size_t n = p.putString(CFG_KEY, json);
    p.end();
    Preferences pr;
    String got;
    if (pr.begin(CFG_NS, true))
    {
      got = pr.getString(CFG_KEY, "");
      pr.end();
    }
    bool ok = (n > 0) && (got == json);
    Serial.printf("[CFG] save %s: %s\n", ok ? "VERIFIED" : "FAILED", json.c_str());
    return ok;
  }

  bool nvsGetString(const char *key, char *out, size_t outsz) override
  {
    JsonDocument doc;
    if (!readBlob(doc))
    {
      Serial.printf("[CFG] no master config yet (key '%s') - using default\n", key);
      return false;
    }
    const char *v = doc[key] | "";
    if (v[0] == '\0')
      return false;
    strncpy(out, v, outsz - 1);
    out[outsz - 1] = '\0';
    Serial.printf("[CFG] get '%s' = '%s'\n", key, out);
    return true;
  }

  bool nvsPutString(const char *key, const char *val) override
  {
    // Read-modify-write the single master-config blob: load the current config
    // (an empty doc if none exists yet), set this one field, then persist and
    // verify the whole blob. String(val) forces a copy into the doc so we never
    // serialize a dangling pointer into the inbound command buffer.
    JsonDocument doc;
    readBlob(doc);
    doc[key] = String(val ? val : "");
    return writeBlobVerified(doc);
  }
};
static PrefsNvs g_nvs;

// Load the friendly name from NVS into g_senderName (default Device-<id>).
static void loadSenderName()
{
  bpLoadSenderName(g_senderName, sizeof(g_senderName), DEVICE_ID_INT, g_nvs);
}

// Debug serial on spare pin (for battery operation monitoring)
#define DEBUG_SERIAL_ENABLED true // Set to false to disable
#define DEBUG_TX_PIN 6            // D5 - Connect to RX of USB-Serial adapter

// V3: home location lives on the receiver. The collar only sends raw lat/lon;
// the receiver computes distance/bearing on each inbound packet. This keeps
// every collar agnostic to where "home" is — change it once at the base station
// and every cat tracks against the new value with no reflash needed.

// ─────────────────────────────────────────────
// CSV Logging Configuration
// ─────────────────────────────────────────────
#define CSV_LOG_ENABLED true          // Enable CSV logging to flash
#define CSV_LOG_FILE "/track_log.csv" // Log file path in LittleFS
#define CSV_MAX_FILE_SIZE_KB 3072     // Max log file size (3MB of 4MB available)
#define CSV_LOG_HEADER "timestamp,msg_id,device,mode,status,lat,lon,dist_m,bearing,battery_v,rssi,snr"

// ─────────────────────────────────────────────
// Pin mapping (Seeeduino XIAO ESP32S3 + SX1262 B2B)
// ─────────────────────────────────────────────
#define LORA_NSS 41
#define LORA_SCK 7
#define LORA_MOSI 9
#define LORA_MISO 8
#define LORA_RST 42
#define LORA_BUSY 40
#define LORA_DIO1 39

#define GPS_RX D7
#define GPS_TX D6
#define GPS_WAKEUP D0 // L76K WAKE_UP (HIGH = awake, LOW = standby; does not switch 3V3)
#define LED_PIN 48

// ─────────────────────────────────────────────
// LoRa / GPS globals
// ─────────────────────────────────────────────
SPIClass LoRaSPI(HSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Debug serial for battery operation
#if DEBUG_SERIAL_ENABLED
HardwareSerial DebugSerial(2); // Use UART2
#define DEBUG_PRINT(x) DebugSerial.print(x)
#define DEBUG_PRINTLN(x) DebugSerial.println(x)
#define DEBUG_PRINTF(...) DebugSerial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

// ─────────────────────────────────────────────
// RTOS primitives: queues + event bits
// ─────────────────────────────────────────────
struct GpsFix
{
  double lat = 0;
  double lon = 0;
  bool valid = false;
  uint32_t unixTime = 0; // GPS UTC as Unix seconds; 0 when unavailable
};

struct TxReq
{
  char json[320];
  bool isTelemetry; // true = main telemetry packet, false = command ACK/response
};

static QueueHandle_t gpsFixQ;     // latest fix (overwrite)
static QueueHandle_t txReqQ;      // TX requests
static EventGroupHandle_t evBits; // state flags

#define EV_FIX (1 << 0)      // have recent valid GPS fix
#define EV_HOME (1 << 1)     // BLE beacon seen this cycle
#define EV_TXDONE (1 << 2)   // LoRa TX finished

// Convert a validated GPS UTC date/time to Unix seconds without relying on the
// ESP32 system clock or timezone configuration.
static uint32_t gpsToUnixTime(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second)
{
  static const uint16_t daysBeforeMonth[] =
      {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  uint32_t y = year;
  uint32_t days = (y - 1970) * 365;
  days += (y - 1969) / 4;
  days -= (y - 1901) / 100;
  days += (y - 1601) / 400;
  days += daysBeforeMonth[month - 1];
  if (month > 2 && (y % 4 == 0) && ((y % 100 != 0) || (y % 400 == 0)))
    days++;
  days += day - 1;
  return days * 86400UL + hour * 3600UL + minute * 60UL + second;
}

// Persisted counters and state across deep sleep (RTC memory - fast but cleared on reset)
RTC_DATA_ATTR uint32_t g_msgCounter = 0;
RTC_DATA_ATTR bool g_gpsWarmedUp = false;        // Tracks if GPS has achieved initial lock
RTC_DATA_ATTR uint8_t g_homeBeaconCycles = 0;    // Count consecutive cycles at home (BLE detected)
RTC_DATA_ATTR char g_currentMode[16] = "developer"; // Current operating mode name
RTC_DATA_ATTR bool g_lastGpsValid = false;        // Last valid fix retained across deep sleep
RTC_DATA_ATTR double g_lastGpsLat = 0.0;
RTC_DATA_ATTR double g_lastGpsLon = 0.0;
RTC_DATA_ATTR uint32_t g_lastGpsUnixTime = 0;
// V3 fix: previously stored a millis()-based timestamp ("g_lostModeStartTime"),
// but millis() resets every deep-sleep wake, so the 2-hour timeout never fired
// correctly. Track total seconds in lost mode by ACCUMULATING across wakes
// instead. Incremented by (this wake's runtime + upcoming sleep) right before
// each deep_sleep_start. Cleared on mode change.
RTC_DATA_ATTR uint32_t g_lostModeAccumS = 0;     // Total seconds spent in lost mode (0 = not in lost mode)

// Geofence state (persisted across deep sleep)
RTC_DATA_ATTR bool g_geofenceEnabled = false;
RTC_DATA_ATTR double g_geofenceLat = 0.0;
RTC_DATA_ATTR double g_geofenceLon = 0.0;
RTC_DATA_ATTR float g_geofenceRadiusM = GEOFENCE_DEFAULT_RADIUS_M;
RTC_DATA_ATTR bool g_outsideGeofence = false;
RTC_DATA_ATTR char g_preGeofenceMode[16] = "";   // Mode before geofence auto-escalation

// First boot flag — forces full GPS acquisition + TX regardless of BLE home,
// so the base station discovers this collar and its initial location.
static bool g_firstBoot = false;
static bool g_buttonWake = false;
static bool g_buttonSuppressUntilRelease = false;
static volatile bool g_forceFullCycleRequested = false;

// Developer mode helper — returns true when the active mode is "developer"
static inline bool isDevMode()
{
  return strcmp(g_currentMode, "developer") == 0;
}

static inline const char *modeToWire(const char *mode)
{
  return strcmp(mode, "developer") == 0 ? "dev" : mode;
}

static inline const char *modeFromWire(const char *mode)
{
  return mode && strcmp(mode, "dev") == 0 ? "developer" : mode;
}

// Current active mode (loaded from NVS/RTC on boot)
const OperatingMode *g_activeMode = &MODE_DEVELOPER;

// NVS backup for msg counter (Flash memory - survives any reset including USB resets)
Preferences prefs;

static void saveOperatingMode(const char *modeName);

// ─────────────────────────────────────────────
// Non-blocking double-press button detection
// ─────────────────────────────────────────────
// State machine polled by TaskButton throughout the whole awake cycle.
// Short press → release → press → release toggles dev/normal.
// A deliberate hold requests a full wake cycle and queues an immediate presence ping.
static enum { BTN_IDLE, BTN_WAIT_RELEASE1, BTN_WAIT_RELEASE_LONG, BTN_WAIT_PRESS2, BTN_WAIT_RELEASE2 } g_btnState = BTN_IDLE;
static uint32_t g_btnTimestamp = 0;
static uint32_t g_lastButtonPresenceMs = 0;

static bool queuePresencePing(const char *reason, bool buttonInitiated)
{
  if (txReqQ == nullptr)
    return false;

  StaticJsonDocument<160> pres;
  pres["type"] = "ping";
  pres["src"] = DEVICE_ID_INT;
  pres["dst"] = BASE_ID;
  pres["mid"] = g_msgCounter++;
  pres["uptime_ms"] = millis();
  if (buttonInitiated)
    pres["btn"] = true;

  TxReq req{};
  req.isTelemetry = false;
  serializeJson(pres, req.json, sizeof(req.json));
  if (xQueueSend(txReqQ, &req, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    if (reason && reason[0])
      Serial.printf("[BTN] Presence queued (%s): %s\n", reason, req.json);
    else
      Serial.printf("[BTN] Presence queued: %s\n", req.json);
    DEBUG_PRINTLN("[BTN] Presence queued");
    return true;
  }

  Serial.println("[BTN] Presence queue full — dropped");
  return false;
}

static void requestForcedFullCycle(const char *reason, bool queueImmediatePresence)
{
  g_forceFullCycleRequested = true;
  Serial.printf("[BTN] Forced full cycle requested%s%s%s\n",
                reason && reason[0] ? " (" : "",
                reason && reason[0] ? reason : "",
                reason && reason[0] ? ")" : "");
  if (queueImmediatePresence)
    queuePresencePing(reason, true);
}

static void flashDeveloperModeLed()
{
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(55);
    digitalWrite(LED_PIN, LOW);
    delay(55);
  }
}

static void flashNormalModeLed()
{
  digitalWrite(LED_PIN, HIGH);
  delay(750);
  digitalWrite(LED_PIN, LOW);
}

static void toggleDeveloperMode()
{
  if (isDevMode())
  {
    saveOperatingMode("normal");
    g_activeMode = getModeByName("normal");
    Serial.println("[BTN] *** Developer Mode OFF — switched to Normal ***");
  }
  else
  {
    saveOperatingMode("developer");
    g_activeMode = getModeByName("developer");
    Serial.println("[BTN] *** Developer Mode ON ***");
  }

  if (isDevMode())
    flashDeveloperModeLed();
  else
    flashNormalModeLed();
}

static void pollButtonToggle()
{
  bool pressed = (digitalRead(DEV_MODE_BUTTON_PIN) == LOW);
  uint32_t now = millis();

  if (g_buttonSuppressUntilRelease)
  {
    if (!pressed)
      g_buttonSuppressUntilRelease = false;
    return;
  }

  switch (g_btnState)
  {
  case BTN_IDLE:
    if (pressed)
    {
      g_btnState = BTN_WAIT_RELEASE1;
      g_btnTimestamp = now;
    }
    break;

  case BTN_WAIT_RELEASE1:
    if (pressed && now - g_btnTimestamp >= DEV_MODE_LONG_PRESS_MS)
    {
      if (now - g_lastButtonPresenceMs > 1500U)
      {
        requestForcedFullCycle("button hold", true);
        if (g_forceFullCycleRequested)
          g_lastButtonPresenceMs = now;
      }
      g_btnState = BTN_WAIT_RELEASE_LONG;
    }
    else if (!pressed)
    {
      g_btnState = BTN_WAIT_PRESS2;
      g_btnTimestamp = now;
    }
    else if (now - g_btnTimestamp > 2000)
    {
      g_btnState = BTN_IDLE; // held too long, reset
    }
    break;

  case BTN_WAIT_RELEASE_LONG:
    if (!pressed)
    {
      g_btnState = BTN_IDLE;
    }
    break;

  case BTN_WAIT_PRESS2:
    if (pressed)
    {
      g_btnState = BTN_WAIT_RELEASE2;
    }
    else if (now - g_btnTimestamp > DEV_MODE_DOUBLE_PRESS_MS)
    {
      g_btnState = BTN_IDLE; // timed out waiting for second press
    }
    break;

  case BTN_WAIT_RELEASE2:
    if (!pressed)
    {
      // Double-press detected! Toggle developer mode.
      toggleDeveloperMode();
      g_btnState = BTN_IDLE;
    }
    break;
  }
}

// A sleeping collar wakes on the first press before the normal polling state
// machine is running. Give that click its usual double-click window during
// boot. Without a second click, continue into the standard presence cycle.
static void handleButtonWakeGesture()
{
  if (!g_buttonWake)
    return;

  Serial.println("[BTN] User button woke collar — waiting briefly for second click");

  uint32_t pressStart = millis();
  while (digitalRead(DEV_MODE_BUTTON_PIN) == LOW &&
         millis() - pressStart < DEV_MODE_LONG_PRESS_MS)
    delay(10);
  uint32_t heldMs = millis() - pressStart;

  if (digitalRead(DEV_MODE_BUTTON_PIN) == LOW)
  {
    g_buttonSuppressUntilRelease = true;
    requestForcedFullCycle("sleep button hold", false);
    Serial.printf("[BTN] Sleep wake held for %lu ms — forced presence cycle starts now\n",
                  (unsigned long)heldMs);
    return;
  }

  uint32_t secondPressDeadline = millis() + DEV_MODE_DOUBLE_PRESS_MS;
  while ((int32_t)(secondPressDeadline - millis()) > 0)
  {
    if (digitalRead(DEV_MODE_BUTTON_PIN) == LOW)
    {
      uint32_t secondReleaseDeadline = millis() + 2000U;
      while (digitalRead(DEV_MODE_BUTTON_PIN) == LOW &&
             (int32_t)(secondReleaseDeadline - millis()) > 0)
        delay(10);
      toggleDeveloperMode();
      Serial.println("[BTN] Button wake handled as developer-mode double-click");
      return;
    }
    delay(10);
  }

  if (heldMs >= DEV_MODE_LONG_PRESS_MS)
  {
    requestForcedFullCycle("sleep button press", false);
    Serial.printf("[BTN] Single hold (%lu ms) — starting forced presence/wake cycle\n",
                  (unsigned long)heldMs);
  }
  else
  {
    requestForcedFullCycle("sleep button press", false);
    Serial.printf("[BTN] Short wake press (%lu ms) — starting forced presence/wake cycle\n",
                  (unsigned long)heldMs);
  }
}

void TaskButton(void *pv)
{
  (void)pv;
  for (;;)
  {
    pollButtonToggle();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ─────────────────────────────────────────────
// L76K GNSS LED control (proprietary binary commands)
// ─────────────────────────────────────────────
static const uint8_t L76K_LED_OFF[] = {
    0xBA, 0xCE, 0x10, 0x00, 0x06, 0x03, 0x40,
    0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00,
    0x00,
    0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00,
    0xF0,
    0xC8, 0x17, 0x08};

static void disableL76kLed()
{
  // The L76K can briefly ignore configuration while its UART is settling.
  // Send the vendor LED-off command twice so the 1PPS indicator remains dark
  // throughout acquisition and after the module is powered down.
  gpsSerial.write(L76K_LED_OFF, sizeof(L76K_LED_OFF));
  gpsSerial.flush();
  delay(100);
  gpsSerial.write(L76K_LED_OFF, sizeof(L76K_LED_OFF));
  gpsSerial.flush();
}

static uint8_t nmeaChecksum(const char *body)
{
  uint8_t checksum = 0;
  while (*body != '\0')
    checksum ^= static_cast<uint8_t>(*body++);
  return checksum;
}

static void requestL76kStandby(uint16_t seconds)
{
  char body[24];
  char command[32];
  snprintf(body, sizeof(body), "PCAS12,%u", seconds);
  snprintf(command, sizeof(command), "$%s*%02X\r\n", body, nmeaChecksum(body));
  gpsSerial.print(command);
  gpsSerial.flush();
  Serial.printf("[GPS] Requested L76K standby for %u seconds\n", seconds);
}

// ─────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────
// (cardinalDirection removed in V3 — the receiver computes the cardinal from
// the bearing once it has done its own haversine. Collars no longer need it.)

// LED flicker for successful transmission
static void led_flicker()
{
  uint8_t flashCount = g_activeMode->led_flash_count;
  for (int i = 0; i < flashCount; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// LED beacon mode (continuous flashing for lost mode)
static void led_beacon_pulse()
{
  if (g_activeMode->led_beacon_mode)
  {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(LED_PIN, LOW);
  }
}

// Load operating mode from NVS or use default
static void loadOperatingMode()
{
  prefs.begin("cattracker", true); // Read-only
  String modeName = prefs.getString("op_mode", "developer");
  prefs.end();

  strncpy(g_currentMode, modeName.c_str(), sizeof(g_currentMode) - 1);
  g_currentMode[sizeof(g_currentMode) - 1] = '\0';
  g_activeMode = getModeByName(g_currentMode);

  Serial.printf("[MODE] Loaded: %s (Power: %ddBm, Sleep: %ds)\n",
                g_activeMode->name, g_activeMode->lora_power_dbm, g_activeMode->sleep_interval_s);
  DEBUG_PRINTF("[MODE] %s P%d S%d\n",
               g_activeMode->name, g_activeMode->lora_power_dbm, g_activeMode->sleep_interval_s);
}

// Save operating mode to NVS
static void saveOperatingMode(const char *modeName)
{
  prefs.begin("cattracker", false);
  prefs.putString("op_mode", modeName);
  prefs.end();

  // Update RTC mode
  strncpy(g_currentMode, modeName, sizeof(g_currentMode) - 1);
  g_currentMode[sizeof(g_currentMode) - 1] = '\0';

  // Update active mode
  g_activeMode = getModeByName(modeName);

  Serial.printf("[MODE] Saved: %s\n", modeName);
  DEBUG_PRINTF("[MODE] Saved: %s\n", modeName);
}

// Check lost mode timeout and auto-revert if needed.
//
// Called once near the top of each wake (after RTC vars are restored). Uses
// the accumulator g_lostModeAccumS, which is incremented before deep_sleep_start
// (see accumulateLostModeTime()). This survives deep sleep correctly — millis()
// alone does not, because it resets to 0 on every wake.
static void checkLostModeTimeout()
{
  if (strcmp(g_currentMode, "lost") != 0)
  {
    g_lostModeAccumS = 0; // Not in lost mode — reset the accumulator
    return;
  }

  if (g_lostModeAccumS >= LOST_MODE_MAX_DURATION_S)
  {
    uint32_t elapsedTime = g_lostModeAccumS;
    Serial.printf("[MODE] Lost mode timeout after %u seconds - reverting to %s\n",
                  elapsedTime, LOST_MODE_FALLBACK_MODE);
    DEBUG_PRINTF("[MODE] Lost timeout -> %s\n", LOST_MODE_FALLBACK_MODE);

    // Silent revert. We do NOT send a special "alert" packet — the previous
    // version did, but that packet had no `status` field, so the receiver's
    // JSON normaliser tagged it "Error" and the cat dropped off the map.
    // The next routine telemetry packet (with status "roaming" or similar
    // and now mode="active") tells the receiver everything it needs to know.
    saveOperatingMode(LOST_MODE_FALLBACK_MODE);
    g_lostModeAccumS = 0;
  }
  else
  {
    Serial.printf("[MODE] Lost mode active: %u / %u s\n",
                  g_lostModeAccumS, (uint32_t)LOST_MODE_MAX_DURATION_S);
  }
}

// Accumulate time spent in lost mode. Call this immediately before
// esp_deep_sleep_start so we account for this wake's runtime and the upcoming
// sleep interval. Safe to call when not in lost mode — it's a no-op.
static void accumulateLostModeTime(uint32_t upcomingSleepS)
{
  if (strcmp(g_currentMode, "lost") != 0) return;
  uint32_t thisWakeS = millis() / 1000;
  g_lostModeAccumS += thisWakeS + upcomingSleepS;
  Serial.printf("[MODE] Lost mode accum: +%u (wake) +%u (sleep) = %u / %u s\n",
                thisWakeS, upcomingSleepS, g_lostModeAccumS,
                (uint32_t)LOST_MODE_MAX_DURATION_S);
}

// ─────────────────────────────────────────────
// Geofence Functions
// ─────────────────────────────────────────────

static double haversineDistanceM(double lat1, double lon1, double lat2, double lon2)
{
  const double R = 6371000.0; // Earth radius in metres
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

static void loadGeofence()
{
  Preferences p;
  if (!p.begin("cattracker", true)) return;
  g_geofenceEnabled = p.getBool("gf_on", false);
  if (g_geofenceEnabled)
  {
    // Preferences doesn't have getDouble, so store as string
    String latStr = p.getString("gf_lat", "0");
    String lonStr = p.getString("gf_lon", "0");
    g_geofenceLat = atof(latStr.c_str());
    g_geofenceLon = atof(lonStr.c_str());
    g_geofenceRadiusM = p.getFloat("gf_rad", GEOFENCE_DEFAULT_RADIUS_M);
  }
  p.end();

  if (g_geofenceEnabled)
  {
    Serial.printf("[GEOFENCE] Loaded: center=%.6f,%.6f radius=%.0fm\n",
                  g_geofenceLat, g_geofenceLon, g_geofenceRadiusM);
    DEBUG_PRINTF("[GF] on %.6f,%.6f r%.0f\n", g_geofenceLat, g_geofenceLon, g_geofenceRadiusM);
  }
}

static bool saveGeofence(double lat, double lon, float radiusM, bool enabled)
{
  Preferences p;
  if (!p.begin("cattracker", false)) return false;
  p.putBool("gf_on", enabled);
  if (enabled)
  {
    char buf[20];
    snprintf(buf, sizeof(buf), "%.8f", lat);
    p.putString("gf_lat", buf);
    snprintf(buf, sizeof(buf), "%.8f", lon);
    p.putString("gf_lon", buf);
    p.putFloat("gf_rad", radiusM);
  }
  p.end();

  g_geofenceEnabled = enabled;
  g_geofenceLat = lat;
  g_geofenceLon = lon;
  g_geofenceRadiusM = radiusM;

  if (enabled)
  {
    Serial.printf("[GEOFENCE] Saved: center=%.6f,%.6f radius=%.0fm\n", lat, lon, radiusM);
    DEBUG_PRINTF("[GF] saved %.6f,%.6f r%.0f\n", lat, lon, radiusM);
  }
  else
  {
    Serial.println("[GEOFENCE] Disabled");
    DEBUG_PRINTLN("[GF] off");
    g_outsideGeofence = false;
    g_preGeofenceMode[0] = '\0';
  }
  return true;
}

// Check a GPS fix against the geofence. Returns "inside", "outside", or nullptr
// if geofence is disabled. Handles auto-escalation and de-escalation.
static const char *checkGeofence(double lat, double lon)
{
  if (!g_geofenceEnabled) return nullptr;

  double dist = haversineDistanceM(lat, lon, g_geofenceLat, g_geofenceLon);
  Serial.printf("[GEOFENCE] Distance: %.1fm (radius: %.0fm)\n", dist, g_geofenceRadiusM);
  DEBUG_PRINTF("[GF] dist=%.0f/%.0f\n", dist, g_geofenceRadiusM);

  bool wasOutside = g_outsideGeofence;

  if (dist > g_geofenceRadiusM)
  {
    g_outsideGeofence = true;

    if (!wasOutside)
    {
      Serial.println("[GEOFENCE] BREACH — cat left the geofence");
      DEBUG_PRINTLN("[GF] BREACH");

      // Auto-escalate if in a low-urgency mode
      if (strcmp(g_currentMode, "normal") == 0 || strcmp(g_currentMode, "powersave") == 0)
      {
        strncpy(g_preGeofenceMode, g_currentMode, sizeof(g_preGeofenceMode) - 1);
        g_preGeofenceMode[sizeof(g_preGeofenceMode) - 1] = '\0';
        saveOperatingMode(GEOFENCE_ESCALATE_MODE);
        Serial.printf("[GEOFENCE] Auto-escalated from '%s' to '%s'\n",
                      g_preGeofenceMode, GEOFENCE_ESCALATE_MODE);
        DEBUG_PRINTF("[GF] escalate -> %s\n", GEOFENCE_ESCALATE_MODE);
      }
    }
    return "outside";
  }
  else if (dist < (g_geofenceRadiusM - GEOFENCE_HYSTERESIS_M))
  {
    // Inside with hysteresis margin — de-escalate if we previously auto-escalated
    g_outsideGeofence = false;

    if (wasOutside && g_preGeofenceMode[0] != '\0')
    {
      Serial.printf("[GEOFENCE] Returned inside — reverting to '%s'\n", g_preGeofenceMode);
      DEBUG_PRINTF("[GF] revert -> %s\n", g_preGeofenceMode);
      saveOperatingMode(g_preGeofenceMode);
      g_preGeofenceMode[0] = '\0';
    }
    return "inside";
  }
  else
  {
    // In hysteresis band — keep current state
    return g_outsideGeofence ? "outside" : "inside";
  }
}

// ─────────────────────────────────────────────
// CSV Logging Functions
// ─────────────────────────────────────────────

static bool g_csvReady = false;

// Initialize LittleFS and create CSV header if needed
static bool initCSVLogging()
{
#if CSV_LOG_ENABLED
  if (!LittleFS.begin(true))
  {
    Serial.println("[CSV] LittleFS mount failed — run 'pio run -t erase' once to flash partition table");
    DEBUG_PRINTLN("[CSV] Mount failed");
    g_csvReady = false;
    return false;
  }

  g_csvReady = true;

  Serial.printf("[CSV] LittleFS mounted - Total: %d KB, Used: %d KB\n",
                LittleFS.totalBytes() / 1024, LittleFS.usedBytes() / 1024);
  DEBUG_PRINTF("[CSV] FS: %dKB/%dKB\n", LittleFS.usedBytes() / 1024, LittleFS.totalBytes() / 1024);

  // Check if log file exists
  if (!LittleFS.exists(CSV_LOG_FILE))
  {
    // Create new file with header
    File logFile = LittleFS.open(CSV_LOG_FILE, "w");
    if (!logFile)
    {
      Serial.println("[CSV] Failed to create log file");
      DEBUG_PRINTLN("[CSV] Create failed");
      return false;
    }
    logFile.println(CSV_LOG_HEADER);
    logFile.close();
    Serial.println("[CSV] Created new log file with header");
    DEBUG_PRINTLN("[CSV] New log created");
  }
  else
  {
    // Check file size and rotate if needed
    File logFile = LittleFS.open(CSV_LOG_FILE, "r");
    if (logFile)
    {
      size_t fileSize = logFile.size();
      logFile.close();

      if (fileSize > (CSV_MAX_FILE_SIZE_KB * 1024))
      {
        Serial.printf("[CSV] Log file too large (%d KB), rotating...\n", fileSize / 1024);
        DEBUG_PRINTLN("[CSV] Rotating log");

        // Backup old log
        LittleFS.remove("/track_log_old.csv");
        LittleFS.rename(CSV_LOG_FILE, "/track_log_old.csv");

        // Create new log with header
        File newLog = LittleFS.open(CSV_LOG_FILE, "w");
        if (newLog)
        {
          newLog.println(CSV_LOG_HEADER);
          newLog.close();
          Serial.println("[CSV] Log rotated successfully");
        }
      }
      else
      {
        Serial.printf("[CSV] Log file ready (%d KB)\n", fileSize / 1024);
        DEBUG_PRINTF("[CSV] Log: %dKB\n", fileSize / 1024);
      }
    }
  }

  return true;
#else
  return false;
#endif
}

// Log transmission to CSV file
static void logTransmissionToCSV(const char *json, int rssi = 0, float snr = 0.0)
{
#if CSV_LOG_ENABLED
  if (!g_csvReady) return;

  // Parse the JSON to extract fields
  StaticJsonDocument<320> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.printf("[CSV] JSON parse error: %s\n", error.c_str());
    return;
  }

  // Open file in append mode
  File logFile = LittleFS.open(CSV_LOG_FILE, "a");
  if (!logFile)
  {
    Serial.println("[CSV] Failed to open log file for append");
    DEBUG_PRINTLN("[CSV] Append failed");
    return;
  }

  // Build CSV line: timestamp,msg_id,device,mode,status,lat,lon,dist_m,bearing,battery_v,rssi,snr
  char csvLine[256];

  // The wire carries GPS time as Unix seconds. Fall back to uptime if absent.
  uint32_t timestamp = doc["time"] | (uint32_t)0;
  snprintf(csvLine, sizeof(csvLine), "%lu,",
           (unsigned long)(timestamp ? timestamp : millis() / 1000));

  // Add remaining fields
  char temp[200];
  // v3.9.x: telemetry now uses SHORT wire keys (seq=msg_id, st=status); this
  // local CSV parses the same doc so it reads the short keys too.
  snprintf(temp, sizeof(temp), "%u,%s,%s,%s,%.6f,%.6f,%.1f,%s,%.2f,%d,%.1f",
           doc["seq"] | 0,
           doc["name"] | (const char *)g_senderName,
           g_currentMode,
           doc["st"] | "unknown",
           doc["lat"] | 0.0,
           doc["lon"] | 0.0,
           doc["dist_m"] | 0.0,
           doc["bearing"] | "0-N",
           0.0, // battery_v (TODO: add battery monitoring)
           rssi,
           snr);

  strcat(csvLine, temp);

  // Write to file
  logFile.println(csvLine);
  logFile.close();

  Serial.printf("[CSV] Logged: seq=%u, st=%s\n",
                doc["seq"] | 0, doc["st"] | "?");
  DEBUG_PRINTF("[CSV] Log: %u\n", doc["seq"] | 0);

#endif
}

// Get CSV log file info (for status requests)
static void getCSVLogInfo(char *info, size_t maxLen)
{
#if CSV_LOG_ENABLED
  if (!g_csvReady)
  {
    snprintf(info, maxLen, "FS not mounted");
    return;
  }

  if (!LittleFS.exists(CSV_LOG_FILE))
  {
    snprintf(info, maxLen, "No log file");
    return;
  }

  File logFile = LittleFS.open(CSV_LOG_FILE, "r");
  if (!logFile)
  {
    snprintf(info, maxLen, "Cannot open log");
    return;
  }

  size_t fileSize = logFile.size();
  int lineCount = 0;

  // Count lines
  while (logFile.available())
  {
    if (logFile.read() == '\n')
      lineCount++;
  }
  logFile.close();

  snprintf(info, maxLen, "%d entries, %d KB", lineCount - 1, fileSize / 1024); // -1 for header
#else
  snprintf(info, maxLen, "Logging disabled");
#endif
}

// ─────────────────────────────────────────────
// Task handles (for cleanup before sleep)
// ─────────────────────────────────────────────
static TaskHandle_t hGPS = nullptr;
static TaskHandle_t hBLE = nullptr;
static TaskHandle_t hLoRa = nullptr;
static TaskHandle_t hPower = nullptr;
static TaskHandle_t hButton = nullptr;
static bool g_gpsStartedThisWake = false;

// ─────────────────────────────────────────────
// Task forward declarations (defined later)
// ─────────────────────────────────────────────
void TaskGPS(void *);
void TaskBLE(void *);
void TaskLoRa(void *);
void TaskPower(void *);

// Start the GPS only after the initial BLE home decision says it is needed.
// Keeping the UART and parser task off during Phase 1 is the main at-home
// power saving: ordinary home cycles never wake the L76K from standby.
static bool startGpsForAcquisition()
{
  if (g_gpsStartedThisWake)
    return true;

  Serial.println("[GPS] Waking after BLE scan");
  DEBUG_PRINTLN("[GPS] Wake after BLE");

  pinMode(GPS_WAKEUP, OUTPUT);
  digitalWrite(GPS_WAKEUP, HIGH);
  delay(500); // Let the module wake and its UART stabilise.

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(100);
  while (gpsSerial.available())
    gpsSerial.read();

  // Keep the onboard GNSS/1PPS LED permanently disabled. This is the vendor
  // OffState command; do not send the RecoverState command anywhere.
  disableL76kLed();

  BaseType_t created = xTaskCreatePinnedToCore(
      TaskGPS, "gps", 4096, nullptr, 2, &hGPS, APP_CPU_NUM);
  if (created != pdPASS)
  {
    Serial.println("[GPS] ERROR: failed to create GPS reader task");
    gpsSerial.end();
    pinMode(GPS_TX, OUTPUT);
    digitalWrite(GPS_TX, LOW);
    digitalWrite(GPS_WAKEUP, LOW);
    hGPS = nullptr;
    return false;
  }

  g_gpsStartedThisWake = true;
  Serial.println("[GPS] UART ready, parser task started");
  DEBUG_PRINTLN("[GPS] UART/task ready");
  return true;
}

// ─────────────────────────────────────────────
// Setup & Main loop
// ─────────────────────────────────────────────
void setup()
{
  // Check wake reason BEFORE Serial init
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  // Release GPIO hold from deep sleep
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)GPS_WAKEUP);
  gpio_hold_dis((gpio_num_t)GPS_TX);
  gpio_hold_dis((gpio_num_t)LORA_NSS);
  gpio_hold_dis((gpio_num_t)LORA_RST);
  gpio_hold_dis((gpio_num_t)LORA_SCK);
  gpio_hold_dis((gpio_num_t)LORA_MOSI);
  rtc_gpio_deinit((gpio_num_t)DEV_MODE_BUTTON_PIN);

  Serial.begin(115200);
  delay(100); // Give serial time to initialize

  // Initialize debug serial for battery operation
#if DEBUG_SERIAL_ENABLED
  DebugSerial.begin(115200, SERIAL_8N1, -1, DEBUG_TX_PIN); // TX only
  delay(50);
  DEBUG_PRINTLN("\n\n=== DEBUG SERIAL ACTIVE ===");
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(DEV_MODE_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("\n\n╔═══════════════════════════════════════════╗");
  Serial.println("║   BluePawz CatTracker TX (FreeRTOS)       ║");
  Serial.printf( "║   FW: %-36s ║\n", FIRMWARE_VERSION);
  Serial.printf( "║   Built: %-33s ║\n", BUILD_TIMESTAMP);
  Serial.println("╚═══════════════════════════════════════════╝");
  DEBUG_PRINTF("[BOOT] FW: %s\n", FIRMWARE_VERSION);

  // Derive unique device ID from MAC address
  initDeviceId();

  Serial.printf("[BOOT] Device ID: %d (MAC: %02X:%02X:%02X:%02X:%02X:%02X)\n",
                DEVICE_ID_INT, g_mac[0], g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5]);
  Serial.printf("[BOOT] Chip: %s  Rev: %d  Cores: %d\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());
  Serial.printf("[BOOT] Flash: %d KB  Heap free: %d bytes\n",
                ESP.getFlashChipSize() / 1024, ESP.getFreeHeap());
  Serial.printf("[BOOT] Reset reason: %d  Wakeup cause: %d\n",
                esp_reset_reason(), wakeup_reason);
  DEBUG_PRINTF("[BOOT] Reset: %d  Wake: %d\n", esp_reset_reason(), wakeup_reason);
  g_buttonWake = (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0);

  // Load persistent counter from NVS (survives all resets)
  prefs.begin("cattracker", false);
  uint32_t nvsCounter = prefs.getUInt("msg_id", 0);

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_TIMER:
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.printf("[BOOT] Wake from DEEP SLEEP via %s (RTC msg_id: %d)\n",
                  g_buttonWake ? "USER BUTTON" : "TIMER", g_msgCounter);
    DEBUG_PRINTF("[BOOT] Wake from %s (msg_id: %d)\n",
                 g_buttonWake ? "BUTTON" : "TIMER", g_msgCounter);
    g_firstBoot = false;
    // Use RTC counter if valid (faster), otherwise fall back to NVS
    if (g_msgCounter < nvsCounter)
    {
      g_msgCounter = nvsCounter;
      Serial.printf("[BOOT] RTC counter was stale, restored from NVS: %d\n", g_msgCounter);
      DEBUG_PRINTF("[BOOT] Restored from NVS: %d\n", g_msgCounter);
    }
    break;
  case ESP_SLEEP_WAKEUP_UNDEFINED:
  default:
    Serial.println("[BOOT] *** FIRST BOOT / POWER-ON RESET ***");
    Serial.println("[BOOT] Will force full GPS acquisition + TX so base station discovers this collar");
    DEBUG_PRINTLN("[BOOT] FIRST BOOT - forced TX");
    g_firstBoot = true;
    // RTC lost, restore from NVS flash
    g_msgCounter = nvsCounter;
    g_gpsWarmedUp = false;
    g_homeBeaconCycles = prefs.getUChar("home_cycles", 0);
    Serial.printf("[BOOT] Restored from NVS: msg_id=%d, home_cycles=%d\n",
                  g_msgCounter, g_homeBeaconCycles);
    DEBUG_PRINTF("[BOOT] NVS: msg_id=%d hc=%d\n", g_msgCounter, g_homeBeaconCycles);
    break;
  }
  prefs.end();

  // Load operating mode from NVS
  loadOperatingMode();

  // V3: load friendly name from NVS (default "Device-<id>" if unset)
  loadSenderName();

  // Load geofence configuration from NVS
  loadGeofence();

  // Check lost mode timeout (auto-revert if exceeded)
  checkLostModeTimeout();

  // Resolve a button wake as either a single forced cycle or the first click
  // of the existing developer-mode double-click gesture.
  g_btnState = BTN_IDLE;
  handleButtonWakeGesture();

  // Initialize CSV logging (LittleFS)
  initCSVLogging();

  // ── Boot config summary ──
  Serial.println("[BOOT] ─── Configuration ───");
  Serial.printf("[BOOT]   Name:       %s\n", g_senderName);
  Serial.printf("[BOOT]   Device ID:  %d\n", DEVICE_ID_INT);
  Serial.printf("[BOOT]   Mode:       %s (TX %d dBm, sleep %d s)\n",
                g_activeMode->name, g_activeMode->lora_power_dbm, g_activeMode->sleep_interval_s);
  Serial.printf("[BOOT]   LoRa:       SF%d BW%.0f CR4/%d  Freq %.1f MHz\n",
                LORA_SF, LORA_BW_KHZ, LORA_CR, LORA_FREQ_MHZ);
  Serial.printf("[BOOT]   BLE:        Beacon '%s'  RSSI threshold %d dBm\n",
                BEACON_NAME, HOME_RSSI_THRESHOLD_DBM);
  Serial.printf("[BOOT]   GPS:        Cold %ds  Warm %ds  Fixes needed %d\n",
                GPS_COLD_START_TIMEOUT / 1000, GPS_WARM_START_TIMEOUT / 1000, GPS_VALID_COUNT_REQUIRED);
  Serial.printf("[BOOT]   GPS warmed: %s\n", g_gpsWarmedUp ? "yes" : "no");
  Serial.printf("[BOOT]   Geofence:   %s", g_geofenceEnabled ? "ON" : "OFF");
  if (g_geofenceEnabled)
    Serial.printf(" (%.6f, %.6f  r=%.0fm)", g_geofenceLat, g_geofenceLon, g_geofenceRadiusM);
  Serial.println();
  Serial.printf("[BOOT]   msg_id:     %d\n", g_msgCounter);
  Serial.printf("[BOOT]   First boot: %s\n", g_firstBoot ? "YES — will force TX" : "no");
  if (isDevMode())
  {
    Serial.println("[BOOT]   *** DEVELOPER MODE ACTIVE ***");
    Serial.println("[BOOT]   Extra diagnostics in telemetry + serial");
    Serial.println("[BOOT]   Double-press USER button anytime to return to Normal");
  }
  else
  {
    Serial.println("[BOOT]   Double-press USER button anytime to enter Developer Mode");
  }
  Serial.println("[BOOT] ────────────────────");

  // Queues & events
  gpsFixQ = xQueueCreate(1, sizeof(GpsFix)); // latest fix (overwrite)
  txReqQ = xQueueCreate(4, sizeof(TxReq));   // TX requests
  evBits = xEventGroupCreate();              // state flags (EV_FIX, EV_HOME, EV_TXDONE)

  // Keep the L76K in standby until the initial BLE home scan fails or a
  // scheduled home heartbeat requires a location update. The module's 3V3
  // supply remains present because the XIAO GNSS board exposes WAKE_UP, not a
  // switched power-enable signal.
  pinMode(GPS_WAKEUP, OUTPUT);
  digitalWrite(GPS_WAKEUP, LOW);
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_TX, LOW);
  g_gpsStartedThisWake = false;
  Serial.println("[INIT] GPS held off pending BLE decision");
  DEBUG_PRINTLN("[INIT] GPS off pending BLE");

  // LoRa radio init (will be re-done in TaskLoRa, but SPI setup here)
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  Serial.println("[LORA] SPI ready");

  // BLE init (scanner only) — suppress noisy GAP event warnings
  esp_log_level_set("BLEScan", ESP_LOG_ERROR);
  BLEDevice::init("");
  Serial.println("[BLE] stack init done");

  // Create tasks
  xTaskCreatePinnedToCore(TaskButton, "button", 3072, nullptr, 2, &hButton, APP_CPU_NUM);
  xTaskCreatePinnedToCore(TaskBLE, "ble", 4096, nullptr, 1, &hBLE, APP_CPU_NUM); // BLE on APP CPU
  xTaskCreatePinnedToCore(TaskLoRa, "lora", 4096, nullptr, 2, &hLoRa, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(TaskPower, "power", 4096, nullptr, 3, &hPower, PRO_CPU_NUM);

  Serial.println("[BOOT] RTOS tasks started");
}

// Post-TX RX window: after telemetry goes out, keep the LoRa radio listening
// briefly so the base can deliver an OTAP command + receive the collar's ACK
// before the collar deep-sleeps. The collar shuts down GPS/BLE immediately but
// holds the radio open for this grace period. Each command received extends the
// window so a burst (and the ACK round-trip) all lands.
#define POST_TX_LISTEN_MS 5000U // 5 s grace listen window before sleep
#define POST_TX_EXTEND_MS 3000U // extend per command received (ACK round-trip)

// millis() of the last inbound OTAP command for THIS device (set in
// handleInboundLoRa, read by loop()'s listen window to extend it). Plain global
// (within-wake only) — volatile because it crosses TaskLoRa → loop().
static volatile uint32_t g_lastCmdRxMs = 0;

// Graceful radio shutdown handshake (loop() ↔ TaskLoRa). When the listen window
// closes, loop() raises g_loraStopReq instead of force-deleting TaskLoRa; the
// task self-deletes at its next IDLE point (any in-flight ACK transmit finished,
// txReqQ drained, radio re-armed) — never mid-SPI — and confirms via
// g_loraStopped. This deterministically closes the ACK-TX teardown race that a
// fixed time budget (LBT backoff can blow past it) could not.
static volatile bool g_loraStopReq = false;  // loop() → TaskLoRa: stop when idle
static volatile bool g_loraStopped = false;  // TaskLoRa → loop(): radio asleep, ending
static volatile int16_t g_loraSleepResult = RADIOLIB_ERR_NONE;

// Every deep-sleep wake performs a complete lora.begin() and configuration, so
// retaining the SX1262 modem configuration serves no purpose. Cold sleep turns
// off its RTC and discards configuration for the radio's lowest sleep state.
static int16_t putLoRaToColdSleep()
{
  int16_t state = lora.sleep(false);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[SLEEP] SX1262 cold-sleep command failed (%d), retrying\n", state);
    delay(2);
    state = lora.sleep(false);
  }
  return state;
}

void loop()
{
  // Wait for Power task to signal cycle complete
  EventBits_t bits = xEventGroupWaitBits(evBits, EV_TXDONE, pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & EV_TXDONE)
  {
    // Needed while the GNSS UART is still open so its timed standby can match
    // the ESP32 profile sleep.
    uint16_t sleepSeconds = g_activeMode->sleep_interval_s;

    // Cycle complete (telemetry sent, or a home-skip). Open a short post-TX
    // LISTEN WINDOW before sleeping: shut down the power-hungry peripherals
    // (GPS, BLE) NOW, but KEEP the LoRa radio alive + listening so the base can
    // deliver a late OTAP command and receive its ACK before we deep-sleep. The
    // radio re-armed RX after the last TX, so TaskLoRa is already listening;
    // handleInboundLoRa applies + ACKs in that task while loop() waits out the
    // window here.
    uint32_t listenStart = millis();
    Serial.println("[MAIN] Cycle complete — GPS/BLE down, LoRa listening before sleep");

    // ── graceful peripheral shutdown (everything EXCEPT the radio) ──
    if (hGPS)   { vTaskDelete(hGPS);   hGPS = nullptr; }
    if (hBLE)   { vTaskDelete(hBLE);   hBLE = nullptr; }
    if (hPower) { vTaskDelete(hPower); hPower = nullptr; } // already self-suspended
    BLEDevice::deinit(true);

    // Use both documented standby controls: the timed CASIC command and the
    // active-low WAKEUP pin. The margin covers the post-TX listen window and
    // shutdown work before the ESP32 actually enters deep sleep.
    if (g_gpsStartedThisWake)
    {
      uint32_t standbySeconds = static_cast<uint32_t>(sleepSeconds) + 10U;
      if (standbySeconds > UINT16_MAX)
        standbySeconds = UINT16_MAX;

      disableL76kLed();
      requestL76kStandby(static_cast<uint16_t>(standbySeconds));
      delay(150);
      digitalWrite(GPS_WAKEUP, LOW);
      delay(50);
      gpsSerial.end();
      g_gpsStartedThisWake = false;
    }
    digitalWrite(GPS_WAKEUP, LOW);
    pinMode(GPS_TX, OUTPUT);
    digitalWrite(GPS_TX, LOW);
    Serial.printf("[SLEEP] GPS/BLE down; GNSS WAKEUP GPIO%d=%d, UART TX LOW\n",
                  GPS_WAKEUP, digitalRead(GPS_WAKEUP));

    // ── post-TX listen window ──
    // Stay open POST_TX_LISTEN_MS (5 s). Each inbound command stamps
    // g_lastCmdRxMs and extends the window by POST_TX_EXTEND_MS so the ACK
    // round-trip (and any follow-up command in the same burst) completes before
    // we kill the radio — avoids tearing down TaskLoRa mid-ACK.
    for (;;)
    {
      uint32_t now = millis();
      uint32_t windowEnd = listenStart + POST_TX_LISTEN_MS;
      if (g_lastCmdRxMs > listenStart && (g_lastCmdRxMs + POST_TX_EXTEND_MS) > windowEnd)
        windowEnd = g_lastCmdRxMs + POST_TX_EXTEND_MS;
      if (now >= windowEnd)
        break;
      delay(50);
    }
    Serial.printf("[MAIN] Listen window closed after %lu ms — sleeping\n",
                  (unsigned long)(millis() - listenStart));
    if (hButton){ vTaskDelete(hButton);hButton = nullptr; }

    // ── stop the radio SAFELY ──
    // Ask TaskLoRa to self-delete at its next IDLE point — it finishes any
    // in-flight ACK transmit + drains txReqQ first, then sleeps the radio — so we
    // never vTaskDelete it mid-SPI on the shared bus (the ACK can clear LBT
    // backoff + airtime well after the listen window's time budget). Wait with a
    // hard cap so a wedged radio can't block deep sleep forever.
    g_loraStopReq = true;
    uint32_t stopWait = millis();
    while (!g_loraStopped && (millis() - stopWait) < 4000U)
      delay(20);
    if (g_loraStopped)
    {
      hLoRa = nullptr; // TaskLoRa self-deleted and already slept the radio
    }
    else if (hLoRa) // cap hit — degenerate fallback (radio wedged): force it down
    {
      vTaskDelete(hLoRa);
      hLoRa = nullptr;
      g_loraSleepResult = putLoRaToColdSleep();
    }
    if (g_loraSleepResult == RADIOLIB_ERR_NONE)
      Serial.println("[SLEEP] SX1262 confirmed in cold sleep");
    else
      Serial.printf("[SLEEP] WARNING: SX1262 sleep command failed (%d)\n",
                    g_loraSleepResult);

    // The ESP32 SPI peripheral powers down in deep sleep. Release it explicitly,
    // then hold the SX1262 input lines at defined idle levels.
    LoRaSPI.end();
    pinMode(LORA_SCK, OUTPUT);
    digitalWrite(LORA_SCK, LOW);
    pinMode(LORA_MOSI, OUTPUT);
    digitalWrite(LORA_MOSI, LOW);

    // Save persistent state to NVS before sleep (survives USB resets)
    prefs.begin("cattracker", false);
    prefs.putUInt("msg_id", g_msgCounter);
    prefs.putUChar("home_cycles", g_homeBeaconCycles);
    prefs.end();

    // Hold GPIO states during deep sleep
    gpio_hold_en((gpio_num_t)GPS_WAKEUP);   // Keep L76K WAKE_UP LOW (standby; 3V3 remains powered)
    gpio_hold_en((gpio_num_t)GPS_TX);       // Keep UART TX LOW (no backfeed to L76K)
    gpio_hold_en((gpio_num_t)LORA_NSS);     // Keep NSS HIGH (SX1262 deselected)
    gpio_hold_en((gpio_num_t)LORA_RST);     // Keep RST HIGH (SX1262 not in reset)
    gpio_hold_en((gpio_num_t)LORA_SCK);     // Keep radio clock LOW (no floating input)
    gpio_hold_en((gpio_num_t)LORA_MOSI);    // Keep radio data input LOW
    gpio_deep_sleep_hold_en();

    // V3: accumulate lost-mode time across deep-sleep cycles so the 2-hour
    // auto-revert actually fires. No-op when not in lost mode.
    accumulateLostModeTime(sleepSeconds);

    // Enter deep sleep
    Serial.printf("[SLEEP] Deep sleeping for %d s in '%s' mode (msg_id: %d)\n",
                  sleepSeconds, g_activeMode->name, g_msgCounter);
    DEBUG_PRINTF("[SLEEP] %ds (%s) msg_id:%d\n", sleepSeconds, g_activeMode->name, g_msgCounter);
    Serial.flush();
#if DEBUG_SERIAL_ENABLED
    DebugSerial.flush();
#endif
    delay(100); // Ensure serial buffer is flushed

// Disable old wake sources, then arm both the profile timer and the active-low
// user button. EXT0 keeps the RTC peripheral domain powered so its internal
// pull-up remains available; moving to EXT1 would require an external pull-up.
#ifdef CONFIG_IDF_TARGET_ESP32S3
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#endif

    rtc_gpio_pullup_en((gpio_num_t)DEV_MODE_BUTTON_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)DEV_MODE_BUTTON_PIN);
    esp_sleep_enable_timer_wakeup((uint64_t)sleepSeconds * 1000000ULL);
    esp_err_t buttonWakeResult =
        esp_sleep_enable_ext0_wakeup((gpio_num_t)DEV_MODE_BUTTON_PIN, 0);
    if (buttonWakeResult != ESP_OK)
      Serial.printf("[SLEEP] WARNING: button wake setup failed (%d)\n", buttonWakeResult);
    esp_deep_sleep_start();
    // Never returns - ESP32 will restart and run setup() again
  }
}

// ─────────────────────────────────────────────
// Task: GPS reader (non‑blocking)
//  • Continuously parses NMEA and publishes latest fix
// ─────────────────────────────────────────────
void TaskGPS(void *)
{
  GpsFix fix; // local working copy
  uint32_t lastStatusTime = millis();
  bool lastReportedValid = false;

  for (;;)
  {
    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      gps.encode(c);
    }

    // Periodic GPS status update (5s in developer mode, 10s otherwise)
    uint32_t statusInterval = isDevMode() ? 5000 : 10000;
    if (millis() - lastStatusTime >= statusInterval)
    {
      lastStatusTime = millis();
      bool hasLoc = gps.location.isValid();
      uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
      uint32_t hdop = gps.hdop.isValid() ? gps.hdop.value() : 9999;

      if (hasLoc)
      {
        Serial.printf("[GPS] Fix: %.6f, %.6f  Sats: %d  HDOP: %.1f\n",
                      gps.location.lat(), gps.location.lng(), sats, hdop / 100.0);
      }
      else
      {
        Serial.printf("[GPS] Acquiring... Sats: %d  HDOP: %.1f  Chars: %lu\n",
                      sats, hdop / 100.0, gps.charsProcessed());
      }
      if (isDevMode())
      {
        Serial.printf("[DEV] Heap: %d  Sentences: %lu  Failed: %lu\n",
                      ESP.getFreeHeap(), gps.sentencesWithFix(), gps.failedChecksum());
      }
    }

    // When location updates, refresh state
    if (gps.location.isUpdated())
    {
      fix.lat = gps.location.lat();
      fix.lon = gps.location.lng();
      fix.valid = gps.location.isValid();
      if (fix.valid)
      {
        g_lastGpsValid = true;
        g_lastGpsLat = fix.lat;
        g_lastGpsLon = fix.lon;
      }

      // Convert valid GPS UTC directly to compact Unix seconds for the wire.
      if (gps.date.isValid() && gps.time.isValid())
      {
        fix.unixTime = gpsToUnixTime(
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
        if (fix.valid)
          g_lastGpsUnixTime = fix.unixTime;
      }
      else
      {
        fix.unixTime = 0;
      }

      if (fix.valid && !lastReportedValid)
      {
        lastReportedValid = true;
        xEventGroupSetBits(evBits, EV_FIX);
        Serial.printf("[GPS] *** FIRST FIX: %.6f, %.6f ***\n", fix.lat, fix.lon);
        DEBUG_PRINTF("[GPS] Fix: %.6f, %.6f\n", fix.lat, fix.lon);
      }
      else if (fix.valid)
      {
        xEventGroupSetBits(evBits, EV_FIX);
      }
      // Overwrite latest fix in queue (drop older value if present)
      xQueueOverwrite(gpsFixQ, &fix);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ─────────────────────────────────────────────
// Task: BLE scanner
//  • Performs initial 10s scan on wake
//  • Continues scanning during GPS acquisition
//  • Sets EV_HOME when beacon detected
// ─────────────────────────────────────────────
void TaskBLE(void *)
{
  BLEScan *scan = BLEDevice::getScan();
  scan->setActiveScan(true);

  for (;;)
  {
    BLEScanResults res = scan->start(BLE_SCAN_WINDOW_S, false);
    for (int i = 0; i < res.getCount(); i++)
    {
      BLEAdvertisedDevice dev = res.getDevice(i);
      if (!dev.haveName()) continue;
      if (String(dev.getName().c_str()) != BEACON_NAME) continue;

      // V3 RSSI gate: don't count a faint distant beacon as "home". The
      // beacon transmits at -12 dBm intentionally; the threshold corresponds
      // to "you should be inside the building". See config.h to tune.
      int rssi = dev.haveRSSI() ? dev.getRSSI() : -127;
      if (rssi < HOME_RSSI_THRESHOLD_DBM)
      {
        Serial.printf("[BLE] 'Home' beacon seen but RSSI %d dBm < threshold %d dBm — ignoring\n",
                      rssi, HOME_RSSI_THRESHOLD_DBM);
        DEBUG_PRINTF("[BLE] Home faint: %d\n", rssi);
        continue;
      }

      xEventGroupSetBits(evBits, EV_HOME);
      Serial.printf("[BLE] 'Home' beacon detected! RSSI=%d dBm (>= %d)\n",
                    rssi, HOME_RSSI_THRESHOLD_DBM);
      DEBUG_PRINTF("[BLE] Home OK %d\n", rssi);
      break;
    }
    scan->clearResults();

    vTaskDelay(pdMS_TO_TICKS(500)); // Short delay between scans
  }
}

// ─────────────────────────────────────────────
// OTAP inbound command path (re-instated in Phase 1)
// ─────────────────────────────────────────────
// SX1262 RX is interrupt-driven: DIO1 fires when a packet lands and this ISR
// just flips a flag. TaskLoRa (the SOLE radio owner) polls the flag, reads the
// packet, and re-arms RX. The ISR must stay in IRAM and do nothing else.
static volatile bool g_loraRxFlag = false;
void IRAM_ATTR onLoRaDio1()
{
  g_loraRxFlag = true;
}

// Reserved envelope fields that are NOT configurable parameters. Everything
// else in a command doc is treated as a key/value config change by the generic
// apply loop below.
static bool isEnvelopeKey(const char *k)
{
  // Short keys are current; long/legacy aliases remain accepted during rollout.
  return !strcmp(k, "type") || !strcmp(k, "src") || !strcmp(k, "dst") ||
         !strcmp(k, "mid") || !strcmp(k, "seq") || !strcmp(k, "did") ||
         !strcmp(k, "source_id") || !strcmp(k, "destination_id") ||
         !strcmp(k, "message_id") || !strcmp(k, "msg_id") ||
         !strcmp(k, "device_id") || !strcmp(k, "cmd") ||
         !strcmp(k, "profile") || !strcmp(k, "device") ||
         !strcmp(k, "ping") || !strcmp(k, "timestamp") || !strcmp(k, "time");
}

// Valid OTAP-settable power profiles. NOTE: getModeByName() can't validate — it
// silently falls back to MODE_NORMAL for an unknown name — so we check the value
// explicitly here. "developer" IS remotely settable (useful for debugging: it
// adds dev diagnostics to telemetry, lowest awake power, 30 s cycle); it can
// also still be toggled locally by the button.
static bool bpValidProfile(const char *p)
{
  return p && (!strcmp(p, "powersave") || !strcmp(p, "normal") ||
               !strcmp(p, "active") || !strcmp(p, "lost") ||
               !strcmp(p, "developer"));
}

// Parse one inbound LoRa frame (already NUL-terminated). If it is a COMMAND
// addressed to this collar (or the broadcast ID), apply every known parameter
// generically and queue an ACK/NACK back to the base, echoing the command's
// message_id so the base can match it. Runs inside TaskLoRa, so it may queue a
// TxReq but must NEVER touch the radio directly.
static void handleInboundLoRa(const char *json)
{
  Serial.println(String("[RX] received ") + json);
  DEBUG_PRINTLN(String("[RX] ") + json);

  StaticJsonDocument<320> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err)
  {
    // Can't address a response without a parseable message — drop it.
    Serial.printf("[RX] drop: invalid_json (%s)\n", err.c_str());
    return;
  }

  const char *type = doc["type"] | "";
  const char *legacyCmd = doc["cmd"] | "";
  if (strcmp(type, "CMD") != 0 && strcmp(type, "command") != 0 &&
      legacyCmd[0] == '\0')
  {
    // Overheard telemetry / ACK / presence from elsewhere — ignore quietly.
    DEBUG_PRINTF("[RX] ignore type '%s'\n", type);
    return;
  }

  uint16_t dest = doc["dst"] | (uint16_t)(doc["destination_id"] | (uint16_t)(doc["device_id"] | 0));
  const char *legacyDevice = doc["device"] | "";
  if (dest == 0 && legacyDevice[0] != '\0' &&
      (strcmp(legacyDevice, g_senderName) == 0 || strcmp(legacyDevice, "broadcast") == 0))
  {
    dest = (strcmp(legacyDevice, "broadcast") == 0) ? BROADCAST_ID : DEVICE_ID_INT;
  }
  if (dest != DEVICE_ID_INT && dest != BROADCAST_ID)
  {
    Serial.printf("[CMD] not-for-me (dest=%u, me=%u)\n", dest, DEVICE_ID_INT);
    return;
  }

  uint16_t src = doc["src"] | (uint16_t)(doc["source_id"] | BASE_ID);
  uint32_t msgId = doc["mid"] | (uint32_t)(doc["message_id"] | (uint32_t)(doc["msg_id"] | 0));
  Serial.printf("[CMD] for-me dst=%u src=%u mid=%lu\n",
                dest, src, (unsigned long)msgId);
  // Stamp the time so loop()'s post-TX listen window extends to cover the
  // reply round-trip + any follow-up command in the same burst.
  g_lastCmdRxMs = millis();

  // ── PING → immediate solicited telemetry reply ───────────────────────
  // No separate ping/pong subsystem: a ping just makes us emit a normal
  // telemetry packet from the collar's CURRENT values (fresh GPS if locked this
  // wake, otherwise the retained fix explicitly marked "last"), tagged
  // pong:true + echoing the ping's mid so the base can match it. Queued
  // isTelemetry=false so it never triggers sleep. No ACK and no apply loop.
  if (doc["ping"].is<bool>() || doc["ping"].is<int>() || strcmp(legacyCmd, "ping") == 0)
  {
    StaticJsonDocument<320> pong;
    bool retainedFix = false;
    pong["type"] = "tel";
    pong["src"] = DEVICE_ID_INT;
    pong["dst"] = src;
    pong["seq"] = g_msgCounter++;
    pong["name"] = (const char *)g_senderName;
    pong["md"] = modeToWire(g_currentMode);
    pong["pong"] = true; // solicited-response marker
    pong["mid"] = msgId; // echo the ping's id
    if (gps.location.isValid())
    {
      pong["st"] = "roam";
      pong["lat"] = gps.location.lat();
      pong["lon"] = gps.location.lng();
      if (gps.date.isValid() && gps.time.isValid())
      {
        pong["time"] = gpsToUnixTime(
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
      }
    }
    else if (g_lastGpsValid)
    {
      pong["st"] = "last";
      pong["lat"] = g_lastGpsLat;
      pong["lon"] = g_lastGpsLon;
      retainedFix = true;
      if (g_lastGpsUnixTime != 0)
        pong["time"] = g_lastGpsUnixTime;
    }
    else
    {
      pong["st"] = "invalidGPSLoc"; // awake but no fix yet this wake
    }
    // Satellite count only describes the current GPS receiver state. It would
    // be misleading beside a retained fix from an earlier wake.
    if (isDevMode() && !retainedFix)
      pong["sats"] = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;

    TxReq out{};
    out.isTelemetry = false;
    serializeJson(pong, out.json, sizeof(out.json));
    if (xQueueSend(txReqQ, &out, pdMS_TO_TICKS(100)) == pdTRUE)
      Serial.printf("[PONG] solicited telemetry queued: %s\n", out.json);
    else
      Serial.println("[PONG] queue full — reply dropped");
    return;
  }

  // Normalize the previous command shape into the compact generic parameter.
  if (strcmp(legacyCmd, "mode") == 0 && doc["profile"].is<const char *>())
    doc["md"] = doc["profile"].as<const char *>();

  if (doc["md"].is<const char *>())
  {
    const char *wireMode = doc["md"].as<const char *>();
    const char *mode = modeFromWire(wireMode);
    if (mode != wireMode)
      doc["md"] = mode;
  }

  // ── Generic apply loop ───────────────────────────────────────────────
  // Walk every key/value; skip envelope fields; apply each known config
  // parameter. The FIRST failure short-circuits to a NACK. An unknown key is
  // unsupported_command; a command carrying NO config key at all is likewise
  // unsupported (nothing to do). Phase-1 known key set = { name }.
  const char *nackReason = nullptr;
  int appliedCount = 0;

  for (JsonPair kv : doc.as<JsonObject>())
  {
    const char *key = kv.key().c_str();
    if (isEnvelopeKey(key))
      continue;

    if (strcmp(key, "name") == 0)
    {
      const char *newName = kv.value().is<const char *>() ? kv.value().as<const char *>() : nullptr;
      if (!newName || !bpValidSenderName(newName))
      {
        Serial.printf("[CFG] reject name '%s' -> invalid_value\n",
                      newName ? newName : "<non-string>");
        nackReason = "invalid_value";
        break;
      }
      // Validation already passed, so a false return here is a PERSISTENCE
      // failure (writeBlobVerified prints [CFG] save FAILED) → invalid_parameter.
      if (!bpSaveSenderName(newName, g_senderName, sizeof(g_senderName), g_nvs))
      {
        Serial.println("[CFG] name write FAILED -> invalid_parameter");
        nackReason = "invalid_parameter";
        break;
      }
      Serial.printf("[CFG] name applied + persisted: '%s'\n", g_senderName);
      appliedCount++;
    }
    else if (strcmp(key, "md") == 0)
    {
      // Change the operating/power profile (md = mode, short wire key).
      const char *prof = kv.value().is<const char *>() ? kv.value().as<const char *>() : nullptr;
      if (!bpValidProfile(prof))
      {
        Serial.printf("[CFG] reject profile '%s' -> invalid_value\n", prof ? prof : "<non-string>");
        nackReason = "invalid_value";
        break;
      }
      // Persist to NVS + update g_currentMode/g_activeMode (survives deep sleep).
      // The new sleep interval applies on THIS wake's sleep; LED-beacon/heartbeat
      // behaviour applies next wake. Update the radio's TX power immediately so
      // the ACK + any further TX this wake go out at the new mode's power (we are
      // inside TaskLoRa, the radio owner, so this is safe).
      saveOperatingMode(prof);
      lora.setOutputPower(g_activeMode->lora_power_dbm);
      Serial.printf("[CFG] profile applied + persisted: '%s' (%d dBm, sleep %ds)\n",
                    g_currentMode, g_activeMode->lora_power_dbm, g_activeMode->sleep_interval_s);
      appliedCount++;
    }
    else
    {
      Serial.printf("[CMD] unsupported parameter '%s'\n", key);
      nackReason = "unsupported_command";
      break;
    }
  }

  if (!nackReason && appliedCount == 0)
    nackReason = "unsupported_command"; // command had no recognised parameter

  // ── Build the response (ACK on success, NACK on failure) — SHORT keys ─
  StaticJsonDocument<256> resp;
  resp["type"] = nackReason ? "nack" : "ack";
  resp["src"] = DEVICE_ID_INT;       // source_id
  resp["dst"] = src;                 // destination_id (the base)
  resp["mid"] = msgId;               // echo the command's message_id for matching
  if (nackReason)
    resp["reason"] = nackReason;
  else
  {
    // Echo the current name + mode so the base sees exactly what was applied
    // (and can confirm a rename or a profile change from one ACK).
    resp["name"] = (const char *)g_senderName;
    resp["md"] = modeToWire(g_currentMode);
  }

  TxReq out{};
  out.isTelemetry = false; // response packet — must NOT trigger deep sleep
  serializeJson(resp, out.json, sizeof(out.json));
  if (xQueueSend(txReqQ, &out, pdMS_TO_TICKS(100)) != pdTRUE)
  {
    Serial.println("[ACK] queue full — response dropped");
    return;
  }
  Serial.printf("[%s] queued: %s\n", nackReason ? "NACK" : "ACK", out.json);
  DEBUG_PRINTLN(String(nackReason ? "[NACK] " : "[ACK] ") + out.json);
}

// ─────────────────────────────────────────────
// Task: LoRa owner
//  • Receives commands from the base station (DIO1 ISR → handleInboundLoRa)
//  • Transmits telemetry + presence + ACK/NACK with mode-based power
//  • Sole owner of the radio — all RX/TX calls live here
// ─────────────────────────────────────────────
void TaskLoRa(void *)
{
  // Initialize LoRa radio (sole owner) - use mode-based power
  Serial.println("[LoRa] Initializing radio...");
  int s = lora.begin(LORA_FREQ_MHZ);
  if (s != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[LORA] init failed (%d)\n", s);
    vTaskDelete(nullptr);
    return;
  }

  // Use power from active mode
  lora.setOutputPower(g_activeMode->lora_power_dbm);
  lora.setSpreadingFactor(LORA_SF);
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setCodingRate(LORA_CR);
  lora.setCRC(LORA_USE_CRC);
  lora.setPreambleLength(LORA_PREAMBLE);
  lora.setSyncWord(LORA_SYNC_WORD);

  Serial.printf("[LORA] configured (SF%d, BW%.0f, PWR%d dBm)\n",
                LORA_SF, LORA_BW_KHZ, g_activeMode->lora_power_dbm);
  DEBUG_PRINTF("[LORA] SF%d BW%.0f P%d\n", LORA_SF, LORA_BW_KHZ, g_activeMode->lora_power_dbm);

  // OTAP: listen for inbound commands for the WHOLE wake. SX1262 RX is
  // interrupt-driven via DIO1; the loop polls g_loraRxFlag and re-arms RX after
  // every received packet and every TX so the collar never goes deaf mid-wake.
  lora.setDio1Action(onLoRaDio1);
  lora.startReceive();
  Serial.println("[LoRa] RX armed (listening for commands)");
  DEBUG_PRINTLN("[LoRa] RX armed");

  for (;;)
  {
    // ─────────────────────────────────────────────
    // PRIORITY 2: LED beacon pulse in lost mode
    // ─────────────────────────────────────────────
    if (g_activeMode->led_beacon_mode)
    {
      static uint32_t lastBeacon = 0;
      if (millis() - lastBeacon >= g_activeMode->led_beacon_interval_ms)
      {
        led_beacon_pulse();
        lastBeacon = millis();
      }
    }

    // ─────────────────────────────────────────────
    // PRIORITY 2.5: Inbound command (DIO1 ISR set the flag)
    // ─────────────────────────────────────────────
    if (g_loraRxFlag)
    {
      g_loraRxFlag = false;
      char rxBuf[256];
      size_t len = lora.getPacketLength();
      if (len > sizeof(rxBuf) - 1)
        len = sizeof(rxBuf) - 1;
      int rs = lora.readData((uint8_t *)rxBuf, len);
      rxBuf[len] = '\0'; // NUL-terminate at the true length (buf is 256)
      if (rs == RADIOLIB_ERR_NONE && len > 0)
      {
        handleInboundLoRa(rxBuf);
      }
      else if (rs != RADIOLIB_ERR_NONE)
      {
        Serial.printf("[RX] readData error: %d\n", rs);
      }
      // (len==0 → spurious/empty packet, e.g. a stray IRQ after a TX; ignore.)
      lora.startReceive(); // re-arm to keep listening for the rest of the wake
    }

    // ─────────────────────────────────────────────
    // PRIORITY 3: Handle TX requests
    // ─────────────────────────────────────────────
    TxReq req;
    if (xQueueReceive(txReqQ, &req, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      lora.standby();

#if LBT_ENABLED
      // Listen Before Talk: check if channel is clear
      bool channelClear = false;
      int retryCount = 0;

      while (!channelClear && retryCount < LBT_MAX_RETRIES)
      {
        // Start channel activity detection (CAD)
        int scanResult = lora.scanChannel();

        if (scanResult == RADIOLIB_CHANNEL_FREE)
        {
          channelClear = true;
          Serial.println("[LoRa] LBT: Channel clear, proceeding with TX");
          DEBUG_PRINTLN("[LoRa] LBT: Clear");
        }
        else if (scanResult == RADIOLIB_PREAMBLE_DETECTED)
        {
          retryCount++;
          // Random backoff to avoid synchronized retries from multiple devices
          int delayMs = random(LBT_RETRY_DELAY_MIN_MS, LBT_RETRY_DELAY_MAX_MS);
          Serial.printf("[LoRa] LBT: Channel busy (retry %d/%d), waiting %d ms\n",
                        retryCount, LBT_MAX_RETRIES, delayMs);
          DEBUG_PRINTF("[LoRa] LBT: Busy, retry %d\n", retryCount);
          vTaskDelay(pdMS_TO_TICKS(delayMs));
        }
        else
        {
          // Scan failed - proceed anyway but log it
          Serial.printf("[LoRa] LBT: Scan error (%d), proceeding with TX\n", scanResult);
          DEBUG_PRINTLN("[LoRa] LBT: Error");
          channelClear = true; // Fail-safe: transmit anyway
        }
      }

      if (!channelClear)
      {
        Serial.printf("[LoRa] LBT: Channel still busy after %d retries, transmitting anyway\n",
                      LBT_MAX_RETRIES);
        DEBUG_PRINTLN("[LoRa] LBT: Forced TX");
      }
#endif

      int ts = lora.transmit(req.json);
      if (ts == RADIOLIB_ERR_NONE)
      {
        // LED flicker to indicate successful transmission
        led_flicker();

        Serial.println(String("[LoRa] TX SUCCESS: ") + req.json);
        DEBUG_PRINTLN(String("[TX] ") + req.json);
        Serial.printf("[LoRa] Next msg_id will be: %d\n", g_msgCounter);
        DEBUG_PRINTF("[TX] Next msg_id: %d\n", g_msgCounter);

        // Get RSSI and SNR after transmission
        int16_t rssi = lora.getRSSI();
        float snr = lora.getSNR();

        // Log transmission to CSV file
        logTransmissionToCSV(req.json, rssi, snr);

        // Save to NVS every 10 messages to reduce flash wear
        if (g_msgCounter % 10 == 0)
        {
          prefs.begin("cattracker", false);
          prefs.putUInt("msg_id", g_msgCounter);
          prefs.end();
          Serial.printf("[LoRa] Saved msg_id to NVS: %d\n", g_msgCounter);
        }
      }
      else
      {
        Serial.print("[LoRa] TX error: ");
        Serial.println(ts);
      }

      // Return to RX mode after TX
      lora.startReceive();
      Serial.println("[LoRa] Returned to RX mode");
      DEBUG_PRINTLN("[LoRa] RX resume");

      // Signal "telemetry done → may sleep" ONLY after the radio work for this
      // packet is completely finished (re-arm included). loop() tears the LoRa
      // task down the instant EV_TXDONE is set, so setting it earlier (before
      // the RSSI/CSV/startReceive calls above) risked vTaskDelete killing
      // TaskLoRa mid-SPI on the shared bus. Non-telemetry packets (presence,
      // ACK/NACK) never set it, so the collar keeps listening the whole wake.
      //
      // CRUCIAL: set it even when the transmit FAILED. Otherwise a rejected
      // telemetry packet (e.g. RADIOLIB_ERR_PACKET_TOO_LONG, -4) would leave the
      // collar hung at "Waiting for TX completion" forever, awake and draining
      // the battery. On failure we just lose this cycle's packet and send a
      // fresh one next wake.
      if (req.isTelemetry)
      {
        if (ts != RADIOLIB_ERR_NONE)
          Serial.printf("[LoRa] telemetry TX failed (%d) — sleeping anyway, retry next wake\n", ts);
        xEventGroupSetBits(evBits, EV_TXDONE);
      }
    }

    // Graceful shutdown handshake. loop() raises g_loraStopReq once the post-TX
    // listen window closes. Honour it ONLY at this IDLE point — after RX + any
    // TX for this iteration, radio re-armed, and the TX queue EMPTY (so any
    // pending ACK has already gone out) — and never mid-SPI. Sleep the radio and
    // self-delete so loop() can deep-sleep safely.
    if (g_loraStopReq && uxQueueMessagesWaiting(txReqQ) == 0)
    {
      g_loraSleepResult = putLoRaToColdSleep();
      if (g_loraSleepResult == RADIOLIB_ERR_NONE)
        Serial.println("[LoRa] stop requested — radio in cold sleep, task ending");
      else
        Serial.printf("[LoRa] stop requested — sleep failed (%d), task ending\n",
                      g_loraSleepResult);
      g_loraStopped = true;
      vTaskDelete(nullptr); // never returns
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ─────────────────────────────────────────────
// Task: Power / Orchestrator
//  • New power-saving strategy:
//    1) Wake → BLE scan 10s for 'Home' beacon
//    2) If Home detected → sleep immediately (skip GPS/TX)
//       - After N home cycles (per mode), acquire GPS and TX st:"home" heartbeat
//    3) If no Home → enable GPS, continue BLE scanning during GPS acquisition
//       - If Home appears during GPS → abort TX and sleep
//       - If no Home → transmit with GPS data and st:"roam"
// ─────────────────────────────────────────────
void TaskPower(void *)
{
  // Reset cycle state
  xEventGroupClearBits(evBits, EV_FIX | EV_HOME | EV_TXDONE);

  Serial.println("\n[POWER] === New wake cycle ===");
  DEBUG_PRINTLN("\n[POWER] Wake cycle");
  if (g_forceFullCycleRequested)
    Serial.println("[POWER] Forced full cycle active — button request will bypass home skip");

  // ─────────────────────────────────────────────
  // OTAP presence packet — announce "I'm awake" at the very start of the wake
  // ─────────────────────────────────────────────
  // A tiny, GPS-free packet sent BEFORE BLE/GPS work so the base learns this
  // collar is reachable and can start delivering any QUEUED command into the
  // whole wake window (BLE 10s + GPS 20–60s + stabilise 15s). isTelemetry=false
  // → it does NOT set EV_TXDONE, so it never short-circuits the wake to sleep.
  queuePresencePing("cycle start", g_forceFullCycleRequested || g_buttonWake);

  // ─────────────────────────────────────────────
  // PHASE 1: Initial 10-second BLE + LoRa RX scan
  // ─────────────────────────────────────────────
  Serial.printf("[POWER] Phase 1: BLE + LoRa RX scan for %d seconds...\n", BLE_INITIAL_SCAN_S);
  DEBUG_PRINTF("[POWER] BLE+LoRa scan %ds\n", BLE_INITIAL_SCAN_S);

  uint32_t bleStartTime = millis();
  bool homeDetectedInitial = false;
  uint32_t lastHomePrintTime = 0;

  while (millis() - bleStartTime < (BLE_INITIAL_SCAN_S * 1000))
  {
    EventBits_t bits = xEventGroupGetBits(evBits);

    // Check for BLE home beacon
    if (bits & EV_HOME)
    {
      if (!homeDetectedInitial)
      {
        homeDetectedInitial = true;
        lastHomePrintTime = millis();
        Serial.printf("[POWER] Home beacon detected after %d ms\n", millis() - bleStartTime);
        DEBUG_PRINTLN("[POWER] Home found (initial)");
      }
      else if (millis() - lastHomePrintTime >= 5000)
      {
        lastHomePrintTime = millis();
        Serial.printf("[POWER] Home beacon still present (%d ms elapsed)\n", millis() - bleStartTime);
      }
      // Don't break - continue scanning for LoRa commands
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // ─────────────────────────────────────────────
  // PHASE 2: Priority Decision Logic
  // ─────────────────────────────────────────────

  bool homeHeartbeat = false; // Flag: this cycle is a home heartbeat with GPS

  // A physical button request means "run the full wake/report cycle now".
  // If the home beacon is present, still acquire GPS and report as a home
  // heartbeat instead of taking the low-power home skip.
  if (g_forceFullCycleRequested && homeDetectedInitial)
  {
    Serial.println("[POWER] Button-forced cycle — home beacon present, sending GPS heartbeat anyway");
    DEBUG_PRINTLN("[POWER] Button force overrides home skip");
    homeDetectedInitial = false;
    homeHeartbeat = true;
    g_homeBeaconCycles = 0;
  }

  // Home detected
  // On first boot, skip home shortcut — always acquire GPS and TX so the
  // base station discovers this collar and its initial location.
  if (g_firstBoot && homeDetectedInitial)
  {
    Serial.println("[POWER] First boot — ignoring home beacon, forcing GPS acquisition + TX");
    DEBUG_PRINTLN("[POWER] First boot override");
    homeDetectedInitial = false;
  }

  if (homeDetectedInitial)
  {
    uint8_t heartbeatThreshold = g_activeMode->home_heartbeat_cycles;
    g_homeBeaconCycles++;
    Serial.printf("[POWER] At home (cycle %d/%d)\n", g_homeBeaconCycles, heartbeatThreshold);
    DEBUG_PRINTF("[POWER] Home cycle %d/%d\n", g_homeBeaconCycles, heartbeatThreshold);

    if (g_homeBeaconCycles >= heartbeatThreshold)
    {
      Serial.printf("[POWER] Heartbeat cycle — acquiring GPS for BLEHome location update\n");
      DEBUG_PRINTLN("[POWER] Heartbeat GPS");
      g_homeBeaconCycles = 0;
      homeHeartbeat = true;
      // Fall through to Phase 3 (GPS acquisition) with BLEHome status
    }
    else
    {
      Serial.println("[POWER] Skipping GPS/TX, going to sleep");
      DEBUG_PRINTLN("[POWER] Sleep (home)");

      xEventGroupSetBits(evBits, EV_TXDONE);

      vTaskSuspend(nullptr);
      return;
    }
  }

  // ─────────────────────────────────────────────
  // PHASE 3: Not at home - enable GPS and continue
  // ─────────────────────────────────────────────
  Serial.println("[POWER] No home beacon - starting GPS acquisition");
  DEBUG_PRINTLN("[POWER] GPS start");

  // Reset home beacon counter (device has left home)
  if (!homeHeartbeat && g_homeBeaconCycles > 0)
  {
    Serial.printf("[POWER] Leaving home (was at home for %d cycles)\n", g_homeBeaconCycles);
    DEBUG_PRINTLN("[POWER] Left home");
    g_homeBeaconCycles = 0;
  }

  // Only now energise GPS and launch its parser task. Home-skip cycles returned
  // above without ever powering the module.
  if (!startGpsForAcquisition())
  {
    Serial.println("[POWER] GPS startup failed - telemetry will report invalidGPSLoc");
    DEBUG_PRINTLN("[POWER] GPS startup failed");
  }

  GpsFix fix{};

  // Adaptive GPS acquisition: Cold start vs Warm start
  uint32_t timeout = g_gpsWarmedUp ? GPS_WARM_START_TIMEOUT : GPS_COLD_START_TIMEOUT;
  Serial.printf("[POWER] Waiting for GPS fix (%s start, %d s timeout)...\n",
                g_gpsWarmedUp ? "WARM" : "COLD", timeout / 1000);
  DEBUG_PRINTF("[POWER] GPS %s (%ds)\n", g_gpsWarmedUp ? "warm" : "cold", timeout / 1000);

  uint32_t gpsStartTime = millis();
  int validCount = 0;
  bool gotStableFix = false;
  bool homeDetectedDuringGPS = false;

  // Wait for required number of consecutive valid fixes
  while (millis() - gpsStartTime < timeout)
  {
    EventBits_t bits = xEventGroupGetBits(evBits);

    // Check if home beacon appeared during GPS acquisition
    // (skip on first boot or heartbeat cycle — we must TX regardless)
    if (!g_firstBoot && !homeHeartbeat && !g_forceFullCycleRequested && (bits & EV_HOME))
    {
      homeDetectedDuringGPS = true;
      Serial.printf("[POWER] Home beacon appeared during GPS (after %d ms) - aborting TX\n",
                    millis() - gpsStartTime);
      DEBUG_PRINTLN("[POWER] Home during GPS - abort");
      break;
    }

    // Check for GPS fix
    if (xQueueReceive(gpsFixQ, &fix, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      if (fix.valid)
      {
        validCount++;
        Serial.printf("[POWER] GPS valid fix #%d: %.6f, %.6f\n",
                      validCount, fix.lat, fix.lon);
        DEBUG_PRINTF("[POWER] Fix #%d\n", validCount);

        if (validCount >= GPS_VALID_COUNT_REQUIRED)
        {
          gotStableFix = true;
          Serial.printf("[POWER] GPS lock stable (%d valid fixes)\n", validCount);
          DEBUG_PRINTLN("[POWER] GPS locked");

          // Mark GPS as warmed up for future cycles
          if (!g_gpsWarmedUp)
          {
            g_gpsWarmedUp = true;
            Serial.println("[POWER] GPS warmup complete - future cycles will be faster");
            DEBUG_PRINTLN("[POWER] GPS warmed up");
          }
          break;
        }
      }
      else
      {
        // Reset count if we get an invalid fix
        if (validCount > 0)
        {
          Serial.printf("[POWER] GPS fix lost, resetting count (was %d)\n", validCount);
          validCount = 0;
        }
      }
    }
  }

  // ─────────────────────────────────────────────
  // PHASE 4: Handle GPS result or home abort
  // ─────────────────────────────────────────────
  if (homeDetectedDuringGPS)
  {
    // Home beacon appeared - abort transmission and sleep
    g_homeBeaconCycles = 1; // Start counting home cycles
    Serial.println("[POWER] TX aborted due to home beacon - going to sleep");
    DEBUG_PRINTLN("[POWER] Abort - sleep");

    // Signal sleep without TX
    xEventGroupSetBits(evBits, EV_TXDONE);
    vTaskSuspend(nullptr);
    return;
  }

  if (!gotStableFix)
  {
    Serial.printf("[POWER] GPS timeout after %d ms (valid count: %d/%d)\n",
                  millis() - gpsStartTime, validCount, GPS_VALID_COUNT_REQUIRED);
    DEBUG_PRINTF("[POWER] GPS timeout (%d/%d)\n", validCount, GPS_VALID_COUNT_REQUIRED);
  }

  if (gotStableFix)
  {
    // Stabilization period with continued home beacon check
    if (GPS_STABILISE_MS > 0)
    {
      Serial.printf("[POWER] Stabilizing for %d ms (checking for home beacon)...\n", GPS_STABILISE_MS);
      DEBUG_PRINTF("[POWER] Stabilize %dms\n", GPS_STABILISE_MS);

      uint32_t stabilizeStart = millis();
      while (millis() - stabilizeStart < GPS_STABILISE_MS)
      {
        EventBits_t bits = xEventGroupGetBits(evBits);
        if (!g_firstBoot && !homeHeartbeat && !g_forceFullCycleRequested && (bits & EV_HOME))
        {
          Serial.println("[POWER] Home beacon detected during stabilization - aborting TX");
          DEBUG_PRINTLN("[POWER] Home in stabilize - abort");
          g_homeBeaconCycles = 1;
          xEventGroupSetBits(evBits, EV_TXDONE);
          vTaskSuspend(nullptr);
          return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }

    (void)xQueueReceive(gpsFixQ, &fix, 0); // Get freshest fix

    // Build full JSON with GPS data
    StaticJsonDocument<384> doc;
    // OTAP unified envelope — every packet is self-describing. SHORT wire keys
    // to save airtime/power (the base expands them back to long keys for the UI
    // + logs): src=source/device identity, dst=destination, seq=telemetry ID,
    // st=status and md=mode. GPS time is Unix seconds. The receiver expands
    // these into its descriptive internal/UI fields.
    doc["type"] = "tel";
    doc["src"] = DEVICE_ID_INT;
    doc["dst"] = BASE_ID;
    doc["seq"] = g_msgCounter++;
    doc["name"] = (const char *)g_senderName; // editable friendly label
    doc["st"] = homeHeartbeat ? "home" : "roam";
    doc["md"] = modeToWire(g_currentMode);
    doc["lat"] = fix.lat;
    doc["lon"] = fix.lon;
    if (fix.unixTime != 0)
    {
      doc["time"] = fix.unixTime;
    }
    if (g_firstBoot)
    {
      doc["first_boot"] = true;
    }
    if (g_forceFullCycleRequested)
    {
      doc["btn"] = true;
    }

    // Geofence check — adds "geofence" field and may auto-escalate mode
    const char *gfStatus = checkGeofence(fix.lat, fix.lon);
    if (gfStatus)
    {
      doc["geofence"] = gfStatus;
    }

    // Developer mode: GPS-quality diagnostics in telemetry (kept compact to fit
    // the 255-byte LoRa frame alongside the envelope).
    if (isDevMode())
    {
      doc["sats"] = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
    }

    TxReq req{};
    req.isTelemetry = true;
    serializeJson(doc, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);

    Serial.println("[POWER] GPS fix acquired - queued for transmission");
    if (g_firstBoot) Serial.println("[POWER] First boot discovery packet — base station will see this collar");
    if (isDevMode()) Serial.printf("[DEV] Packet size: %d bytes  Heap: %d\n",
                                   strlen(req.json), ESP.getFreeHeap());
    DEBUG_PRINTLN("[POWER] TX queued");
    g_firstBoot = false;
    g_buttonWake = false;
    g_forceFullCycleRequested = false;
  }
  else
  {
    // GPS timeout - send invalid status (still TX so base station sees us)
    StaticJsonDocument<320> doc;
    // OTAP unified envelope, SHORT wire keys (see the GPS-fix builder).
    doc["type"] = "tel";
    doc["src"] = DEVICE_ID_INT;
    doc["dst"] = BASE_ID;
    doc["seq"] = g_msgCounter++;
    doc["name"] = (const char *)g_senderName; // editable label
    doc["st"] = "invalidGPSLoc";
    doc["md"] = modeToWire(g_currentMode);
    if (g_firstBoot)
    {
      doc["first_boot"] = true;
    }
    if (g_forceFullCycleRequested)
    {
      doc["btn"] = true;
    }
    if (isDevMode())
    {
      doc["sats"] = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
    }

    TxReq req{};
    req.isTelemetry = true;
    serializeJson(doc, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);

    Serial.println("[POWER] GPS timeout - sending invalidGPSLoc");
    if (g_firstBoot) Serial.println("[POWER] First boot — TX anyway so base station discovers this collar");
    if (isDevMode()) Serial.printf("[DEV] Packet size: %d bytes  Sats: %d\n",
                                   strlen(req.json),
                                   gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
    DEBUG_PRINTLN("[POWER] TX invalid");
    g_firstBoot = false;
    g_buttonWake = false;
    g_forceFullCycleRequested = false;
  }

  // Wait for TX completion (EV_TXDONE set by TaskLoRa)
  Serial.println("[POWER] Waiting for TX completion...");
  DEBUG_PRINTLN("[POWER] Wait TX done");

  // Task done - will be deleted by main loop
  vTaskSuspend(nullptr);
}
