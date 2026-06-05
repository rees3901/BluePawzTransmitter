/*
  ┌─────────────────────────────────────────────────────────────┐
  │  CAT TRACKER TX (ESP32S3 + FreeRTOS)                       │
  │  SX1262 LoRa + TinyGPSPlus + BLE "Home" beacon             │
  │  Behaviour: wake → GPS/ BLE → build JSON → LoRa TX → sleep │
  │  JSON fields: msg_id, device_id, id, status, lat/lon/time… │
  └─────────────────────────────────────────────────────────────┘

  High‑level RTOS design
  ───────────────────────
  • TaskGPS  : reads UART1, feeds TinyGPSPlus, publishes latest fix.
  • TaskBLE  : short active scan window; sets Event bit when beacon "Home" seen.
  • TaskLoRa : sole owner of RadioLib; sends payloads queued by the Power task; handles RX later.
  • TaskPower: orchestrates one acquisition/decision cycle then enters sleep.

  Sleep policy
  ────────────
  • Uses deep sleep via timer wake (default 30 s). Adjust SLEEP_SECONDS below.
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
// producing a stable 3-digit number (100–999) unique to each physical chip.
// No manual programming needed — flash the same firmware on any collar and
// it gets its own ID automatically.
// The human-friendly NAME ("Podge", "Macy", etc.) is a runtime value
// (g_senderName) stored in NVS and changed at any time via the `set_name`
// command from the receiver. Default if NVS is empty: "Device-<id>".
static uint16_t DEVICE_ID_INT = 0;

static uint8_t g_mac[6];

static void initDeviceId()
{
  esp_efuse_mac_get_default(g_mac);
  uint32_t hash = g_mac[0];
  for (int i = 1; i < 6; i++)
    hash = hash * 31 + g_mac[i];
  DEVICE_ID_INT = 100 + (hash % 900); // 100–999
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
//     (and shows up as set_name ok:false over the air) instead of vanishing.
//
// The hardware-independent core (name_store / cmd_inbound) still talks to the
// generic INvs get/put-string interface; on the device each "key" maps to a
// field of the master-config blob. The native sandbox backs the same interface
// with an in-RAM map, so the tested command logic is unchanged.
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
#define GPS_EN 1 // D2 → GPS Enable (HIGH = ON)
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
  char dateTime[24] = ""; // Format: "YYYY-MM-DD HH:MM:SS"
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
#define EV_LORA_CMD (1 << 3) // LoRa command received (HIGHEST PRIORITY)

// Persisted counters and state across deep sleep (RTC memory - fast but cleared on reset)
RTC_DATA_ATTR uint32_t g_msgCounter = 0;
RTC_DATA_ATTR bool g_gpsWarmedUp = false;        // Tracks if GPS has achieved initial lock
RTC_DATA_ATTR uint8_t g_homeBeaconCycles = 0;    // Count consecutive cycles at home (BLE detected)
RTC_DATA_ATTR char g_currentMode[16] = "developer"; // Current operating mode name
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

// Developer mode helper — returns true when the active mode is "developer"
static inline bool isDevMode()
{
  return strcmp(g_currentMode, "developer") == 0;
}

// Current active mode (loaded from NVS/RTC on boot)
const OperatingMode *g_activeMode = &MODE_DEVELOPER;

// NVS backup for msg counter (Flash memory - survives any reset including USB resets)
Preferences prefs;

// LoRa RX interrupt flag
volatile bool rxFlag = false;

static void saveOperatingMode(const char *modeName);

// ─────────────────────────────────────────────
// Non-blocking double-press button detection
// ─────────────────────────────────────────────
// State machine polled from active wait loops throughout the wake cycle.
// Detects: press → release → press → release within DEV_MODE_DOUBLE_PRESS_MS.
static enum { BTN_IDLE, BTN_WAIT_RELEASE1, BTN_WAIT_PRESS2, BTN_WAIT_RELEASE2 } g_btnState = BTN_IDLE;
static uint32_t g_btnTimestamp = 0;

static void pollButtonToggle()
{
  bool pressed = (digitalRead(DEV_MODE_BUTTON_PIN) == LOW);
  uint32_t now = millis();

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
    if (!pressed)
    {
      g_btnState = BTN_WAIT_PRESS2;
      g_btnTimestamp = now;
    }
    else if (now - g_btnTimestamp > 2000)
    {
      g_btnState = BTN_IDLE; // held too long, reset
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

      int flashes = isDevMode() ? 5 : 2;
      for (int i = 0; i < flashes; i++)
      {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
      }

      g_btnState = BTN_IDLE;
    }
    break;
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

static const uint8_t L76K_LED_RECOVER[] = {
    0xBA, 0xCE, 0x10, 0x00, 0x06, 0x03, 0x40,
    0x42, 0x0F, 0x00, 0xA0, 0x86, 0x01, 0x00,
    0x03,
    0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00,
    0xF3,
    0xC8, 0x17, 0x08};

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

// LoRa RX interrupt handler (ISR - keep minimal!)
void IRAM_ATTR setRxFlag(void)
{
  rxFlag = true;
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

  // Get current time from GPS if available, otherwise use uptime
  const char *timestamp = doc["time"] | "";
  if (strlen(timestamp) == 0)
  {
    snprintf(csvLine, sizeof(csvLine), "%lu,", millis() / 1000); // Uptime in seconds
  }
  else
  {
    snprintf(csvLine, sizeof(csvLine), "%s,", timestamp);
  }

  // Add remaining fields
  char temp[200];
  snprintf(temp, sizeof(temp), "%u,%s,%s,%s,%.6f,%.6f,%.1f,%s,%.2f,%d,%.1f",
           doc["msg_id"] | 0,
           doc["name"] | (const char *)g_senderName,
           g_currentMode,
           doc["status"] | "unknown",
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

  Serial.printf("[CSV] Logged: msg_id=%u, status=%s\n",
                doc["msg_id"] | 0, doc["status"] | "?");
  DEBUG_PRINTF("[CSV] Log: %u\n", doc["msg_id"] | 0);

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

// ─────────────────────────────────────────────
// Task forward declarations (defined later)
// ─────────────────────────────────────────────
void TaskGPS(void *);
void TaskBLE(void *);
void TaskLoRa(void *);
void TaskPower(void *);

// ─────────────────────────────────────────────
// Setup & Main loop
// ─────────────────────────────────────────────
void setup()
{
  // Check wake reason BEFORE Serial init
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  // Release GPIO hold from deep sleep
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)GPS_EN);
  gpio_hold_dis((gpio_num_t)GPS_TX);
  gpio_hold_dis((gpio_num_t)LORA_NSS);
  gpio_hold_dis((gpio_num_t)LORA_RST);

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

  // Load persistent counter from NVS (survives all resets)
  prefs.begin("cattracker", false);
  uint32_t nvsCounter = prefs.getUInt("msg_id", 0);

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.printf("[BOOT] Wake from DEEP SLEEP (RTC msg_id: %d)\n", g_msgCounter);
    DEBUG_PRINTF("[BOOT] Wake from DEEP SLEEP (msg_id: %d)\n", g_msgCounter);
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

  // Button state machine reset (polled continuously during wake cycle)
  g_btnState = BTN_IDLE;

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

  // GPS Enable and UART
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH); // Power on GPS
  Serial.println("[INIT] GPS power enabled");
  DEBUG_PRINTLN("[INIT] GPS power enabled");
  delay(500); // Let GPS module fully power up and stabilize

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Flush any stale data in UART buffer from previous session
  delay(100);
  while (gpsSerial.available())
  {
    gpsSerial.read();
  }
  Serial.println("[INIT] GPS UART started and buffer flushed");
  DEBUG_PRINTLN("[INIT] GPS UART ready");

  // Restore L76K LED to 1PPS blink mode (it turns solid-on during sleep)
  gpsSerial.write(L76K_LED_RECOVER, sizeof(L76K_LED_RECOVER));
  Serial.println("[INIT] L76K LED restored to 1PPS mode");
  DEBUG_PRINTLN("[INIT] L76K LED on");

  // LoRa radio init (will be re-done in TaskLoRa, but SPI setup here)
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  Serial.println("[LORA] SPI ready");

  // BLE init (scanner only) — suppress noisy GAP event warnings
  esp_log_level_set("BLEScan", ESP_LOG_ERROR);
  BLEDevice::init("");
  Serial.println("[BLE] stack init done");

  // Create tasks
  xTaskCreatePinnedToCore(TaskGPS, "gps", 4096, nullptr, 2, &hGPS, APP_CPU_NUM); // GPS on APP CPU
  xTaskCreatePinnedToCore(TaskBLE, "ble", 4096, nullptr, 1, &hBLE, APP_CPU_NUM); // BLE on APP CPU
  xTaskCreatePinnedToCore(TaskLoRa, "lora", 4096, nullptr, 2, &hLoRa, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(TaskPower, "power", 4096, nullptr, 3, &hPower, PRO_CPU_NUM);

  Serial.println("[BOOT] RTOS tasks started");
}

// Post-TX RX window: how long to keep TaskLoRa alive after telemetry has gone
// out, so the receiver can opportunistically push queued commands now that it
// knows the collar is awake. Class-A LoRaWAN-style pattern.
#define POST_TX_LISTEN_MS 20000U // Base window
#define POST_TX_EXTEND_MS 3000U  // Extension per command received (so bursts land)

void loop()
{
  // Wait for Power task to signal cycle complete
  EventBits_t bits = xEventGroupWaitBits(evBits, EV_TXDONE, pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & EV_TXDONE)
  {
    // V3.8.0: remote commands removed — the collar is TRANSMIT-ONLY now. Once
    // telemetry has gone out there is nothing to listen for, so go straight to
    // sleep cleanup (no post-TX RX window, no command handling).
    Serial.println("[MAIN] TX done — cycle complete, cleaning up for sleep");

    // Delete all tasks
    if (hGPS)
    {
      vTaskDelete(hGPS);
      hGPS = nullptr;
    }
    if (hBLE)
    {
      vTaskDelete(hBLE);
      hBLE = nullptr;
    }
    if (hLoRa)
    {
      vTaskDelete(hLoRa);
      hLoRa = nullptr;
    }
    if (hPower)
    {
      vTaskDelete(hPower);
      hPower = nullptr;
    }

    // Deinitialize BLE to save power
    BLEDevice::deinit(true);

    // Put SX1262 into sleep mode (~0.2 µA vs ~4.5 mA in RX)
    lora.sleep();
    Serial.println("[SLEEP] SX1262 radio sleeping");

    // Turn off L76K LED before sleep
    // Send the command twice with generous delays — the L76K needs time
    // to process the proprietary binary frame at 9600 baud.
    gpsSerial.write(L76K_LED_OFF, sizeof(L76K_LED_OFF));
    delay(200);
    gpsSerial.write(L76K_LED_OFF, sizeof(L76K_LED_OFF));
    delay(200);
    Serial.println("[SLEEP] L76K LED off command sent");
    DEBUG_PRINTLN("[SLEEP] L76K LED off");

    // Power off GPS module
    digitalWrite(GPS_EN, LOW);
    Serial.println("[SLEEP] GPS power disabled");

    // End GPS serial to release pins
    gpsSerial.end();

    // Hold UART TX pin LOW during sleep — prevents floating pin from
    // backfeeding current through L76K ESD diodes and lighting the LED
    pinMode(GPS_TX, OUTPUT);
    digitalWrite(GPS_TX, LOW);

    Serial.println("[SLEEP] GPS UART closed, TX held LOW");

    // Save persistent state to NVS before sleep (survives USB resets)
    prefs.begin("cattracker", false);
    prefs.putUInt("msg_id", g_msgCounter);
    prefs.putUChar("home_cycles", g_homeBeaconCycles);
    prefs.end();

    // Hold GPIO states during deep sleep
    gpio_hold_en((gpio_num_t)GPS_EN);       // Keep GPS_EN LOW (GPS off)
    gpio_hold_en((gpio_num_t)GPS_TX);       // Keep UART TX LOW (no backfeed to L76K)
    gpio_hold_en((gpio_num_t)LORA_NSS);     // Keep NSS HIGH (SX1262 deselected)
    gpio_hold_en((gpio_num_t)LORA_RST);     // Keep RST HIGH (SX1262 not in reset)
    gpio_deep_sleep_hold_en();

    // Get sleep interval from active mode
    uint16_t sleepSeconds = g_activeMode->sleep_interval_s;

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

// Disable USB serial as wakeup source
#ifdef CONFIG_IDF_TARGET_ESP32S3
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#endif

    esp_sleep_enable_timer_wakeup((uint64_t)sleepSeconds * 1000000ULL);
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

      // Format date/time as human-readable string
      if (gps.date.isValid() && gps.time.isValid())
      {
        snprintf(fix.dateTime, sizeof(fix.dateTime),
                 "%04d-%02d-%02d %02d:%02d:%02d",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
      }
      else
      {
        fix.dateTime[0] = '\0'; // Empty if invalid
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
// Task: LoRa owner
//  • Receives mode commands from base station
//  • Transmits packets with mode-based power
//  • Handles RX and TX
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

  // V3.8.0: transmit-only collar — no RX, no command interrupt, no command
  // handling. The radio is used solely to transmit telemetry below.
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
        if (req.isTelemetry)
        {
          xEventGroupSetBits(evBits, EV_TXDONE);
        }

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
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ─────────────────────────────────────────────
// Task: Power / Orchestrator
//  • New power-saving strategy:
//    1) Wake → BLE scan 10s for 'Home' beacon
//    2) If Home detected → sleep immediately (skip GPS/TX)
//       - After N home cycles (per mode), acquire GPS and TX "BLEHome" heartbeat
//    3) If no Home → enable GPS, continue BLE scanning during GPS acquisition
//       - If Home appears during GPS → abort TX and sleep
//       - If no Home → transmit with GPS data and "roaming" status
// ─────────────────────────────────────────────
void TaskPower(void *)
{
  // Reset cycle state
  xEventGroupClearBits(evBits, EV_FIX | EV_HOME | EV_TXDONE);

  Serial.println("\n[POWER] === New wake cycle ===");
  DEBUG_PRINTLN("\n[POWER] Wake cycle");

  // ─────────────────────────────────────────────
  // PHASE 1: Initial 10-second BLE + LoRa RX scan
  // ─────────────────────────────────────────────
  Serial.printf("[POWER] Phase 1: BLE + LoRa RX scan for %d seconds...\n", BLE_INITIAL_SCAN_S);
  DEBUG_PRINTF("[POWER] BLE+LoRa scan %ds\n", BLE_INITIAL_SCAN_S);

  uint32_t bleStartTime = millis();
  bool homeDetectedInitial = false;
  bool loraCommandReceived = false;
  uint32_t lastHomePrintTime = 0;

  while (millis() - bleStartTime < (BLE_INITIAL_SCAN_S * 1000))
  {
    EventBits_t bits = xEventGroupGetBits(evBits);

    // PRIORITY 1: Check for LoRa command (overrides everything)
    if (bits & EV_LORA_CMD)
    {
      loraCommandReceived = true;
      Serial.printf("[POWER] LoRa command received after %d ms - PRIORITY MODE\n", millis() - bleStartTime);
      DEBUG_PRINTLN("[POWER] LoRa CMD priority");
      break; // Exit scan immediately
    }

    // PRIORITY 2: Check for BLE home
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

    pollButtonToggle();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // ─────────────────────────────────────────────
  // PHASE 2: Priority Decision Logic
  // ─────────────────────────────────────────────

  // CASE 1: LoRa command received - apply settings and continue cycle normally
  if (loraCommandReceived)
  {
    Serial.println("[POWER] LoRa command takes precedence - continuing cycle with new settings");
    DEBUG_PRINTLN("[POWER] LoRa CMD applied");

    // Clear the command flag
    xEventGroupClearBits(evBits, EV_LORA_CMD);

    // Settings already applied by handleModeCommand()
    // Continue to GPS acquisition phase (don't sleep even if home detected)
    homeDetectedInitial = false; // Override BLE home detection
  }

  // CASE 2: Home detected (and NO LoRa command)
  // On first boot, skip home shortcut — always acquire GPS and TX so the
  // base station discovers this collar and its initial location.
  if (g_firstBoot && homeDetectedInitial)
  {
    Serial.println("[POWER] First boot — ignoring home beacon, forcing GPS acquisition + TX");
    DEBUG_PRINTLN("[POWER] First boot override");
    homeDetectedInitial = false;
  }

  bool homeHeartbeat = false; // Flag: this cycle is a home heartbeat with GPS

  if (homeDetectedInitial && !loraCommandReceived)
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

  // GPS is already powered on from setup(), just wait for fix
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

    // Check for LoRa commands during GPS acquisition
    if (bits & EV_LORA_CMD)
    {
      xEventGroupClearBits(evBits, EV_LORA_CMD);
      Serial.printf("[POWER] LoRa command received during GPS acquisition (%d ms)\n",
                    millis() - gpsStartTime);
      DEBUG_PRINTLN("[POWER] LoRa CMD during GPS");
    }

    // Check if home beacon appeared during GPS acquisition
    // (skip on first boot or heartbeat cycle — we must TX regardless)
    if (!g_firstBoot && !homeHeartbeat && (bits & EV_HOME))
    {
      homeDetectedDuringGPS = true;
      Serial.printf("[POWER] Home beacon appeared during GPS (after %d ms) - aborting TX\n",
                    millis() - gpsStartTime);
      DEBUG_PRINTLN("[POWER] Home during GPS - abort");
      break;
    }

    pollButtonToggle();

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
        if (!g_firstBoot && !homeHeartbeat && (bits & EV_HOME))
        {
          Serial.println("[POWER] Home beacon detected during stabilization - aborting TX");
          DEBUG_PRINTLN("[POWER] Home in stabilize - abort");
          g_homeBeaconCycles = 1;
          xEventGroupSetBits(evBits, EV_TXDONE);
          vTaskSuspend(nullptr);
          return;
        }
        pollButtonToggle();
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }

    (void)xQueueReceive(gpsFixQ, &fix, 0); // Get freshest fix

    // Build full JSON with GPS data
    StaticJsonDocument<384> doc;
    doc["msg_id"] = g_msgCounter++;
    // V3.6.0 field model: device_id = immutable UID (identity); name =
    // editable friendly label. The legacy ambiguous "id" field is gone.
    doc["device_id"] = DEVICE_ID_INT;
    doc["name"] = (const char *)g_senderName;
    doc["status"] = homeHeartbeat ? "BLEHome" : "roaming";
    doc["mode"] = g_currentMode;
    doc["lat"] = fix.lat;
    doc["lon"] = fix.lon;
    if (fix.dateTime[0] != '\0')
    {
      doc["time"] = fix.dateTime;
    }
    if (g_firstBoot)
    {
      doc["first_boot"] = true;
    }

    // Geofence check — adds "geofence" field and may auto-escalate mode
    const char *gfStatus = checkGeofence(fix.lat, fix.lon);
    if (gfStatus)
    {
      doc["geofence"] = gfStatus;
    }

    // Developer mode: extra diagnostics in telemetry
    if (isDevMode())
    {
      doc["sats"] = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
      doc["hdop"] = gps.hdop.isValid() ? gps.hdop.value() / 100.0 : 99.99;
      doc["heap"] = ESP.getFreeHeap();
      doc["uptime_ms"] = millis();
      doc["fw"] = FIRMWARE_VERSION;
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
  }
  else
  {
    // GPS timeout - send invalid status (still TX so base station sees us)
    StaticJsonDocument<320> doc;
    doc["msg_id"] = g_msgCounter++;
    doc["device_id"] = DEVICE_ID_INT; // immutable UID
    doc["name"] = (const char *)g_senderName; // editable label
    doc["status"] = "invalidGPSLoc";
    doc["mode"] = g_currentMode;
    if (g_firstBoot)
    {
      doc["first_boot"] = true;
    }
    if (isDevMode())
    {
      doc["sats"] = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
      doc["heap"] = ESP.getFreeHeap();
      doc["uptime_ms"] = millis();
      doc["fw"] = FIRMWARE_VERSION;
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
  }

  // Wait for TX completion (EV_TXDONE set by TaskLoRa)
  Serial.println("[POWER] Waiting for TX completion...");
  DEBUG_PRINTLN("[POWER] Wait TX done");

  // Task done - will be deleted by main loop
  vTaskSuspend(nullptr);
}
