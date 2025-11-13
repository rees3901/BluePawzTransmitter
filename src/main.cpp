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
#include "config.h" // Operating modes and shared configuration

// ─────────────────────────────────────────────
// Device Identity (per-node configuration)
// ─────────────────────────────────────────────
// Uncomment ONE device name per tracker
// #define SENDER_ID "Macy"
#define SENDER_ID "Podge"
// #define SENDER_ID "Gizmo"
// #define SENDER_ID "Simba"
// #define SENDER_ID "Carrie"

#define DEVICE_ID_INT 4 // Numeric device ID (read-only, not changeable via remote)

// Debug serial on spare pin (for battery operation monitoring)
#define DEBUG_SERIAL_ENABLED true // Set to false to disable
#define DEBUG_TX_PIN 6            // D5 - Connect to RX of USB-Serial adapter

// Home location (for distance/bearing when available)
const double HOME_LAT = 51.87370573411073;
const double HOME_LON = -2.2396017778476716;

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
};

static QueueHandle_t gpsFixQ;     // latest fix (overwrite)
static QueueHandle_t txReqQ;      // TX requests
static EventGroupHandle_t evBits; // state flags

#define EV_FIX (1 << 0)    // have recent valid GPS fix
#define EV_HOME (1 << 1)   // BLE beacon seen this cycle
#define EV_TXDONE (1 << 2) // LoRa TX finished

// Persisted counters and state across deep sleep (RTC memory - fast but cleared on reset)
RTC_DATA_ATTR uint32_t g_msgCounter = 0;
RTC_DATA_ATTR bool g_gpsWarmedUp = false;        // Tracks if GPS has achieved initial lock
RTC_DATA_ATTR uint8_t g_homeBeaconCycles = 0;    // Count consecutive cycles at home (BLE detected)
RTC_DATA_ATTR char g_currentMode[16] = "normal"; // Current operating mode name
RTC_DATA_ATTR uint32_t g_lostModeStartTime = 0;  // Timestamp when lost mode activated (0 = not in lost mode)

// Current active mode (loaded from NVS/RTC on boot)
const OperatingMode *g_activeMode = &MODE_NORMAL;

// NVS backup for msg counter (Flash memory - survives any reset including USB resets)
Preferences prefs;

// ─────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────
static String cardinalDirection(double bearing)
{
  static const char *dirs[] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int idx = (int)((bearing + 11.25) / 22.5) % 16;
  return String(dirs[idx]);
}

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
  String modeName = prefs.getString("op_mode", "normal");
  prefs.end();

  // Update RTC-backed mode name
  strncpy(g_currentMode, modeName.c_str(), sizeof(g_currentMode) - 1);
  g_currentMode[sizeof(g_currentMode) - 1] = '\0';

  // Set active mode pointer
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

// Check lost mode timeout and auto-revert if needed
static void checkLostModeTimeout()
{
  if (strcmp(g_currentMode, "lost") != 0)
  {
    g_lostModeStartTime = 0; // Not in lost mode
    return;
  }

  // If lost mode start time not set, initialize it
  if (g_lostModeStartTime == 0)
  {
    g_lostModeStartTime = millis() / 1000; // Store in seconds
    Serial.printf("[MODE] Lost mode activated at uptime %d s\n", g_lostModeStartTime);
    DEBUG_PRINTLN("[MODE] Lost mode started");
    return;
  }

  // Check if timeout exceeded
  uint32_t currentTime = millis() / 1000;
  uint32_t elapsedTime = currentTime - g_lostModeStartTime;

  if (elapsedTime >= LOST_MODE_MAX_DURATION_S)
  {
    Serial.printf("[MODE] Lost mode timeout after %d seconds - reverting to %s\n",
                  elapsedTime, LOST_MODE_FALLBACK_MODE);
    DEBUG_PRINTF("[MODE] Lost timeout -> %s\n", LOST_MODE_FALLBACK_MODE);

    saveOperatingMode(LOST_MODE_FALLBACK_MODE);
    g_lostModeStartTime = 0;

    // Send alert to base station
    StaticJsonDocument<128> alert;
    alert["alert"] = "lost_mode_timeout";
    alert["duration_s"] = elapsedTime;
    alert["new_mode"] = LOST_MODE_FALLBACK_MODE;

    TxReq req{};
    serializeJson(alert, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, 0); // Non-blocking queue
  }
}

// Parse and handle mode change command from base station
static bool handleModeCommand(const char *json)
{
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.printf("[RX] JSON parse error: %s\n", error.c_str());
    DEBUG_PRINTLN("[RX] Parse error");
    return false;
  }

  const char *cmd = doc["cmd"];
  if (!cmd)
  {
    Serial.println("[RX] Missing 'cmd' field");
    return false;
  }

  // Handle mode change command
  if (strcmp(cmd, "mode") == 0)
  {
    const char *profile = doc["profile"];
    if (!profile)
    {
      Serial.println("[RX] Missing 'profile' field");
      return false;
    }

    // Validate profile name
    const OperatingMode *newMode = getModeByName(profile);
    if (newMode == nullptr)
    {
      Serial.printf("[RX] Unknown profile: %s\n", profile);
      DEBUG_PRINTF("[RX] Unknown: %s\n", profile);
      return false;
    }

    // Save new mode
    saveOperatingMode(profile);

    // Reset lost mode timer if switching to lost mode
    if (strcmp(profile, "lost") == 0)
    {
      g_lostModeStartTime = 0; // Will be initialized on next wake
    }

    // Send ACK
    StaticJsonDocument<192> ack;
    ack["ack"] = "mode";
    ack["profile"] = profile;
    ack["power"] = newMode->lora_power_dbm;
    ack["sleep"] = newMode->sleep_interval_s;
    ack["device"] = SENDER_ID;

    TxReq req{};
    serializeJson(ack, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);

    Serial.printf("[MODE] Changed to '%s' (ACK queued)\n", profile);
    DEBUG_PRINTF("[MODE] -> %s\n", profile);

    return true;
  }

  // Handle status request
  else if (strcmp(cmd, "get_status") == 0)
  {
    StaticJsonDocument<256> status;
    status["status"] = "ok";
    status["device"] = SENDER_ID;
    status["mode"] = g_currentMode;
    status["power"] = g_activeMode->lora_power_dbm;
    status["sleep"] = g_activeMode->sleep_interval_s;
    status["msg_id"] = g_msgCounter;
    status["gps_warm"] = g_gpsWarmedUp;
    status["home_cycles"] = g_homeBeaconCycles;

    if (g_lostModeStartTime > 0)
    {
      uint32_t elapsed = (millis() / 1000) - g_lostModeStartTime;
      status["lost_mode_s"] = elapsed;
    }

    TxReq req{};
    serializeJson(status, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);

    Serial.println("[RX] Status request - response queued");
    DEBUG_PRINTLN("[RX] Status sent");

    return true;
  }

  Serial.printf("[RX] Unknown command: %s\n", cmd);
  return false;
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

  Serial.println("\n\n[BOOT] CatTracker TX (RTOS)");
  DEBUG_PRINTLN("[BOOT] CatTracker TX (RTOS)");
  Serial.printf("[BOOT] Reset reason: %d\n", esp_reset_reason());
  DEBUG_PRINTF("[BOOT] Reset reason: %d\n", esp_reset_reason());

  // Load persistent counter from NVS (survives all resets)
  prefs.begin("cattracker", false);
  uint32_t nvsCounter = prefs.getUInt("msg_id", 0);

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.printf("[BOOT] ✓ Wake from DEEP SLEEP (RTC msg_id: %d)\n", g_msgCounter);
    DEBUG_PRINTF("[BOOT] Wake from DEEP SLEEP (msg_id: %d)\n", g_msgCounter);
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
    Serial.printf("[BOOT] ✗ POWER-ON RESET (cause: %d)\n", wakeup_reason);
    DEBUG_PRINTF("[BOOT] POWER-ON RESET (cause: %d)\n", wakeup_reason);
    // RTC lost, restore from NVS flash
    g_msgCounter = nvsCounter;
    g_gpsWarmedUp = false;
    Serial.printf("[BOOT] Restored msg_id from NVS flash: %d\n", g_msgCounter);
    DEBUG_PRINTF("[BOOT] msg_id from NVS: %d\n", g_msgCounter);
    break;
  }
  prefs.end();

  // Load operating mode from NVS
  loadOperatingMode();

  // Check lost mode timeout (auto-revert if exceeded)
  checkLostModeTimeout();

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

  // LoRa radio init (will be re-done in TaskLoRa, but SPI setup here)
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  Serial.println("[LORA] SPI ready");

  // BLE init (scanner only)
  BLEDevice::init("");
  Serial.println("[BLE] stack init done");

  // Create tasks
  xTaskCreatePinnedToCore(TaskGPS, "gps", 4096, nullptr, 2, &hGPS, APP_CPU_NUM); // GPS on APP CPU
  xTaskCreatePinnedToCore(TaskBLE, "ble", 4096, nullptr, 1, &hBLE, APP_CPU_NUM); // BLE on APP CPU
  xTaskCreatePinnedToCore(TaskLoRa, "lora", 4096, nullptr, 2, &hLoRa, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(TaskPower, "power", 4096, nullptr, 3, &hPower, PRO_CPU_NUM);

  Serial.println("[BOOT] RTOS tasks started");
}

void loop()
{
  // Wait for Power task to signal cycle complete
  EventBits_t bits = xEventGroupWaitBits(evBits, EV_TXDONE, pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & EV_TXDONE)
  {
    Serial.println("[MAIN] Cycle complete, cleaning up for sleep");

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

    // Power off GPS completely
    digitalWrite(GPS_EN, LOW);
    Serial.println("[SLEEP] GPS power disabled");

    // End GPS serial to release pins
    gpsSerial.end();
    Serial.println("[SLEEP] GPS UART closed");

    // Save msg_id to NVS before sleep (survives USB resets)
    prefs.begin("cattracker", false);
    prefs.putUInt("msg_id", g_msgCounter);
    prefs.end();

    // Hold GPIO states during deep sleep (keeps GPS_EN LOW)
    gpio_hold_en((gpio_num_t)GPS_EN);
    gpio_deep_sleep_hold_en();

    // Get sleep interval from active mode
    uint16_t sleepSeconds = g_activeMode->sleep_interval_s;

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
  uint32_t lastCharTime = millis();
  int charCount = 0;

  for (;;)
  {
    int avail = gpsSerial.available();
    if (avail > 0)
    {
      charCount += avail;
      if (millis() - lastCharTime > 5000)
      {
        Serial.printf("[GPS] Received %d chars in last 5s\n", charCount);
        charCount = 0;
        lastCharTime = millis();
      }
    }

    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      gps.encode(c);
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

      if (fix.valid)
      {
        xEventGroupSetBits(evBits, EV_FIX);
        Serial.printf("[GPS] Valid fix: %.6f, %.6f\n", fix.lat, fix.lon);
        DEBUG_PRINTF("[GPS] Fix: %.6f, %.6f\n", fix.lat, fix.lon);
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
      if (dev.haveName() && String(dev.getName().c_str()) == BEACON_NAME)
      {
        xEventGroupSetBits(evBits, EV_HOME);
        Serial.println("[BLE] 'Home' beacon detected!");
        DEBUG_PRINTLN("[BLE] Home detected");
        break;
      }
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

  // TODO: Enable RX mode for command reception (future enhancement)
  // lora.setDio1Action(rxISR);
  // lora.startReceive();

  for (;;)
  {
    // LED beacon pulse in lost mode
    if (g_activeMode->led_beacon_mode)
    {
      static uint32_t lastBeacon = 0;
      if (millis() - lastBeacon >= g_activeMode->led_beacon_interval_ms)
      {
        led_beacon_pulse();
        lastBeacon = millis();
      }
    }

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
        xEventGroupSetBits(evBits, EV_TXDONE);

        // LED flicker to indicate successful transmission
        led_flicker();

        Serial.println(String("[LoRa] TX SUCCESS: ") + req.json);
        DEBUG_PRINTLN(String("[TX] ") + req.json);
        Serial.printf("[LoRa] Next msg_id will be: %d\n", g_msgCounter);
        DEBUG_PRINTF("[TX] Next msg_id: %d\n", g_msgCounter);

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
      // Return to standby/receive if you later add RX
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ─────────────────────────────────────────────
// Task: Power / Orchestrator
//  • New power-saving strategy:
//    1) Wake → BLE scan 10s for 'Home' beacon
//    2) If Home detected → sleep immediately (skip GPS/TX)
//       - After 5 consecutive home cycles, transmit "BLEHome" status
//    3) If no Home → enable GPS, continue BLE scanning during GPS acquisition
//       - If Home appears during GPS → abort TX and sleep
//       - If no Home → transmit with GPS data and "outanabout" status
// ─────────────────────────────────────────────
void TaskPower(void *)
{
  // Reset cycle state
  xEventGroupClearBits(evBits, EV_FIX | EV_HOME | EV_TXDONE);

  Serial.println("\n[POWER] === New wake cycle ===");
  DEBUG_PRINTLN("\n[POWER] Wake cycle");

  // ─────────────────────────────────────────────
  // PHASE 1: Initial 10-second BLE scan
  // ─────────────────────────────────────────────
  Serial.printf("[POWER] Phase 1: BLE scan for %d seconds...\n", BLE_INITIAL_SCAN_S);
  DEBUG_PRINTF("[POWER] BLE scan %ds\n", BLE_INITIAL_SCAN_S);

  uint32_t bleStartTime = millis();
  bool homeDetectedInitial = false;

  while (millis() - bleStartTime < (BLE_INITIAL_SCAN_S * 1000))
  {
    EventBits_t bits = xEventGroupGetBits(evBits);
    if (bits & EV_HOME)
    {
      homeDetectedInitial = true;
      Serial.printf("[POWER] Home beacon detected after %d ms\n", millis() - bleStartTime);
      DEBUG_PRINTLN("[POWER] Home found (initial)");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // ─────────────────────────────────────────────
  // PHASE 2: Handle home detection
  // ─────────────────────────────────────────────
  if (homeDetectedInitial)
  {
    g_homeBeaconCycles++;
    Serial.printf("[POWER] At home (cycle %d/%d)\n", g_homeBeaconCycles, HOME_SLEEP_CYCLES);
    DEBUG_PRINTF("[POWER] Home cycle %d/%d\n", g_homeBeaconCycles, HOME_SLEEP_CYCLES);

    if (g_homeBeaconCycles >= HOME_SLEEP_CYCLES)
    {
      // 5th cycle at home - transmit with "BLEHome" status
      Serial.println("[POWER] 5th home cycle - transmitting 'BLEHome' status");
      DEBUG_PRINTLN("[POWER] TX BLEHome");

      StaticJsonDocument<320> doc;
      doc["msg_id"] = g_msgCounter++;
      doc["device_id"] = DEVICE_ID_INT;
      doc["id"] = SENDER_ID;
      doc["status"] = "BLEHome";

      TxReq req{};
      serializeJson(doc, req.json, sizeof(req.json));
      xQueueSend(txReqQ, &req, portMAX_DELAY);

      // Reset counter for next home detection sequence
      g_homeBeaconCycles = 0;

      // Wait for TX completion before sleep
      Serial.println("[POWER] Waiting for TX completion...");
    }
    else
    {
      // Cycles 1-4: skip GPS and TX, go straight to sleep
      Serial.println("[POWER] Skipping GPS/TX, going to sleep");
      DEBUG_PRINTLN("[POWER] Sleep (home)");

      // Signal sleep without TX
      xEventGroupSetBits(evBits, EV_TXDONE);
    }

    // Task done - will be deleted by main loop
    vTaskSuspend(nullptr);
    return;
  }

  // ─────────────────────────────────────────────
  // PHASE 3: Not at home - enable GPS and continue
  // ─────────────────────────────────────────────
  Serial.println("[POWER] No home beacon - starting GPS acquisition");
  DEBUG_PRINTLN("[POWER] GPS start");

  // Reset home beacon counter (device has left home)
  if (g_homeBeaconCycles > 0)
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
    // Check if home beacon appeared during GPS acquisition
    EventBits_t bits = xEventGroupGetBits(evBits);
    if (bits & EV_HOME)
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
        if (bits & EV_HOME)
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
    StaticJsonDocument<320> doc;
    doc["msg_id"] = g_msgCounter++;
    doc["device_id"] = DEVICE_ID_INT;
    doc["id"] = SENDER_ID;
    doc["status"] = "outanabout";
    doc["lat"] = fix.lat;
    doc["lon"] = fix.lon;
    if (fix.dateTime[0] != '\0')
    {
      doc["time"] = fix.dateTime;
    }

    // Distance & bearing to home
    double dist = TinyGPSPlus::distanceBetween(fix.lat, fix.lon, HOME_LAT, HOME_LON);
    double brng = TinyGPSPlus::courseTo(fix.lat, fix.lon, HOME_LAT, HOME_LON);
    String dir = String((int)brng) + "-" + cardinalDirection(brng);
    doc["dist_m"] = dist;
    doc["bearing"] = dir;

    TxReq req{};
    serializeJson(doc, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);

    Serial.println("[POWER] GPS fix acquired - queued for transmission");
    DEBUG_PRINTLN("[POWER] TX queued");
  }
  else
  {
    // GPS timeout - send invalid status
    StaticJsonDocument<192> doc;
    doc["msg_id"] = g_msgCounter++;
    doc["device_id"] = DEVICE_ID_INT;
    doc["id"] = SENDER_ID;
    doc["status"] = "invalidGPSLoc";

    TxReq req{};
    serializeJson(doc, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);

    Serial.println("[POWER] GPS timeout - sending invalidGPSLoc");
    DEBUG_PRINTLN("[POWER] TX invalid");
  }

  // Wait for TX completion (EV_TXDONE set by TaskLoRa)
  Serial.println("[POWER] Waiting for TX completion...");
  DEBUG_PRINTLN("[POWER] Wait TX done");

  // Task done - will be deleted by main loop
  vTaskSuspend(nullptr);
}
