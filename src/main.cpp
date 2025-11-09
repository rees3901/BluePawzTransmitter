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

// ─────────────────────────────────────────────
// Build‑time configuration / constants
// ─────────────────────────────────────────────
#define SENDER_ID "Macy" // Human‑readable name
// #define SENDER_ID "Podge" // Human‑readable name
// #define SENDER_ID "Gizmo" // Human‑readable name
// #define SENDER_ID "Simba" // Human‑readable name
// #define SENDER_ID "Carrie" // Human‑readable name
#define DEVICE_ID_INT 4 // Numeric device id in JSON
#define LORA_FREQ_MHZ 915.0
#define LORA_POWER_DBM 22
#define LORA_SF 8
#define LORA_BW_KHZ 250.0
#define LORA_CR 5 // 4/5
#define LORA_PREAMBLE 16
#define LORA_USE_CRC 1

// LBT (Listen Before Talk) configuration
#define LBT_ENABLED true           // Enable channel activity detection before TX
#define LBT_RSSI_THRESHOLD -100    // dBm - if RSSI > this, channel is busy (-100 = sensitive)
#define LBT_SCAN_TIME_US 5000      // Microseconds to listen (5ms = 5000us)
#define LBT_MAX_RETRIES 5          // Number of retry attempts if channel busy
#define LBT_RETRY_DELAY_MIN_MS 50  // Minimum random delay between retries
#define LBT_RETRY_DELAY_MAX_MS 500 // Maximum random delay between retries

#define SLEEP_SECONDS 30             // Deep sleep interval after each cycle
#define GPS_COLD_START_TIMEOUT 60000 // 60s for initial cold start acquisition
#define GPS_WARM_START_TIMEOUT 20000 // 20s for subsequent warm starts
#define GPS_VALID_COUNT_REQUIRED 5   // Number of consecutive valid fixes to confirm lock
#define GPS_STABILISE_MS 15000       // Extra wait after achieving required valid count (15s)
#define BLE_SCAN_WINDOW_S 3          // Active scan seconds per cycle
#define BEACON_NAME "Home"           // BLE device name that means "at home"

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
RTC_DATA_ATTR bool g_gpsWarmedUp = false; // Tracks if GPS has achieved initial lock

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
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
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

    // Enter deep sleep
    Serial.printf("[SLEEP] Deep sleeping for %d s (msg_id saved: %d)\n", SLEEP_SECONDS, g_msgCounter);
    DEBUG_PRINTF("[SLEEP] Sleeping %ds (msg_id: %d)\n", SLEEP_SECONDS, g_msgCounter);
    Serial.flush();
#if DEBUG_SERIAL_ENABLED
    DebugSerial.flush();
#endif
    delay(100); // Ensure serial buffer is flushed

// Disable USB serial as wakeup source
#ifdef CONFIG_IDF_TARGET_ESP32S3
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#endif

    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
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
// Task: BLE scanner (short active window)
//  • Scans for BEACON_NAME; sets EV_HOME when seen
// ─────────────────────────────────────────────
void TaskBLE(void *)
{
  BLEScan *scan = BLEDevice::getScan();
  scan->setActiveScan(true);

  for (;;)
  {
    // Reset HOME bit each cycle; Power task decides final status
    xEventGroupClearBits(evBits, EV_HOME);

    BLEScanResults res = scan->start(BLE_SCAN_WINDOW_S, false);
    for (int i = 0; i < res.getCount(); i++)
    {
      BLEAdvertisedDevice dev = res.getDevice(i);
      if (dev.haveName() && String(dev.getName().c_str()) == BEACON_NAME)
      {
        xEventGroupSetBits(evBits, EV_HOME);
        break;
      }
    }
    scan->clearResults();

    vTaskDelay(pdMS_TO_TICKS(1000)); // short idle until next cycle
  }
}

// ─────────────────────────────────────────────
// Task: LoRa owner (TX only for now)
//  • Dequeues payloads and transmits
//  • Blinks LED on success
//  • Initializes radio on first run
// ─────────────────────────────────────────────
void TaskLoRa(void *)
{
  // Initialize LoRa radio (sole owner)
  Serial.println("[LoRa] Initializing radio...");
  int s = lora.begin(LORA_FREQ_MHZ);
  if (s != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[LORA] init failed (%d)\n", s);
    vTaskDelete(nullptr); // Kill this task
    return;
  }
  lora.setOutputPower(LORA_POWER_DBM);
  lora.setSpreadingFactor(LORA_SF);
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setCodingRate(LORA_CR);
  lora.setCRC(LORA_USE_CRC);
  lora.setPreambleLength(LORA_PREAMBLE);
  Serial.println("[LORA] configured and ready");

  for (;;)
  {
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
//  • One full cycle:
//    1) Clear state; allow BLE/GPS to work
//    2) If HOME seen → send JSON with status=home (GPS optional)
//    3) Else wait up to 30s for first valid fix; then +10s stabilise
//    4) If still invalid → send status=invalidGPSLoc
//    5) Queue LoRa TX; wait for TX completion
//    6) Signal main loop to initiate sleep
// ─────────────────────────────────────────────
void TaskPower(void *)
{
  // Reset cycle state
  xEventGroupClearBits(evBits, EV_FIX | EV_HOME | EV_TXDONE);

  // Let BLE & GPS run in parallel for a brief moment
  uint32_t tStart = millis();

  // Poll for fast HOME win
  bool homeSeen = false;
  while (millis() - tStart < 500)
  {
    EventBits_t b = xEventGroupGetBits(evBits);
    if (b & EV_HOME)
    {
      homeSeen = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  GpsFix fix{};

  if (homeSeen)
  {
    // Build minimal JSON with home status; GPS optional if already valid
    (void)xQueueReceive(gpsFixQ, &fix, 0);
    StaticJsonDocument<320> doc;
    doc["msg_id"] = g_msgCounter++;
    doc["device_id"] = DEVICE_ID_INT;
    doc["id"] = SENDER_ID;
    doc["status"] = "home";
    if (fix.valid)
    {
      doc["lat"] = fix.lat;
      doc["lon"] = fix.lon;
      if (fix.dateTime[0] != '\0')
      {
        doc["time"] = fix.dateTime;
      }
    }
    TxReq req{};
    serializeJson(doc, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);
  }
  else
  {
    // Adaptive GPS acquisition: Cold start (first time) vs Warm start (subsequent)
    uint32_t timeout = g_gpsWarmedUp ? GPS_WARM_START_TIMEOUT : GPS_COLD_START_TIMEOUT;
    Serial.printf("[POWER] Waiting for GPS fix (%s start, %d s timeout)...\n",
                  g_gpsWarmedUp ? "WARM" : "COLD", timeout / 1000);

    uint32_t tWaitStart = millis();
    int validCount = 0;
    bool gotStableFix = false;

    // Wait for required number of consecutive valid fixes
    while (millis() - tWaitStart < timeout)
    {
      if (xQueueReceive(gpsFixQ, &fix, pdMS_TO_TICKS(200)) == pdTRUE)
      {
        if (fix.valid)
        {
          validCount++;
          Serial.printf("[POWER] GPS valid fix #%d: %.6f, %.6f\n",
                        validCount, fix.lat, fix.lon);

          if (validCount >= GPS_VALID_COUNT_REQUIRED)
          {
            gotStableFix = true;
            Serial.printf("[POWER] GPS lock stable (%d valid fixes)\n", validCount);

            // Mark GPS as warmed up for future cycles
            if (!g_gpsWarmedUp)
            {
              g_gpsWarmedUp = true;
              Serial.println("[POWER] GPS warmup complete - future cycles will be faster");
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

    if (!gotStableFix)
    {
      Serial.printf("[POWER] GPS timeout after %d ms (valid count: %d/%d)\n",
                    millis() - tWaitStart, validCount, GPS_VALID_COUNT_REQUIRED);
    }

    bool gotFix = gotStableFix;
    if (gotFix)
    {
      // Short stabilization period, then get freshest fix
      if (GPS_STABILISE_MS > 0)
      {
        Serial.printf("[POWER] Stabilizing for %d ms...\n", GPS_STABILISE_MS);
        vTaskDelay(pdMS_TO_TICKS(GPS_STABILISE_MS));
      }
      (void)xQueueReceive(gpsFixQ, &fix, 0); // refresh latest

      // Prepare full JSON with range/bearing if possible
      StaticJsonDocument<320> doc;
      doc["msg_id"] = g_msgCounter++;
      doc["device_id"] = DEVICE_ID_INT;
      doc["id"] = SENDER_ID;
      doc["status"] = (xEventGroupGetBits(evBits) & EV_HOME) ? "home" : "outanabout";
      doc["lat"] = fix.lat;
      doc["lon"] = fix.lon;
      if (fix.dateTime[0] != '\0')
      {
        doc["time"] = fix.dateTime;
      }
      // Distance & bearing (TinyGPSPlus helpers are static)
      double dist = TinyGPSPlus::distanceBetween(fix.lat, fix.lon, HOME_LAT, HOME_LON);
      double brng = TinyGPSPlus::courseTo(fix.lat, fix.lon, HOME_LAT, HOME_LON);
      String dir = String((int)brng) + "-" + cardinalDirection(brng);
      doc["dist_m"] = dist;
      doc["bearing"] = dir;
      TxReq req{};
      serializeJson(doc, req.json, sizeof(req.json));
      xQueueSend(txReqQ, &req, portMAX_DELAY);
    }
    else
    {
      // Timeout → send invalid status
      StaticJsonDocument<192> doc;
      doc["msg_id"] = g_msgCounter++;
      doc["device_id"] = DEVICE_ID_INT;
      doc["id"] = SENDER_ID;
      doc["status"] = "invalidGPSLoc";
      TxReq req{};
      serializeJson(doc, req.json, sizeof(req.json));
      xQueueSend(txReqQ, &req, portMAX_DELAY);
    }
  }

  // Wait for TX completion (EV_TXDONE set by TaskLoRa)
  // Main loop() will handle the actual sleep entry
  Serial.println("[POWER] Cycle complete, waiting for TX");

  // Task is done - it will be deleted by main loop
  vTaskSuspend(nullptr);
}
