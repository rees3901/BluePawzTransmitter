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

// ─────────────────────────────────────────────
// Build‑time configuration / constants
// ─────────────────────────────────────────────
#define SENDER_ID "Podge" // Human‑readable name
#define DEVICE_ID_INT 1   // Numeric device id in JSON
#define LORA_FREQ_MHZ 915.0
#define LORA_POWER_DBM 22
#define LORA_SF 8
#define LORA_BW_KHZ 250.0
#define LORA_CR 5 // 4/5
#define LORA_PREAMBLE 16
#define LORA_USE_CRC 1

#define SLEEP_SECONDS 30          // Deep sleep interval after each cycle
#define GPS_STABILISE_MS 10000    // Extra wait after first valid fix
#define GPS_ACQUIRE_TIMEOUT 30000 // Max wait for first valid fix
#define BLE_SCAN_WINDOW_S 3       // Active scan seconds per cycle
#define BEACON_NAME "Home"        // BLE device name that means "at home"

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

#define GPS_RX 0 // D0 → GPS TX
#define GPS_TX 1 // D1 → GPS RX
#define LED_PIN 48

// ─────────────────────────────────────────────
// LoRa / GPS globals
// ─────────────────────────────────────────────
SPIClass LoRaSPI(HSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ─────────────────────────────────────────────
// RTOS primitives: queues + event bits
// ─────────────────────────────────────────────
struct GpsFix
{
  double lat = 0;
  double lon = 0;
  bool valid = false;
  uint32_t unixTime = 0; // TinyGPSPlus time.value()
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

// Persisted counters across deep sleep
RTC_DATA_ATTR uint32_t g_msgCounter = 0;

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
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(50);
  Serial.println("[BOOT] CatTracker TX (RTOS)");

  // Queues & events
  gpsFixQ = xQueueCreate(1, sizeof(GpsFix)); // latest fix (overwrite)  
  txReqQ = xQueueCreate(4, sizeof(TxReq)); // TX requests
  evBits = xEventGroupCreate(); // state flags (EV_FIX, EV_HOME, EV_TXDONE)  

  // GPS UART (independent from USB CDC)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[INIT] GPS UART started");

  // LoRa radio init (will be re-done in TaskLoRa, but SPI setup here)
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  Serial.println("[LORA] SPI ready");

  // BLE init (scanner only)
  BLEDevice::init("");
  Serial.println("[BLE] stack init done");

  // Create tasks
  xTaskCreatePinnedToCore(TaskGPS, "gps", 4096, nullptr, 2, &hGPS, APP_CPU_NUM);  // GPS on APP CPU 
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

    // Enter deep sleep
    Serial.printf("[SLEEP] Deep sleeping for %d s\n", SLEEP_SECONDS);
    Serial.flush();

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
  for (;;)
  {
    while (gpsSerial.available())
    {
      gps.encode(gpsSerial.read());
    }

    // When location updates, refresh state
    if (gps.location.isUpdated())
    {
      fix.lat = gps.location.lat();
      fix.lon = gps.location.lng();
      fix.valid = gps.location.isValid();
      fix.unixTime = gps.time.isValid() ? gps.time.value() : 0;

      if (fix.valid)
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
    if (xQueueReceive(txReqQ, &req, pdMS_TO_TICKS(10)) == pdTRUE) // 
    {
      lora.standby();
      int ts = lora.transmit(req.json);
      if (ts == RADIOLIB_ERR_NONE)
      {
        xEventGroupSetBits(evBits, EV_TXDONE);
        // rapid strobe 5x
        for (int i = 0; i < 5; i++)
        {
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(50));
          digitalWrite(LED_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        Serial.println(String("[LoRa] sent: ") + req.json);
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
      doc["time"] = fix.unixTime;
    }
    TxReq req{};
    serializeJson(doc, req.json, sizeof(req.json));
    xQueueSend(txReqQ, &req, portMAX_DELAY);
  }
  else
  {
    // Wait up to GPS_ACQUIRE_TIMEOUT for first valid fix
    uint32_t tWaitStart = millis();
    bool gotFix = false;
    while (millis() - tWaitStart < GPS_ACQUIRE_TIMEOUT)
    {
      (void)xQueueReceive(gpsFixQ, &fix, pdMS_TO_TICKS(200)); // get latest if any
      if (fix.valid)
      {
        gotFix = true;
        break;
      }
    }

    if (gotFix)
    {
      // Stabilise for additional GPS_STABILISE_MS
      vTaskDelay(pdMS_TO_TICKS(GPS_STABILISE_MS));
      (void)xQueueReceive(gpsFixQ, &fix, 0); // refresh latest

      // Prepare full JSON with range/bearing if possible
      StaticJsonDocument<320> doc;
      doc["msg_id"] = g_msgCounter++;
      doc["device_id"] = DEVICE_ID_INT;
      doc["id"] = SENDER_ID;
      doc["status"] = (xEventGroupGetBits(evBits) & EV_HOME) ? "home" : "outanabout";
      doc["lat"] = fix.lat;
      doc["lon"] = fix.lon;
      doc["time"] = fix.unixTime;
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
