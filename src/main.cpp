/*
  ┌──────────────────────────────────────────────┐
  │ 🐾 CAT TRACKER TX — LoRa GPS Collar           │
  │ 📡 SX1262 + TinyGPSPlus + BLE home detection  │
  └──────────────────────────────────────────────┘
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h> // Ensuring latest version compatibility
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFiManager.h>    // For configuration portal
#include <EEPROM.h>         // For storing configuration
#include "secrets.h"        // Wi-Fi credentials
#include <SoftwareSerial.h> // For GPS module communication
// Include web server and filesystem libraries
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#define EEPROM_SIZE 512
#define CONFIG_VERSION "CT1" // Configuration version - change when format changes

// Forward declarations
void saveConfigToEEPROM();
void startConfigPortal();
String cardinalDirection(double bearing);
void colorPrint(const String &message, const char *color = NULL);
void applyProfile(const char *mode);
void applyLoraParams();

// Configuration structure to be stored in EEPROM
struct CatTrackerConfig
{
  char configVersion[4];         // Version of config format
  char senderId[16];             // Cat name/ID (e.g., "Gizmo")
  double homeLat;                // Home latitude
  double homeLon;                // Home longitude
  unsigned long sendInterval;    // LoRa transmission interval (ms)
  unsigned long bleScanInterval; // BLE scan interval (ms)
  unsigned long beaconInterval;  // BLE beacon interval (ms)
  int beaconDuration;            // Seconds to advertise as beacon
  int bleSeenThreshold;          // How many times beacon must be seen to be "home"
  int bleMissedThreshold;        // How many times beacon must be missed to be "away"
  char mode[16];                 // "sleepy", "normal", "lost"
  int loraPower;                 // dBm
  int loraPreamble;              // preamble length
};

// Default configuration
CatTrackerConfig config = {
    CONFIG_VERSION,
    "Simba",             // Default cat name
    51.87370573411073,   // Default HOME_LAT
    -2.2396017778476716, // Default HOME_LON
    60000,               // Default send interval (60 sec)
    120000,              // Default BLE scan interval (120 sec)
    120000,              // Default beacon interval (120 sec)
    3,                   // Default beacon duration (3 sec)
    3,                   // Default seen threshold (3 times)
    5,                   // Default missed threshold (5 times)
    "normal",            // mode
    18,                  // loraPower
    8                    // loraPreamble
};

// Define old globals as references to config values for backward compatibility
#define SENDER_ID (config.senderId)
#define HOME_LAT (config.homeLat)
#define HOME_LON (config.homeLon)

// LoRa Pins
#define LORA_NSS 41
#define LORA_SCK 7
#define LORA_MOSI 9
#define LORA_MISO 8
#define LORA_RST 42
#define LORA_BUSY 40
#define LORA_DIO1 39

// GPS Pins
#define GPS_RX 44        // D7 = GPIO 44
#define GPS_TX 43        // D6 = GPIO 43
#define GPS_BAUD 9600    // Baud rate for GPS module
#define GPS_SLEEP_WAKE 1 // D0 = GPIO 1 (Wake pin for GPS module)
#define GPS_RESET 21     // D10 = GPIO 21 (Reset pin for GPS module)

// #define NODE_ADDRESS 0x01      // This device's address
// #define RECEIVER_ADDRESS 0xFF  // Base station address
// volatile bool messageReceived = false;

#define ANSI_RED "\033[31m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_BLUE "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN "\033[36m"
#define ANSI_WHITE "\033[37m"
#define ANSI_BRIGHT_RED "\033[91m"
#define ANSI_BRIGHT_GREEN "\033[92m"
#define ANSI_BRIGHT_YELLOW "\033[93m"
#define ANSI_BRIGHT_BLUE "\033[94m"
#define ANSI_BRIGHT_MAGENTA "\033[95m"
#define ANSI_BRIGHT_CYAN "\033[96m"
#define ANSI_BRIGHT_WHITE "\033[97m"
#define ANSI_BG_BLUE "\033[44m"
#define ANSI_BOLD "\033[1m"
#define ANSI_RESET "\033[0m"

// Button pin for status report
#define STATUS_BUTTON_PIN 21
bool lastButtonState = HIGH; // Pulled high by default, LOW when pressed

// Flag to detect new serial connections
bool serialConnected = false;
unsigned long lastSerialActivity = 0;
const unsigned long SERIAL_TIMEOUT = 10000; // 10 seconds timeout to detect disconnect

// Configuration flag
bool shouldSaveConfig = false;

// Add this global variable near the other globals:
bool gpsIsAwake = true;

// Add these globals for GPS wake timing
unsigned long gpsWakeLeadTime = 20000; // 20 seconds before LoRa send
unsigned long gpsWakeTime = 0;
bool gpsShouldBeAwake = false;

// Add these globals for manual transmit
bool manualTxRequested = false;
unsigned long manualTxStartTime = 0;
bool manualTxInProgress = false;

// ──────────────────────────────--
// 🛠️ SETUP INITIALISATION
// ──────────────────────────────
void printStatusReport();

void colorPrint(const String &message, const char *color)
{
  if (color != NULL)
  {
    Serial.print(color);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else if (message.indexOf("[ERROR]") >= 0)
  {
    Serial.print(ANSI_RED);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else if (message.indexOf("[OK]") >= 0)
  {
    Serial.print(ANSI_GREEN);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else if (message.indexOf("[STATUS]") >= 0)
  {
    Serial.print(ANSI_BRIGHT_CYAN);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else if (message.indexOf("[BLE]") >= 0)
  {
    Serial.print(ANSI_BLUE);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else if (message.indexOf("[LORA]") >= 0)
  {
    Serial.print(ANSI_MAGENTA);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else if (message.indexOf("[GPS]") >= 0)
  {
    Serial.print(ANSI_YELLOW);
    Serial.print(message);
    Serial.println(ANSI_RESET);
  }
  else
  {
    Serial.println(message);
  }
}

SPIClass LoRaSPI(HSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;
// Define GPS serial on UART1
SoftwareSerial gpsSerial1(GPS_RX, GPS_TX);

const unsigned long sendInterval = 60000;
const unsigned long bleScanInterval = 120000; // 2 minutes (120000ms) for BLE scan
unsigned long lastSendTime = 0;
unsigned long lastBleScanTime = 0;
unsigned long lastStatusPrint = 0;
bool isHome = false;
// Update global variables from config
// Variable for the name to advertise when beaconing
String advertisedBeaconName = SENDER_ID;          // Use cat name/ID for beaconing
String SearchingForBeaconName = "CAT_TRACKER_HQ"; // Name of the beacon to look for; not transmitted
// String beaconAddress = "00:00:00:00:00:00"; // MAC address of the beacon to look for
BLEScan *pBLEScan;

const int BLE_SCAN_DURATION = 7; // BLE scan duration in seconds
bool bleScanning = false;
unsigned long bleScanStartTime = 0;

// Remove these global variables as they conflict with config struct
// const unsigned long beaconInterval = 120000; // 2 minutes between beaconing
// const int BEACON_DURATION = 3; // Seconds to advertise as a beacon
bool isBeaconing = false;
unsigned long beaconStartTime = 0;
BLEAdvertising *pAdvertising = nullptr; // Pointer to BLEAdvertising object

// Forward declarations
void setupBLEAdvertising();
void startBeaconing();
void stopBeaconing();

// Global server instance
AsyncWebServer server(80); // Create a web server on port 80

// API Handler for getting status
void handleGetStatus(AsyncWebServerRequest *request)
{
  JsonDocument doc;

  // Basic info
  doc["id"] = config.senderId;
  doc["uptime"] = millis() / 1000; // Convert to seconds
  doc["isHome"] = isHome;
  doc["lastSendTime"] = lastSendTime / 1000;
  doc["chipId"] = (uint32_t)(ESP.getEfuseMac() >> 32);
  doc["freeHeap"] = ESP.getFreeHeap();

  // GPS data
  if (gps.location.isValid())
  {
    doc["gpsValid"] = true;
    doc["lat"] = gps.location.lat();
    doc["lon"] = gps.location.lng();
    doc["satellites"] = gps.satellites.value();
    doc["satsSeen"] = gps.satellites.value();
    // Calculate distance and direction from home
    double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), config.homeLat, config.homeLon);
    double bearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), config.homeLat, config.homeLon);
    doc["distance"] = (int)dist;
    doc["direction"] = cardinalDirection(bearing);
  }
  else
  {
    doc["gpsValid"] = false;
    doc["satsSeen"] = 0;
  }

  // BLE info
  doc["bleMode"] = bleScanning ? "scanning" : (isBeaconing ? "beaconing" : "idle");
  doc["beaconName"] = advertisedBeaconName ? advertisedBeaconName : beaconName; // Use cat name/ID for beaconing
  ;
  doc["beaconSeenCount"] = isHome ? config.bleSeenThreshold : 0; // Simplified - would need to expose counter from class

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// API Handler for getting configuration
void handleGetConfig(AsyncWebServerRequest *request)
{
  JsonDocument doc;

  doc["senderId"] = config.senderId;
  doc["homeLat"] = config.homeLat;
  doc["homeLon"] = config.homeLon;
  doc["sendInterval"] = config.sendInterval;
  doc["bleScanInterval"] = config.bleScanInterval;
  doc["beaconInterval"] = config.beaconInterval;
  doc["beaconDuration"] = config.beaconDuration;
  doc["beaconName"] = advertisedBeaconName;
  doc["bleSeenThreshold"] = config.bleSeenThreshold;
  doc["bleMissedThreshold"] = config.bleMissedThreshold;
  doc["mode"] = config.mode;
  doc["loraPower"] = config.loraPower;
  doc["loraPreamble"] = config.loraPreamble;

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// API Handler for updating configuration
void handleUpdateConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  // Store data in temporary buffer for processing
  static uint8_t *jsonBuffer = NULL;
  static size_t jsonBufferLength = 0;
  static unsigned long bufferTimestamp = 0;

  // Check if there's a stale buffer (from a previous incomplete request)
  unsigned long now = millis();
  if (jsonBuffer != NULL && now - bufferTimestamp > 30000)
  {
    // Free the stale buffer if it's been more than 30 seconds
    free(jsonBuffer);
    jsonBuffer = NULL;
    jsonBufferLength = 0;
    colorPrint("[API] Freed stale JSON buffer", ANSI_YELLOW);
  }

  if (index == 0)
  {
    // If this is the first packet, allocate the buffer
    if (jsonBuffer != NULL)
    {
      free(jsonBuffer); // Free any previous buffer to prevent memory leaks
    }
    jsonBuffer = (uint8_t *)malloc(total);
    jsonBufferLength = 0;
    bufferTimestamp = now;
  }

  // Ensure buffer exists before copying data
  if (jsonBuffer == NULL)
  {
    request->send(500, "application/json", "{\"success\":false,\"message\":\"Internal buffer error\"}");
    return;
  }

  // Copy data to buffer
  memcpy(jsonBuffer + jsonBufferLength, data, len);
  jsonBufferLength += len;
  bufferTimestamp = now; // Update timestamp with each packet

  if (index + len == total)
  {
    // If this is the last packet, process the complete JSON
    String jsonStr = String((char *)jsonBuffer, jsonBufferLength);
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);

    if (!error)
    {
      // Update config with new values - use modern ArduinoJson syntax
      if (doc["mode"].is<const char *>())
      {
        applyProfile(doc["mode"]);
      }
      if (doc["senderId"].is<const char *>())
      {
        strncpy(config.senderId, doc["senderId"], sizeof(config.senderId) - 1);
      }
      if (doc["homeLat"].is<double>())
      {
        config.homeLat = doc["homeLat"];
      }
      if (doc["homeLon"].is<double>())
      {
        config.homeLon = doc["homeLon"];
      }
      if (doc["sendInterval"].is<unsigned long>())
      {
        config.sendInterval = doc["sendInterval"];
      }
      if (doc["bleScanInterval"].is<unsigned long>())
      {
        config.bleScanInterval = doc["bleScanInterval"];
      }
      if (doc["beaconInterval"].is<unsigned long>())
      {
        config.beaconInterval = doc["beaconInterval"];
      }
      if (doc["beaconDuration"].is<int>())
      {
        config.beaconDuration = doc["beaconDuration"];
      }
      if (doc["beaconName"].is<const char *>())
      {
        advertisedBeaconName = doc["beaconName"].as<String>();
      }
      if (doc["bleSeenThreshold"].is<int>())
      {
        config.bleSeenThreshold = doc["bleSeenThreshold"];
      }
      if (doc["bleMissedThreshold"].is<int>())
      {
        config.bleMissedThreshold = doc["bleMissedThreshold"];
      }
      if (doc["loraPower"].is<int>())
      {
        config.loraPower = doc["loraPower"];
      }
      if (doc["loraPreamble"].is<int>())
      {
        config.loraPreamble = doc["loraPreamble"];
      }
      // Apply LoRa parameters after config update
      applyLoraParams();
      // Save to EEPROM
      saveConfigToEEPROM();
      // Send success response
      request->send(200, "application/json", "{\"success\":true}");
    }
    else
    {
      // Error parsing JSON
      request->send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON\"}");
    }

    // Free the buffer
    free(jsonBuffer);
    jsonBuffer = NULL;
    jsonBufferLength = 0;
  }
}

// API Handler for resetting configuration
void handleResetConfig(AsyncWebServerRequest *request)
{
  // Reset to default configuration
  strncpy(config.configVersion, CONFIG_VERSION, sizeof(config.configVersion));
  strncpy(config.senderId, "Gizmo", sizeof(config.senderId));
  config.homeLat = 51.87370573411073;
  config.homeLon = -2.2396017778476716;
  config.sendInterval = 60000;
  config.bleScanInterval = 120000;
  config.beaconInterval = 120000;
  config.beaconDuration = 3;
  config.bleSeenThreshold = 3;
  config.bleMissedThreshold = 5;

  // Update global variables
  SearchingForBeaconName = "CAT_TRACKER_HQ";

  // Save to EEPROM
  saveConfigToEEPROM();

  // Send success response
  request->send(200, "application/json", "{\"success\":true}");
}

// API Handler for device info
void handleGetInfo(AsyncWebServerRequest *request)
{
  JsonDocument doc;

  doc["id"] = config.senderId;
  doc["uptime"] = millis() / 1000; // Convert to seconds

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// Handler for restart
void handleRestart(AsyncWebServerRequest *request)
{
  request->send(200, "text/html", "<html><body><h1>Restarting device...</h1><script>setTimeout(function() { window.location.href = '/'; }, 5000);</script></body></html>");

  // Schedule a restart after we send the response
  delay(500);
  ESP.restart();
}

// Initialize LittleFS and web server
void setupWebServer()
{
  // Initialize LittleFS
  if (!LittleFS.begin(true))
  {
    colorPrint("[ERROR] Failed to mount LittleFS", ANSI_RED);
    return;
  }

  colorPrint("[INIT] LittleFS mounted successfully", ANSI_GREEN);

  // List all files in LittleFS
  File root = LittleFS.open("/");
  File file = root.openNextFile();

  colorPrint("[INIT] Files in LittleFS:", ANSI_BRIGHT_CYAN);
  while (file)
  {
    colorPrint("  - " + String(file.name()) + " (" + String(file.size()) + " bytes)", ANSI_BRIGHT_WHITE);
    file = root.openNextFile();
  }

  // ESP32-S3 specific workaround - initialize WiFi before AsyncWebServer
  WiFi.mode(WIFI_AP);
  WiFi.softAP("CatTrackerSetup", "cattracker");
  delay(100); // Give it time to initialize

  IPAddress IP = WiFi.softAPIP();
  colorPrint("[INIT] AP IP address: " + IP.toString(), ANSI_BRIGHT_CYAN);

  // Set up web server routes
  // Serve static files
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // API endpoints
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleUpdateConfig);
  server.on("/api/reset", HTTP_POST, handleResetConfig);
  server.on("/api/info", HTTP_GET, handleGetInfo);
  server.on("/restart", HTTP_GET, handleRestart);

  // Redirect to main page for WiFi setup
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    request->send(200, "text/html", "<html><body><h1>Starting WiFi Setup...</h1><script>setTimeout(function() { window.location.href = '192.168.4.1'; }, 1000);</script></body></html>");
    // Schedule WiFi setup after sending the response
    WiFi.disconnect();
    delay(500);
    startConfigPortal(); });

  // 404 handler
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(404, "text/html", "<html><body><h1>404 Not Found</h1><p>The requested URL was not found on this server.</p><p><a href='/'>Go to Home</a></p></body></html>"); });

  // Start server with a small delay to ensure everything is ready
  delay(100);
  server.begin();
  colorPrint("[INIT] Web server started on port 80", ANSI_GREEN);
}

// ──────────────────────────────
// 📡 BLE SCANNER CALLBACK CLASS
// ──────────────────────────────
class BeaconScanner : public BLEAdvertisedDeviceCallbacks
{
private:
  int beaconSeenCount = 0;   // Counter for consecutive beacon detections
  int beaconMissedCount = 0; // Counter for consecutive missed scans

public:
  void onResult(BLEAdvertisedDevice advertisedDevice) override
  {
    // Create a complete device report string first
    String deviceReport = "[BLE] Device: ";

    // Always include address (all devices have this)
    deviceReport += advertisedDevice.getAddress().toString().c_str();

    // Add name if available
    if (advertisedDevice.haveName())
    {
      String advName = advertisedDevice.getName().c_str();
      deviceReport += " Name: \"" + advName + "\"";

      // Check if this is our target beacon
      if (advName == SearchingForBeaconName)
      {
        beaconSeenCount++;
        beaconMissedCount = 0; // Reset missed count if beacon is seen

        // Add this info to our report
        deviceReport += " [HOME BEACON] Seen count: " + String(beaconSeenCount);

        // Use config value for threshold instead of hardcoded value
        if (beaconSeenCount >= config.bleSeenThreshold)
        {
          isHome = true;
          deviceReport += " - HOME DETECTED!";
        }
      }
    }

    // Add signal strength (RSSI)
    deviceReport += " RSSI: " + String(advertisedDevice.getRSSI()) + "dBm";

    // Print the complete device report in one go
    Serial.println(deviceReport);
  }

  void onScanComplete(BLEScanResults scanResults)
  {
    // Use config value for threshold instead of hardcoded value
    if (beaconSeenCount < config.bleSeenThreshold)
    {
      beaconMissedCount++;
      Serial.println("[BLE] Home Beacon not seen in this scan. Missed count: " + String(beaconMissedCount));
      // Use config value for threshold instead of hardcoded value
      if (beaconMissedCount >= config.bleMissedThreshold)
      {
        isHome = false;
        beaconSeenCount = 0; // Reset seen count when leaving home
        Serial.println("[BLE] Beacon missed " + String(config.bleMissedThreshold) + " times in a row. Not at home anymore.");
      }
    }
  }
};

// ──────────────────────────────
// 🧭 BEARING TO CARDINAL DIRECTION
// ──────────────────────────────
String cardinalDirection(double bearing)
{
  const char *directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                              "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int index = (int)((bearing + 11.25) / 22.5) % 16;
  return String(directions[index]);
}

// ──────────────────────────────
// 📥 LORA RECEIVE CALLBACK HANDLER
// ──────────────────────────────
/*
void onReceive()
{
  messageReceived = true;  // Set flag only, don't handle message in ISR
}

void handleIncomingMessage()
{
  if (!messageReceived)
    return;

  String incoming;
  int state = lora.readData(incoming);
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.print("[LORA RX] Received: ");
    Serial.println(incoming);
  }
  messageReceived = false;
  lora.startReceive(); // Resume receiving
}
*/

// ──────────────────────────────
// 🛠️ BLE ADVERTISING SETUP
// ──────────────────────────────
void setupBLEAdvertising()
{
  colorPrint("[INIT] Setting up BLE advertising...");
  pAdvertising = BLEDevice::getAdvertising();

  BLEAdvertisementData advData;
  advData.setName(SENDER_ID);
  advData.setFlags(0x06); // BR_EDR_NOT_SUPPORTED | LE General Discoverable Mode

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(advData);

  // Make the advertising less aggressive to save power
  pAdvertising->setMinInterval(0x100); // 160ms
  pAdvertising->setMaxInterval(0x200); // 320ms

  colorPrint("[OK] BLE advertising setup complete");
}

// ──────────────────────────────
// 📊 STATUS REPORT FUNCTION
// ──────────────────────────────
void printStatusReport()
{
  unsigned long uptime = millis() / 1000; // Convert to seconds
  unsigned long days = uptime / (24 * 60 * 60);
  uptime %= (24 * 60 * 60);
  unsigned long hours = uptime / (60 * 60);
  uptime %= (60 * 60);
  unsigned long minutes = uptime / 60;
  unsigned long seconds = uptime % 60;

  // Line separators for visual clarity
  String separator = "────────────────────────────────────────────";

  Serial.println();
  Serial.print(ANSI_BG_BLUE);
  Serial.print(ANSI_BOLD);
  Serial.print(" CAT TRACKER STATUS REPORT ");
  Serial.println(ANSI_RESET);
  Serial.println(separator);

  // Cat and location status
  Serial.print(ANSI_BRIGHT_CYAN);
  Serial.print("🐱 ");
  Serial.print(SENDER_ID);
  Serial.print(" is ");
  if (isHome)
  {
    Serial.print(ANSI_BRIGHT_GREEN);
    Serial.print("HOME");
  }
  else
  {
    Serial.print(ANSI_BRIGHT_YELLOW);
    Serial.print("outnabout");
  }
  Serial.println(ANSI_RESET);

  // GPS Status
  Serial.print(ANSI_BRIGHT_CYAN);
  Serial.print("📍 GPS: ");
  if (gps.location.isValid())
  {
    Serial.print(ANSI_BRIGHT_GREEN);
    Serial.print("FIX OK");
    Serial.print(ANSI_RESET);
    Serial.print(" (");
    Serial.print(gps.satellites.value());
    Serial.print(" sats, Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Lon: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print(")");
  }
  else
  {
    Serial.print(ANSI_BRIGHT_RED);
    Serial.print("NO FIX");
  }
  Serial.println(ANSI_RESET);

  // Distance from home
  if (gps.location.isValid())
  {
    double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    double bearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    Serial.print(ANSI_BRIGHT_CYAN);
    Serial.print("🏠 Distance: ");
    Serial.print(ANSI_BRIGHT_WHITE);
    Serial.print(dist);
    Serial.print("m ");
    Serial.print(cardinalDirection(bearing));
    Serial.println(ANSI_RESET);
  }

  // System status
  Serial.print(ANSI_BRIGHT_CYAN);
  Serial.print("⏱️ Uptime: ");
  Serial.print(ANSI_BRIGHT_WHITE);
  if (days > 0)
  {
    Serial.print(days);
    Serial.print("d ");
  }
  Serial.print(hours);
  Serial.print("h ");
  Serial.print(minutes);
  Serial.print("m ");
  Serial.print(seconds);
  Serial.println("s");

  // LoRa status
  Serial.print(ANSI_BRIGHT_CYAN);
  Serial.print("📻 LoRa: ");
  Serial.print(ANSI_BRIGHT_MAGENTA);
  Serial.print("TX every ");
  Serial.print(sendInterval / 1000);
  Serial.print("s (last ");
  Serial.print((millis() - lastSendTime) / 1000);
  Serial.print("s ago)");
  Serial.println(ANSI_RESET);

  // BLE status
  Serial.print(ANSI_BRIGHT_CYAN);
  Serial.print("📱 BLE: ");
  if (bleScanning)
  {
    Serial.print(ANSI_BRIGHT_BLUE);
    Serial.print("SCANNING");
  }
  else if (isBeaconing)
  {
    Serial.print(ANSI_BRIGHT_GREEN);
    Serial.print("BEACONING as '");
    Serial.print(SENDER_ID);
    Serial.print("'");
  }
  else
  {
    Serial.print(ANSI_YELLOW);
    Serial.print("IDLE");
  }
  Serial.println(ANSI_RESET);

  Serial.println(separator);
  Serial.println();
}

// ──────────────────────────────
// 💾 CONFIG SAVE & LOAD FUNCTIONS
// ──────────────────────────────
void saveConfigToEEPROM()
{
  EEPROM.put(0, config);
  EEPROM.commit();
  colorPrint("[CONFIG] Settings saved to EEPROM", ANSI_GREEN);
}

bool loadConfigFromEEPROM()
{
  CatTrackerConfig storedConfig;
  EEPROM.get(0, storedConfig);

  // Check if the config version matches
  if (String(storedConfig.configVersion) == String(CONFIG_VERSION))
  {
    config = storedConfig;
    colorPrint("[CONFIG] Valid settings loaded from EEPROM", ANSI_GREEN);
    return true;
  }

  colorPrint("[CONFIG] No valid config in EEPROM, using defaults", ANSI_YELLOW);
  return false;
}

// Callback notifying us of the need to save config
void saveConfigCallback()
{
  colorPrint("[CONFIG] Configuration changed - will save", ANSI_GREEN);
  shouldSaveConfig = true;
}

// Profile application function
void applyProfile(const char *mode)
{
  if (strcmp(mode, "sleepy") == 0)
  {
    config.sendInterval = 120000;
    config.bleScanInterval = 240000;
    config.beaconInterval = 240000;
    config.beaconDuration = 2;
    config.loraPower = 10;
    config.loraPreamble = 8;
  }
  else if (strcmp(mode, "normal") == 0)
  {
    config.sendInterval = 60000;
    config.bleScanInterval = 120000;
    config.beaconInterval = 120000;
    config.beaconDuration = 3;
    config.loraPower = 18;
    config.loraPreamble = 8;
  }
  else if (strcmp(mode, "lost") == 0)
  {
    config.sendInterval = 15000;
    config.bleScanInterval = 30000;
    config.beaconInterval = 30000;
    config.beaconDuration = 5;
    config.loraPower = 22;
    config.loraPreamble = 16;
  }
  strncpy(config.mode, mode, sizeof(config.mode) - 1);
}

void applyLoraParams()
{
  lora.setOutputPower(config.loraPower);
  lora.setPreambleLength(config.loraPreamble);
}

// ──────────────────────────────
// ⚙️ WIFI CONFIG PORTAL FUNCTIONS
// ──────────────────────────────
String getParam(String name, String placeholder, String defaultValue, int length)
{
  String html = "<p>";
  html += "<label for='" + name + "'>" + placeholder + "</label>";
  html += "<input id='" + name + "' name='" + name + "' maxlength='" + length + "' value='" + defaultValue + "'>";
  html += "</p>";
  return html;
}

String getNumberParam(String name, String placeholder, String defaultValue, String min, String max)
{
  String html = "<p>";
  html += "<label for='" + name + "'>" + placeholder + "</label>";
  html += "<input type='number' id='" + name + "' name='" + name + "' min='" + min + "' max='" + max + "' value='" + defaultValue + "'>";
  html += "</p>";
  return html;
}

void startConfigPortal()
{
  // Turn on the LED to show we're in configuration mode
  digitalWrite(LED_BUILTIN, HIGH);
  shouldSaveConfig = false;

  WiFiManager wifiManager;

  // Configure WiFiManager
  wifiManager.setTitle("🐱 Cat Tracker Configuration");
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Set timeout for the configuration portal (3 minutes)
  wifiManager.setConfigPortalTimeout(180);

  // ID and Location Parameters
  WiFiManagerParameter custom_html_header_1("<h3>🐾 Cat Identity</h3>");
  WiFiManagerParameter custom_cat_name(getParam("cat_name", "Cat Name", config.senderId, 15).c_str());

  WiFiManagerParameter custom_html_header_2("<h3>🏠 Home Location</h3>");
  WiFiManagerParameter custom_home_lat(getParam("home_lat", "Home Latitude", String(config.homeLat, 8).c_str(), 12).c_str());
  WiFiManagerParameter custom_home_lon(getParam("home_lon", "Home Longitude", String(config.homeLon, 8).c_str(), 12).c_str());

  // Timing Parameters
  WiFiManagerParameter custom_html_header_3("<h3>⏱️ Timing Parameters</h3>");
  WiFiManagerParameter custom_send_interval(getNumberParam("send_interval", "LoRa Send Interval (seconds)", String(config.sendInterval / 1000), "10", "3600").c_str());
  WiFiManagerParameter custom_ble_scan_interval(getNumberParam("ble_scan_interval", "BLE Scan Interval (seconds)", String(config.bleScanInterval / 1000), "10", "3600").c_str());
  WiFiManagerParameter custom_beacon_interval(getNumberParam("beacon_interval", "Beacon Interval (seconds)", String(config.beaconInterval / 1000), "10", "3600").c_str());
  WiFiManagerParameter custom_beacon_duration(getNumberParam("beacon_duration", "Beacon Duration (seconds)", String(config.beaconDuration), "1", "30").c_str());

  // Home Detection Parameters
  WiFiManagerParameter custom_html_header_4("<h3>🏠 Home Detection</h3>");
  WiFiManagerParameter custom_beacon_name(getParam("beacon_name", "Home Beacon Name", SearchingForBeaconName.c_str(), 20).c_str());
  WiFiManagerParameter custom_ble_seen(getNumberParam("ble_seen", "Home Seen Threshold", String(config.bleSeenThreshold), "1", "10").c_str());
  WiFiManagerParameter custom_ble_missed(getNumberParam("ble_missed", "Home Missed Threshold", String(config.bleMissedThreshold), "1", "15").c_str());

  // Add all parameters to WiFiManager
  wifiManager.addParameter(&custom_html_header_1);
  wifiManager.addParameter(&custom_cat_name);

  wifiManager.addParameter(&custom_html_header_2);
  wifiManager.addParameter(&custom_home_lat);
  wifiManager.addParameter(&custom_home_lon);

  wifiManager.addParameter(&custom_html_header_3);
  wifiManager.addParameter(&custom_send_interval);
  wifiManager.addParameter(&custom_ble_scan_interval);
  wifiManager.addParameter(&custom_beacon_interval);
  wifiManager.addParameter(&custom_beacon_duration);

  wifiManager.addParameter(&custom_html_header_4);
  wifiManager.addParameter(&custom_beacon_name);
  wifiManager.addParameter(&custom_ble_seen);
  wifiManager.addParameter(&custom_ble_missed);

  // Start the config portal
  colorPrint("[CONFIG] Starting WiFiManager config portal...", ANSI_BRIGHT_CYAN);
  colorPrint("[CONFIG] Connect to WiFi network: 'CatTrackerSetup' with password: 'cattracker'", ANSI_BRIGHT_GREEN);

  // Use "CatTracker" as the AP name and "cattracker" as the password
  bool portalResult = wifiManager.startConfigPortal("CatTrackerSetup", "cattracker");

  if (portalResult || shouldSaveConfig)
  {
    colorPrint("[CONFIG] Configuration portal completed", ANSI_GREEN);

    // Save the parameters to our config structure
    strncpy(config.senderId, custom_cat_name.getValue(), sizeof(config.senderId) - 1);
    config.homeLat = atof(custom_home_lat.getValue());
    config.homeLon = atof(custom_home_lon.getValue());

    config.sendInterval = atol(custom_send_interval.getValue()) * 1000;
    config.bleScanInterval = atol(custom_ble_scan_interval.getValue()) * 1000;
    config.beaconInterval = atol(custom_beacon_interval.getValue()) * 1000;
    config.beaconDuration = atoi(custom_beacon_duration.getValue());

    SearchingForBeaconName = custom_beacon_name.getValue();
    config.bleSeenThreshold = atoi(custom_ble_seen.getValue());
    config.bleMissedThreshold = atoi(custom_ble_missed.getValue());

    // Save to EEPROM
    saveConfigToEEPROM();

    // Print the new configuration
    colorPrint("[CONFIG] New settings:", ANSI_BRIGHT_CYAN);
    colorPrint("  Cat Name: " + String(config.senderId), ANSI_BRIGHT_WHITE);
    colorPrint("  Home Location: " + String(config.homeLat, 8) + ", " + String(config.homeLon, 8), ANSI_BRIGHT_WHITE);
    colorPrint("  Send Interval: " + String(config.sendInterval / 1000) + "s", ANSI_BRIGHT_WHITE);
    colorPrint("  BLE Scan Interval: " + String(config.bleScanInterval / 1000) + "s", ANSI_BRIGHT_WHITE);
    colorPrint("  Beacon Interval: " + String(config.beaconInterval / 1000) + "s", ANSI_BRIGHT_WHITE);
    colorPrint("  Beacon Duration: " + String(config.beaconDuration) + "s", ANSI_BRIGHT_WHITE);
    colorPrint("  Home Beacon Name: " + SearchingForBeaconName, ANSI_BRIGHT_WHITE);
    colorPrint("  Seen Threshold: " + String(config.bleSeenThreshold), ANSI_BRIGHT_WHITE);
    colorPrint("  Missed Threshold: " + String(config.bleMissedThreshold), ANSI_BRIGHT_WHITE);
  }
  else
  {
    colorPrint("[CONFIG] Configuration portal canceled or timed out", ANSI_YELLOW);
  }

  // Turn off the LED
  digitalWrite(LED_BUILTIN, LOW);

  // Restart the device to apply changes
  colorPrint("[CONFIG] Restarting device to apply changes...", ANSI_BRIGHT_CYAN);
  delay(1000);
  ESP.restart();
}

// ███████╗███████╗████████╗██╗   ██╗██████╗
// ██╔════╝██╔════╝╚══██╔══╝██║   ██║██╔══██╗
// ███████╗█████╗     ██║   ██║   ██║██████╔╝
// ╚════██║██╔══╝     ██║   ██║   ██║██╔═══╝
// ███████║███████╗   ██║   ╚██████╔╝██║
// ╚══════╝╚══════╝   ╚═╝    ╚═════╝ ╚═╝
// =============================================
// 🛠️ DEVICE INITIALIZATION & CONFIGURATION 🛠️
// =============================================

void setup()
{
  Serial.begin(115200);
  delay(50);
  colorPrint("[BOOT] Initialising CAT TRACKER TX...");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);           // Turn off the built-in LED
  pinMode(48, OUTPUT);                      // Simple LED pin setup
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up
  // Set up GPS and LoRa pins
  pinMode(GPS_RESET, OUTPUT);         // Set GPS reset pin as output
  digitalWrite(GPS_RESET, HIGH);      // Set GPS reset pin HIGH to disable reset
  pinMode(GPS_SLEEP_WAKE, OUTPUT);    // Set GPS sleep/wake pin as output
  digitalWrite(GPS_SLEEP_WAKE, HIGH); // Set GPS wake pin as output and HIGH
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Check if button is pressed at startup to enter config mode
  if (digitalRead(STATUS_BUTTON_PIN) == LOW)
  {
    colorPrint("[CONFIG] Button pressed at startup - entering configuration mode", ANSI_BRIGHT_CYAN);
    // Flash LED to indicate config mode
    for (int i = 0; i < 8; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    startConfigPortal();
    // startConfigPortal will restart the ESP when done
    // so execution won't continue past this point
  }

  // Load settings from EEPROM
  loadConfigFromEEPROM();

  //  ┌─────────────────────────────────┐
  //  │ 🛰️  GPS INITIALIZATION          │
  //  └─────────────────────────────────┘
  // Initialize GPS serial first so we can check for fix during warmup
  gpsSerial1.begin(GPS_BAUD);
  delay(100);         // Wait for serial to initialize
  gpsSerial1.flush(); // Now flush any garbage data

  // GPS warm-up period with early exit if fix is obtained
  colorPrint("[GPS] Warming up GPS (max 20 secs) to get first fix...");
  unsigned long gpsWarmupStart = millis();
  bool fixFound = false;

  // Loop for up to 20 seconds or until a valid fix is obtained
  while (millis() - gpsWarmupStart < 20000 && !fixFound)
  {
    // Check for GPS data
    while (gpsSerial1.available() > 0)
    {
      gps.encode(gpsSerial1.read());
    }

    // Check if we have a valid fix
    if (gps.location.isValid())
    {
      fixFound = true;
      colorPrint("[GPS] Valid fix obtained early ✔ Lat: " + String(gps.location.lat(), 6) +
                     ", Lon: " + String(gps.location.lng(), 6),
                 ANSI_BRIGHT_GREEN);
      break;
    }

    // Show progress dots
    Serial.print(".");
    delay(1000);
  }

  if (!fixFound)
  {
    colorPrint("[GPS] Warmup complete without Geetting fix. (still indoors?) Continuing anyway...", ANSI_YELLOW);
  }

  colorPrint("[INIT] GPS initialized");

  //  ┌─────────────────────────────────┐
  //  │ 📡 LORA RADIO INITIALIZATION    │
  //  └─────────────────────────────────┘
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  colorPrint("[INIT] SPI for LoRa initialised");

  colorPrint("[INIT] Starting LoRa...");
  if (lora.begin(915.0) != RADIOLIB_ERR_NONE) // 915.0 MHz for US915 band
  {
    colorPrint("[ERROR] LoRa failed to initialise. Proceeding with caution...");
  }
  else
  {
    colorPrint("[OK] LoRa initialised successfully");
  }

  {
    Serial.println("INIT} setting Lora parameters...");
  }
  lora.setOutputPower(22);
  lora.setSpreadingFactor(8);
  lora.setBandwidth(250.0);
  lora.setCodingRate(5);
  lora.setCRC(true);
  lora.setPreambleLength(8);
  colorPrint("[INIT] LoRa PaRams configured");

  //  ┌─────────────────────────────────┐
  //  │ 📱 BLE SETUP & CONFIGURATION    │
  //  └─────────────────────────────────┘
  colorPrint("[INIT] Starting BLE scan setup...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new BeaconScanner());
  pBLEScan->setActiveScan(true); // Active scan to get more data from devices nearby
  colorPrint("[OK] BLE scanner ready");

  // Initialize BLE advertising setup for beaconing
  setupBLEAdvertising();

  //  ┌─────────────────────────────────┐
  //  │ 🌐 WEB INTERFACE SETUP          │
  //  └─────────────────────────────────┘
  // Initialize LittleFS and web server
  setupWebServer();
  colorPrint("[INIT] Web interface ready at http://192.168.4.1/", ANSI_BRIGHT_GREEN);

  // ════════════════════════════════════════
  // 🚀 SETUP COMPLETE - READY TO TRACK 🐱
  // ════════════════════════════════════════
}

// Add this function to start beaconing
void startBeaconing()
{
  if (!isBeaconing && pAdvertising != nullptr)
  {
    colorPrint("[BLE] Starting to beacon as '" + String(SENDER_ID) + "'");
    pAdvertising->start();
    isBeaconing = true;
    beaconStartTime = millis();
  }
}

// Add this function to stop beaconing
void stopBeaconing() // Function to stop beaconing as a beacon
{
  if (isBeaconing && pAdvertising != nullptr)
  {
    pAdvertising->stop();
    isBeaconing = false;
    colorPrint("[BLE] Stopped beaconing");
  }
}

// GPS power management functions
void gpsWake()
{
  if (!gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, HIGH);
    colorPrint("[GPS] Waking up GPS module", ANSI_YELLOW);
    gpsIsAwake = true;
  }
}

void gpsSleep()
{
  if (gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, LOW);
    colorPrint("[GPS] Putting GPS module to sleep", ANSI_YELLOW);
    gpsIsAwake = false;
  }
}

// GPS power management variables
unsigned long lastGpsActiveTime = 0;  // Last time GPS was actively used
unsigned long homeTimeWithoutGps = 0; // Time spent at home with GPS off
int consecutiveGpsOffCycles = 0;      // Counter for GPS off cycles
bool forcedGpsActivation = false;     // Flag for forced GPS activation

// Function to check if we need GPS data now
bool isGpsNeeded()
{
  unsigned long now = millis();

  // FAILSAFE #1: Forced activation override
  // If GPS has been manually activated (e.g., by button press or periodic check)
  if (forcedGpsActivation)
  {
    static unsigned long forcedActivationTime = now;
    if (now - forcedActivationTime > 60000)
    {                              // 1 minute of forced activation
      forcedGpsActivation = false; // Reset after timeout
    }
    return true;
  }

  // FAILSAFE #2: Periodic mandatory GPS check regardless of home status
  // Extend the interval to 10 minutes as requested
  static unsigned long lastMandatoryCheck = 0;
  if (now - lastMandatoryCheck > 600000)
  { // 10 minutes
    lastMandatoryCheck = now;
    consecutiveGpsOffCycles = 0; // Reset counter when we do a mandatory check
    return true;
  }

  // FAILSAFE #3: Counter-based activation
  // After 5 loops with GPS off, force it on for one cycle
  if (consecutiveGpsOffCycles >= 5)
  {
    consecutiveGpsOffCycles = 0;
    return true;
  }

  // FAILSAFE #4: Time-based maximum - never keep GPS off for more than 30 minutes total
  if (now - lastGpsActiveTime > 1800000)
  { // 30 minutes max GPS off time
    lastGpsActiveTime = now;
    return true;
  }

  // Main logic - optimize for power saving when at home
  if (isHome)
  {
    // We're home, so we can usually save power by leaving GPS off
    homeTimeWithoutGps += (now - lastBleScanTime);

    // FAILSAFE #5: Don't trust "home" status indefinitely
    // If home for more than 2 hours without GPS confirmation, check GPS
    if (homeTimeWithoutGps > 7200000)
    { // 2 hours
      homeTimeWithoutGps = 0;
      return true;
    }

    // Normal home power-saving mode
    consecutiveGpsOffCycles++;
    return false;
  }
  else
  {
    // Not at home, reset home time counter
    homeTimeWithoutGps = 0;
  }

  // Regular logic: Wake GPS before sending data or if no valid fix
  bool needed = (now - lastSendTime > (config.sendInterval - 15000)) ||
                (!gps.location.isValid());

  if (needed)
  {
    lastGpsActiveTime = now;
    consecutiveGpsOffCycles = 0;
  }
  else
  {
    consecutiveGpsOffCycles++;
  }

  return needed;
}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                          █
// █    ███╗   ███╗ █████╗ ██╗███╗   ██╗    ██╗      ██████╗  █
// █    ████╗ ████║██╔══██╗██║████╗  ██║    ██║     ██╔═══██╗ █
// █    ██╔████╔██║███████║██║██╔██╗ ██║    ██║     ██║   ██║ █
// █    ██║╚██╔╝██║██╔══██║██║██║╚██╗██║    ██║     ██║   ██║ █
// █    ██║ ╚═╝ ██║██║  ██║██║██║ ╚████║    ███████╗╚██████╔╝ █
// █    ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝    ╚══════╝ ╚═════╝  █
// █                                                          █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀

// ┌─────────────────────────────────────────────────────────────┐
// │ 🔄 MAIN PROGRAM EXECUTION LOOP - CAT TRACKING OPERATIONS    │
// └─────────────────────────────────────────────────────────────┘
void loop()
{
  unsigned long now = millis();

  // Calculate when to wake GPS before LoRa send
  if ((lastSendTime + config.sendInterval - gpsWakeLeadTime <= now) && !gpsShouldBeAwake)
  {
    gpsWake();
    gpsShouldBeAwake = true;
    gpsWakeTime = now;
  }

  // Only put GPS to sleep after LoRa send
  if (gpsShouldBeAwake && (now - lastSendTime < 1000)) // Just after LoRa send
  {
    gpsSleep();
    gpsShouldBeAwake = false;
  }

  // Check button state for status report and manual transmit
  bool buttonState = digitalRead(STATUS_BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH)
  {
    // Button pressed, print status report
    printStatusReport();
    // Manual transmit request
    manualTxRequested = true;
    manualTxStartTime = now;
    manualTxInProgress = true;
    gpsWake();
    colorPrint("[MANUAL] Manual transmit requested via button press", ANSI_BRIGHT_CYAN);
  }
  lastButtonState = buttonState;

  // Manual transmit logic
  if (manualTxRequested && manualTxInProgress)
  {
    // Wait for GPS warmup (max 20s or until fix)
    bool fixFound = false;
    unsigned long elapsed = now - manualTxStartTime;
    while (gpsSerial1.available() > 0)
    {
      gps.encode(gpsSerial1.read());
    }
    if (gps.location.isValid())
    {
      fixFound = true;
    }
    if (fixFound || elapsed > 20000)
    {
      // Send LoRa packet (same as normal send)
      colorPrint("[MANUAL] Sending manual LoRa packet", ANSI_BRIGHT_CYAN);
      lastSendTime = now; // Update send time to avoid double send
      lora.standby();
      static uint32_t messageId = 0;
      JsonDocument doc;
      doc["msg_id"] = messageId++;
      doc["device_id"] = 4;
      doc["id"] = SENDER_ID;
      double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), config.homeLat, config.homeLon);
      double bearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), config.homeLat, config.homeLon);
      String dir = String((int)bearing) + "-" + cardinalDirection(bearing);
      doc["lat"] = gps.location.lat();
      doc["lon"] = gps.location.lng();
      doc["time"] = gps.time.value();
      doc["dist_m"] = dist;
      doc["bearing"] = dir;
      // Add status with three possible states: home, outanabout, or error (if no valid GPS fix)
      doc["status"] = !gps.location.isValid() ? "error" : (isHome ? "home" : "outanabout");
      String out;
      serializeJson(doc, out);
      colorPrint("Sending: " + out);
      int txState = lora.transmit(out);
      if (txState == RADIOLIB_ERR_NONE)
      {
        Serial.print("[LORA] msg [");
        // Flash the LED five times to visually indicate LoRa packet transmission
        for (int i = 0; i < 5; i++)
        {
          digitalWrite(48, HIGH);
          delay(50);
          digitalWrite(48, LOW);
          delay(50);
        }
        Serial.print(messageId - 1);
        colorPrint("] sent");
      }
      else
      {
        Serial.print(ANSI_RED);
        Serial.print("[LORA] Transmit failed, code: ");
        Serial.print(txState);
        Serial.println(ANSI_RESET);
      }
      doc.clear();

      // Start BLE beaconing
      startBeaconing();
      // Reset manual transmit state
      manualTxRequested = false;
      manualTxInProgress = false;
      // Optionally, put GPS to sleep after manual send
      gpsSleep();
    }
    // else, keep waiting for fix or timeout
  }

  // Process GPS data while available
  while (gpsSerial1.available() > 0)
  {
    gps.encode(gpsSerial1.read()); // Read and decode GPS data from UART
  }

  // 🛰️ Periodic GPS status diagnostics
  static unsigned long lastGpsDataTime = 0;

  while (gpsSerial1.available() > 0)
  {
    gps.encode(gpsSerial1.read());
    lastGpsDataTime = now; // Update timestamp when data is received
  }

  if (now - lastStatusPrint > 60000)
  {
    lastStatusPrint = now;
    if (!gpsIsAwake)
    {
      colorPrint("[GPS] GPS is asleep", ANSI_YELLOW);
    }
    else if (now - lastGpsDataTime > 10000) // Check if no data received in the last 10 seconds
      colorPrint("[GPS] No data on UART", ANSI_YELLOW);
    else if (!gps.location.isValid())
      colorPrint("[GPS] Invalid fix", ANSI_YELLOW);
    else
      colorPrint("[GPS] Valid fix with coordinates", ANSI_GREEN);
  }

  // 📱 BLE scanning logic (non-blocking)
  if (!bleScanning && now - lastBleScanTime > config.bleScanInterval) // Use config value
  {
    // Start a new scan
    colorPrint("[BLE] Starting new scan for beacons...");
    pBLEScan->clearResults();
    pBLEScan->start(BLE_SCAN_DURATION, false); // Start async scan
    bleScanning = true;
    bleScanStartTime = now;
    lastBleScanTime = now;
  }

  // Check if we need to stop scanning (scan duration elapsed)
  if (bleScanning && now - bleScanStartTime > (BLE_SCAN_DURATION * 1000))
  {
    // Scan should be complete by now
    BLEScanResults results = pBLEScan->getResults();

    // Print summary of scan results
    colorPrint("[BLE] Scan complete. Found " + String(results.getCount()) + " devices");

    // Create a static reference to our BeaconScanner instance
    static BeaconScanner *beaconScanner = nullptr;

    // Initialize the scanner if it's the first time
    if (beaconScanner == nullptr)
    {
      beaconScanner = new BeaconScanner();
    }

    // Call onScanComplete directly on our BeaconScanner instance
    if (beaconScanner != nullptr)
    {
      beaconScanner->onScanComplete(results);
    }

    // Mark scan as complete and clean up
    bleScanning = false;
    pBLEScan->clearResults(); // Free memory
  }

  // ┌─────────────────────────────────────────┐
  // │    📡 LORA TRANSMISSION BLOCK 📡       │
  // │                                         │
  // │    ██╗      ██████╗ ██████╗  █████╗    │
  // │    ██║     ██╔═══██╗██╔══██╗██╔══██╗   │
  // │    ██║     ██║   ██║██████╔╝███████║   │
  // │    ██║     ██║   ██║██╔══██╗██╔══██║   │
  // │    ███████╗╚██████╔╝██║  ██║██║  ██║   │
  // │    ╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝   │
  // │                                         │
  // └─────────────────────────────────────────┘
  //
  // Check if it's time to send a LoRa transmission based on configured interval
  if (now - lastSendTime > config.sendInterval) // Use config value
  {
    // Send data regardless of GPS fix status
    // This will transmit even when GPS location is not valid
    lastSendTime = now;
    colorPrint("[LORA] Preparing for transmit...");
    lora.standby(); // Put radio in standby mode before transmission
    static uint32_t messageId = 0;
    JsonDocument doc;
    // Build the JSON message with tracking information
    doc["msg_id"] = messageId++;
    doc["device_id"] = 4;
    doc["id"] = SENDER_ID;
    // Calculate distance and bearing from home coordinates
    double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), config.homeLat, config.homeLon); // Use config values
    double bearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), config.homeLat, config.homeLon);     // Use config values
    String dir = String((int)bearing) + "-" + cardinalDirection(bearing);
    doc["lat"] = gps.location.lat();
    doc["lon"] = gps.location.lng();
    doc["time"] = gps.time.value();
    doc["dist_m"] = dist;
    doc["bearing"] = dir;
    doc["satellite_Count"] = gps.satellites.value(); // Add satellite count to JSON
    // Add status with three possible states: home, outanabout, or error (if no valid GPS fix)
    doc["status"] = !gps.location.isValid() ? "error" : (isHome ? "home" : "outanabout");
    String out;
    serializeJson(doc, out);
    colorPrint("Sending: " + out);
    // Transmit the JSON message via LoRa
    int txState = lora.transmit(out);
    if (txState == RADIOLIB_ERR_NONE)
    {
      Serial.print("[LORA] msg [");
      // Visual indication of transmission - flash LED 5 times
      for (int i = 0; i < 5; i++)
      {
        digitalWrite(48, HIGH);
        delay(50);
        digitalWrite(48, LOW);
        delay(50);
      }
      Serial.print(messageId - 1);
      colorPrint("] sent");
    }
    else
    {
      Serial.print(ANSI_RED);
      Serial.print("[LORA] Transmit failed, code: ");
      Serial.print(txState);
      Serial.println(ANSI_RESET);
    }
    doc.clear(); // Free memory used by the JSON document
  }

  // ┌────────────────────────┐
  // │ 📱 BLE BEACON CONTROL  │
  // └────────────────────────┘

  if (now - lastSendTime < 1000 && !isBeaconing)
  {
    // Start beaconing immediately after LoRa transmission
    startBeaconing();
  }

  // Stop beaconing after configured duration to save power
  if (isBeaconing && now - beaconStartTime > (config.beaconDuration * 1000))
  {
    stopBeaconing();
  }

  // Declare static lastBeaconTime to track beacon timing between loop iterations
  static unsigned long lastBeaconTime = 0;

  // Periodically beacon based on configured interval when not scanning or already beaconing
  if (!isBeaconing && !bleScanning && now - lastBeaconTime > config.beaconInterval)
  {
    startBeaconing();
    lastBeaconTime = now;
  }
} // End of loop()
