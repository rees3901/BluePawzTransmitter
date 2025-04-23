/*
  ┌──────────────────────────────────────────────┐
  │ 🐾 CAT TRACKER TX — LoRa GPS Collar           │
  │ 📡 SX1262 + TinyGPSPlus                       │ // <-- Removed BLE reference
  └──────────────────────────────────────────────┘
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Define LED_BUILTIN if not already defined (common for ESP32)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // Adjust if your board uses a different pin
#endif

// Pin Definitions (Ensure these match your hardware)
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
#define GPS_SLEEP_WAKE 1 // d0 = GPIO 1  - used for sleep/wake control
#define GPS_RESET 9      // D10 = GPIO 9

// Button pin for status report
#define STATUS_BUTTON_PIN 21 //

// Hardcoded Configuration Values (Replaces JSON config)
#define SENDER_ID "Gizmo"
#define HOME_LAT 51.87370573411073
#define HOME_LON -2.2396017778476716
#define SEND_INTERVAL 60000 // milliseconds (60 seconds)
#define LORA_POWER 18       // dBm
#define LORA_PREAMBLE 8

// ANSI Color Codes
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

// Hardware Instances
SPIClass LoRaSPI(HSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;
SoftwareSerial gpsSerial1(GPS_RX, GPS_TX); // Use HardwareSerial if possible for better reliability

// Global State Variables
bool lastButtonState = HIGH;
bool gpsIsAwake = true;                // Assume awake initially after setup
unsigned long gpsWakeLeadTime = 20000; // Reduced lead time (20s) - adjust as needed
unsigned long lastSendTime = 0;
unsigned long lastStatusPrint = 0;
// bool isHome = true;              // Always consider "home" (BLE functionality removed)
static uint32_t messageId = 0;   // Global message counter for LoRa
bool manualTxRequested = false;  // Flag for button press request
bool manualTxInProgress = false; // Flag to indicate manual sequence active

// Forward Declarations
void printStatusReport();
void colorPrint(const String &message, const char *color = ANSI_RESET);
void gpsWake();
void gpsSleep();
String cardinalDirection(double bearing);
void handleManualTransmit();
void handleRegularTransmit();
void processGps();
void checkButton();
void periodicStatusUpdate();
void transmitLora(String payload); // Changed signature
String buildJsonPayload();         // Changed signature
void sendLoraPacket(bool isManual);

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                              █
// █   ███████ ███████ ████████ ██    ██ ██████      ██████  ██    ██ ███    ██  █
// █   ██      ██         ██    ██    ██ ██   ██     ██   ██ ██    ██ ████   ██  █
// █   ███████ █████      ██    ██    ██ ██████      ██████  ██    ██ ██ ██  ██  █
// █        ██ ██         ██    ██    ██ ██          ██   ██ ██    ██ ██  ██ ██  █
// █   ███████ ███████    ██     ██████  ██          ██    ██  ██████  ██   ████  █
// █                                                                              █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
void setup()
{
  Serial.begin(115200);
  delay(50);
  colorPrint("[BOOT] Initialising CAT TRACKER TX...");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(48, OUTPUT); // Status LED
  // pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP);

  // GPS Pin Setup
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH); // Keep GPS out of reset
  pinMode(GPS_SLEEP_WAKE, OUTPUT);
  digitalWrite(GPS_SLEEP_WAKE, HIGH); // Start with GPS awake
  gpsIsAwake = true;

  // GPS Init
  gpsSerial1.begin(GPS_BAUD);
  delay(100);
  colorPrint("[GPS] Warming up GPS...");
  unsigned long gpsWarmupStart = millis();
  bool fixFound = false;
  while (millis() - gpsWarmupStart < 60000 && !fixFound)
  { // 60 sec warmup
    while (gpsSerial1.available() > 0)
    {
      if (gps.encode(gpsSerial1.read()) && gps.location.isValid())
      {
        fixFound = true;
        colorPrint("[GPS] Valid fix obtained early ✔ Lat: " + String(gps.location.lat(), 6) +
                       ", Lon: " + String(gps.location.lng(), 6),
                   ANSI_BRIGHT_GREEN);
        break;
      }
    }
    if (!fixFound)
    {
      Serial.print(".");
      delay(1000);
    }
  }
  if (!fixFound)
  {
    colorPrint("[GPS] Warmup Expired without Getting fix.", ANSI_RED);
  }
  else
  {
    colorPrint("[GPS] Initialized.", ANSI_GREEN);
  }

  // LoRa Init
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  colorPrint("[INIT] SPI for LoRa initialised");
  int initState = lora.begin(915.0); // Use appropriate frequency
  if (initState != RADIOLIB_ERR_NONE)
  {
    colorPrint("[ERROR] LoRa failed to initialise. Code: " + String(initState), ANSI_RED);
  }
  else
  {
    colorPrint("[OK] LoRa initialised successfully");
    // Apply LoRa parameters from hardcoded values
    lora.setOutputPower(LORA_POWER);       // Use defined constant
    lora.setSpreadingFactor(8);            // Sets the spreading factor to 8 (default)
    lora.setBandwidth(250.0);              // sets the bandwidth to 250 kHz (default)
    lora.setCodingRate(5);                 // sets the coding rate to 4/5 (default)
    lora.setCRC(true);                     // sets the CRC to true (default)
    lora.setPreambleLength(LORA_PREAMBLE); // Use defined constant
    colorPrint("[INIT] LoRa Params configured: Power=" + String(LORA_POWER) +
               ", Preamble=" + String(LORA_PREAMBLE)); // Use defined constants
  }

  colorPrint("════════════════════════════════════════", ANSI_BOLD);
  colorPrint("🚀 SETUP COMPLETE - READY TO TRACK 🐱", ANSI_BOLD);
  colorPrint("════════════════════════════════════════", ANSI_BOLD);

  // Set initial lastSendTime to allow first send after interval
  lastSendTime = millis() - SEND_INTERVAL + 5000; // Use defined constant
}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                              █
// █   ███    ███  █████  ██ ███    ██     ██       ██████   ██████  ██████      █
// █   ████  ████ ██   ██ ██ ████   ██     ██      ██    ██ ██    ██ ██   ██     █
// █   ██ ████ ██ ███████ ██ ██ ██  ██     ██      ██    ██ ██    ██ ██████      █
// █   ██  ██  ██ ██   ██ ██ ██  ██ ██     ██      ██    ██ ██    ██ ██          █
// █   ██      ██ ██   ██ ██ ██   ████     ███████  ██████   ██████  ██          █
// █                                                                              █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
void loop()
{
  unsigned long now = millis();

  // --- Core Tasks ---
  checkButton();          // Handle button press for status/manual TX
  processGps();           // Process incoming GPS data if awake
  periodicStatusUpdate(); // Print status periodically

  // --- State Machine Logic ---
  static bool preparingToSend = false; // Flag to manage the send preparation phase

  // 1. Manual Transmit Check (Highest Priority)
  if (manualTxInProgress)
  {
    handleManualTransmit(); // Execute manual transmit sequence
    // handleManualTransmit will reset flags and state when done
    preparingToSend = false; // Ensure regular preparation stops if manual TX occurs
    return;                  // Skip regular cycle checks for this iteration
  }

  // 2. Regular Send Cycle Preparation Trigger
  // Check if it's time to START preparing for the next send
  if (!preparingToSend && (now >= lastSendTime + SEND_INTERVAL - gpsWakeLeadTime)) // Use defined constant
  {
    colorPrint("[CYCLE] Preparing for next send...", ANSI_CYAN);
    preparingToSend = true;

    // Wake GPS
    gpsWake(); // Function handles the check if already awake
  }

  // 3. Regular Send Cycle Execution Trigger
  // Check if the preparation phase is active AND the actual send interval has passed
  if (preparingToSend && (now >= lastSendTime + SEND_INTERVAL)) // Use defined constant
  {
    handleRegularTransmit(); // Execute the regular transmit sequence
    preparingToSend = false; // End the preparation/send cycle state
  }

  // 4. GPS Sleep Logic (Idle State)
  // Put GPS to sleep if it's awake, not preparing/sending, and not manually triggered
  if (gpsIsAwake && !preparingToSend && !manualTxInProgress)
  {
    // Add a small delay after the last send before sleeping
    // This ensures LoRa TX and beacon start have completed
    if (now > lastSendTime + 2000)
    {             // Wait 2 seconds after last send
      gpsSleep(); // Function handles the check if already asleep
    }
  }

} // End of loop()

// ──────────────────────────────
// │  HELPER FUNCTIONS FOR LOOP │
// ──────────────────────────────

// --- Button Handling ---
void checkButton()
{
  bool buttonState = digitalRead(STATUS_BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH)
  {
    printStatusReport();
    if (!manualTxInProgress)
    { // Prevent triggering multiple times if held
      manualTxRequested = true;
      manualTxInProgress = true; // Start the manual process immediately
      colorPrint("[MANUAL] Manual transmit requested via button press", ANSI_BRIGHT_CYAN);
      // GPS will be woken by handleManualTransmit if needed
    }
  }
  lastButtonState = buttonState;
}

// --- Process GPS Data ---
void processGps()
{
  static unsigned long lastGpsDataTime = 0;
  if (gpsIsAwake)
  {
    while (gpsSerial1.available() > 0)
    {
      if (gps.encode(gpsSerial1.read()))
      {                             // encode() returns true on full sentence
        lastGpsDataTime = millis(); // Update timestamp when a sentence is processed
      }
    }
    // Optional: Add diagnostic check for no data while awake
    // if (millis() - lastGpsDataTime > 15000) { // 15 seconds no data
    //     colorPrint("[GPS] Warning: No GPS data received recently while awake.", ANSI_YELLOW);
    // }
  }
}

// --- Periodic Status Update ---
void periodicStatusUpdate()
{
  unsigned long now = millis();
  if (now - lastStatusPrint > 60000)
  { // Print status every 60 seconds
    lastStatusPrint = now;
    // Simple status: GPS state and Home state
    Serial.print("[STATUS] Uptime: ");
    Serial.print(now / 1000);
    Serial.print("s");
    Serial.print(" | GPS: ");
    Serial.print(gpsIsAwake ? "Awake" : "Asleep");
    if (gpsIsAwake)
    {
      Serial.print(gps.location.isValid() ? " (Valid Fix)" : " (No Fix)");
      Serial.print(" Sats: ");
      Serial.print(gps.satellites.value());
    }
    Serial.print(" | Home: ");
    // Serial.print(isHome ? "YES" : "NO");
    Serial.print(" | Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println();
    // printStatusReport(); // Call the full report if desired
  }
}

// --- Build JSON Payload (Common Logic) ---
// Update signature to return String, remove ArduinoJson dependency
String buildJsonPayload()
{
  // Manual JSON string construction
  String payload = "{";
  payload += "\"msg_id\":" + String(messageId++) + ",";                             // Increment global message counter
  payload += "\"device_id\":4,";                                                    // Assuming this is fixed
  payload += "\"id\":\"" + String(SENDER_ID) + "\",";                               // Use defined constant
  payload += "\"time\":" + String(gps.time.isValid() ? gps.time.value() : 0) + ","; // Use GPS time if valid
  payload += "\"satellite_Count\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";

  bool locationValid = gps.location.isValid();
  double currentLat = locationValid ? gps.location.lat() : 0.0;
  double currentLon = locationValid ? gps.location.lng() : 0.0;
  double dist = 0.0;
  String bearingStr = "N/A";

  // Status depends on GPS validity
  if (locationValid)
  {
    colorPrint("[JSON] Status: Out. Using GPS coordinates.", ANSI_YELLOW);
    payload += "\"status\":\"outanabout\",";
    payload += "\"lat\":" + String(currentLat, 6) + ",";
    payload += "\"lon\":" + String(currentLon, 6) + ",";
    dist = TinyGPSPlus::distanceBetween(currentLat, currentLon, HOME_LAT, HOME_LON);    // Use defined constants
    double bearing = TinyGPSPlus::courseTo(currentLat, currentLon, HOME_LAT, HOME_LON); // Use defined constants
    bearingStr = String((int)bearing) + "-" + cardinalDirection(bearing);
    payload += "\"dist_m\":" + String(dist, 2) + ",";
    payload += "\"bearing\":\"" + bearingStr + "\"";
  }
  else
  {
    colorPrint("[JSON] Status: Out but GPS fix INVALID. Sending error status.", ANSI_YELLOW);
    payload += "\"status\":\"error\","; // No valid GPS fix while out
    payload += "\"lat\":0.0,";
    payload += "\"lon\":0.0,";
    payload += "\"dist_m\":0.0,";
    payload += "\"bearing\":\"N/A\"";
  }

  payload += "}";
  return payload;
}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                █
// █   ██       ██████  ██████   █████      ████████ ██   ██       █
// █   ██      ██    ██ ██   ██ ██   ██        ██     ██ ██        █
// █   ██      ██    ██ ██████  ███████        ██      ███         █
// █   ██      ██    ██ ██   ██ ██   ██        ██     ██ ██        █
// █   ███████  ██████  ██   ██ ██   ██        ██    ██   ██       █
// █                                                                █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
// --- Transmit LoRa Packet (Common Logic) ---
// Update signature to accept String payload
void transmitLora(String payload)
{
  colorPrint("[LORA] Sending: " + payload, ANSI_MAGENTA);

  lora.standby(); // Ensure radio is ready
  int txState = lora.transmit(payload);

  if (txState == RADIOLIB_ERR_NONE)
  {
    Serial.print("[LORA] msg [");
    // Flash LED
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(48, HIGH);
      delay(50);
      digitalWrite(48, LOW);
      delay(50);
    }
    Serial.print(messageId - 1); // Print the ID that was just sent
    colorPrint("] sent successfully", ANSI_GREEN);
  }
  else
  {
    colorPrint("[LORA] Transmit failed, code: " + String(txState), ANSI_RED);
  }
}

// --- Build and Send LoRa Packet (Unified Function) ---
void sendLoraPacket(bool isManual)
{
  String payload = ""; // Use String instead of StaticJsonDocument

  if (isManual)
  {
    colorPrint("[LORA] Building MANUAL payload...", ANSI_BRIGHT_CYAN);
    // Manual payload logic (manual JSON string construction)
    payload = "{";
    payload += "\"msg_id\":" + String(messageId++) + ","; // Increment global counter
    payload += "\"device_id\":4,";
    payload += "\"id\":\"" + String(SENDER_ID) + "\","; // Use defined constant
    payload += "\"time\":" + String(gps.time.isValid() ? gps.time.value() : 0) + ",";
    payload += "\"satellite_Count\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";

    if (gps.location.isValid())
    {
      payload += "\"status\":\"manual\","; // Indicate manual send
      payload += "\"lat\":" + String(gps.location.lat(), 6) + ",";
      payload += "\"lon\":" + String(gps.location.lng(), 6) + ",";
      double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),
                                                 HOME_LAT, HOME_LON); // Use defined constants
      double bearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(),
                                             HOME_LAT, HOME_LON); // Use defined constants
      payload += "\"dist_m\":" + String(dist, 2) + ",";
      payload += "\"bearing\":\"" + String((int)bearing) + "-" + cardinalDirection(bearing) + "\"";
    }
    else
    {
      payload += "\"status\":\"manual_error\","; // Indicate manual send w/ no fix
      payload += "\"lat\":0.0,";
      payload += "\"lon\":0.0,";
      payload += "\"dist_m\":0.0,";
      payload += "\"bearing\":\"N/A\"";
    }
    payload += "}";
  }
  else
  {
    colorPrint("[LORA] Building REGULAR payload...", ANSI_MAGENTA);
    // Regular payload logic (calls existing build function)
    payload = buildJsonPayload(); // buildJsonPayload increments messageId
  }

  // Transmit
  transmitLora(payload); // Use common transmit function

  // Update last send time *after* transmission attempt
  lastSendTime = millis();
}

// --- Manual Transmit Sequence ---
void handleManualTransmit()
{
  unsigned long now = millis();
  colorPrint("[MANUAL] Starting manual transmit sequence...", ANSI_BRIGHT_CYAN);

  // 1. Wake GPS & Attempt Fix
  gpsWake(); // Ensure GPS is awake
  colorPrint("[MANUAL] Attempting GPS fix...", ANSI_BRIGHT_CYAN);
  unsigned long manualFixAttemptStart = now;
  bool fixFound = false;
  const unsigned long manualFixTimeout = gpsWakeLeadTime; // Use lead time as timeout

  while (millis() - manualFixAttemptStart < manualFixTimeout)
  {
    processGps(); // Process any incoming data
    if (gps.location.isValid())
    {
      fixFound = true;
      colorPrint("[MANUAL] GPS fix acquired!", ANSI_BRIGHT_GREEN);
      break;
    }
    delay(100); // Small delay
  }

  if (!fixFound)
  {
    colorPrint("[MANUAL] GPS fix timeout. Sending last known or invalid data.", ANSI_YELLOW);
  }

  // 2. Build and Transmit Manual Packet using the new function
  sendLoraPacket(true); // Pass true for manual send

  // 3. Post-Manual-Transmission Actions
  // lastSendTime is updated within sendLoraPacket()

  // 4. Reset manual flags
  manualTxRequested = false;
  manualTxInProgress = false;
  colorPrint("[MANUAL] Manual transmit sequence complete.", ANSI_BRIGHT_CYAN);
}

// --- Regular Transmit Sequence ---
void handleRegularTransmit()
{
  unsigned long now = millis(); // Keep 'now' for potential future use, though not strictly needed here anymore
  colorPrint("[CYCLE] Send interval reached. Executing transmit...", ANSI_MAGENTA);

  // Note: GPS should have been woken up earlier in the 'preparingToSend' phase.

  // 1. Check GPS Fix (it had time to warm up)
  if (gpsIsAwake && !gps.location.isValid())
  {
    colorPrint("[CYCLE] GPS fix still invalid after warmup period.", ANSI_RED);
    // Optional: Could add a brief final attempt here if desired, but likely covered by processGps()
  }
  else if (!gpsIsAwake)
  {
    colorPrint("[CYCLE] GPS is asleep during send cycle? This shouldn't happen.", ANSI_RED);
    // Consider waking GPS here as a fallback? Or rely on the state machine logic.
  }

  // 2. Build and Transmit Regular Packet using the new function
  sendLoraPacket(false); // Pass false for regular send

  // 3. Post-Transmission Actions
  // lastSendTime is updated within sendLoraPacket()

  colorPrint("[CYCLE] Regular transmit sequence complete.", ANSI_MAGENTA);
}

// --- Other Helper Functions (Placeholders - ensure these exist from original code) ---

void printStatusReport()
{
  // ... (Full status report printing code from original file) ...
  colorPrint("[STATUS] Placeholder: Full status report would print here.", ANSI_YELLOW);
  Serial.println("-------------------- STATUS REPORT --------------------");
  Serial.print("  Uptime: ");
  Serial.println(millis() / 1000);
  Serial.print("  GPS Awake: ");
  Serial.println(gpsIsAwake ? "Yes" : "No");
  Serial.print("  GPS Fix: ");
  Serial.println(gps.location.isValid() ? "Valid" : "Invalid");
  Serial.print("  Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("  Is Home: ");
  // Serial.println(isHome ? "Yes" : "No");
  Serial.print("  Free Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.println("-------------------------------------------------------");
}

void colorPrint(const String &message, const char *color)
{
  Serial.print(color);
  Serial.println(message);
  Serial.print(ANSI_RESET);
}

void gpsWake()
{
  if (!gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, HIGH);
    colorPrint("[GPS] Waking up GPS module", ANSI_YELLOW);
    gpsIsAwake = true;
    // Add a small delay for the module to stabilize after wake-up
    delay(100);
  }
  else
  {
    // colorPrint("[GPS] Already awake.", ANSI_YELLOW);
  }
}

void gpsSleep()
{
  if (gpsIsAwake)
  {
    // Before sleeping, ensure any pending serial data is sent (optional)
    Serial.flush();
    digitalWrite(GPS_SLEEP_WAKE, LOW);
    colorPrint("[GPS] Putting GPS module to sleep", ANSI_YELLOW);
    gpsIsAwake = false;
  }
  else
  {
    // colorPrint("[GPS] Already asleep.", ANSI_YELLOW);
  }
}

String cardinalDirection(double bearing)
{
  const char *directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                              "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  // Normalize bearing to 0-360
  bearing = fmod(bearing + 360.0, 360.0);
  int index = (int)round(bearing / 22.5) % 16;
  return String(directions[index]);
}
