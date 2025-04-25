/*
  ┌──────────────────────────────────────────────┐
  │ 🐾 CAT TRACKER TX — LoRa GPS Collar          │
  │ 📡 SX1262 + TinyGPSPlus                      │ // <-- Removed BLE reference
  └──────────────────────────────────────────────┘
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <esp_sleep.h>      // Include ESP sleep library
#include <ArduinoJson.h>    // <-- Add ArduinoJson library
#include <stdio.h>          // <-- Add for sprintf
#include <HardwareSerial.h> // <-- ADD THIS LINE

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
#define LORA_DIO1 39 // Used for LoRa RxDone interrupt wake-up

// GPS Pins
#define GPS_RX 44        // D7 = GPIO 44
#define GPS_TX 43        // D6 = GPIO 43
#define GPS_BAUD 9600    // Baud rate for GPS module
#define GPS_SLEEP_WAKE 1 // d0 = GPIO 1  - used for sleep/wake control
#define GPS_RESET 9      // D10 = GPIO 9

// Button pin for status report and manual wake/transmit
#define STATUS_BUTTON_PIN GPIO_NUM_21 // Use GPIO_NUM_x for sleep functions


// #define SENDER_ID "Podge"
#define SENDER_ID "Macy"
// #define SENDER_ID "Simba"
// #define SENDER_ID "Gizmo"
#define HOME_LAT 51.87370573411073
#define HOME_LON -2.2396017778476716
// #define SEND_INTERVAL 60000 // milliseconds (60 seconds) - Commented out, using sleep timer

// Sleep Configuration
// ┌─────────────────┐
// │ Sleep Settings  │
// └─────────────────┘


#define SLEEP_DURATION_US (30 * 1000000ULL) // 30 seconds in microseconds




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
HardwareSerial gpsSerial(1); // Use UART peripheral 1 (adjust if needed)
// HardwareSerial gpsSerial(44, 43); // <-- Constructor doesn't take pins

// Global State Variables
bool gpsIsAwake = true; // Assume awake initially after setup
unsigned long gpsWakeLeadTime = 60000;
unsigned long InitialgpsWakeLeadTime = 120000; // Time to wait for GPS fix after wake (60s)
// unsigned long lastSendTime = 0; // No longer needed for interval timing
unsigned long lastStatusPrint = 0;
static uint32_t messageId = 0; // Global message counter for LoRa
String LoRaRxMsg = "";         // Buffer for received LoRa message
// bool manualTxRequested = false;  // Replaced by button wake-up logic
// bool manualTxInProgress = false; // Replaced by button wake-up logic
// bool lastButtonState = HIGH; // Replaced by interrupt wake-up

// Forward Declarations
void printStatusReport();
void colorPrint(const String &message, const char *color = ANSI_RESET); // Ensure this matches definition
void gpsWake();
void gpsSleep();
String cardinalDirection(double bearing);
void processGps();
void periodicStatusUpdate(); // Keep for periodic updates while awake
void transmitLora(String payload);
String buildJsonPayload(); // Removed flag
void craftLoraPacket();    // Renamed from sendLoraPacket
void handleWakeupReason();
void handleLoraReception();
void performTransmissionSequence(); // Removed flag
void goToLightSleep();
void ledFlicker();     // <-- Add forward declaration
bool isChannelClear(); // <-- Add forward declaration for CAD function

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                             █
// █   ███████ ███████ ████████ ██    ██ ██████      ██████  ██    ██ ███    ██  █
// █   ██      ██         ██    ██    ██ ██   ██     ██   ██ ██    ██ ████   ██  █
// █   ███████ █████      ██    ██    ██ ███████     ██████  ██    ██ ██ ██  ██  █
// █        ██ ██         ██    ██    ██ ██          ██   ██ ██    ██ ██  ██ ██  █
// █   ███████ ███████    ██     ██████  ██          ██    █  ██████  ██   ████  █
// █                                                                             █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
void setup()
{
  Serial.begin(115200);
  delay(1000);                                                                    // Wait for serial monitor
  Serial.println("\n[BOOT] Serial connection delay complete. Starting setup..."); // Added this line
  delay(200);                                                                     // Give some time for the serial monitor to open
  colorPrint("[BOOT] Initialising CAT TRACKER TX v2 (Sleep Enabled)...");
  delay(200); // Give some time for the serial monitor to open
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(48, OUTPUT); // Status LED

  // --- Button Pin Setup ---
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP); // Configure button pin

  // --- GPS Pin Setup ---
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH); // Keep GPS out of reset
  pinMode(GPS_SLEEP_WAKE, OUTPUT);
  digitalWrite(GPS_SLEEP_WAKE, HIGH); // Start with GPS awake
  gpsIsAwake = true;

  // --- GPS Init ---
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX); // Initialize HardwareSerial with custom pins
  // gpsSerial.begin(GPS_BAUD); // Simpler begin without pin specification
  delay(100);
  colorPrint("[GPS] Waking up GPS module(60s) for initial setup...");
  // gpsWake(); // gpsWake also calls begin, avoid calling it right after begin here. Let the warmup loop handle it.

  // --- Add check for GPS serial activity ---
  colorPrint("[GPS] Checking for initial serial activity...", ANSI_YELLOW);
  delay(1000); // Wait 1 second for GPS to start sending data

  if (gpsSerial.available() > 0)
  {
    colorPrint("[GPS] Serial data detected! Module appears awake. ✔ ", ANSI_BRIGHT_GREEN);
    // Optionally read and discard initial potentially garbled data
    while (gpsSerial.available() > 0)
    {
      gpsSerial.read();
    }
  }
  else
  {
    colorPrint("[GPS] No serial data detected after 1s. Check wiring/power.", ANSI_BRIGHT_RED);
    // Consider halting or adding more robust error handling here if needed
  }
  // --- End check ---

  colorPrint("[GPS] Warming up GPS (waiting for fix)..."); // Proceed with warmup/fix attempt
  unsigned long gpsWarmupStart = millis();
  bool fixFound = false;
  bool firstFixDetectedSetup = false; // Flag for first fix detection in setup
  unsigned long firstFixTimestampSetup = 0; // Timestamp for first fix in setup

  while (millis() - gpsWarmupStart < 120000) // 2 min timeout
  {
    processGps(); // Process data during warmup

    // Check if a valid fix is obtained and we haven't detected one before
    if (!firstFixDetectedSetup && gps.location.isValid() && gps.location.age() < 5000)
    {
      firstFixDetectedSetup = true;
      firstFixTimestampSetup = millis();
      colorPrint("[GPS] Initial valid fix obtained during warmup! Waiting 10s for stability...", ANSI_BRIGHT_GREEN);
      // Do not break yet, continue processing
    }

    // If we have detected a fix, check if 10 seconds have passed since then
    if (firstFixDetectedSetup && (millis() - firstFixTimestampSetup >= 10000))
    {
      colorPrint("[GPS] 10s stabilization period complete during warmup.", ANSI_BRIGHT_GREEN);
      fixFound = true; // Mark that we successfully waited the stabilization period
      break;           // Exit the loop now
    }

    delay(1); // Reduced delay significantly to process GPS data more frequently
  }

  // After the loop (timeout or stabilization complete)
  if (fixFound)
  {
    // We successfully detected a fix and waited 10 seconds
    colorPrint("[GPS] Initialized with stabilized fix.", ANSI_GREEN);
    processGps(); // Process one last time
  }
  else if (firstFixDetectedSetup)
  {
    // We detected a fix, but the 120s timeout occurred before the 10s stabilization finished
    colorPrint("[GPS] Initialized with early fix (stabilization incomplete).", ANSI_YELLOW);
    processGps(); // Process one last time
  }
  else
  {
    // 120s timeout occurred without ever detecting a valid fix
    colorPrint("[GPS] Warmup Expired without Getting fix.", ANSI_RED);
  }
  // Put GPS to sleep after initial warmup/fix attempt before first sleep cycle

  // --- LoRa Init ---
  pinMode(LORA_DIO1, INPUT); // Set DIO1 pin as input for interrupt
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  colorPrint("[INIT] setting up SPI for LoRa ...");
  int initState = lora.begin(915.0); // Use appropriate frequency
  if (initState != RADIOLIB_ERR_NONE)
  {
    colorPrint("[ERROR] LoRa failed to initialise. Code: " + String(initState), ANSI_RED);
    while (true)
      ; // Halt on critical error
  }
  else
  {
    colorPrint("[OK] LoRa initialised successfully");
    // Apply LoRa parameters
    lora.setOutputPower(22);
    lora.setSpreadingFactor(8);
    lora.setBandwidth(250.0);
    lora.setCodingRate(5);
    lora.setCRC(true);
    lora.setPreambleLength(8);
    // Set DIO1 mask for RxDone interrupt
    lora.setDio1Action(handleLoraReception); // Call this function when DIO1 goes HIGH

    colorPrint("[INIT] LoRa Params configured.");
  }

  colorPrint("════════════════════════════════════════", ANSI_BOLD);
  colorPrint("🚀 SETUP COMPLETE - Entering initial sleep cycle 😴", ANSI_BOLD);
  colorPrint("════════════════════════════════════════", ANSI_BOLD);

  // Set initial lastSendTime to allow first send after interval - No longer needed
  // lastSendTime = millis() - SEND_INTERVAL + 5000;
  gpsSleep();
  delay(1000);
  lora.standby();
  ledFlicker(); // Go to sleep for the first time
  goToLightSleep();
}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                             █
// █   ███    ███  █████  ██ ███    ██     ██       ██████   ██████  ██████      █
// █   ████  ████ ██   ██ ██ ████   ██     ██      ██    ██ ██    ██ ██   ██     █
// █   ██ ████ ██ ███████ ██ ██ ██  ██     ██      ██    ██ ██    ██ ██████      █
// █   ██  ██  ██ ██   ██ ██ ██  ██ ██     ██      ██    ██ ██    ██ ██          █
// █   ██      ██ ██   ██ ██ ██   ████     ███████  ██████   ██████  ██          █
// █                                                                             █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
void loop()
{
  Serial.begin(115200);
  // --- Code execution resumes here after wake-up ---
  colorPrint("\n☀️ Woke up!", ANSI_BRIGHT_YELLOW);

  // --- Handle Wake-up Reason ---
  handleWakeupReason();

  // --- Perform Periodic Tasks (if awake for a while, e.g., during GPS fix) ---
  // Note: The main activity now happens within the wake-up handlers
  // and the transmission sequence. We might add short tasks here if needed
  // before going back to sleep, but the core logic is triggered by wake-up.
  // processGps(); // Process GPS only when explicitly woken/needed
  // periodicStatusUpdate(); // Print status only on button press or specific debug needs

  delay(1000); // --- Go Back to Sleep ---
  goToLightSleep();
}

// ──────────────────────────────
// │ WAKE UP / SLEEP FUNCTIONS  │
// ──────────────────────────────

void handleWakeupReason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_TIMER:
    colorPrint("[WAKE] Reason: Timer ⏰", ANSI_CYAN);
    performTransmissionSequence(); // Perform standard transmission (no flag)
    break;
  case ESP_SLEEP_WAKEUP_EXT0: // Connected to STATUS_BUTTON_PIN (GPIO 21)
    colorPrint("[WAKE] Reason: Button Press 👆", ANSI_BRIGHT_CYAN);
    printStatusReport();           // Print status report first
    performTransmissionSequence(); // Perform standard transmission (no flag)
    ledFlicker();                  // Flicker LED after sequence
    break;
  case ESP_SLEEP_WAKEUP_GPIO: // Connected to LORA_DIO1 (GPIO 39)
    colorPrint("[WAKE] Reason: LoRa DIO1 Interrupt 📡", ANSI_BRIGHT_MAGENTA);
    handleLoraReception();         // Read the received message
    performTransmissionSequence(); // Perform standard transmission after receiving (no flag)
    break;
  default:
    colorPrint("[WAKE] Reason: Unknown (" + String(wakeup_reason) + ")", ANSI_RED);
    // Potentially add a short delay or specific handling for unexpected wakeups
    delay(1000);
    break;
  }
}

void handleLoraReception()
{
  colorPrint("[LORA RX] Interrupt received. Reading message...", ANSI_MAGENTA);
  int state = lora.readData(LoRaRxMsg); // Read message into the global buffer

  if (state == RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA RX] Received: " + LoRaRxMsg, ANSI_BRIGHT_GREEN);
    // TODO: Add parsing logic here if needed based on LoRaRxMsg content
    // Example: Parse JSON, check commands, etc.
    // For now, we just store and print it.
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  {
    colorPrint("[LORA RX] CRC error!", ANSI_RED);
    LoRaRxMsg = ""; // Clear buffer on error
  }
  else
  {
    colorPrint("[LORA RX] Failed, code: " + String(state), ANSI_RED);
    LoRaRxMsg = ""; // Clear buffer on error
  }
  // No need to clear IRQ flags manually here if using RadioLib's ISR handling (setDio1Action)
}

// Removed isButtonTriggered parameter
void performTransmissionSequence()
{
  colorPrint("[SEQUENCE] Starting Transmission Sequence...", ANSI_MAGENTA); // Use standard color

  // 1. Wake GPS & Attempt Fix/Update
  gpsWake(); // Ensure GPS is awake and serial is initialized
  colorPrint("[SEQUENCE] Attempting GPS fix/update (Max Wait: 60s)...", ANSI_YELLOW);
  unsigned long fixAttemptStart = millis();
  bool fixObtainedDuringWait = false; // Flag to track if we completed the stabilization wait
  bool firstFixDetected = false;      // Flag to track if we've seen the *first* fix
  unsigned long firstFixTimestamp = 0; // Timestamp of the first fix detection

  // Wait for up to 60 seconds total, processing GPS data
  while (millis() - fixAttemptStart < 60000)
  {
    processGps(); // Continuously process GPS data

    // Check if a valid fix is obtained and we haven't detected one before
    if (!firstFixDetected && gps.location.isValid() && gps.location.age() < 5000)
    {
      // This is the first time we've detected a valid, recent fix in this sequence
      firstFixDetected = true;
      firstFixTimestamp = millis();
      colorPrint("[SEQUENCE] Initial GPS fix detected! Waiting 10s for stability...", ANSI_GREEN);
      // Do not break yet, continue processing in the loop
    }

    // If we have detected a fix, check if 10 seconds have passed since then
    if (firstFixDetected && (millis() - firstFixTimestamp >= 10000))
    {
      colorPrint("[SEQUENCE] 10s stabilization period complete.", ANSI_GREEN);
      fixObtainedDuringWait = true; // Mark that we successfully waited the stabilization period
      break;                        // Exit the loop now, we have waited enough
    }

    delay(1); // Minimal delay to yield, allowing other tasks if any (mostly processes serial)
  }

  // After the loop (either 60s timeout, or 10s stabilization finished)
  if (fixObtainedDuringWait)
  {
    // We successfully detected a fix and waited 10 seconds
    colorPrint("[SEQUENCE] Proceeding with stabilized GPS fix.", ANSI_GREEN);
    processGps(); // Process one last time to ensure latest data is encoded
  }
  else if (firstFixDetected)
  {
    // We detected a fix, but the 60s timeout occurred before the 10s stabilization finished
    colorPrint("[SEQUENCE] Proceeding with initial GPS fix (stabilization incomplete).", ANSI_YELLOW);
    processGps(); // Process one last time
  }
  else
  {
    // 60s timeout occurred without ever detecting a valid fix
    colorPrint("[SEQUENCE] GPS fix timeout after 60s. Proceeding without valid fix.", ANSI_YELLOW);
    // No need to processGps() here as there was no valid data
  }
  // Proceed to build and transmit LoRa packet regardless of fix status (buildJsonPayload handles invalid data)

  // 2. Build and Transmit LoRa Packet
  craftLoraPacket(); // Call renamed function

  // 3. Post-Transmission: Sleep GPS
  gpsSleep();

  colorPrint("[SEQUENCE] Transmission Sequence Complete.", ANSI_MAGENTA); // Use standard color
}

void goToLightSleep()
{
  colorPrint("[SLEEP] Preparing for light sleep...", ANSI_BLUE);

  // Clear received message buffer before sleeping
  LoRaRxMsg = "";

  // Put LoRa into receive mode to listen for wake-up messages
  colorPrint("[SLEEP] Setting LoRa to receive mode...", ANSI_BLUE);
  int rxState = lora.startReceive();
  if (rxState != RADIOLIB_ERR_NONE)
  {
    colorPrint("[SLEEP] Failed to start LoRa receive, code: " + String(rxState), ANSI_RED);
    // Decide handling: retry? deep sleep? For now, proceed to sleep anyway.
  }
  else
  {
    colorPrint("[SLEEP] LoRa is listening 👂", ANSI_BLUE);
  }

  // Configure wake-up sources
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // Disable all first, then will renable 

  // Timer Wakeup
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  colorPrint("[SLEEP] Wakeup enabled: Timer (" + String(SLEEP_DURATION_US / 1000000ULL) + "s)", ANSI_BLUE);

  // Button Wakeup (EXT0) - Requires the pin number and level (0 for LOW)
  esp_sleep_enable_ext0_wakeup(STATUS_BUTTON_PIN, 0);
  colorPrint("[SLEEP] Wakeup enabled: Button (GPIO " + String(STATUS_BUTTON_PIN) + " LOW)", ANSI_BLUE);

  // LoRa DIO1 Wakeup (GPIO) - Requires enabling GPIO wakeup and configuring the specific pin/level
  esp_sleep_enable_gpio_wakeup();
  // Wake up if DIO1 (GPIO 39) goes HIGH (e.g., on RxDone)
  gpio_wakeup_enable(GPIO_NUM_39, GPIO_INTR_HIGH_LEVEL);
  colorPrint("[SLEEP] Wakeup enabled: LoRa DIO1 (GPIO " + String(LORA_DIO1) + " HIGH)", ANSI_BLUE);

  colorPrint("😴 Entering light sleep...", ANSI_BOLD);
  Serial.flush(); // Ensure all serial messages are sent before sleeping
  gpsSleep(); // Ensure GPS is in sleep mode before going to light sleep
  // Enter light sleep
  esp_light_sleep_start();

  // --- Execution resumes in loop() after wake-up ---
}

// ──────────────────────────────
// │  HELPER FUNCTIONS          │
// ──────────────────────────────

// --- Button Handling (Now handled by wake-up interrupt) ---
/*
void checkButton()
{
  // This function is no longer needed as button press triggers EXT0 wake-up
}
*/

// --- Process GPS Data ---
void processGps()
{
  if (gpsIsAwake)
  {
    // Read all available characters from the GPS serial port
    while (gpsSerial.available() > 0)
    {
      // Read ONE character from the serial port
      char c = gpsSerial.read();
      // Feed that character to the TinyGPS++ library for processing
      gps.encode(c);
    }
    // The gps object (TinyGPSPlus instance) updates its internal state
    // (location, time, satellites, etc.) automatically when a complete
    // NMEA sentence is successfully parsed by gps.encode().
    // We check the validity of the data (e.g., gps.location.isValid())
    // elsewhere in the code when we need to use it (like in buildJsonPayload).
  }
}

// --- Build JSON Payload ---
String buildJsonPayload()
{
  // Increased size slightly to accommodate timestamp, distance, bearing
  StaticJsonDocument<300> doc; // Adjusted size

  // Add static fields
  doc["msg_id"] = messageId;
  doc["device_id"] = 4; // Or use a variable if needed
  doc["id"] = SENDER_ID;

  // --- Core GPS Data ---
  bool loc_valid = gps.location.isValid();
  unsigned long loc_age = gps.location.age(); // Keep age to determine staleness
  bool sat_valid = gps.satellites.isValid();
  unsigned long sat_value = gps.satellites.value();
  bool time_valid = gps.time.isValid();
  unsigned long time_age = gps.time.age();

  // --- Determine Status based on Validity and Age ---
  // Consider location stale if valid but age is > 60 seconds (adjust as needed)
  bool isStale = loc_valid && loc_age > 60000;

  if (loc_valid && !isStale)
  {
    double currentLat = gps.location.lat();
    double currentLon = gps.location.lng();

    doc["status"] = "outanabout"; // Changed status back
    doc["lat"] = currentLat;
    doc["lon"] = currentLon;
    doc["sats"] = sat_valid ? sat_value : 0; // Include satellite count

    // Calculate distance/bearing
    double dist = TinyGPSPlus::distanceBetween(currentLat, currentLon, HOME_LAT, HOME_LON);
    double bearing = TinyGPSPlus::courseTo(currentLat, currentLon, HOME_LAT, HOME_LON);
    String bearingStr = String((int)bearing) + "-" + cardinalDirection(bearing);
    doc["dist_m"] = round(dist * 100.0) / 100.0; // Restore distance
    doc["bearing"] = bearingStr;                 // Restore bearing
  }
  else // Location Invalid or Stale
  {
    doc["status"] = "error"; // Simplified error status
    doc["lat"] = 0.0;
    doc["lon"] = 0.0;
    doc["sats"] = sat_valid ? sat_value : 0; // Still report sats if available
    doc["dist_m"] = 0.0;
    doc["bearing"] = "N/A";
  }

  // --- Add Timestamp if valid and recent ---
  if (time_valid && time_age < 60000) { // Check age too
    char isoTimestamp[25];
    snprintf(isoTimestamp, sizeof(isoTimestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    doc["time"] = isoTimestamp; // Restore timestamp
  } else {
    doc["time"] = "error"; // Indicate if time is invalid/stale
  }


  String payload;
  serializeJson(doc, payload);
  doc.clear(); // Clear JSON doc memory

  // --- Debug Print (only if Serial is somehow available) ---
  // colorPrint("[JSON] Payload (" + String(payload.length()) + " bytes): " + payload, ANSI_CYAN);

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
void transmitLora(String payload)
{
  colorPrint("[LORA TX] Preparing to send: " + payload, ANSI_MAGENTA);

  bool channelClearToSend = false;
  for (int attempt = 0; attempt < 5; attempt++)
  {
    if (isChannelClear())
    {
      channelClearToSend = true;
      break; // Exit loop, channel is clear
    }
    else
    {
      // Channel busy or CAD failed
      if (attempt < 4)
      { // Don't wait after the last attempt
        colorPrint("[LORA LBT] Channel busy. Waiting 2s (Attempt " + String(attempt + 1) + "/5)...", ANSI_YELLOW);
        delay(2000);
      }
    }
  }

  if (!channelClearToSend)
  {
    colorPrint("[LORA LBT] Channel busy after 5 attempts. Transmitting anyway.", ANSI_YELLOW);
  }

  // --- Proceed with Transmission ---
  colorPrint("[LORA TX] Starting transmission...", ANSI_MAGENTA);

  delay(20); // Ensure radio is in standby before transmitting (might already be from CAD)
  lora.standby();

  int txState = lora.transmit(payload);
  // ... (rest of the existing transmitLora function: error handling, LED blinking, etc.) ...
  unsigned long txStart = millis();

  if (txState == RADIOLIB_ERR_NONE)
  {
    // Wait for TX to complete - monitor DIO1 or use timeout
    // Note: SX1262 doesn't typically use DIO1 for TxDone by default with RadioLib's basic transmit.
    // We'll rely on the blocking nature or add a calculated delay if needed.
    // For now, assume transmit() blocks or finishes quickly enough.
    // A more robust method would involve checking BUSY pin or using TxDone interrupt on DIOx.
    colorPrint("[LORA TX] Transmission started...", ANSI_GREEN);

    // Simple visual indicator while transmitting
    digitalWrite(48, HIGH); // Turn on status LED
    // No delay here, assume transmit handles timing or is fast

    Serial.print("[LORA TX] msg [");
    Serial.print(messageId); // Print the ID that was just sent
    colorPrint("] sent successfully!", ANSI_GREEN);
    digitalWrite(48, LOW); // Turn off status LED

    // Add specific success blink using built-in LED
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(60);
        digitalWrite(LED_BUILTIN, LOW);
        delay(120);
    }
  }
  else
  {
    colorPrint("[LORA TX] Transmit failed, code: " + String(txState), ANSI_RED);
    // Flash status LED (pin 48) rapidly 20 times for error
    for (int i = 0; i < 20; i++) // Changed loop count to 20
    {
      digitalWrite(48, HIGH);
      delay(50); // Short delay for rapid blink
      digitalWrite(48, LOW);
      delay(50); // Short delay for rapid blink
    }
  }

  // It's good practice to return to standby after TX attempt
  lora.standby();
}

// --- Build and Send LoRa Packet (Unified Function) ---
// Renamed from sendLoraPacket
void craftLoraPacket()
{
  // Increment message ID for this packet *before* building payload
  messageId++;

  // Debug: Print GPS state *before* building payload

  String payload = buildJsonPayload(); // Build payload (no longer needs trigger type)

  // Transmit
  transmitLora(payload); // Use common transmit function
  // Transmit

  // lastSendTime update is no longer needed as we use sleep timer
  // lastSendTime = millis();
}

void ledFlicker()
{
  colorPrint("[LED] Flickering status LED...", ANSI_BLUE);
  pinMode(48, OUTPUT); // Ensure pin is output
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(48, HIGH);
    delay(50); // ~1/12th second ON
    digitalWrite(48, LOW);
    delay(50); // ~1/12th second OFF (Total cycle ~1/6th sec, 6 cycles ~1 sec)
  }
}

void printStatusReport()
{
  colorPrint("-------------------- STATUS REPORT --------------------", ANSI_BOLD);
  Serial.print("  Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println("s");
  Serial.print("  GPS Awake: ");
  Serial.println(gpsIsAwake ? "Yes" : "No");
  if (gps.location.isValid())
  {
    Serial.print("  Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("  Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println("m");
  }
  else
  {
    Serial.println("  Location: Invalid");
  }
  Serial.print("  Satellites: ");
  Serial.println(gps.satellites.value()); // Check validity
  Serial.print("  HDOP: ");
  Serial.println(gps.hdop.value()); // Check validity
  Serial.print("  Last Rx Msg: ");
  Serial.println(LoRaRxMsg.length() > 0 ? LoRaRxMsg : "None");
  Serial.print("  Free Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("  Wakeup Cause: ");
  Serial.println(esp_sleep_get_wakeup_cause()); // This should be fine
  colorPrint("-------------------------------------------------------", ANSI_BOLD);
}

void colorPrint(const String &message, const char *color) // Match declaration
{
  Serial.print(color);
  Serial.println(message);
  Serial.print(ANSI_RESET); // Reset color
  Serial.flush();           // Ensure message is printed, especially before sleep
}

void gpsWake()
{
  if (!gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, HIGH);
    colorPrint("[GPS] Setting wake pin HIGH...", ANSI_YELLOW);
    gpsIsAwake = true;
    delay(100); // Small delay for wake pin stabilization

    colorPrint("[GPS] Initializing HardwareSerial for GPS...", ANSI_YELLOW);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(500); // Increased delay AFTER serial begin to allow GPS UART to stabilize
  }
  else
  {
    colorPrint("[GPS] Already awake.", ANSI_YELLOW);
    colorPrint("[GPS] Re-initializing HardwareSerial (just in case)...", ANSI_YELLOW);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100); // Shorter delay if already awake is likely fine
  }
}

void gpsSleep()
{
  if (gpsIsAwake)
  {
    // Before sleeping, ensure any pending serial data is sent (optional but good practice)
    Serial.flush();
    gpsSerial.end(); // <-- Keep this, SoftwareSerial has an end() method

    digitalWrite(GPS_SLEEP_WAKE, LOW);
    colorPrint("[GPS] Putting GPS module to sleep 😴", ANSI_YELLOW);
    gpsIsAwake = false;
    // gpsSerial1.end(); // Optional: formally close serial, may save minuscule power
  }
  else
  {
    colorPrint("[GPS] Already asleep.", ANSI_YELLOW);
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

// --- Check LoRa Channel Activity (CAD) ---
bool isChannelClear()
{
  colorPrint("[LORA CAD] Checking channel activity...", ANSI_BLUE);

  // Ensure radio is in standby for CAD
  int state = lora.standby();
  if (state != RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA CAD] Failed to enter standby before CAD: " + String(state), ANSI_RED);
    return false; // Indicate failure, maybe transmit anyway later? For now, treat as busy.
  }

  // Start CAD
  state = lora.startChannelScan();
  if (state != RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA CAD] Failed to start CAD: " + String(state), ANSI_RED);
    return false; // Indicate failure
  }

  // Wait for CAD to complete. This duration depends on LoRa settings (BW, SF).
  // A simple delay is used here; a more robust method might use DIO interrupts if configured.
  // This delay might be too short or too long depending on settings. Increased slightly.
  delay(100); // Adjusted fixed delay for CAD completion

  // Check CAD result
  state = lora.getChannelScanResult();

  if (state == RADIOLIB_CHANNEL_FREE)
  {
    colorPrint("[LORA CAD] Channel is free!", ANSI_GREEN);
    return true;
  }
  else if (state == RADIOLIB_LORA_DETECTED)
  {
    colorPrint("[LORA CAD] LoRa signal detected!", ANSI_YELLOW);
    return false;
  }
  else
  {
    colorPrint("[LORA CAD] CAD failed or unknown result: " + String(state), ANSI_RED);
    return false; // Treat errors or unexpected results as busy
  }
}
// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                             █
// █                             END OF CODE                                     █
// █                                                                             █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀