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
#include <esp_sleep.h>   // Include ESP sleep library
#include <ArduinoJson.h> // <-- Add ArduinoJson library

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

// Hardcoded Configuration Values (Replaces JSON config)
#define SENDER_ID "Gizmo"
#define HOME_LAT 51.87370573411073
#define HOME_LON -2.2396017778476716
// #define SEND_INTERVAL 60000 // milliseconds (60 seconds) - Commented out, using sleep timer

// Sleep Configuration
// ┌─────────────────┐
// │ Sleep Settings  │
// └─────────────────┘
#define SLEEP_DURATION_US (60 * 1000000ULL) // 1  minute in microseconds

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
bool gpsIsAwake = true;                // Assume awake initially after setup
unsigned long gpsWakeLeadTime = 30000; // Time to wait for GPS fix after wake (30s)
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
void sendLoraPacket();     // Removed flag
void handleWakeupReason();
void handleLoraReception();
void performTransmissionSequence(); // Removed flag
void goToLightSleep();
void ledFlicker(); // <-- Add forward declaration

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                             █
// █   ███████ ███████ ████████ ██    ██ ██████      ██████  ██    ██ ███    ██  █
// █   ██      ██         ██    ██    ██ ██   ██     ██   ██ ██    ██ ████   ██  █
// █   ███████ █████      ██    ██    ██ ██████      ██████  ██    ██ ██ ██  ██  █
// █        ██ ██         ██    ██    ██ ██          ██   ██ ██    ██ ██  ██ ██  █
// █   ███████ ███████    ██     ██████  ██          ██    █  ██████  ██  ████   █
// █                                                                             █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
void setup()
{
  Serial.begin(115200);
  delay(50); // Wait for serial monitor
  colorPrint("[BOOT] Initialising CAT TRACKER TX v2 (Sleep Enabled)...");

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
  gpsSerial1.begin(GPS_BAUD);
  delay(100);
  colorPrint("[GPS] Warming up GPS...");
  unsigned long gpsWarmupStart = millis();
  bool fixFound = false;
  while (millis() - gpsWarmupStart < 60000 && !fixFound)
  {               // 60 sec warmup
    processGps(); // Process data during warmup
    if (gps.location.isValid())
    {
      fixFound = true;
      colorPrint("[GPS] Valid fix obtained during warmup ✔ Lat: " + String(gps.location.lat(), 6) +
                     ", Lon: " + String(gps.location.lng(), 6),
                 ANSI_BRIGHT_GREEN);
      break;
    }
    Serial.print(".");
    delay(1000);
  }
  if (!fixFound)
  {
    colorPrint("[GPS] Warmup Expired without Getting fix.", ANSI_RED);
  }
  else
  {
    colorPrint("[GPS] Initialized.", ANSI_GREEN);
  }
  // Put GPS to sleep after initial warmup/fix attempt before first sleep cycle
  gpsSleep();

  // --- LoRa Init ---
  pinMode(LORA_DIO1, INPUT); // Set DIO1 pin as input for interrupt
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  colorPrint("[INIT] SPI for LoRa initialised");
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

  delay(1000); // Go to sleep for the first time
  goToLightSleep();
}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                              █
// █   ███    ███  █████  ██ ███    ██     ██       ██████   ██████  ██████      █
// █   ████  ████ ██   ██ ██ ████   ██     ██      ██    ██ ██    ██ ██   ██     █
// █   ██ ████ ██ ███████ ██ ██ ██  ██     ██      ██    ██ ██    ██ ██████      █
// █   ██  ██  ██ ██   ██ ██ ██  ██ ██     ██      ██    ██ ██    ██ ██          █
// █   ██      ██ ██   ██ ██ ██   ████     ███████  ██████   ██████  ██          █
// █                                                                              █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
void loop()
{
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

  // 1. Wake GPS & Attempt Fix
  gpsWake(); // Ensure GPS is awake
  colorPrint("[SEQUENCE] Attempting GPS fix (Max Wait: " + String(gpsWakeLeadTime / 1000) + "s)...", ANSI_YELLOW);
  unsigned long fixAttemptStart = millis();
  bool fixFound = false;

  while (millis() - fixAttemptStart < gpsWakeLeadTime)
  {
    processGps(); // Process any incoming data
    if (gps.location.isValid())
    {
      fixFound = true;
      colorPrint("[SEQUENCE] GPS fix acquired! ✔", ANSI_BRIGHT_GREEN);
      break;
    }
    // Add a small delay and maybe a status print during wait
    Serial.print(".");
    delay(500);
  }
  Serial.println(); // Newline after dots

  if (!fixFound)
  {
    colorPrint("[SEQUENCE] GPS fix timeout. Sending last known or invalid data.", ANSI_YELLOW);
  }

  // 2. Build and Transmit LoRa Packet
  sendLoraPacket(); // Call simplified function (no flag)

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
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // Disable all first

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
  // static unsigned long lastGpsDataTime = 0; // Keep for diagnostics if needed
  if (gpsIsAwake)
  {
    while (gpsSerial1.available() > 0)
    {
      gps.encode(gpsSerial1.read());
      // We check gps.location.isValid() where needed, encode handles sentence processing.
      // lastGpsDataTime = millis(); // Update timestamp if diagnostics are re-enabled
    }
    // Optional: Add diagnostic check for no data while awake
    // if (millis() - lastGpsDataTime > 15000) { // 15 seconds no data
    //     colorPrint("[GPS] Warning: No GPS data received recently while awake.", ANSI_YELLOW);
    // }
  }
}

// --- Periodic Status Update (Called less frequently now) ---
void periodicStatusUpdate()
{
  unsigned long now = millis();
  if (now - lastStatusPrint > 60000)
  { // Print status every 60 seconds IF AWAKE
    lastStatusPrint = now;
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
    Serial.print(" | Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println();
  }
}

// --- Build JSON Payload ---
// Uses ArduinoJson library
// Removed isManualTrigger parameter
String buildJsonPayload()
{
  // Allocate JSON document (adjust size if needed, 256 seems reasonable)
  StaticJsonDocument<256> doc;

  // Add static fields
  doc["msg_id"] = messageId; // Note: messageId incremented in sendLoraPacket
  doc["device_id"] = 4;
  doc["id"] = SENDER_ID;
  doc["time"] = gps.time.isValid() ? gps.time.value() : 0;
  doc["satellite_Count"] = gps.satellites.isValid() ? gps.satellites.value() : 0;

  bool locationValid = gps.location.isValid();
  double currentLat = locationValid ? gps.location.lat() : 0.0;
  double currentLon = locationValid ? gps.location.lng() : 0.0;
  String status = "unknown"; // Default status

  if (locationValid)
  {
    double dist = TinyGPSPlus::distanceBetween(currentLat, currentLon, HOME_LAT, HOME_LON);
    double bearing = TinyGPSPlus::courseTo(currentLat, currentLon, HOME_LAT, HOME_LON);
    String bearingStr = String((int)bearing) + "-" + cardinalDirection(bearing);

    // Simplified status: always "outanabout" if GPS is valid
    status = "outanabout";
    colorPrint("[JSON] Status: Out (GPS Valid).", ANSI_YELLOW);

    doc["status"] = status;
    doc["lat"] = currentLat; // ArduinoJson handles precision
    doc["lon"] = currentLon;
    doc["dist_m"] = round(dist * 100.0) / 100.0; // Round to 2 decimal places
    doc["bearing"] = bearingStr;
  }
  else // Location Invalid
  {
    // Simplified status: always "error" if GPS is invalid
    status = "error";
    colorPrint("[JSON] Status: Error (GPS Invalid).", ANSI_RED);

    doc["status"] = status;
    doc["lat"] = 0.0;
    doc["lon"] = 0.0;
    doc["dist_m"] = 0.0;
    doc["bearing"] = "N/A";
  }

  // Serialize JSON to string
  String payload;
  serializeJson(doc, payload);
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
  colorPrint("[LORA TX] Sending: " + payload, ANSI_MAGENTA);

  // Ensure LoRa is not in receive mode before transmitting
  lora.standby(); // Go to standby mode first

  // Potentially re-apply TX parameters if sleep affects them, though often not needed for light sleep
  // lora.setOutputPower(22);
  // ... other params if needed ...

  int txState = lora.transmit(payload);
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
  }
  else
  {
    colorPrint("[LORA TX] Transmit failed, code: " + String(txState), ANSI_RED);
    // Flash LED rapidly for error
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(48, HIGH);
      delay(100);
      digitalWrite(48, LOW);
      delay(100);
    }
  }

  // It's good practice to return to standby after TX attempt
  lora.standby();
}

// --- Build and Send LoRa Packet (Unified Function) ---
// Removed isManualTrigger flag
void sendLoraPacket()
{
  // Increment message ID for this packet *before* building payload
  messageId++;

  String payload = buildJsonPayload(); // Build payload (no longer needs trigger type)

  // Transmit
  transmitLora(payload); // Use common transmit function

  // lastSendTime update is no longer needed as we use sleep timer
  // lastSendTime = millis();
}

// --- Manual Transmit Sequence (Now part of performTransmissionSequence) ---
/*
void handleManualTransmit()
{
  // Logic moved into performTransmissionSequence(true) triggered by button wake-up
}
*/

// --- Regular Transmit Sequence (Now part of performTransmissionSequence) ---
/*
void handleRegularTransmit()
{
 // Logic moved into performTransmissionSequence(false) triggered by timer wake-up
}
*/

// --- Other Helper Functions ---

void ledFlicker()
{
  colorPrint("[LED] Flickering status LED...", ANSI_BLUE);
  pinMode(48, OUTPUT); // Ensure pin is output
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(48, HIGH);
    delay(83); // ~1/12th second ON
    digitalWrite(48, LOW);
    delay(83); // ~1/12th second OFF (Total cycle ~1/6th sec, 6 cycles ~1 sec)
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
  Serial.println(gps.satellites.isValid() ? String(gps.satellites.value()) : "N/A"); // Check validity
  Serial.print("  HDOP: ");
  Serial.println(gps.hdop.isValid() ? String(gps.hdop.value()) : "N/A"); // Check validity
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
    colorPrint("[GPS] Waking up GPS module...", ANSI_YELLOW);
    gpsIsAwake = true;
    // Add a small delay for the module to stabilize after wake-up
    delay(100);
    // Re-initialize serial connection? Usually not needed after light sleep, but test.
    // gpsSerial1.begin(GPS_BAUD); // Uncomment if GPS communication fails after wake
  }
  else
  {
    colorPrint("[GPS] Already awake.", ANSI_YELLOW);
  }
}

void gpsSleep()
{
  if (gpsIsAwake)
  {
    // Before sleeping, ensure any pending serial data is sent (optional but good practice)
    Serial.flush();
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
