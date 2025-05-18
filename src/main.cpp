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
#include <BLEDevice.h>      // <-- Add BLE headers
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_attr.h> // For RTC_DATA_ATTR

// Remove FlickerType enum and ledFlicker(FlickerType)
// Add three simple flicker functions

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
#define STATUS_LED_PIN 48             // Pin for the status LED used in ledFlicker

// #define SENDER_ID "Podge"
#define SENDER_ID "Macy"
// #define SENDER_ID "Simba"
// #define SENDER_ID "Gizmo"
#define HOME_LAT 51.87370573411073
#define HOME_LON -2.2396017778476716
// #define SEND_INTERVAL 60000 // milliseconds (60 seconds) - Commented out, using sleep timer

// --- Device ID Mapping ---
uint16_t DEVICE_ID_HEX = 0x0000; // Default/Unknown ID

// Sleep Configuration
// ┌─────────────────┐
// │ Sleep Settings  │
// └─────────────────┘

// #define SLEEP_DURATION_US (30 * 1000000ULL) // 30 seconds in microseconds <-- REMOVE OLD DEFINE
#define SLEEP_DURATION_AWAY_US (10 * 1000000ULL)                   // 10 seconds when away
#define SLEEP_DURATION_HOME_US (10 * 1000000ULL)                   // 10 seconds when home
volatile uint64_t currentSleepDurationUs = SLEEP_DURATION_AWAY_US; // Variable to hold current sleep duration, default to away

// BLE Configuration
const char *targetDeviceName = "CAT_TRACKER_HQ";
BLEScan *pBLEScan = nullptr;  // Initialize to nullptr
volatile bool isHome = false; // Flag to indicate if home beacon is detected

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

// Forward Declarations
void colorPrint(const String &message, const char *color = ANSI_RESET); // Moved earlier
void printStatusReport();
void gpsWake();
void gpsSleep();
String cardinalDirection(double bearing);
void processGps();
void periodicStatusUpdate();
void transmitLora(String payload);
String buildJsonPayload();
void craftLoraPacket();
void handleWakeupReason();
void handleLoraReception();
void performTransmissionSequence();
void goToLightSleep();
bool scanForHomeBeacon(uint32_t scanDurationSeconds);

void flickerShort()
{
  colorPrint("[LED] Flickering status LED (short)...", ANSI_BLUE);
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(30);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(30);
  }
}
void flickerMedium()
{
  colorPrint("[LED] Flickering status LED (medium)...", ANSI_BLUE);
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(50);
  }
}
void flickerLong()
{
  colorPrint("[LED] Flickering status LED (long)...", ANSI_BLUE);
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 12; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(80);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
  }
}

// --- RTC Memory Flag ---
RTC_DATA_ATTR int bootFlag = 0; // 0 = cold boot, 1 = woke from sleep

// --- BLE Callback Class ---
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str()); // Debug: Print all found devices
    if (advertisedDevice.haveName() && advertisedDevice.getName() == targetDeviceName)
    {
      colorPrint("[BLE] Found Home Beacon! (" + String(targetDeviceName) + ")", ANSI_BRIGHT_GREEN);
      isHome = true;
      // Stop scan early if target is found
      // Check if pBLEScan is initialized before using
      if (pBLEScan != nullptr) // Simplified check
      {                        // Check if scanning before stopping
        // No need to check isScanning() if stop() is idempotent (safe to call multiple times)
        pBLEScan->stop();
        colorPrint("[BLE] Scan stopped early.", ANSI_BLUE);
      }
    }
  }
};

// Hardware Instances
SPIClass LoRaSPI(HSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use UART peripheral 1 (adjust if needed)
// HardwareSerial gpsSerial(44, 43); // <-- Constructor doesn't take pins

// --- Create single BLE Callback instance ---
MyAdvertisedDeviceCallbacks bleCallbacks; // Create one instance globally

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

// --- SETUP ---
// ┌─────────────────┐
// │   Setup Loop    │
// └─────────────────┘
void setup()
{
  // --- Check RTC flag and wakeup reason ---
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (bootFlag == 1 && (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 || wakeup_reason == ESP_SLEEP_WAKEUP_GPIO))
  {
    // Woke from sleep, skip full setup
    Serial.begin(115200);
    colorPrint("[WAKE] Skipping full setup, resuming from sleep...", ANSI_BRIGHT_YELLOW);
    // Reset flag for next sleep cycle
    bootFlag = 0;
    return;
  }
  bootFlag = 0; // Ensure flag is cleared on cold boot

  delay(1000);                                                                 // Wait for serial monitor
  Serial.begin(115200);                                                        // <-- Initialize Serial communication first
  Serial.println("\n[BOOT] Serial connection established. Starting setup..."); // Adjusted message
  delay(200);                                                                  // Give some time for the serial monitor to open
  colorPrint("[BOOT] Initialising CAT TRACKER TX v2 (Sleep Enabled)...");
  flickerShort(); // Flicker short on initial startup
  delay(200);     // Give some time for the serial monitor to open
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT); // Status LED

  // --- Button Pin Setup ---
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP); // Configure button pin

  // --- Determine Device ID based on SENDER_ID ---
  String sender = SENDER_ID; // Convert macro to String for comparison
  if (sender == "Podge")
  {
    DEVICE_ID_HEX = 0x1111;
  }
  else if (sender == "Macy")
  {
    DEVICE_ID_HEX = 0x2222;
  }
  else if (sender == "Gizmo")
  {
    DEVICE_ID_HEX = 0x3333;
  }
  else if (sender == "Simba")
  {
    DEVICE_ID_HEX = 0x4444;
  }
  else if (sender == "Carrie")
  {
    DEVICE_ID_HEX = 0x5555;
  }
  else if (sender == "Chloe")
  {
    DEVICE_ID_HEX = 0x6666;
  }
  else
  {
    DEVICE_ID_HEX = 0xFFFF; // Indicate an unknown/unmapped ID
    colorPrint("[WARN] Unknown SENDER_ID: " + sender + ". Using default hex ID.", ANSI_YELLOW);
  }
  colorPrint("[INIT] Device ID set to: 0x" + String(DEVICE_ID_HEX, HEX), ANSI_BLUE);

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
  bool firstFixDetectedSetup = false;       // Flag for first fix detection in setup
  unsigned long firstFixTimestampSetup = 0; // Timestamp for first fix in setup

  while (millis() - gpsWarmupStart < 60000) // Wait for 60 seconds
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
    flickerLong(); // Flicker long for error
    while (true)
      ; // Halt on critical error
  }
  else
  {
    colorPrint("[OK] LoRa initialised successfully");
    // Apply LoRa parameters
    lora.setOutputPower(18); // Set output power (dBm)
    lora.setSpreadingFactor(8);
    lora.setBandwidth(250.0);
    lora.setCodingRate(5);
    lora.setCRC(true);
    lora.setPreambleLength(8);
    // Set DIO1 mask for RxDone interrupt
    lora.setDio1Action(handleLoraReception); // Call this function when DIO1 goes HIGH

    colorPrint("[INIT] LoRa Params configured.");
  }

  // --- BLE Init ---
  colorPrint("[INIT] Initializing BLE...", ANSI_BLUE);
  BLEDevice::init("");             // Initialize BLE device with no name (scanner role)
  pBLEScan = BLEDevice::getScan(); // Get the scanner instance
  if (pBLEScan == nullptr)
  {
    colorPrint("[INIT ERROR] Failed to get BLE Scanner instance!", ANSI_RED);
    flickerLong(); // Flicker long for error
    // Handle error appropriately - maybe halt or disable BLE feature
  }
  else
  {
    pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks); // Use address of the global instance
    pBLEScan->setActiveScan(true);                         // Active scan uses more power but is needed to get device names
    pBLEScan->setInterval(100);                            // Scan interval (milliseconds)
    pBLEScan->setWindow(99);                               // Scan window (milliseconds), must be <= interval
    colorPrint("[INIT] BLE Scanner Initialized.", ANSI_BLUE);
  }

  colorPrint("════════════════════════════════════════", ANSI_BOLD);
  colorPrint("🚀 SETUP COMPLETE - Entering initial sleep cycle 😴", ANSI_BOLD);
  colorPrint("════════════════════════════════════════", ANSI_BOLD);

  // Set initial lastSendTime to allow first send after interval - No longer needed
  // lastSendTime = millis() - SEND_INTERVAL + 5000;
  gpsSleep();
  delay(1000);
  lora.standby();
  // flickerMedium(); // Go to sleep for the first time
  bootFlag = 1;
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
  // Reinitialize Serial after waking up
  // Serial.begin(115200);                                        // Reopen the serial port
  Serial.println("\n[WAKE] Serial connection reestablished."); // Log wake-up

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
    flickerShort();                // Flicker LED after sequence
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

  // --- 1. Attempt BLE Home Beacon Scan (First 10 seconds) ---
  colorPrint("[SEQUENCE] Scanning for Home Beacon (10s)...", ANSI_YELLOW);
  bool foundHome = scanForHomeBeacon(10); // Scan for 10 seconds, returns true if home found

  // --- Adjust Sleep Duration based on BLE Scan ---
  if (foundHome)
  {
    currentSleepDurationUs = SLEEP_DURATION_HOME_US;
    colorPrint("[SEQUENCE] Home detected. Setting sleep duration to " + String(currentSleepDurationUs / 1000000ULL) + "s.", ANSI_BRIGHT_GREEN);
  }
  else
  {
    currentSleepDurationUs = SLEEP_DURATION_AWAY_US;
    colorPrint("[SEQUENCE] Home not detected. Setting sleep duration to " + String(currentSleepDurationUs / 1000000ULL) + "s.", ANSI_YELLOW);
  }

  // --- 2. GPS Fix Attempt (Only if Home Beacon NOT Found) ---
  if (!foundHome)
  { // Use the return value from the scan function
    colorPrint("[SEQUENCE] Home Beacon not found. Proceeding with GPS fix attempt...", ANSI_YELLOW);
    // Wake GPS & Attempt Fix/Update
    gpsWake(); // Ensure GPS is awake and serial is initialized
    colorPrint("[SEQUENCE] Attempting GPS fix/update (Max Wait: 60s)...", ANSI_YELLOW);
    unsigned long fixAttemptStart = millis();
    bool fixObtainedDuringWait = false;  // Flag to track if we completed the stabilization wait
    bool firstFixDetected = false;       // Flag to track if we've seen the *first* fix
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

    // Put GPS to sleep AFTER potential fix attempt
    gpsSleep();
  }
  else
  {
    // Home beacon WAS found
    colorPrint("[SEQUENCE] Home Beacon detected. Skipping GPS fix attempt.", ANSI_BRIGHT_GREEN);
    gpsSleep(); // Ensure GPS is put back to sleep if it wasn't already
  }

  // --- 3. Build and Transmit LoRa Packet ---
  craftLoraPacket(); // Call renamed function

  // --- 4. Post-Transmission ---
  // GPS is already put to sleep either after successful scan or after GPS attempt.

  colorPrint("[SEQUENCE] Transmission Sequence Complete.", ANSI_MAGENTA); // Use standard color
}

void goToLightSleep()
{
  colorPrint("[SLEEP] Preparing for light sleep...", ANSI_BLUE);

  // Clear received message buffer before sleeping
  LoRaRxMsg = "";

  // Reset isHome flag before sleeping
  // Note: The decision on sleep duration is already made in performTransmissionSequence based on the scan result.
  // We still reset the flag here for the next cycle's scan logic.
  if (isHome)
  { // Only print if it was true
    colorPrint("[SLEEP] Resetting Home Beacon flag.", ANSI_BLUE);
    isHome = false;
  }

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

  // Timer Wakeup - Use the dynamically set duration
  esp_sleep_enable_timer_wakeup(currentSleepDurationUs);
  colorPrint("[SLEEP] Wakeup enabled: Timer (" + String(currentSleepDurationUs / 1000000ULL) + "s)", ANSI_BLUE); // Use variable here

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
  // Serial.end();   // Removed to prevent boot loop
  gpsSleep(); // Ensure GPS is in sleep mode before going to light sleep

  // --- Optional: Stop BLE Scan explicitly before sleep ---
  // Although the scan started in scanForHomeBeacon should have stopped,
  // it's safer to ensure it's stopped before sleeping.
  // if (pBLEScan != nullptr && pBLEScan->isScanning()) // <-- Commenting out potentially problematic check
  if (pBLEScan != nullptr) // Simplified check: just ensure pBLEScan is not null before stopping
  {
    // Check if it *was* scanning before stopping (optional, for logging)
    // bool was_scanning = pBLEScan->isScanning(); // Assuming isScanning() doesn't exist or is unreliable
    // if(was_scanning) { // Log only if it was scanning
    colorPrint("[SLEEP] Stopping any active BLE scan...", ANSI_BLUE);
    pBLEScan->stop();
    // }
  }
  // --- Optional: Deinitialize BLE to save more power ---
  BLEDevice::deinit(true);                             // Set to true to release memory
  colorPrint("[SLEEP] BLE Deinitialized.", ANSI_BLUE); // <-- Add log message
  // Note: If deinitialized, BLEDevice::init() must be called again after wake-up if needed.
  // For simplicity now, let's not deinit unless power saving is critical.

  bootFlag = 1; // Set RTC flag before sleeping
  // Enter light sleep
  esp_light_sleep_start();

  // --- Execution resumes in loop() after wake-up ---
}

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
  StaticJsonDocument<300> doc; // Corrected for ArduinoJson v7: specify capacity as template argument

  // Add static fields with shorter keys
  doc["mid"] = messageId;                         // msg_id -> mid
  doc["did"] = "0x" + String(DEVICE_ID_HEX, HEX); // device_id -> did
  doc["id"] = SENDER_ID;                          // id remains id

  // --- Check if Home Beacon was detected ---
  if (isHome)
  {
    doc["stat"] = "H"; // status -> stat, "Home" -> "H"
    // Truncate home coordinates before assigning
    double truncatedHomeLat = round(HOME_LAT * 100000.0) / 100000.0;
    double truncatedHomeLon = round(HOME_LON * 100000.0) / 100000.0;
    doc["lat"] = truncatedHomeLat;                                 // Assign truncated value
    doc["lon"] = truncatedHomeLon;                                 // Assign truncated value
    doc["sat"] = -1;                                               // sats -> sat
    doc["dst"] = 0.0;                                              // dist_m -> dst
    doc["dir"] = "NA";                                             // bearing -> dir, "N/A" -> "NA"
    doc["ts"] = "NA";                                              // time -> ts, "N/A" -> "NA"
    colorPrint("[JSON] Building payload: Status=Home", ANSI_CYAN); // Added log
  }
  else
  {
    // --- Home Beacon NOT detected, use GPS Data ---
    bool loc_valid = gps.location.isValid();
    unsigned long loc_age = gps.location.age(); // Keep age to determine staleness
    bool sat_valid = gps.satellites.isValid();
    unsigned long sat_value = gps.satellites.value();
    bool time_valid = gps.time.isValid();
    unsigned long time_age = gps.time.age();

    // Consider location stale if valid but age is > 60 seconds
    bool isStale = loc_valid && loc_age > 60000;

    if (loc_valid && !isStale)
    {
      double currentLat = gps.location.lat();
      double currentLon = gps.location.lng();

      doc["stat"] = "O"; // status -> stat, "outanabout" -> "O"
      // Truncate current coordinates before assigning
      double truncatedCurrentLat = round(currentLat * 100000.0) / 100000.0;
      double truncatedCurrentLon = round(currentLon * 100000.0) / 100000.0;
      doc["lat"] = truncatedCurrentLat;       // Assign truncated value
      doc["lon"] = truncatedCurrentLon;       // Assign truncated value
      doc["sat"] = sat_valid ? sat_value : 0; // sats -> sat

      // Calculate distance/bearing
      double dist = TinyGPSPlus::distanceBetween(currentLat, currentLon, HOME_LAT, HOME_LON);
      double bearing = TinyGPSPlus::courseTo(currentLat, currentLon, HOME_LAT, HOME_LON);
      // Keep bearing calculation, but use shorter key and value
      String bearingStr = String((int)bearing) + "-" + cardinalDirection(bearing);
      doc["dst"] = round(dist * 100.0) / 100.0;                                        // dist_m -> dst
      doc["dir"] = bearingStr;                                                         // bearing -> dir
      colorPrint("[JSON] Building payload: Status=outanabout (Valid GPS)", ANSI_CYAN); // Added log
    }
    else // Location Invalid or Stale
    {
      doc["stat"] = "E"; // status -> stat, "error" -> "E"
      doc["lat"] = 0.0;  // No truncation needed for 0.0
      doc["lon"] = 0.0;
      doc["sat"] = sat_valid ? sat_value : 0;                                               // sats -> sat
      doc["dst"] = 0.0;                                                                     // dist_m -> dst
      doc["dir"] = "NA";                                                                    // bearing -> dir, "N/A" -> "NA"
      colorPrint("[JSON] Building payload: Status=error (Invalid/Stale GPS)", ANSI_YELLOW); // Added log
    }

    // --- Add Timestamp if valid and recent ---
    if (time_valid && time_age < 60000)
    { // Check age too
      char isoTimestamp[25];
      snprintf(isoTimestamp, sizeof(isoTimestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second());
      doc["ts"] = isoTimestamp; // time -> ts
    }
    else
    {
      doc["ts"] = "E"; // time -> ts, "error" -> "E"
    }
  } // End of else (isHome == false)

  char payload[256];
  serializeJson(doc, payload, sizeof(payload));
  doc.clear(); // Clear JSON doc memory

  // --- Debug Print (only if Serial is somehow available) ---
  // colorPrint("[JSON] Payload (" + String(strlen(payload)) + " bytes): " + String(payload), ANSI_CYAN);

  return String(payload);
}

// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                █
// █   ██       ██████  ██████   █████      ████████ ██   ██       █
// █   ██      ██    ██ ██   ██ ██   ██        ██     ██ ██        █
// █   ██      ██    ██ ██████  ███████        ██      ███         █
// █   ██      ██    ██ ██   ██ ██   ██        ██     ██ ██        █
// █   ███████  ██████  ██   ██ ██   ██        ██    ██   ██       █
// ██ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █ █                                                                 █
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
  // Check validity before printing value
  Serial.println(gps.satellites.isValid() ? String(gps.satellites.value()) : "Invalid");
  Serial.print("  HDOP: ");
  // Check validity before printing value
  Serial.println(gps.hdop.isValid() ? String(gps.hdop.value() / 100.0) : "Invalid"); // HDOP is often scaled by 100
  Serial.print("  Last Rx Msg: ");
  Serial.println(LoRaRxMsg.length() > 0 ? LoRaRxMsg : "None");
  Serial.print("  Free Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("  Wakeup Cause: ");
  Serial.println(esp_sleep_get_wakeup_cause()); // This should be fine
  colorPrint("-------------------------------------------------------", ANSI_BOLD);
}

void colorPrint(const String &message, const char *color)
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

// --- BLE Scan for Home Beacon ---
bool scanForHomeBeacon(uint32_t scanDurationSeconds)
{
  // --- Re-initialize BLE after waking from sleep (if deinitialized) ---
  colorPrint("[BLE] Initializing BLE for scan...", ANSI_BLUE);
  BLEDevice::init("");             // Initialize BLE device
  pBLEScan = BLEDevice::getScan(); // Get the scanner instance
  if (pBLEScan == nullptr)
  {
    colorPrint("[BLE ERROR] Failed to get BLE Scanner instance after re-init!", ANSI_RED);
    return false; // Cannot proceed without scanner
  }
  // Re-apply callbacks and scan parameters
  // Note: If BLEDevice::deinit() is NOT used in goToLightSleep, some of this re-init might be skippable.
  // However, re-applying ensures a known state after wake-up.
  pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks); // Use address of the global instance
  pBLEScan->setActiveScan(true);                         // Active scan uses more power but is needed to get device names
  pBLEScan->setInterval(100);                            // Scan interval (milliseconds)
  pBLEScan->setWindow(99);                               // Scan window (milliseconds), must be <= interval
  colorPrint("[BLE] BLE Scanner Re-initialized.", ANSI_BLUE);
  // --- End Re-initialization ---

  // No need for the null check here again as it's done after re-init
  // if (pBLEScan == nullptr)
  // {
  //   colorPrint("[BLE ERROR] Scanner not initialized! Cannot scan.", ANSI_RED);
  //   return false;
  // }

  colorPrint("[BLE] Starting scan for \"" + String(targetDeviceName) + "\" (" + String(scanDurationSeconds) + "s)...", ANSI_BLUE);
  isHome = false; // Reset flag before each scan

  // REMOVE Redundant/Partial Re-init lines:
  // BLEDevice::init("");
  // pBLEScan = BLEDevice::getScan();
  // pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // pBLEScan->setActiveScan(true);
  // pBLEScan->setInterval(100);
  // pBLEScan->setWindow(99);

  // Start the scan. The scan runs for scanDurationSeconds.
  // The callback (MyAdvertisedDeviceCallbacks::onResult) will set isHome = true
  // and call pBLEScan->stop() if the target device is found.
  // The 'false' argument means the scan is non-blocking.
  pBLEScan->start(scanDurationSeconds, false);

  // We need to wait here for the scan to complete or be stopped by the callback.
  // Since the scan runs in the background, we can use a simple delay loop.
  // Check isHome periodically to allow early exit if the beacon is found quickly.
  unsigned long scanStartTime = millis();
  while (millis() - scanStartTime < (scanDurationSeconds * 1000))
  {
    if (isHome)
    {
      // Beacon found by callback, which should have also stopped the scan.
      break; // Exit the wait loop early
    }
    delay(100); // Wait a bit before checking again
  }
  // After scan duration or early exit, check the flag
  if (isHome)
  {
    colorPrint("[BLE] Scan complete. Home Beacon FOUND!", ANSI_BRIGHT_GREEN);
  }
  else
  {
    colorPrint("[BLE] Scan complete. Home Beacon NOT found.", ANSI_YELLOW);

    // Try to gracefully stop the scan
    try
    {
      // Add a short delay to allow any pending BLE operations to complete
      delay(50);

      colorPrint("[BLE] Attempting to stop scan...", ANSI_BLUE);
      // Try to stop the scan but handle any errors that might occur
      if (pBLEScan != nullptr)
      {
        pBLEScan->stop();
        colorPrint("[BLE] Scan stopped (no error code available)", ANSI_GREEN);
      }
    }
    catch (...)
    {
      // In case of any unexpected exception, just log it and continue
      colorPrint("[BLE] Exception while stopping scan - continuing anyway", ANSI_RED);
    }
  }

  // Optional: Deinitialize BLE here if deep power saving is needed between scans
  // BLEDevice::deinit(true);

  return isHome;
}

// ...existing code...
// --- transmitLora function ---
void transmitLora(String payload)
{
  colorPrint("[LORA TX] Preparing to transmit...", ANSI_BLUE);
  // --- TRANSMIT MARKER ---
  colorPrint("=== TRANSMITTING PAYLOAD OVER LORA ===", "\033[38;5;208m"); // ORANGE, CAPS
  colorPrint("[LORA TX] Transmitting packet (" + String(payload.length()) + " bytes)...", ANSI_BLUE);
  colorPrint("  Payload: " + payload, ANSI_CYAN);
  int state = lora.standby();
  if (state != RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX ERROR] Failed to enter standby before transmit: " + String(state), ANSI_RED);
    flickerLong(); // Flicker long for error
  }
  state = lora.transmit(payload);
  if (state == RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX] Transmission successful!", ANSI_BRIGHT_GREEN);
    flickerMedium(); // Flicker medium on successful transmission
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    colorPrint("[LORA TX ERROR] Packet too long!", ANSI_RED);
    flickerLong(); // Flicker long for error
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    colorPrint("[LORA TX ERROR] Transmission timeout!", ANSI_RED);
    flickerLong(); // Flicker long for error
  }
  else
  {
    colorPrint("[LORA TX ERROR] Transmission failed, code: " + String(state), ANSI_RED);
    flickerLong(); // Flicker long for error
  }

  // 3. Put LoRa back into a low-power state or receive mode after transmission
  // For this application, we usually go back to sleep, which sets receive mode.
  // If not sleeping immediately, explicitly set standby or receive here.
  // lora.standby(); // Or lora.startReceive() if needed immediately
}
// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// █                                                                             █
// █                             END OF CODE                                     █
// █                                                                             █
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀