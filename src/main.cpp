/*
  ┌──────────────────────────────────────────────┐
  │ CAT TRACKER TX — LoRa GPS Collar             │
  │ SX1262 + TinyGPSPlus + Binary TLV Protocol   │
  └──────────────────────────────────────────────┘

  This is the main firmware for the pet tracker COLLAR (transmitter).
  It runs on a Seeeduino XIAO ESP32S3 with:
    - SX1262 LoRa radio module (long-range communication)
    - NEO-6M GPS module (location tracking)
    - BLE (Bluetooth) for detecting a home beacon
    - Status LED + button for user interaction

  WHAT IT DOES (each wake cycle):
  1. Scans for the BLE home beacon to check if the pet is at home
  2. If not home, gets a GPS fix to determine location
  3. Builds a binary telemetry packet with status/location/battery info
  4. Transmits the packet over LoRa to the base station
  5. Opens a 2-second receive window to listen for commands from base station
  6. Goes back to sleep until the next cycle

  The base station can remotely change the collar's operating mode
  (normal/powersave/active/lost) by sending command packets.

  Binary TLV protocol compatible with BluePawzReceiver.
  protocol.h and config.h MUST be identical on TX and RX.
*/

#include <Arduino.h>            // Core Arduino functions (digitalWrite, millis, delay, Serial, etc.)
#include <RadioLib.h>           // LoRa radio library — drives the SX1262 module
#include <TinyGPS++.h>          // GPS NMEA sentence parser — turns raw GPS data into lat/lon/speed/etc.
#include <esp_sleep.h>          // ESP32 sleep modes — light sleep with timer/GPIO wakeup
#include <HardwareSerial.h>     // Hardware UART — used for GPS serial communication
#include <BLEDevice.h>          // ESP32 BLE — initialize Bluetooth Low Energy
#include <BLEUtils.h>           // BLE utility classes
#include <BLEScan.h>            // BLE scanning — used to find the home beacon
#include <BLEAdvertisedDevice.h> // BLE advertised device info (name, RSSI, etc.)
#include <esp_attr.h>           // ESP32 attributes — RTC_DATA_ATTR for data that survives sleep

#include "protocol.h" // Binary packet format, CRC, TLV builders/parsers (shared with receiver)
#include "config.h"   // Operating modes, LoRa params, GPS/BLE settings (shared with receiver)

// ═══════════════════════════════════════════════
// Device Identity — Change this per collar!
// ═══════════════════════════════════════════════
// Each collar needs a unique ID so the base station can tell them apart.
// Valid range: 0x0001 to 0xFFFE (0x0000 = base station, 0xFFFF = broadcast).
// IMPORTANT: Change this before flashing each new collar!
#define MY_DEVICE_ID 0x0001

// ═══════════════════════════════════════════════
// Pin Definitions — Hardware wiring
// ═══════════════════════════════════════════════
// These define which ESP32 GPIO pins connect to which hardware.
// If you rewire anything, update these to match.

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // Fallback built-in LED pin (usually not used, we use STATUS_LED_PIN instead)
#endif

// SX1262 LoRa radio module — connected via SPI bus
#define LORA_NSS 41   // SPI Chip Select (active low) — tells the LoRa module "I'm talking to you"
#define LORA_SCK 7    // SPI Clock — synchronizes data transfer
#define LORA_MOSI 9   // SPI Master Out Slave In — data FROM ESP32 TO LoRa module
#define LORA_MISO 8   // SPI Master In Slave Out — data FROM LoRa module TO ESP32
#define LORA_RST 42   // Hardware reset pin — pull low to reset the LoRa module
#define LORA_BUSY 40  // Busy indicator — LoRa module pulls this HIGH when it's processing
#define LORA_DIO1 39  // Digital I/O 1 — LoRa module triggers this on RX/TX complete (interrupt)

// NEO-6M GPS module — connected via UART (serial)
#define GPS_RX 44         // ESP32 RX pin ← GPS TX (receives NMEA sentences from GPS)
#define GPS_TX 43         // ESP32 TX pin → GPS RX (sends commands to GPS, rarely used)
#define GPS_BAUD 9600     // GPS module communicates at 9600 baud (standard for NEO-6M)
#define GPS_SLEEP_WAKE 1  // GPIO to control GPS power — HIGH = awake, LOW = sleep
#define GPS_RESET 3       // GPS hardware reset pin — pull LOW to reset, keep HIGH for normal operation

// User interface pins
#define STATUS_BUTTON_PIN GPIO_NUM_21 // Physical button — press to wake from sleep and show status
#define STATUS_LED_PIN 48             // RGB/status LED — flashes to show transmission status and lost mode beacon

// ═══════════════════════════════════════════════
// Home Location — Where "home" is on the map
// ═══════════════════════════════════════════════
// When the collar gets a GPS fix, it calculates distance to these coordinates.
// If the pet is within HOME_RADIUS_M meters, it's considered "at home" via GPS.
// (BLE beacon detection is preferred when available — more reliable and saves battery.)
#define HOME_LAT 51.87370573411073    // Home latitude in decimal degrees
#define HOME_LON -2.2396017778476716  // Home longitude in decimal degrees
#define HOME_RADIUS_M 20.0            // "At home" radius in meters. If within this distance, pet is home.

// ═══════════════════════════════════════════════
// Timing
// ═══════════════════════════════════════════════
#define COMMAND_LISTEN_MS 2000 // After each TX, listen for this many ms for commands from base station

// ═══════════════════════════════════════════════
// ANSI Color Codes — Pretty serial debug output
// ═══════════════════════════════════════════════
// These escape codes make the serial monitor output colorful and easier to read.
// Each category of message (GPS, BLE, LoRa, errors, etc.) gets its own color.
// Only works in terminals that support ANSI codes (PlatformIO monitor, PuTTY, etc.).
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
#define ANSI_BOLD "\033[1m"
#define ANSI_RESET "\033[0m"

// ═══════════════════════════════════════════════
// Forward Declarations
// ═══════════════════════════════════════════════
// These tell the compiler about functions that are defined later in this file.
// Needed because some functions call each other in ways the compiler can't
// resolve by just reading top-to-bottom.
void colorPrint(const String &message, const char *color = ANSI_RESET);
void printStatusReport();
void gpsWake();
void gpsSleep();
void processGps();
void performTransmissionSequence();
void goToLightSleep();
bool scanForHomeBeacon(uint32_t scanDurationSeconds);
void handleWakeupReason();
void handleLoraReception();

// Binary protocol packet senders
void sendTelemetry();                                        // Build and send a PKT_TELEMETRY with location/status
void sendModeAck(uint32_t cmdMsgSeq);                       // Acknowledge a mode change command from base station
void sendStatusResponse(uint32_t cmdMsgSeq);                // Respond to a status query from base station
void sendLostModeTimeoutAlert();                            // Alert base station that lost mode timed out
void listenForCommands();                                   // Open RX window and process any incoming commands
void handleReceivedCommand(const uint8_t *buf, uint8_t len); // Parse and act on a received binary command
void applyProfile(bp_profile_t profile);                    // Switch to a different operating mode
void transmitBinaryPacket(uint8_t *buf, uint8_t len);       // Send a raw binary packet over LoRa

// LED indicator functions
void flickerShort();  // 3 quick flashes — initialization/status feedback
void flickerMedium(); // Mode-dependent number of flashes — successful transmission
void flickerLong();   // 12 flashes — error indicator (LoRa init failed, etc.)
void ledBeacon();     // Single flash — continuous beacon in lost mode

// ═══════════════════════════════════════════════
// Hardware Instances — The actual hardware objects
// ═══════════════════════════════════════════════
// These are the driver objects that talk to the physical hardware.

SPIClass LoRaSPI(HSPI);  // SPI bus instance for LoRa (using the HSPI peripheral, not the default VSPI)
// Create the SX1262 LoRa radio object, telling RadioLib which pins to use
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;              // GPS parser — feed it raw NMEA bytes and it gives you lat/lon/speed/time
HardwareSerial gpsSerial(1);  // UART1 for GPS communication (ESP32 has 3 hardware UARTs: 0, 1, 2)

// ═══════════════════════════════════════════════
// BLE — Bluetooth Low Energy for home beacon detection
// ═══════════════════════════════════════════════
BLEScan *pBLEScan = nullptr;   // Pointer to the BLE scanner instance (initialized in setup())
volatile bool isHome = false;  // Set to true by BLE callback when home beacon is found. Volatile because it's set in a callback.
uint8_t homeCycleCount = 0;    // Counts how many consecutive cycles the BLE home beacon was detected

// BLE scan callback class — gets called for every BLE device found during a scan.
// We only care about devices with the name "HOME" (or whatever BEACON_NAME is set to).
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // Check if this BLE device has a name and if that name matches our home beacon
    if (advertisedDevice.haveName() && advertisedDevice.getName() == BEACON_NAME)
    {
      colorPrint("[BLE] Found Home Beacon! (" + String(BEACON_NAME) + ")", ANSI_BRIGHT_GREEN);
      isHome = true; // Signal to the main code that we're home
      // Stop scanning early — no point continuing once we've found the beacon
      if (pBLEScan != nullptr)
      {
        pBLEScan->stop();
        colorPrint("[BLE] Scan stopped early.", ANSI_BLUE);
      }
    }
  }
};
MyAdvertisedDeviceCallbacks bleCallbacks; // Instance of our callback class

// ═══════════════════════════════════════════════
// Global State — Variables that track the collar's current state
// ═══════════════════════════════════════════════
bool gpsIsAwake = true;        // Whether the GPS module is currently powered on
bool gpsWarmStart = false;     // True after the first successful GPS fix — enables faster warm starts
static uint32_t messageSeq = 0; // Monotonically increasing message counter. Each TX gets a unique sequence number.

// RTC_DATA_ATTR means this variable survives light sleep (stored in RTC memory, not main RAM).
// Used to detect if we're waking from sleep (bootFlag=1) vs. a fresh power-on (bootFlag=0).
RTC_DATA_ATTR int bootFlag = 0;

// Operating mode state — which profile is currently active
const OperatingMode *currentMode = &MODE_NORMAL;  // Pointer to the current mode's settings (from config.h)
bp_profile_t currentProfile = PROFILE_NORMAL;      // Enum value of the current profile (for sending in packets)

// Lost mode tracking — monitors how long we've been in lost mode
unsigned long lostModeStartTime = 0; // millis() timestamp when lost mode was activated
bool inLostMode = false;             // Whether lost mode is currently active

// Timed loop control (sleep disabled for debugging — using millis-based timing instead)
unsigned long lastSendTime = 0; // millis() timestamp of the last transmission cycle

// ═══════════════════════════════════════════════
// LED Functions — Visual feedback via the status LED
// ═══════════════════════════════════════════════

// Quick triple-flash. Used for:
// - Boot/initialization confirmation
// - Button press acknowledgment
// - Minor status events
void flickerShort()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH); // LED on
    delay(30);                          // 30ms on
    digitalWrite(STATUS_LED_PIN, LOW);  // LED off
    delay(30);                          // 30ms off
  }
}

// Mode-dependent flash count. Used after successful transmission.
// The number of flashes comes from currentMode->led_flash_count
// (5 for most modes, 10 for lost mode — more visible).
void flickerMedium()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < currentMode->led_flash_count; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50); // 50ms on — slightly longer than flickerShort for visibility
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(50);
  }
}

// Long error flash — 12 rapid blinks. Used when something goes wrong
// (e.g., LoRa module failed to initialize). Hard to miss.
void flickerLong()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 12; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(80); // 80ms on — slower, more deliberate flashing
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
  }
}

// Single beacon flash. Only active when in lost mode (led_beacon_mode=true).
// Called periodically from the main loop to create a continuous flashing effect
// that helps the owner visually locate the collar in the dark.
void ledBeacon()
{
  if (currentMode->led_beacon_mode)
  {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100); // 100ms flash — bright enough to spot
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

// ═══════════════════════════════════════════════
// SETUP — Runs once on power-on or after deep reset
// ═══════════════════════════════════════════════
// This function initializes all the hardware (GPS, LoRa, BLE, LEDs, pins).
// If we're waking from light sleep (bootFlag=1), we skip the full setup
// because the hardware is already configured — saves time and power.
void setup()
{
  Serial.begin(115200); // Start USB serial for debug output (115200 baud)
  delay(1000);          // Give the serial port time to connect

  // Check if this is a wake-from-sleep rather than a fresh boot.
  // If bootFlag was set to 1 before sleeping, and we woke via timer/button/GPIO,
  // we can skip the expensive full initialization.
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (bootFlag == 1 && (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER ||
                        wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 ||
                        wakeup_reason == ESP_SLEEP_WAKEUP_GPIO))
  {
    colorPrint("[WAKE] Woke from sleep, skipping full setup...", ANSI_BRIGHT_YELLOW);
    bootFlag = 0; // Clear the flag so next fresh boot does full setup
    return;       // Skip directly to loop()
  }

  // ── Full initialization (fresh boot / first power on) ──
  bootFlag = 0;
  Serial.println("\n[BOOT] Serial connection established. Starting setup...");
  delay(200);
  colorPrint("[BOOT] Initialising CAT TRACKER TX v3 (Binary TLV Protocol)...");
  colorPrint("[BOOT] Device: " + String(getDeviceName(MY_DEVICE_ID)) +
             " (ID: 0x" + String(MY_DEVICE_ID, HEX) + ")", ANSI_BRIGHT_CYAN);
  flickerShort(); // Visual confirmation that we're booting
  delay(200);

  // Configure basic I/O pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);               // Built-in LED off
  pinMode(STATUS_LED_PIN, OUTPUT);              // Status LED for flashing
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP);     // Button with internal pull-up (active LOW when pressed)

  // ─── GPS Initialization ───
  // Set up the GPS module's control pins and start serial communication.
  // Then wait for the GPS to get its first fix (can take up to 60 seconds on cold start).
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);       // Keep GPS out of reset (HIGH = normal operation)
  pinMode(GPS_SLEEP_WAKE, OUTPUT);
  digitalWrite(GPS_SLEEP_WAKE, HIGH);  // Wake up GPS (HIGH = awake)
  gpsIsAwake = true;

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX); // Start UART1 at 9600 baud for GPS
  delay(100);
  colorPrint("[GPS] Waking up GPS module for initial setup...");

  // Quick check: is the GPS actually sending data?
  colorPrint("[GPS] Checking for initial serial activity...", ANSI_YELLOW);
  delay(1000); // Wait 1 second for GPS to start transmitting NMEA sentences
  if (gpsSerial.available() > 0)
  {
    colorPrint("[GPS] Serial data detected! Module appears awake.", ANSI_BRIGHT_GREEN);
    while (gpsSerial.available() > 0)
      gpsSerial.read(); // Flush the buffer — we'll start fresh
  }
  else
  {
    colorPrint("[GPS] No serial data detected after 1s. Check wiring/power.", ANSI_BRIGHT_RED);
  }

  // GPS warmup phase — wait for the module to acquire satellites and get a valid fix.
  // A "cold start" (no prior fix) can take up to 60 seconds.
  // We also require a 15-second stabilization period after the first fix
  // because GPS coordinates tend to jump around initially before settling.
  colorPrint("[GPS] Warming up GPS (waiting for fix)...");
  unsigned long gpsWarmupStart = millis();
  bool fixFound = false;
  bool firstFixDetected = false;
  unsigned long firstFixTimestamp = 0;

  while (millis() - gpsWarmupStart < GPS_COLD_START_TIMEOUT) // Loop until timeout (60s)
  {
    processGps(); // Feed GPS serial data to the TinyGPS++ parser
    // Check for first valid fix (location is valid and data is fresh — less than 5 seconds old)
    if (!firstFixDetected && gps.location.isValid() && gps.location.age() < 5000)
    {
      firstFixDetected = true;
      firstFixTimestamp = millis();
      colorPrint("[GPS] Initial valid fix obtained! Waiting for stability...", ANSI_BRIGHT_GREEN);
    }
    // Once we have a fix, wait for the stabilization period (15 seconds)
    if (firstFixDetected && (millis() - firstFixTimestamp >= GPS_STABILISE_MS))
    {
      colorPrint("[GPS] Stabilization period complete.", ANSI_BRIGHT_GREEN);
      fixFound = true;
      gpsWarmStart = true; // Future cycles can use shorter warm start timeout
      break;
    }
    delay(1); // Small delay to prevent tight-looping
  }

  // Report the GPS warmup result
  if (fixFound)
  {
    colorPrint("[GPS] Initialized with stabilized fix.", ANSI_GREEN);
    processGps(); // One more read to get the latest data
  }
  else if (firstFixDetected)
  {
    colorPrint("[GPS] Initialized with early fix (stabilization incomplete).", ANSI_YELLOW);
    processGps();
  }
  else
  {
    colorPrint("[GPS] Warmup expired without getting fix.", ANSI_RED);
  }

  // ─── LoRa Radio Initialization ───
  // Set up the SX1262 LoRa module via SPI.
  // Configure all radio parameters to match the base station (defined in config.h).
  pinMode(LORA_DIO1, INPUT); // DIO1 is an input — LoRa module drives it for interrupts
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS); // Start the SPI bus
  colorPrint("[INIT] Setting up SPI for LoRa...");
  int initState = lora.begin(LORA_FREQ_MHZ); // Initialize the radio at 915 MHz
  if (initState != RADIOLIB_ERR_NONE)
  {
    // LoRa failed to initialize — this is fatal. Flash error LED and halt.
    colorPrint("[ERROR] LoRa failed to initialise. Code: " + String(initState), ANSI_RED);
    flickerLong();
    while (true) // Infinite loop — can't do anything without the radio
      ;
  }

  // Configure all LoRa radio parameters — these MUST match the base station!
  colorPrint("[OK] LoRa initialised successfully");
  lora.setOutputPower(currentMode->lora_power_dbm); // TX power from current mode profile
  lora.setSpreadingFactor(LORA_SF);                  // SF8 — good range/speed balance
  lora.setBandwidth(LORA_BW_KHZ);                   // 250 kHz bandwidth
  lora.setCodingRate(LORA_CR);                       // 4/5 coding rate
  lora.setCRC(LORA_USE_CRC);                         // Enable hardware CRC
  lora.setPreambleLength(LORA_PREAMBLE);             // 16-symbol preamble
  lora.setSyncWord(LORA_SYNC_WORD);                  // 0x12 private network sync word
  colorPrint("[INIT] LoRa params: " + String(LORA_FREQ_MHZ) + "MHz SF" +
             String(LORA_SF) + " BW" + String(LORA_BW_KHZ) + "kHz CR4/" +
             String(LORA_CR) + " Preamble:" + String(LORA_PREAMBLE) +
             " Power:" + String(currentMode->lora_power_dbm) + "dBm", ANSI_BLUE);

  // ─── BLE Initialization ───
  // Set up Bluetooth Low Energy scanning to detect the home beacon.
  // The base station advertises as a BLE device named "HOME".
  // When the collar detects this beacon, it knows the pet is at home.
  colorPrint("[INIT] Initializing BLE...", ANSI_BLUE);
  BLEDevice::init("");                    // Initialize BLE with no device name (we're just scanning)
  pBLEScan = BLEDevice::getScan();        // Get the scanner instance
  if (pBLEScan == nullptr)
  {
    colorPrint("[INIT ERROR] Failed to get BLE Scanner instance!", ANSI_RED);
    flickerLong();
  }
  else
  {
    pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks); // Register our callback for found devices
    pBLEScan->setActiveScan(true);   // Active scan = asks devices for more info (vs passive listening)
    pBLEScan->setInterval(100);      // Scan interval in ms (how often to scan)
    pBLEScan->setWindow(99);         // Scan window in ms (how long each scan lasts — nearly 100% duty cycle)
    colorPrint("[INIT] BLE Scanner Initialized. Beacon name: \"" + String(BEACON_NAME) + "\"", ANSI_BLUE);
  }

  // ─── Setup Complete ───
  colorPrint("════════════════════════════════════════", ANSI_BOLD);
  colorPrint("SETUP COMPLETE - Binary TLV Protocol Active", ANSI_BOLD);
  colorPrint("════════════════════════════════════════", ANSI_BOLD);

  // Schedule the first transmission to happen ~5 seconds after boot
  // (instead of waiting the full sleep_interval_s which could be 5+ minutes)
  lastSendTime = millis() - (currentMode->sleep_interval_s * 1000UL) + 5000;
  delay(1000);
  lora.standby();  // Put LoRa in standby mode (low power, ready to TX)
  bootFlag = 1;    // Set flag so next wake-from-sleep skips full setup
}

// ═══════════════════════════════════════════════
// MAIN LOOP — Runs continuously after setup()
// ═══════════════════════════════════════════════
// The main loop handles three things:
// 1. Heartbeat — prints a status message every 5 seconds (for debugging)
// 2. Lost mode management — checks if lost mode should timeout
// 3. Transmission cycles — triggers a full TX sequence at the configured interval
void loop()
{
  unsigned long currentTime = millis();
  unsigned long intervalMs = (unsigned long)currentMode->sleep_interval_s * 1000UL; // Convert seconds to ms

  // ── Heartbeat ──
  // Print a status line every 5 seconds so you can see the collar is alive in the serial monitor.
  // Shows current mode and countdown to next transmission.
  static unsigned long lastHeartbeat = 0; // Static = persists between loop() calls
  if (currentTime - lastHeartbeat >= 5000)
  {
    long secsLeft = (long)(intervalMs - (currentTime - lastSendTime)) / 1000;
    if (secsLeft < 0) secsLeft = 0; // Don't show negative countdown
    colorPrint("[HB] Active | Mode: " + String(currentMode->name) +
               " | Next TX in " + String(secsLeft) + "s", ANSI_BLUE);
    lastHeartbeat = currentTime;
  }

  // ── Lost Mode Timeout Check ──
  // If we've been in lost mode for longer than LOST_MODE_MAX_DURATION_S (2 hours),
  // automatically revert to active mode to conserve battery.
  // Sends an alert packet to the base station before switching.
  if (inLostMode && (currentTime - lostModeStartTime >= (unsigned long)LOST_MODE_MAX_DURATION_S * 1000UL))
  {
    colorPrint("[LOST] Lost mode timeout! Reverting to active...", ANSI_BRIGHT_RED);
    sendLostModeTimeoutAlert(); // Tell the base station we're timing out
    applyProfile(PROFILE_ACTIVE); // Switch to active mode
  }

  // ── LED Beacon ──
  // In lost mode, flash the LED at regular intervals so the owner can spot the collar.
  // The interval comes from currentMode->led_beacon_interval_ms (default: 2000ms = every 2 seconds).
  if (inLostMode && currentMode->led_beacon_mode)
  {
    static unsigned long lastBeacon = 0;
    if (currentTime - lastBeacon >= currentMode->led_beacon_interval_ms)
    {
      ledBeacon();
      lastBeacon = currentTime;
    }
  }

  // ── Transmission Cycle ──
  // When enough time has passed since the last TX (based on the current mode's sleep interval),
  // run a full transmission sequence: BLE scan → GPS fix → send telemetry → listen for commands.
  if (currentTime - lastSendTime >= intervalMs)
  {
    colorPrint("\n=== TRANSMISSION CYCLE START ===", ANSI_BRIGHT_GREEN);
    performTransmissionSequence();
    lastSendTime = currentTime; // Reset the timer
    colorPrint("=== TRANSMISSION CYCLE COMPLETE ===\n", ANSI_BRIGHT_GREEN);
  }

  processGps(); // Keep feeding GPS data to the parser (even between TX cycles)
  delay(100);   // Small delay to prevent tight-looping and reduce power consumption
}

// ═══════════════════════════════════════════════
// Transmission Sequence — The core duty cycle
// ═══════════════════════════════════════════════
// This is the main work the collar does each cycle:
// Step 1: Scan for BLE home beacon
// Step 2: If not home, try to get a GPS fix
// Step 3: Build and send a telemetry packet
// Step 4: Listen for commands from the base station
void performTransmissionSequence()
{
  colorPrint("[SEQ] Starting Transmission Sequence...", ANSI_MAGENTA);

  // Reset home status for a fresh detection this cycle
  isHome = false;

  // ── Step 1: BLE Home Beacon Scan ──
  // Scan for the home BLE beacon. This is quick (10 seconds) and if found,
  // we can skip the GPS fix entirely (saves ~20-60 seconds of power consumption).
  colorPrint("[SEQ] Scanning for Home Beacon (" + String(BLE_INITIAL_SCAN_S) + "s)...", ANSI_YELLOW);
  bool foundHome = scanForHomeBeacon(BLE_INITIAL_SCAN_S);

  if (foundHome)
  {
    homeCycleCount++; // Track consecutive home detections
    colorPrint("[SEQ] Home detected. Consecutive cycles: " + String(homeCycleCount), ANSI_BRIGHT_GREEN);
  }
  else
  {
    homeCycleCount = 0; // Reset counter — pet left home
    colorPrint("[SEQ] Home not detected.", ANSI_YELLOW);
  }

  // ── Step 2: GPS Fix (only if not at home) ──
  // If BLE says we're home, skip GPS to save battery.
  // Otherwise, wake up the GPS and wait for a fix.
  if (!foundHome)
  {
    gpsWake(); // Power on the GPS module
    colorPrint("[SEQ] Attempting GPS fix (max " + String(GPS_COLD_START_TIMEOUT / 1000) + "s)...", ANSI_YELLOW);
    unsigned long fixStart = millis();
    bool fixObtained = false;
    bool firstFix = false;
    unsigned long firstFixTime = 0;
    // Use shorter timeout if GPS already had a fix before (warm start)
    unsigned long timeout = gpsWarmStart ? GPS_WARM_START_TIMEOUT : GPS_COLD_START_TIMEOUT;

    while (millis() - fixStart < timeout)
    {
      processGps(); // Feed GPS data to the parser
      // Detect first valid fix (fresh data, less than 5 seconds old)
      if (!firstFix && gps.location.isValid() && gps.location.age() < 5000)
      {
        firstFix = true;
        firstFixTime = millis();
        gpsWarmStart = true; // Mark that we've had at least one fix
        colorPrint("[SEQ] GPS fix detected! Stabilizing...", ANSI_GREEN);
      }
      // Wait for stabilization (coordinates stop jumping around)
      if (firstFix && (millis() - firstFixTime >= GPS_STABILISE_MS))
      {
        fixObtained = true;
        break;
      }
      delay(1);
    }

    if (fixObtained)
    {
      colorPrint("[SEQ] Proceeding with stabilized GPS fix.", ANSI_GREEN);
      processGps(); // Final read for latest data
    }
    else if (firstFix)
    {
      colorPrint("[SEQ] Proceeding with initial GPS fix (stabilization incomplete).", ANSI_YELLOW);
      processGps();
    }
    else
    {
      colorPrint("[SEQ] GPS fix timeout. Proceeding without valid fix.", ANSI_YELLOW);
    }
  }
  else
  {
    colorPrint("[SEQ] Home Beacon detected. Skipping GPS fix.", ANSI_BRIGHT_GREEN);
  }

  // ── Step 3: Send telemetry packet ──
  sendTelemetry();

  // ── Step 4: Listen for commands from base station ──
  listenForCommands();

  colorPrint("[SEQ] Transmission Sequence Complete.", ANSI_MAGENTA);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Telemetry (PKT_TELEMETRY)
// ═══════════════════════════════════════════════
// Builds and sends the main telemetry packet containing:
// - Device status (home/out/error)
// - GPS coordinates (if available)
// - Distance and bearing to home
// - Speed, fix age
// - Battery voltage
// - Current operating profile and settings (via TLV)
// - Lost mode elapsed time (if in lost mode)
void sendTelemetry()
{
  messageSeq++; // Increment the message sequence counter

  // ── Determine the device status ──
  // Priority: BLE home > valid GPS > invalid GPS
  bp_status_t status;
  uint16_t flags = PKT_TELEMETRY; // Start with packet type in the flags field

  bool locValid = gps.location.isValid();       // Does the GPS have any fix at all?
  unsigned long locAge = gps.location.age();     // How old is the last fix (milliseconds)?
  bool isStale = locValid && locAge > 60000;     // Fix is "stale" if older than 60 seconds

  if (isHome)
  {
    // BLE beacon detected — pet is definitely at home
    status = STATUS_BLE_HOME;
    flags |= FLAG_BLE_HOME | FLAG_HAS_GPS; // Set BLE flag; we'll fill GPS with home coords
  }
  else if (locValid && !isStale)
  {
    // We have a fresh GPS fix — check if pet is within the home radius
    double dist = TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    if (dist <= HOME_RADIUS_M)
      status = STATUS_BLE_HOME; // GPS says we're home (even though BLE didn't detect it)
    else
      status = STATUS_OUT_AND_ABOUT; // Pet is out and about with good GPS
    flags |= FLAG_HAS_GPS; // Mark that GPS data is valid in this packet
  }
  else
  {
    // No valid or fresh GPS fix available
    status = STATUS_INVALID_GPS;
  }

  // Add warm start flag if GPS has had a prior fix (tells base station the GPS state)
  if (gpsWarmStart)
    flags |= FLAG_GPS_WARM;

  // ── Build the GPS timestamp ──
  // Convert the GPS date/time to a Unix timestamp for the packet header.
  // Only if the GPS time data is valid and fresh (less than 60 seconds old).
  uint32_t unixTime = 0;
  if (gps.time.isValid() && gps.date.isValid() && gps.time.age() < 60000)
  {
    unixTime = gpsToUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                             gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  // ── Assemble the packet ──
  uint8_t buf[BP_MAX_PACKET_SIZE]; // 66-byte buffer (maximum possible packet size)
  pkt_init(buf, MY_DEVICE_ID, messageSeq, unixTime, status, flags); // Fill in the header

  // ── Fill in GPS fields ──
  if (isHome)
  {
    // BLE home detected — send the configured home coordinates instead of GPS
    // (GPS may not even have a fix since we skipped it when at home)
    int32_t lat_e7 = (int32_t)(HOME_LAT * 1e7); // Convert decimal degrees to integer * 1e7
    int32_t lon_e7 = (int32_t)(HOME_LON * 1e7);
    pkt_set_gps(buf, lat_e7, lon_e7, 0, 0); // Distance=0, Bearing=0 (we're at home)
  }
  else if (flags & FLAG_HAS_GPS)
  {
    // We have a valid GPS fix — use the actual coordinates
    int32_t lat_e7 = (int32_t)(gps.location.lat() * 1e7);
    int32_t lon_e7 = (int32_t)(gps.location.lng() * 1e7);

    // Calculate distance and compass bearing from current position to home
    double dist = TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    double bearing = TinyGPSPlus::courseTo(
        gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    uint16_t dist_m = (uint16_t)min(dist, 65535.0);  // Cap at u16 max (65.5 km)
    uint16_t bearing_deg = (uint16_t)bearing;         // 0-359 degrees

    pkt_set_gps(buf, lat_e7, lon_e7, dist_m, bearing_deg);

    // Speed in cm/s — stored at byte offset 28-29
    if (gps.speed.isValid())
    {
      uint16_t speed_cms = (uint16_t)(gps.speed.mps() * 100.0); // Convert m/s to cm/s
      memcpy(&buf[28], &speed_cms, 2);
    }

    // GPS fix age in seconds — stored at byte offset 26-27
    // Tells the base station how old this fix is
    uint16_t fixAge_s = (uint16_t)(locAge / 1000); // Convert ms to seconds
    memcpy(&buf[26], &fixAge_s, 2);
  }

  // Battery voltage in millivolts — stored at byte offset 22-23
  // TODO: Replace this placeholder with actual ADC reading from the battery
  uint16_t batt_mV = 3700; // Hardcoded 3.7V placeholder
  memcpy(&buf[22], &batt_mV, 2);

  // ── Append TLV data ──
  // These optional fields tell the base station about the collar's current settings
  pkt_add_tlv_u8(buf, TLV_PROFILE, currentProfile);                // Which mode we're in
  pkt_add_tlv_i8(buf, TLV_TX_POWER, currentMode->lora_power_dbm);  // Current TX power
  pkt_add_tlv_u16(buf, TLV_SLEEP_INTERVAL, currentMode->sleep_interval_s); // Current sleep interval
  pkt_add_tlv_u8(buf, TLV_GPS_WARM, gpsWarmStart ? 1 : 0);        // GPS warm start state
  pkt_add_tlv_u8(buf, TLV_HOME_CYCLES, homeCycleCount);            // Consecutive BLE home detections

  // If in lost mode, include how long we've been in it
  if (inLostMode)
  {
    uint32_t lostElapsed = (millis() - lostModeStartTime) / 1000; // Seconds in lost mode
    pkt_add_tlv_u32(buf, TLV_LOST_MODE_S, lostElapsed);
  }

  // ── Finalize and transmit ──
  uint8_t pktLen = pkt_finalize(buf); // Compute CRC and get total packet length

  colorPrint("[TX] Sending PKT_TELEMETRY | Status: " +
             String(statusToDisplayString(status)) +
             " | Seq: " + String(messageSeq) +
             " | Size: " + String(pktLen) + "B", ANSI_BRIGHT_CYAN);
  pkt_print_hex(buf, pktLen); // Hex dump for debugging

  transmitBinaryPacket(buf, pktLen); // Actually send it over LoRa
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Mode ACK (PKT_MODE_ACK)
// ═══════════════════════════════════════════════
// After receiving and applying a mode change command from the base station,
// send back an acknowledgment so the base station knows it worked.
// Includes the new profile settings and the original command's sequence ID.
void sendModeAck(uint32_t cmdMsgSeq)
{
  messageSeq++;

  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, 0, STATUS_OK, PKT_MODE_ACK); // timestamp=0 (not needed for ACKs)

  // TLV: Tell the base station what profile is now active and its settings
  pkt_add_tlv_u8(buf, TLV_PROFILE, currentProfile);
  pkt_add_tlv_i8(buf, TLV_TX_POWER, currentMode->lora_power_dbm);
  pkt_add_tlv_u16(buf, TLV_SLEEP_INTERVAL, currentMode->sleep_interval_s);
  pkt_add_tlv_u32(buf, TLV_CMD_MSG_ID, cmdMsgSeq); // Echo back which command we're ACK'ing

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_MODE_ACK for cmd seq " + String(cmdMsgSeq) +
             " | New mode: " + String(currentMode->name), ANSI_BRIGHT_CYAN);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Status Response (PKT_STATUS_RESP)
// ═══════════════════════════════════════════════
// When the base station sends a PKT_CMD_STATUS request, respond with
// the collar's current operating settings and state.
void sendStatusResponse(uint32_t cmdMsgSeq)
{
  messageSeq++;

  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, 0, STATUS_OK, PKT_STATUS_RESP);

  // Pack all current settings into TLV fields
  pkt_add_tlv_u8(buf, TLV_PROFILE, currentProfile);                // Current mode
  pkt_add_tlv_i8(buf, TLV_TX_POWER, currentMode->lora_power_dbm);  // TX power
  pkt_add_tlv_u16(buf, TLV_SLEEP_INTERVAL, currentMode->sleep_interval_s); // Sleep interval
  pkt_add_tlv_u8(buf, TLV_GPS_WARM, gpsWarmStart ? 1 : 0);        // GPS state
  pkt_add_tlv_u8(buf, TLV_HOME_CYCLES, homeCycleCount);            // Home detection count
  pkt_add_tlv_u32(buf, TLV_CMD_MSG_ID, cmdMsgSeq);                 // Which command triggered this response

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_STATUS_RESP for cmd seq " + String(cmdMsgSeq), ANSI_BRIGHT_CYAN);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Lost Mode Timeout Alert (PKT_ALERT)
// ═══════════════════════════════════════════════
// Sent when lost mode auto-expires after LOST_MODE_MAX_DURATION_S (2 hours).
// Tells the base station: "I was in lost mode for X seconds, now switching to active."
void sendLostModeTimeoutAlert()
{
  messageSeq++;

  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, 0, STATUS_LOST_TIMEOUT, PKT_ALERT);

  uint32_t duration = (millis() - lostModeStartTime) / 1000; // Total seconds spent in lost mode
  pkt_add_tlv_u32(buf, TLV_DURATION_S, duration);             // How long lost mode lasted
  pkt_add_tlv_u8(buf, TLV_NEW_MODE, PROFILE_ACTIVE);          // What mode we're reverting to

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_ALERT: Lost mode timeout after " +
             String(duration) + "s, reverting to active", ANSI_BRIGHT_RED);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Listen for Commands — 2-second RX window
// ═══════════════════════════════════════════════
// After transmitting, open a receive window to listen for commands
// from the base station. The base station can send mode change commands
// or status queries during this window.
// Only processes ONE packet per window, then closes.
void listenForCommands()
{
  colorPrint("[RX] Opening " + String(COMMAND_LISTEN_MS) + "ms receive window...", ANSI_MAGENTA);

  // Put the LoRa radio into receive mode
  int rxState = lora.startReceive();
  if (rxState != RADIOLIB_ERR_NONE)
  {
    colorPrint("[RX] Failed to start receive: " + String(rxState), ANSI_RED);
    return;
  }

  unsigned long listenStart = millis();
  while (millis() - listenStart < COMMAND_LISTEN_MS) // Listen for up to 2 seconds
  {
    // Check if the LoRa module has received a packet (via interrupt flag)
    int irqFlags = lora.getIrqStatus();
    if (irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE) // RX_DONE flag means a packet arrived
    {
      uint8_t rxBuf[BP_MAX_PACKET_SIZE];
      size_t rxLen = 0;
      int state = lora.readData(rxBuf, sizeof(rxBuf)); // Read the received data
      if (state == RADIOLIB_ERR_NONE)
      {
        rxLen = lora.getPacketLength();
        colorPrint("[RX] Received " + String(rxLen) + " bytes", ANSI_BRIGHT_MAGENTA);
        pkt_print_hex(rxBuf, rxLen);

        // Sanity check: is this a valid binary protocol packet?
        // Must be at least 38 bytes and start with our protocol version byte.
        if (rxLen >= BP_MIN_PACKET_SIZE && rxBuf[0] == BP_PROTOCOL_VERSION)
        {
          handleReceivedCommand(rxBuf, rxLen); // Parse and act on it
        }
        else
        {
          colorPrint("[RX] Not a binary protocol packet (ignoring)", ANSI_YELLOW);
        }
      }
      else
      {
        colorPrint("[RX] Read error: " + String(state), ANSI_RED);
      }
      break; // Only process one packet per listen window
    }
    delay(10); // Check for packets every 10ms
  }

  lora.standby(); // Return radio to standby mode (low power)
  colorPrint("[RX] Receive window closed.", ANSI_MAGENTA);
}

// ═══════════════════════════════════════════════
// Handle Received Binary Command
// ═══════════════════════════════════════════════
// Called when we receive a valid binary protocol packet during the RX window.
// Validates CRC, checks if the packet is addressed to us, and dispatches
// based on packet type (mode change or status query).
void handleReceivedCommand(const uint8_t *buf, uint8_t len)
{
  // ── Step 1: Validate CRC ──
  // Make sure the packet wasn't corrupted during transmission
  if (!pkt_validate_crc(buf, len))
  {
    colorPrint("[RX] Binary CRC validation failed! Dropping packet.", ANSI_RED);
    return;
  }

  // ── Step 2: Check device ID ──
  // The packet's device ID field tells us who it's intended for.
  // We only accept packets addressed to our specific ID or to broadcast (0xFFFF).
  uint16_t targetId = pkt_device_id(buf);
  if (targetId != MY_DEVICE_ID && targetId != DEVICE_ID_BROADCAST)
  {
    colorPrint("[RX] Packet not for us (target: 0x" + String(targetId, HEX) +
               "), ignoring.", ANSI_YELLOW);
    return;
  }

  // ── Step 3: Dispatch based on packet type ──
  uint16_t pktType = pkt_pkt_type(buf);   // Extract packet type from flags
  uint32_t cmdSeq = pkt_msg_seq(buf);     // Command's sequence number (for ACK matching)

  switch (pktType)
  {
  case PKT_CMD_MODE: // Base station wants us to change operating mode
  {
    colorPrint("[RX] Received PKT_CMD_MODE (seq: " + String(cmdSeq) + ")", ANSI_BRIGHT_MAGENTA);
    uint8_t newProfile;
    // Look for the TLV_PROFILE field to find out which mode to switch to
    if (pkt_tlv_get_u8(buf, TLV_PROFILE, &newProfile))
    {
      colorPrint("[RX] Requested profile: " + String(profileToName((bp_profile_t)newProfile)), ANSI_BRIGHT_MAGENTA);
      applyProfile((bp_profile_t)newProfile); // Actually change the mode
      sendModeAck(cmdSeq);                    // Tell base station we applied it
    }
    else
    {
      colorPrint("[RX] PKT_CMD_MODE missing TLV_PROFILE!", ANSI_RED);
    }
    break;
  }
  case PKT_CMD_STATUS: // Base station wants our current status
  {
    colorPrint("[RX] Received PKT_CMD_STATUS (seq: " + String(cmdSeq) + ")", ANSI_BRIGHT_MAGENTA);
    sendStatusResponse(cmdSeq); // Send back our current settings
    break;
  }
  default:
    colorPrint("[RX] Unknown packet type: 0x" + String(pktType, HEX), ANSI_YELLOW);
    break;
  }
}

// ═══════════════════════════════════════════════
// Apply Operating Profile — Switch modes
// ═══════════════════════════════════════════════
// Changes the collar's operating mode (normal/powersave/active/lost).
// Updates the current mode pointer, adjusts LoRa TX power,
// and manages lost mode start/stop tracking.
void applyProfile(bp_profile_t profile)
{
  const char *name = profileToName(profile);       // Get string name for the profile
  const OperatingMode *mode = getModeByName(name);  // Look up the full mode settings

  colorPrint("[MODE] Changing from " + String(currentMode->name) +
             " to " + String(name), ANSI_BRIGHT_YELLOW);

  // Update global state to the new mode
  currentProfile = profile;
  currentMode = mode;

  // Immediately apply the new TX power setting to the radio
  lora.setOutputPower(currentMode->lora_power_dbm);
  colorPrint("[MODE] TX Power: " + String(currentMode->lora_power_dbm) + "dBm", ANSI_BLUE);
  colorPrint("[MODE] Sleep interval: " + String(currentMode->sleep_interval_s) + "s", ANSI_BLUE);

  // ── Handle lost mode tracking ──
  if (profile == PROFILE_LOST)
  {
    if (!inLostMode) // Only start the timer if we're entering lost mode fresh
    {
      inLostMode = true;
      lostModeStartTime = millis(); // Record when lost mode began (for timeout calculation)
      colorPrint("[MODE] Lost mode ACTIVATED. Timer started.", ANSI_BRIGHT_RED);
    }
    // If already in lost mode, don't reset the timer (consecutive lost commands don't restart it)
  }
  else
  {
    // Switching away from lost mode — deactivate it
    if (inLostMode)
    {
      colorPrint("[MODE] Lost mode DEACTIVATED.", ANSI_GREEN);
    }
    inLostMode = false;
    lostModeStartTime = 0;
  }
}

// ═══════════════════════════════════════════════
// Transmit Binary Packet via LoRa (with LBT)
// ═══════════════════════════════════════════════
// Takes a fully assembled binary packet buffer and sends it over the LoRa radio.
// Handles error cases (timeout, packet too long, etc.) with appropriate LED feedback.
void transmitBinaryPacket(uint8_t *buf, uint8_t len)
{
  colorPrint("[LORA TX] Transmitting " + String(len) + " bytes...", ANSI_BLUE);

  // Put the radio in standby mode first (required before TX)
  int state = lora.standby();
  if (state != RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX WARN] Standby failed: " + String(state) + " (continuing)", ANSI_YELLOW);
  }

  // Transmit the packet — this blocks until transmission is complete
  state = lora.transmit(buf, len);
  if (state == RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX] Transmission successful!", ANSI_BRIGHT_GREEN);
    flickerMedium(); // Visual success feedback (mode-dependent flash count)
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    colorPrint("[LORA TX ERROR] Packet too long!", ANSI_RED);
    flickerLong(); // Error flash
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    // TX timeout is often a false alarm on SX1262 — the packet may have still been sent
    colorPrint("[LORA TX WARN] Timeout (often false positive - check receiver)", ANSI_YELLOW);
    flickerShort();
  }
  else
  {
    colorPrint("[LORA TX ERROR] Failed, code: " + String(state), ANSI_RED);
    flickerLong();
  }
}

// ═══════════════════════════════════════════════
// Wake / Sleep / GPS Helpers
// ═══════════════════════════════════════════════

// Called when the ESP32 wakes from light sleep.
// Checks WHY it woke up and takes the appropriate action:
// - Timer: Normal scheduled wake → do a transmission cycle
// - Button (EXT0): User pressed the status button → show status report, then transmit
// - GPIO (DIO1): LoRa radio received a packet → read it, then transmit
void handleWakeupReason()
{
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason)
  {
  case ESP_SLEEP_WAKEUP_TIMER: // Woke up because the sleep timer expired
    colorPrint("[WAKE] Reason: Timer", ANSI_CYAN);
    performTransmissionSequence();
    break;
  case ESP_SLEEP_WAKEUP_EXT0: // Woke up because the button was pressed
    colorPrint("[WAKE] Reason: Button Press", ANSI_BRIGHT_CYAN);
    printStatusReport();            // Show current status on serial
    performTransmissionSequence();  // Also do a transmission
    flickerShort();                 // Quick flash to acknowledge button press
    break;
  case ESP_SLEEP_WAKEUP_GPIO: // Woke up because LoRa DIO1 triggered (received a packet in sleep)
    colorPrint("[WAKE] Reason: LoRa DIO1 Interrupt", ANSI_BRIGHT_MAGENTA);
    handleLoraReception();          // Read and process the received packet
    performTransmissionSequence();  // Then do a normal transmission
    break;
  default: // Unexpected wake reason
    colorPrint("[WAKE] Reason: Unknown (" + String(reason) + ")", ANSI_RED);
    delay(1000);
    break;
  }
}

// Handle a LoRa packet that was received while the ESP32 was awake
// (different from listenForCommands which actively opens a receive window).
// This is called when DIO1 triggers a wake-from-sleep interrupt.
void handleLoraReception()
{
  colorPrint("[LORA RX] Interrupt received. Reading...", ANSI_MAGENTA);
  uint8_t rxBuf[BP_MAX_PACKET_SIZE];
  int state = lora.readData(rxBuf, sizeof(rxBuf)); // Read whatever the radio has

  if (state == RADIOLIB_ERR_NONE)
  {
    size_t rxLen = lora.getPacketLength();
    colorPrint("[LORA RX] Received " + String(rxLen) + " bytes", ANSI_BRIGHT_GREEN);
    pkt_print_hex(rxBuf, rxLen);

    // If it's a valid protocol packet, process it as a command
    if (rxLen >= BP_MIN_PACKET_SIZE && rxBuf[0] == BP_PROTOCOL_VERSION)
    {
      handleReceivedCommand(rxBuf, rxLen);
    }
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  {
    colorPrint("[LORA RX] CRC error!", ANSI_RED); // Hardware CRC failed (corrupt packet)
  }
  else
  {
    colorPrint("[LORA RX] Failed, code: " + String(state), ANSI_RED);
  }
}

// Read all available bytes from the GPS serial port and feed them to the TinyGPS++ parser.
// Must be called frequently to keep the GPS data up to date.
// Each byte is a character from an NMEA sentence (like "$GPGGA,..." or "$GPRMC,...").
void processGps()
{
  if (gpsIsAwake)
  {
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read()); // Feed each byte to TinyGPS++ for parsing
    }
  }
}

// Power on the GPS module by driving the sleep/wake pin HIGH.
// Includes a short delay for the module to wake up and start sending NMEA data.
void gpsWake()
{
  if (!gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, HIGH); // HIGH = GPS awake
    colorPrint("[GPS] Setting wake pin HIGH...", ANSI_YELLOW);
    gpsIsAwake = true;
    delay(100);  // Short delay for hardware to wake
    colorPrint("[GPS] GPS awakened.", ANSI_YELLOW);
    delay(500);  // Give GPS time to start outputting NMEA sentences
  }
}

// Power off the GPS module by driving the sleep/wake pin LOW.
// Saves significant battery when GPS isn't needed (e.g., pet is at home).
void gpsSleep()
{
  if (gpsIsAwake)
  {
    Serial.flush();                     // Make sure all debug output is sent before sleep
    digitalWrite(GPS_SLEEP_WAKE, LOW);  // LOW = GPS sleep
    colorPrint("[GPS] Putting GPS module to sleep.", ANSI_YELLOW);
    gpsIsAwake = false;
  }
}

// Put the ESP32 into light sleep mode.
// Light sleep preserves RAM contents (all variables keep their values)
// but shuts down the CPU and most peripherals to save power.
//
// Three wake-up sources are configured:
// 1. Timer — wake after the current mode's sleep interval
// 2. Button (EXT0) — wake when the user presses the status button
// 3. GPIO (DIO1) — wake when the LoRa radio receives a packet
void goToLightSleep()
{
  colorPrint("[SLEEP] Preparing for light sleep...", ANSI_BLUE);

  isHome = false; // Clear home status — will be re-scanned on wake

  // Put LoRa radio into receive mode so it can wake us up if a packet arrives
  int rxState = lora.startReceive();
  if (rxState != RADIOLIB_ERR_NONE)
    colorPrint("[SLEEP] Failed to start LoRa receive: " + String(rxState), ANSI_RED);
  else
    colorPrint("[SLEEP] LoRa is listening.", ANSI_BLUE);

  // Calculate sleep duration in microseconds (ESP32 sleep API uses microseconds)
  uint64_t sleepUs = (uint64_t)currentMode->sleep_interval_s * 1000000ULL;

  // Configure wake-up sources
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // Clear any previous wake sources
  esp_sleep_enable_timer_wakeup(sleepUs);                 // Wake after sleep interval
  colorPrint("[SLEEP] Timer wakeup: " + String(currentMode->sleep_interval_s) + "s", ANSI_BLUE);

  esp_sleep_enable_ext0_wakeup(STATUS_BUTTON_PIN, 0);    // Wake on button press (active LOW)
  esp_sleep_enable_gpio_wakeup();                         // Enable GPIO wakeup
  gpio_wakeup_enable(GPIO_NUM_39, GPIO_INTR_HIGH_LEVEL); // Wake on LoRa DIO1 going HIGH (packet received)

  colorPrint("[SLEEP] Entering light sleep...", ANSI_BOLD);
  Serial.flush(); // Ensure all serial data is sent before sleeping

  // Stop BLE scanning before sleep (otherwise it would keep running)
  if (pBLEScan != nullptr)
    pBLEScan->stop();

  bootFlag = 1;              // Set flag so setup() knows to skip full init on wake
  esp_light_sleep_start();   // Actually enter light sleep — execution stops here until wake
}

// ═══════════════════════════════════════════════
// BLE Scan — Search for the home beacon
// ═══════════════════════════════════════════════
// Runs a BLE scan for the specified duration, looking for a device named BEACON_NAME ("HOME").
// Returns true if the beacon was found, false if not.
// The scan can end early if the beacon is found (callback sets isHome=true and stops the scan).
bool scanForHomeBeacon(uint32_t scanDurationSeconds)
{
  if (pBLEScan == nullptr)
  {
    colorPrint("[BLE ERROR] Scanner not initialized!", ANSI_RED);
    return false;
  }

  colorPrint("[BLE] Starting scan for \"" + String(BEACON_NAME) +
             "\" (" + String(scanDurationSeconds) + "s)...", ANSI_BLUE);
  isHome = false; // Reset flag before scanning

  pBLEScan->start(scanDurationSeconds, false); // Start scanning (false = don't store results, we use callback)

  // Wait for scan to complete or beacon to be found
  unsigned long scanStart = millis();
  while (millis() - scanStart < (scanDurationSeconds * 1000))
  {
    if (isHome) // Beacon found — callback set this flag
      break;
    delay(100); // Check every 100ms
  }

  if (isHome)
  {
    colorPrint("[BLE] Home Beacon FOUND!", ANSI_BRIGHT_GREEN);
  }
  else
  {
    colorPrint("[BLE] Home Beacon NOT found.", ANSI_YELLOW);
    if (pBLEScan != nullptr)
    {
      delay(50);
      pBLEScan->stop(); // Make sure scan is stopped if it timed out
    }
  }

  return isHome;
}

// ═══════════════════════════════════════════════
// Status Report — Prints a detailed status summary to serial
// ═══════════════════════════════════════════════
// Triggered by button press or for debugging. Shows all important
// collar state: device info, mode, GPS status, battery, uptime, etc.
void printStatusReport()
{
  colorPrint("──────────── STATUS REPORT ────────────", ANSI_BOLD);
  Serial.printf("  Device: %s (0x%04X)\n", getDeviceName(MY_DEVICE_ID), MY_DEVICE_ID);
  Serial.printf("  Mode: %s\n", currentMode->name);
  Serial.printf("  TX Power: %d dBm\n", currentMode->lora_power_dbm);
  Serial.printf("  Sleep Interval: %d s\n", currentMode->sleep_interval_s);
  Serial.printf("  Uptime: %lu s\n", millis() / 1000);        // Seconds since boot
  Serial.printf("  GPS Awake: %s\n", gpsIsAwake ? "Yes" : "No");
  Serial.printf("  GPS Warm: %s\n", gpsWarmStart ? "Yes" : "No");
  if (gps.location.isValid())
    Serial.printf("  Location: %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
  else
    Serial.println("  Location: Invalid");
  Serial.printf("  Satellites: %s\n",
                gps.satellites.isValid() ? String(gps.satellites.value()).c_str() : "Invalid");
  Serial.printf("  Home Cycles: %d\n", homeCycleCount);
  if (inLostMode)
    Serial.printf("  Lost Mode: %lu s elapsed\n", (millis() - lostModeStartTime) / 1000);
  Serial.printf("  Msg Seq: %lu\n", messageSeq);
  Serial.printf("  Free Heap: %lu\n", ESP.getFreeHeap()); // Available RAM (useful for debugging memory leaks)
  colorPrint("───────────────────────────────────────", ANSI_BOLD);
}

// ═══════════════════════════════════════════════
// Color Print — Serial output with ANSI color codes
// ═══════════════════════════════════════════════
// Wraps a message in ANSI escape codes for colorful serial output.
// Makes it much easier to scan the serial monitor and distinguish
// between GPS messages (yellow), LoRa (blue), errors (red), etc.
void colorPrint(const String &message, const char *color)
{
  Serial.print(color);       // Set the color
  Serial.println(message);   // Print the message with newline
  Serial.print(ANSI_RESET);  // Reset color back to default
  Serial.flush();             // Ensure it's sent immediately (important before sleep)
}
