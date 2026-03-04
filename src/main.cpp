/*
  ┌──────────────────────────────────────────────┐
  │ CAT TRACKER TX — LoRa GPS Collar             │
  │ SX1262 + TinyGPSPlus + Binary TLV Protocol   │
  └──────────────────────────────────────────────┘
  Binary TLV protocol compatible with BluePawzReceiver.
  protocol.h and config.h MUST be identical on TX and RX.
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <esp_sleep.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_attr.h>

#include "protocol.h"
#include "config.h"

// ═══════════════════════════════════════════════
// Device Identity — Change this per collar!
// ═══════════════════════════════════════════════
#define MY_DEVICE_ID 0x0001 // Unique collar ID (1-65534). Change per collar before flashing.

// ═══════════════════════════════════════════════
// Pin Definitions
// ═══════════════════════════════════════════════
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define LORA_NSS 41
#define LORA_SCK 7
#define LORA_MOSI 9
#define LORA_MISO 8
#define LORA_RST 42
#define LORA_BUSY 40
#define LORA_DIO1 39

#define GPS_RX 44
#define GPS_TX 43
#define GPS_BAUD 9600
#define GPS_SLEEP_WAKE 1
#define GPS_RESET 3

#define STATUS_BUTTON_PIN GPIO_NUM_21
#define STATUS_LED_PIN 48

// ═══════════════════════════════════════════════
// Home Location
// ═══════════════════════════════════════════════
#define HOME_LAT 51.87370573411073
#define HOME_LON -2.2396017778476716
#define HOME_RADIUS_M 20.0

// ═══════════════════════════════════════════════
// Timing
// ═══════════════════════════════════════════════
#define COMMAND_LISTEN_MS 2000 // Listen window after each TX

// ═══════════════════════════════════════════════
// ANSI Color Codes
// ═══════════════════════════════════════════════
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

// Binary protocol functions
void sendTelemetry();
void sendModeAck(uint32_t cmdMsgSeq);
void sendStatusResponse(uint32_t cmdMsgSeq);
void sendLostModeTimeoutAlert();
void listenForCommands();
void handleReceivedCommand(const uint8_t *buf, uint8_t len);
void applyProfile(bp_profile_t profile);
void transmitBinaryPacket(uint8_t *buf, uint8_t len);

// LED functions
void flickerShort();
void flickerMedium();
void flickerLong();
void ledBeacon();

// ═══════════════════════════════════════════════
// Hardware Instances
// ═══════════════════════════════════════════════
SPIClass LoRaSPI(HSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, LoRaSPI);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ═══════════════════════════════════════════════
// BLE
// ═══════════════════════════════════════════════
BLEScan *pBLEScan = nullptr;
volatile bool isHome = false;
uint8_t homeCycleCount = 0; // Consecutive BLE home detections

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (advertisedDevice.haveName() && advertisedDevice.getName() == BEACON_NAME)
    {
      colorPrint("[BLE] Found Home Beacon! (" + String(BEACON_NAME) + ")", ANSI_BRIGHT_GREEN);
      isHome = true;
      if (pBLEScan != nullptr)
      {
        pBLEScan->stop();
        colorPrint("[BLE] Scan stopped early.", ANSI_BLUE);
      }
    }
  }
};
MyAdvertisedDeviceCallbacks bleCallbacks;

// ═══════════════════════════════════════════════
// Global State
// ═══════════════════════════════════════════════
bool gpsIsAwake = true;
bool gpsWarmStart = false; // Track if GPS has had a previous fix
static uint32_t messageSeq = 0;
RTC_DATA_ATTR int bootFlag = 0;

// Operating mode state
const OperatingMode *currentMode = &MODE_NORMAL;
bp_profile_t currentProfile = PROFILE_NORMAL;

// Lost mode tracking
unsigned long lostModeStartTime = 0;
bool inLostMode = false;

// Timed loop (sleep disabled for debugging)
unsigned long lastSendTime = 0;

// ═══════════════════════════════════════════════
// LED Functions
// ═══════════════════════════════════════════════
void flickerShort()
{
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
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < currentMode->led_flash_count; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(50);
  }
}

void flickerLong()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 12; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(80);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
  }
}

void ledBeacon()
{
  if (currentMode->led_beacon_mode)
  {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

// ═══════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(1000);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (bootFlag == 1 && (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER ||
                        wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 ||
                        wakeup_reason == ESP_SLEEP_WAKEUP_GPIO))
  {
    colorPrint("[WAKE] Woke from sleep, skipping full setup...", ANSI_BRIGHT_YELLOW);
    bootFlag = 0;
    return;
  }

  bootFlag = 0;
  Serial.println("\n[BOOT] Serial connection established. Starting setup...");
  delay(200);
  colorPrint("[BOOT] Initialising CAT TRACKER TX v3 (Binary TLV Protocol)...");
  colorPrint("[BOOT] Device: " + String(getDeviceName(MY_DEVICE_ID)) +
             " (ID: 0x" + String(MY_DEVICE_ID, HEX) + ")", ANSI_BRIGHT_CYAN);
  flickerShort();
  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP);

  // --- GPS Init ---
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);
  pinMode(GPS_SLEEP_WAKE, OUTPUT);
  digitalWrite(GPS_SLEEP_WAKE, HIGH);
  gpsIsAwake = true;

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(100);
  colorPrint("[GPS] Waking up GPS module for initial setup...");

  colorPrint("[GPS] Checking for initial serial activity...", ANSI_YELLOW);
  delay(1000);
  if (gpsSerial.available() > 0)
  {
    colorPrint("[GPS] Serial data detected! Module appears awake.", ANSI_BRIGHT_GREEN);
    while (gpsSerial.available() > 0)
      gpsSerial.read();
  }
  else
  {
    colorPrint("[GPS] No serial data detected after 1s. Check wiring/power.", ANSI_BRIGHT_RED);
  }

  // GPS warmup with stabilization
  colorPrint("[GPS] Warming up GPS (waiting for fix)...");
  unsigned long gpsWarmupStart = millis();
  bool fixFound = false;
  bool firstFixDetected = false;
  unsigned long firstFixTimestamp = 0;

  while (millis() - gpsWarmupStart < GPS_COLD_START_TIMEOUT)
  {
    processGps();
    if (!firstFixDetected && gps.location.isValid() && gps.location.age() < 5000)
    {
      firstFixDetected = true;
      firstFixTimestamp = millis();
      colorPrint("[GPS] Initial valid fix obtained! Waiting for stability...", ANSI_BRIGHT_GREEN);
    }
    if (firstFixDetected && (millis() - firstFixTimestamp >= GPS_STABILISE_MS))
    {
      colorPrint("[GPS] Stabilization period complete.", ANSI_BRIGHT_GREEN);
      fixFound = true;
      gpsWarmStart = true;
      break;
    }
    delay(1);
  }

  if (fixFound)
  {
    colorPrint("[GPS] Initialized with stabilized fix.", ANSI_GREEN);
    processGps();
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

  // --- LoRa Init ---
  pinMode(LORA_DIO1, INPUT);
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  colorPrint("[INIT] Setting up SPI for LoRa...");
  int initState = lora.begin(LORA_FREQ_MHZ);
  if (initState != RADIOLIB_ERR_NONE)
  {
    colorPrint("[ERROR] LoRa failed to initialise. Code: " + String(initState), ANSI_RED);
    flickerLong();
    while (true)
      ;
  }

  colorPrint("[OK] LoRa initialised successfully");
  lora.setOutputPower(currentMode->lora_power_dbm);
  lora.setSpreadingFactor(LORA_SF);
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setCodingRate(LORA_CR);
  lora.setCRC(LORA_USE_CRC);
  lora.setPreambleLength(LORA_PREAMBLE);
  lora.setSyncWord(LORA_SYNC_WORD);
  colorPrint("[INIT] LoRa params: " + String(LORA_FREQ_MHZ) + "MHz SF" +
             String(LORA_SF) + " BW" + String(LORA_BW_KHZ) + "kHz CR4/" +
             String(LORA_CR) + " Preamble:" + String(LORA_PREAMBLE) +
             " Power:" + String(currentMode->lora_power_dbm) + "dBm", ANSI_BLUE);

  // --- BLE Init ---
  colorPrint("[INIT] Initializing BLE...", ANSI_BLUE);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  if (pBLEScan == nullptr)
  {
    colorPrint("[INIT ERROR] Failed to get BLE Scanner instance!", ANSI_RED);
    flickerLong();
  }
  else
  {
    pBLEScan->setAdvertisedDeviceCallbacks(&bleCallbacks);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    colorPrint("[INIT] BLE Scanner Initialized. Beacon name: \"" + String(BEACON_NAME) + "\"", ANSI_BLUE);
  }

  colorPrint("════════════════════════════════════════", ANSI_BOLD);
  colorPrint("SETUP COMPLETE - Binary TLV Protocol Active", ANSI_BOLD);
  colorPrint("════════════════════════════════════════", ANSI_BOLD);

  // First transmission after a short delay
  lastSendTime = millis() - (currentMode->sleep_interval_s * 1000UL) + 5000;
  delay(1000);
  lora.standby();
  bootFlag = 1;
}

// ═══════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════
void loop()
{
  unsigned long currentTime = millis();
  unsigned long intervalMs = (unsigned long)currentMode->sleep_interval_s * 1000UL;

  // Heartbeat every 5 seconds
  static unsigned long lastHeartbeat = 0;
  if (currentTime - lastHeartbeat >= 5000)
  {
    long secsLeft = (long)(intervalMs - (currentTime - lastSendTime)) / 1000;
    if (secsLeft < 0) secsLeft = 0;
    colorPrint("[HB] Active | Mode: " + String(currentMode->name) +
               " | Next TX in " + String(secsLeft) + "s", ANSI_BLUE);
    lastHeartbeat = currentTime;
  }

  // Lost mode timeout check
  if (inLostMode && (currentTime - lostModeStartTime >= (unsigned long)LOST_MODE_MAX_DURATION_S * 1000UL))
  {
    colorPrint("[LOST] Lost mode timeout! Reverting to active...", ANSI_BRIGHT_RED);
    sendLostModeTimeoutAlert();
    applyProfile(PROFILE_ACTIVE);
  }

  // LED beacon in lost mode
  if (inLostMode && currentMode->led_beacon_mode)
  {
    static unsigned long lastBeacon = 0;
    if (currentTime - lastBeacon >= currentMode->led_beacon_interval_ms)
    {
      ledBeacon();
      lastBeacon = currentTime;
    }
  }

  // Time to transmit?
  if (currentTime - lastSendTime >= intervalMs)
  {
    colorPrint("\n=== TRANSMISSION CYCLE START ===", ANSI_BRIGHT_GREEN);
    performTransmissionSequence();
    lastSendTime = currentTime;
    colorPrint("=== TRANSMISSION CYCLE COMPLETE ===\n", ANSI_BRIGHT_GREEN);
  }

  processGps();
  delay(100);
}

// ═══════════════════════════════════════════════
// Transmission Sequence
// ═══════════════════════════════════════════════
void performTransmissionSequence()
{
  colorPrint("[SEQ] Starting Transmission Sequence...", ANSI_MAGENTA);

  // Reset home status for fresh scan
  isHome = false;

  // 1. BLE Home Beacon Scan
  colorPrint("[SEQ] Scanning for Home Beacon (" + String(BLE_INITIAL_SCAN_S) + "s)...", ANSI_YELLOW);
  bool foundHome = scanForHomeBeacon(BLE_INITIAL_SCAN_S);

  if (foundHome)
  {
    homeCycleCount++;
    colorPrint("[SEQ] Home detected. Consecutive cycles: " + String(homeCycleCount), ANSI_BRIGHT_GREEN);
  }
  else
  {
    homeCycleCount = 0;
    colorPrint("[SEQ] Home not detected.", ANSI_YELLOW);
  }

  // 2. GPS Fix Attempt (only if not at home)
  if (!foundHome)
  {
    gpsWake();
    colorPrint("[SEQ] Attempting GPS fix (max " + String(GPS_COLD_START_TIMEOUT / 1000) + "s)...", ANSI_YELLOW);
    unsigned long fixStart = millis();
    bool fixObtained = false;
    bool firstFix = false;
    unsigned long firstFixTime = 0;
    unsigned long timeout = gpsWarmStart ? GPS_WARM_START_TIMEOUT : GPS_COLD_START_TIMEOUT;

    while (millis() - fixStart < timeout)
    {
      processGps();
      if (!firstFix && gps.location.isValid() && gps.location.age() < 5000)
      {
        firstFix = true;
        firstFixTime = millis();
        gpsWarmStart = true;
        colorPrint("[SEQ] GPS fix detected! Stabilizing...", ANSI_GREEN);
      }
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
      processGps();
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

  // 3. Send binary telemetry
  sendTelemetry();

  // 4. Listen for commands from base station
  listenForCommands();

  colorPrint("[SEQ] Transmission Sequence Complete.", ANSI_MAGENTA);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Telemetry (PKT_TELEMETRY)
// ═══════════════════════════════════════════════
void sendTelemetry()
{
  messageSeq++;

  // Determine status
  bp_status_t status;
  uint16_t flags = PKT_TELEMETRY;

  bool locValid = gps.location.isValid();
  unsigned long locAge = gps.location.age();
  bool isStale = locValid && locAge > 60000;

  if (isHome)
  {
    status = STATUS_BLE_HOME;
    flags |= FLAG_BLE_HOME | FLAG_HAS_GPS; // Include GPS flag since we send home coords
  }
  else if (locValid && !isStale)
  {
    double dist = TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    if (dist <= HOME_RADIUS_M)
      status = STATUS_BLE_HOME; // GPS-based home detection
    else
      status = STATUS_OUT_AND_ABOUT;
    flags |= FLAG_HAS_GPS;
  }
  else
  {
    status = STATUS_INVALID_GPS;
  }

  if (gpsWarmStart)
    flags |= FLAG_GPS_WARM;

  // Build timestamp
  uint32_t unixTime = 0;
  if (gps.time.isValid() && gps.date.isValid() && gps.time.age() < 60000)
  {
    unixTime = gpsToUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                             gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  // Build packet
  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, unixTime, status, flags);

  // GPS fields
  if (isHome)
  {
    // BLE home detected — use home coordinates
    int32_t lat_e7 = (int32_t)(HOME_LAT * 1e7);
    int32_t lon_e7 = (int32_t)(HOME_LON * 1e7);
    pkt_set_gps(buf, lat_e7, lon_e7, 0, 0);
  }
  else if (flags & FLAG_HAS_GPS)
  {
    int32_t lat_e7 = (int32_t)(gps.location.lat() * 1e7);
    int32_t lon_e7 = (int32_t)(gps.location.lng() * 1e7);
    double dist = TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    double bearing = TinyGPSPlus::courseTo(
        gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LON);
    uint16_t dist_m = (uint16_t)min(dist, 65535.0);
    uint16_t bearing_deg = (uint16_t)bearing;

    pkt_set_gps(buf, lat_e7, lon_e7, dist_m, bearing_deg);

    // Speed (cm/s)
    if (gps.speed.isValid())
    {
      uint16_t speed_cms = (uint16_t)(gps.speed.mps() * 100.0);
      memcpy(&buf[28], &speed_cms, 2);
    }

    // Fix age (seconds)
    uint16_t fixAge_s = (uint16_t)(locAge / 1000);
    memcpy(&buf[26], &fixAge_s, 2);
  }

  // Battery (placeholder — no ADC reading yet)
  uint16_t batt_mV = 3700;
  memcpy(&buf[22], &batt_mV, 2);

  // TLV payload
  pkt_add_tlv_u8(buf, TLV_PROFILE, currentProfile);
  pkt_add_tlv_i8(buf, TLV_TX_POWER, currentMode->lora_power_dbm);
  pkt_add_tlv_u16(buf, TLV_SLEEP_INTERVAL, currentMode->sleep_interval_s);
  pkt_add_tlv_u8(buf, TLV_GPS_WARM, gpsWarmStart ? 1 : 0);
  pkt_add_tlv_u8(buf, TLV_HOME_CYCLES, homeCycleCount);

  if (inLostMode)
  {
    uint32_t lostElapsed = (millis() - lostModeStartTime) / 1000;
    pkt_add_tlv_u32(buf, TLV_LOST_MODE_S, lostElapsed);
  }

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_TELEMETRY | Status: " +
             String(statusToDisplayString(status)) +
             " | Seq: " + String(messageSeq) +
             " | Size: " + String(pktLen) + "B", ANSI_BRIGHT_CYAN);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Mode ACK (PKT_MODE_ACK)
// ═══════════════════════════════════════════════
void sendModeAck(uint32_t cmdMsgSeq)
{
  messageSeq++;

  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, 0, STATUS_OK, PKT_MODE_ACK);

  pkt_add_tlv_u8(buf, TLV_PROFILE, currentProfile);
  pkt_add_tlv_i8(buf, TLV_TX_POWER, currentMode->lora_power_dbm);
  pkt_add_tlv_u16(buf, TLV_SLEEP_INTERVAL, currentMode->sleep_interval_s);
  pkt_add_tlv_u32(buf, TLV_CMD_MSG_ID, cmdMsgSeq);

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_MODE_ACK for cmd seq " + String(cmdMsgSeq) +
             " | New mode: " + String(currentMode->name), ANSI_BRIGHT_CYAN);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Status Response (PKT_STATUS_RESP)
// ═══════════════════════════════════════════════
void sendStatusResponse(uint32_t cmdMsgSeq)
{
  messageSeq++;

  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, 0, STATUS_OK, PKT_STATUS_RESP);

  pkt_add_tlv_u8(buf, TLV_PROFILE, currentProfile);
  pkt_add_tlv_i8(buf, TLV_TX_POWER, currentMode->lora_power_dbm);
  pkt_add_tlv_u16(buf, TLV_SLEEP_INTERVAL, currentMode->sleep_interval_s);
  pkt_add_tlv_u8(buf, TLV_GPS_WARM, gpsWarmStart ? 1 : 0);
  pkt_add_tlv_u8(buf, TLV_HOME_CYCLES, homeCycleCount);
  pkt_add_tlv_u32(buf, TLV_CMD_MSG_ID, cmdMsgSeq);

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_STATUS_RESP for cmd seq " + String(cmdMsgSeq), ANSI_BRIGHT_CYAN);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Binary Protocol — Send Lost Mode Timeout Alert (PKT_ALERT)
// ═══════════════════════════════════════════════
void sendLostModeTimeoutAlert()
{
  messageSeq++;

  uint8_t buf[BP_MAX_PACKET_SIZE];
  pkt_init(buf, MY_DEVICE_ID, messageSeq, 0, STATUS_LOST_TIMEOUT, PKT_ALERT);

  uint32_t duration = (millis() - lostModeStartTime) / 1000;
  pkt_add_tlv_u32(buf, TLV_DURATION_S, duration);
  pkt_add_tlv_u8(buf, TLV_NEW_MODE, PROFILE_ACTIVE);

  uint8_t pktLen = pkt_finalize(buf);

  colorPrint("[TX] Sending PKT_ALERT: Lost mode timeout after " +
             String(duration) + "s, reverting to active", ANSI_BRIGHT_RED);
  pkt_print_hex(buf, pktLen);

  transmitBinaryPacket(buf, pktLen);
}

// ═══════════════════════════════════════════════
// Listen for Commands (2-second RX window)
// ═══════════════════════════════════════════════
void listenForCommands()
{
  colorPrint("[RX] Opening " + String(COMMAND_LISTEN_MS) + "ms receive window...", ANSI_MAGENTA);

  int rxState = lora.startReceive();
  if (rxState != RADIOLIB_ERR_NONE)
  {
    colorPrint("[RX] Failed to start receive: " + String(rxState), ANSI_RED);
    return;
  }

  unsigned long listenStart = millis();
  while (millis() - listenStart < COMMAND_LISTEN_MS)
  {
    // Check if a packet was received
    int irqFlags = lora.getIrqStatus();
    if (irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE)
    {
      uint8_t rxBuf[BP_MAX_PACKET_SIZE];
      size_t rxLen = 0;
      int state = lora.readData(rxBuf, sizeof(rxBuf));
      if (state == RADIOLIB_ERR_NONE)
      {
        rxLen = lora.getPacketLength();
        colorPrint("[RX] Received " + String(rxLen) + " bytes", ANSI_BRIGHT_MAGENTA);
        pkt_print_hex(rxBuf, rxLen);

        // Check if it's a binary protocol packet
        if (rxLen >= BP_MIN_PACKET_SIZE && rxBuf[0] == BP_PROTOCOL_VERSION)
        {
          handleReceivedCommand(rxBuf, rxLen);
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
      break; // Process one packet per listen window
    }
    delay(10);
  }

  lora.standby();
  colorPrint("[RX] Receive window closed.", ANSI_MAGENTA);
}

// ═══════════════════════════════════════════════
// Handle Received Binary Command
// ═══════════════════════════════════════════════
void handleReceivedCommand(const uint8_t *buf, uint8_t len)
{
  // Validate CRC
  if (!pkt_validate_crc(buf, len))
  {
    colorPrint("[RX] Binary CRC validation failed! Dropping packet.", ANSI_RED);
    return;
  }

  // Check device ID — must be for us or broadcast
  uint16_t targetId = pkt_device_id(buf);
  if (targetId != MY_DEVICE_ID && targetId != DEVICE_ID_BROADCAST)
  {
    colorPrint("[RX] Packet not for us (target: 0x" + String(targetId, HEX) +
               "), ignoring.", ANSI_YELLOW);
    return;
  }

  uint16_t pktType = pkt_pkt_type(buf);
  uint32_t cmdSeq = pkt_msg_seq(buf);

  switch (pktType)
  {
  case PKT_CMD_MODE:
  {
    colorPrint("[RX] Received PKT_CMD_MODE (seq: " + String(cmdSeq) + ")", ANSI_BRIGHT_MAGENTA);
    uint8_t newProfile;
    if (pkt_tlv_get_u8(buf, TLV_PROFILE, &newProfile))
    {
      colorPrint("[RX] Requested profile: " + String(profileToName((bp_profile_t)newProfile)), ANSI_BRIGHT_MAGENTA);
      applyProfile((bp_profile_t)newProfile);
      sendModeAck(cmdSeq);
    }
    else
    {
      colorPrint("[RX] PKT_CMD_MODE missing TLV_PROFILE!", ANSI_RED);
    }
    break;
  }
  case PKT_CMD_STATUS:
  {
    colorPrint("[RX] Received PKT_CMD_STATUS (seq: " + String(cmdSeq) + ")", ANSI_BRIGHT_MAGENTA);
    sendStatusResponse(cmdSeq);
    break;
  }
  default:
    colorPrint("[RX] Unknown packet type: 0x" + String(pktType, HEX), ANSI_YELLOW);
    break;
  }
}

// ═══════════════════════════════════════════════
// Apply Operating Profile
// ═══════════════════════════════════════════════
void applyProfile(bp_profile_t profile)
{
  const char *name = profileToName(profile);
  const OperatingMode *mode = getModeByName(name);

  colorPrint("[MODE] Changing from " + String(currentMode->name) +
             " to " + String(name), ANSI_BRIGHT_YELLOW);

  currentProfile = profile;
  currentMode = mode;

  // Apply LoRa TX power
  lora.setOutputPower(currentMode->lora_power_dbm);
  colorPrint("[MODE] TX Power: " + String(currentMode->lora_power_dbm) + "dBm", ANSI_BLUE);
  colorPrint("[MODE] Sleep interval: " + String(currentMode->sleep_interval_s) + "s", ANSI_BLUE);

  // Handle lost mode tracking
  if (profile == PROFILE_LOST)
  {
    if (!inLostMode)
    {
      inLostMode = true;
      lostModeStartTime = millis();
      colorPrint("[MODE] Lost mode ACTIVATED. Timer started.", ANSI_BRIGHT_RED);
    }
  }
  else
  {
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
void transmitBinaryPacket(uint8_t *buf, uint8_t len)
{
  colorPrint("[LORA TX] Transmitting " + String(len) + " bytes...", ANSI_BLUE);

  int state = lora.standby();
  if (state != RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX WARN] Standby failed: " + String(state) + " (continuing)", ANSI_YELLOW);
  }

  state = lora.transmit(buf, len);
  if (state == RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX] Transmission successful!", ANSI_BRIGHT_GREEN);
    flickerMedium();
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    colorPrint("[LORA TX ERROR] Packet too long!", ANSI_RED);
    flickerLong();
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
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
void handleWakeupReason()
{
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason)
  {
  case ESP_SLEEP_WAKEUP_TIMER:
    colorPrint("[WAKE] Reason: Timer", ANSI_CYAN);
    performTransmissionSequence();
    break;
  case ESP_SLEEP_WAKEUP_EXT0:
    colorPrint("[WAKE] Reason: Button Press", ANSI_BRIGHT_CYAN);
    printStatusReport();
    performTransmissionSequence();
    flickerShort();
    break;
  case ESP_SLEEP_WAKEUP_GPIO:
    colorPrint("[WAKE] Reason: LoRa DIO1 Interrupt", ANSI_BRIGHT_MAGENTA);
    handleLoraReception();
    performTransmissionSequence();
    break;
  default:
    colorPrint("[WAKE] Reason: Unknown (" + String(reason) + ")", ANSI_RED);
    delay(1000);
    break;
  }
}

void handleLoraReception()
{
  colorPrint("[LORA RX] Interrupt received. Reading...", ANSI_MAGENTA);
  uint8_t rxBuf[BP_MAX_PACKET_SIZE];
  int state = lora.readData(rxBuf, sizeof(rxBuf));

  if (state == RADIOLIB_ERR_NONE)
  {
    size_t rxLen = lora.getPacketLength();
    colorPrint("[LORA RX] Received " + String(rxLen) + " bytes", ANSI_BRIGHT_GREEN);
    pkt_print_hex(rxBuf, rxLen);

    if (rxLen >= BP_MIN_PACKET_SIZE && rxBuf[0] == BP_PROTOCOL_VERSION)
    {
      handleReceivedCommand(rxBuf, rxLen);
    }
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  {
    colorPrint("[LORA RX] CRC error!", ANSI_RED);
  }
  else
  {
    colorPrint("[LORA RX] Failed, code: " + String(state), ANSI_RED);
  }
}

void processGps()
{
  if (gpsIsAwake)
  {
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }
  }
}

void gpsWake()
{
  if (!gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, HIGH);
    colorPrint("[GPS] Setting wake pin HIGH...", ANSI_YELLOW);
    gpsIsAwake = true;
    delay(100);
    colorPrint("[GPS] GPS awakened.", ANSI_YELLOW);
    delay(500);
  }
}

void gpsSleep()
{
  if (gpsIsAwake)
  {
    Serial.flush();
    digitalWrite(GPS_SLEEP_WAKE, LOW);
    colorPrint("[GPS] Putting GPS module to sleep.", ANSI_YELLOW);
    gpsIsAwake = false;
  }
}

void goToLightSleep()
{
  colorPrint("[SLEEP] Preparing for light sleep...", ANSI_BLUE);

  isHome = false;

  // Put LoRa into receive mode for wake-up
  int rxState = lora.startReceive();
  if (rxState != RADIOLIB_ERR_NONE)
    colorPrint("[SLEEP] Failed to start LoRa receive: " + String(rxState), ANSI_RED);
  else
    colorPrint("[SLEEP] LoRa is listening.", ANSI_BLUE);

  uint64_t sleepUs = (uint64_t)currentMode->sleep_interval_s * 1000000ULL;

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(sleepUs);
  colorPrint("[SLEEP] Timer wakeup: " + String(currentMode->sleep_interval_s) + "s", ANSI_BLUE);

  esp_sleep_enable_ext0_wakeup(STATUS_BUTTON_PIN, 0);
  esp_sleep_enable_gpio_wakeup();
  gpio_wakeup_enable(GPIO_NUM_39, GPIO_INTR_HIGH_LEVEL);

  colorPrint("[SLEEP] Entering light sleep...", ANSI_BOLD);
  Serial.flush();

  if (pBLEScan != nullptr)
    pBLEScan->stop();

  bootFlag = 1;
  esp_light_sleep_start();
}

// ═══════════════════════════════════════════════
// BLE Scan
// ═══════════════════════════════════════════════
bool scanForHomeBeacon(uint32_t scanDurationSeconds)
{
  if (pBLEScan == nullptr)
  {
    colorPrint("[BLE ERROR] Scanner not initialized!", ANSI_RED);
    return false;
  }

  colorPrint("[BLE] Starting scan for \"" + String(BEACON_NAME) +
             "\" (" + String(scanDurationSeconds) + "s)...", ANSI_BLUE);
  isHome = false;

  pBLEScan->start(scanDurationSeconds, false);

  unsigned long scanStart = millis();
  while (millis() - scanStart < (scanDurationSeconds * 1000))
  {
    if (isHome)
      break;
    delay(100);
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
      pBLEScan->stop();
    }
  }

  return isHome;
}

// ═══════════════════════════════════════════════
// Status Report
// ═══════════════════════════════════════════════
void printStatusReport()
{
  colorPrint("──────────── STATUS REPORT ────────────", ANSI_BOLD);
  Serial.printf("  Device: %s (0x%04X)\n", getDeviceName(MY_DEVICE_ID), MY_DEVICE_ID);
  Serial.printf("  Mode: %s\n", currentMode->name);
  Serial.printf("  TX Power: %d dBm\n", currentMode->lora_power_dbm);
  Serial.printf("  Sleep Interval: %d s\n", currentMode->sleep_interval_s);
  Serial.printf("  Uptime: %lu s\n", millis() / 1000);
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
  Serial.printf("  Free Heap: %lu\n", ESP.getFreeHeap());
  colorPrint("───────────────────────────────────────", ANSI_BOLD);
}

void colorPrint(const String &message, const char *color)
{
  Serial.print(color);
  Serial.println(message);
  Serial.print(ANSI_RESET);
  Serial.flush();
}
