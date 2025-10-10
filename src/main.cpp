/*
  CAT TRACKER TX — LoRa GPS Collar
  SX1262 + TinyGPSPlus (Seeed XIAO nRF52840)
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>
#include <ArduinoLowPower.h>
#include <math.h>
#include <stdio.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 17
#endif

#ifndef PIN_SPI_SS
#define PIN_SPI_SS 10
#endif
#ifndef PIN_SPI_SCK
#define PIN_SPI_SCK 13
#endif
#ifndef PIN_SPI_MOSI
#define PIN_SPI_MOSI 11
#endif
#ifndef PIN_SPI_MISO
#define PIN_SPI_MISO 12
#endif

#define LORA_NSS 4 // D4 per Wio-SX1262 header
#define LORA_SCK PIN_SPI_SCK
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_MISO PIN_SPI_MISO
#define LORA_RST 2   // D2 from blue digital header
#define LORA_BUSY 3  // D3 from blue digital header
#define LORA_DIO1 1  // D1 matches DIO1 pad
#define LORA_RF_SW 5 // D5 drives RF switch

#ifdef PIN_SERIAL1_RX
#define GPS_RX PIN_SERIAL1_RX
#else
#define GPS_RX 7
#endif
#ifdef PIN_SERIAL1_TX
#define GPS_TX PIN_SERIAL1_TX
#else
#define GPS_TX 6
#endif
#define GPS_BAUD 9600
#define GPS_SLEEP_WAKE 8
#define GPS_RESET 9

#define STATUS_BUTTON_PIN -1
#define STATUS_LED_PIN LED_BUILTIN

#define SENDER_ID "Macy"
#define HOME_LAT 51.87370573411073
#define HOME_LON -2.2396017778476716

#define SLEEP_DURATION_AWAY_US (10ULL * 1000000ULL)
#define SLEEP_DURATION_HOME_US (10ULL * 1000000ULL)

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

void colorPrint(const String &message, const char *color = ANSI_RESET);
void printStatusReport();
void gpsWake();
void gpsSleep();
String cardinalDirection(double bearing);
void processGps();
void transmitLora(const String &payload);
String buildJsonPayload();
void craftLoraPacket();
void performTransmissionSequence();
void goToLightSleep();
bool scanForHomeBeacon(uint32_t scanDurationSeconds);
void handleWakeupReason();
void handleLoraReception();
void buttonWakeISR();

void flickerShort();
void flickerMedium();
void flickerLong();

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
TinyGPSPlus gps;
static uint32_t messageId = 0;
uint16_t DEVICE_ID_HEX = 0x0000;
String loraRxMsg;
volatile bool gpsIsAwake = false;
volatile bool isHome = false;
volatile uint64_t currentSleepDurationUs = SLEEP_DURATION_AWAY_US;

enum WakeReason
{
  WAKE_TIMER,
  WAKE_BUTTON,
  WAKE_LORA
};

volatile WakeReason wakeReason = WAKE_TIMER;
bool bleReady = false;

const char *targetDeviceName = "CAT_TRACKER_HQ";

void colorPrint(const String &message, const char *color)
{
  Serial.print(color);
  Serial.println(message);
  Serial.print(ANSI_RESET);
  Serial.flush();
}

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
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 12; i++)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(80);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(80);
  }
}

void buttonWakeISR()
{
  wakeReason = WAKE_BUTTON;
}

void handleLoraReception()
{
  wakeReason = WAKE_LORA;

  String rxPayload;
  int state = lora.readData(rxPayload);
  if (state == RADIOLIB_ERR_NONE)
  {
    loraRxMsg = rxPayload;
    colorPrint(String("[LORA RX] Received: ") + rxPayload, ANSI_BRIGHT_GREEN);
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  {
    colorPrint("[LORA RX] CRC error", ANSI_RED);
    loraRxMsg = "";
  }
  else
  {
    colorPrint(String("[LORA RX] Failed, code: ") + state, ANSI_RED);
    loraRxMsg = "";
  }
}

void gpsWake()
{
  if (!gpsIsAwake)
  {
    digitalWrite(GPS_SLEEP_WAKE, HIGH);
    delay(50);
#if defined(ARDUINO_SEEED_XIAO_NRF52840) || defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE)
    Serial1.setPins(GPS_RX, GPS_TX);
#endif
    Serial1.begin(GPS_BAUD);
    gpsIsAwake = true;
    colorPrint("[GPS] Woke GPS module", ANSI_YELLOW);
  }
}

void gpsSleep()
{
  if (gpsIsAwake)
  {
    Serial1.end();
    digitalWrite(GPS_SLEEP_WAKE, LOW);
    gpsIsAwake = false;
    colorPrint("[GPS] GPS module sleeping", ANSI_YELLOW);
  }
}

void processGps()
{
  while (Serial1.available() > 0)
  {
    char c = Serial1.read();
    gps.encode(c);
  }
}

String buildJsonPayload()
{
  StaticJsonDocument<300> doc;

  doc["mid"] = messageId;
  doc["did"] = "0x" + String(DEVICE_ID_HEX, HEX);
  doc["id"] = SENDER_ID;

  if (isHome)
  {
    double truncatedHomeLat = round(HOME_LAT * 100000.0) / 100000.0;
    double truncatedHomeLon = round(HOME_LON * 100000.0) / 100000.0;

    doc["stat"] = "H";
    doc["lat"] = truncatedHomeLat;
    doc["lon"] = truncatedHomeLon;
    doc["sat"] = -1;
    doc["dst"] = 0.0;
    doc["dir"] = "NA";
    doc["ts"] = "NA";
  }
  else
  {
    bool locValid = gps.location.isValid();
    bool locFresh = gps.location.isValid() && gps.location.age() < 60000;
    bool satValid = gps.satellites.isValid();
    bool timeValid = gps.time.isValid() && gps.time.age() < 60000;

    if (locValid && locFresh)
    {
      double currentLat = gps.location.lat();
      double currentLon = gps.location.lng();
      double truncatedLat = round(currentLat * 100000.0) / 100000.0;
      double truncatedLon = round(currentLon * 100000.0) / 100000.0;

      doc["stat"] = "O";
      doc["lat"] = truncatedLat;
      doc["lon"] = truncatedLon;
      doc["sat"] = satValid ? gps.satellites.value() : 0;

      double dist = TinyGPSPlus::distanceBetween(currentLat, currentLon, HOME_LAT, HOME_LON);
      double bearing = TinyGPSPlus::courseTo(currentLat, currentLon, HOME_LAT, HOME_LON);

      doc["dst"] = round(dist * 100.0) / 100.0;
      doc["dir"] = String((int)bearing) + "-" + cardinalDirection(bearing);
    }
    else
    {
      doc["stat"] = "E";
      doc["lat"] = 0.0;
      doc["lon"] = 0.0;
      doc["sat"] = satValid ? gps.satellites.value() : 0;
      doc["dst"] = 0.0;
      doc["dir"] = "NA";
    }

    if (timeValid)
    {
      char isoTimestamp[25];
      snprintf(isoTimestamp, sizeof(isoTimestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second());
      doc["ts"] = isoTimestamp;
    }
    else
    {
      doc["ts"] = "E";
    }
  }

  char payload[256];
  serializeJson(doc, payload, sizeof(payload));
  return String(payload);
}

void transmitLora(const String &payload)
{
  colorPrint("[LORA TX] Preparing to transmit...", ANSI_BLUE);
  colorPrint("=== TRANSMITTING PAYLOAD OVER LORA ===", "\033[38;5;208m");
  colorPrint(String("[LORA TX] Transmitting packet (") + payload.length() + " bytes)...", ANSI_BLUE);
  colorPrint(String("  Payload: ") + payload, ANSI_CYAN);

  int state = lora.standby();
  if (state != RADIOLIB_ERR_NONE)
  {
    colorPrint(String("[LORA TX ERROR] Failed to enter standby: ") + state, ANSI_RED);
    flickerLong();
    return;
  }

  state = lora.transmit(payload);
  if (state == RADIOLIB_ERR_NONE)
  {
    colorPrint("[LORA TX] Transmission successful!", ANSI_BRIGHT_GREEN);
    flickerMedium();
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    colorPrint("[LORA TX ERROR] Packet too long", ANSI_RED);
    flickerLong();
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    colorPrint("[LORA TX ERROR] Transmission timeout", ANSI_RED);
    flickerLong();
  }
  else
  {
    colorPrint(String("[LORA TX ERROR] Transmission failed, code: ") + state, ANSI_RED);
    flickerLong();
  }
}

void craftLoraPacket()
{
  messageId++;
  String payload = buildJsonPayload();
  transmitLora(payload);
}

bool scanForHomeBeacon(uint32_t scanDurationSeconds)
{
  if (!bleReady)
  {
    colorPrint("[BLE] Initialising BLE peripheral...", ANSI_BLUE);
    bleReady = BLE.begin();
    if (!bleReady)
    {
      colorPrint("[BLE ERROR] BLE.begin() failed", ANSI_RED);
      return false;
    }
  }

  colorPrint(String("[BLE] Scanning for \"") + targetDeviceName + "\" (" + scanDurationSeconds + "s)...", ANSI_BLUE);
  isHome = false;

  BLE.scanForName(targetDeviceName);
  unsigned long start = millis();
  while (millis() - start < (scanDurationSeconds * 1000UL))
  {
    BLEDevice peripheral = BLE.available();
    if (peripheral)
    {
      if (peripheral.hasLocalName() && peripheral.localName() == targetDeviceName)
      {
        colorPrint(String("[BLE] Found Home Beacon: ") + targetDeviceName, ANSI_BRIGHT_GREEN);
        isHome = true;
        break;
      }
    }
    BLE.poll();
    delay(50);
  }

  BLE.stopScan();
  if (!isHome)
  {
    colorPrint("[BLE] Home Beacon not found", ANSI_YELLOW);
  }

  return isHome;
}

void goToLightSleep()
{
  colorPrint("[SLEEP] Preparing for low-power interval...", ANSI_BLUE);
  loraRxMsg = "";

  if (isHome)
  {
    colorPrint("[SLEEP] Clearing Home Beacon flag", ANSI_BLUE);
    isHome = false;
  }

  int rxState = lora.startReceive();
  if (rxState != RADIOLIB_ERR_NONE)
  {
    colorPrint(String("[SLEEP] Failed to start LoRa receive, code: ") + rxState, ANSI_RED);
  }
  else
  {
    colorPrint("[SLEEP] LoRa listening for wake-up", ANSI_BLUE);
  }

  wakeReason = WAKE_TIMER;

  if (STATUS_BUTTON_PIN >= 0)
  {
    LowPower.attachInterruptWakeup(STATUS_BUTTON_PIN, buttonWakeISR, FALLING);
  }

  uint32_t sleepDurationMs = (uint32_t)(currentSleepDurationUs / 1000ULL);
  colorPrint(String("[SLEEP] Sleeping for ") + (sleepDurationMs / 1000.0f) + " s", ANSI_BLUE);
  LowPower.sleep(sleepDurationMs);

  if (STATUS_BUTTON_PIN >= 0)
  {
    LowPower.detachInterruptWakeup(STATUS_BUTTON_PIN);
  }
}

void handleWakeupReason()
{
  WakeReason reason = wakeReason;
  wakeReason = WAKE_TIMER;

  switch (reason)
  {
  case WAKE_TIMER:
    colorPrint("[WAKE] Reason: Timer", ANSI_CYAN);
    performTransmissionSequence();
    break;
  case WAKE_BUTTON:
    colorPrint("[WAKE] Reason: Button", ANSI_BRIGHT_CYAN);
    printStatusReport();
    performTransmissionSequence();
    flickerShort();
    break;
  case WAKE_LORA:
    colorPrint("[WAKE] Reason: LoRa DIO1", ANSI_BRIGHT_MAGENTA);
    if (!loraRxMsg.isEmpty())
    {
      colorPrint(String("[WAKE] LoRa buffer contains: ") + loraRxMsg, ANSI_BRIGHT_MAGENTA);
    }
    performTransmissionSequence();
    break;
  }
}

void performTransmissionSequence()
{
  colorPrint("[SEQUENCE] Starting transmission sequence", ANSI_MAGENTA);

  bool foundHome = scanForHomeBeacon(10);
  currentSleepDurationUs = foundHome ? SLEEP_DURATION_HOME_US : SLEEP_DURATION_AWAY_US;
  colorPrint(String("[SEQUENCE] Next sleep duration: ") + (currentSleepDurationUs / 1000000ULL) + " s", foundHome ? ANSI_BRIGHT_GREEN : ANSI_YELLOW);

  if (!foundHome)
  {
    gpsWake();
    colorPrint("[SEQUENCE] Attempting GPS fix (max 60s)", ANSI_YELLOW);

    unsigned long fixStart = millis();
    bool fixStabilised = false;
    bool firstFixDetected = false;
    unsigned long firstFixTimestamp = 0;

    while (millis() - fixStart < 60000UL)
    {
      processGps();

      if (!firstFixDetected && gps.location.isValid() && gps.location.age() < 5000)
      {
        firstFixDetected = true;
        firstFixTimestamp = millis();
        colorPrint("[SEQUENCE] Initial GPS fix detected, waiting 10s for stability", ANSI_GREEN);
      }

      if (firstFixDetected && (millis() - firstFixTimestamp >= 10000UL))
      {
        colorPrint("[SEQUENCE] GPS fix stabilised", ANSI_GREEN);
        fixStabilised = true;
        break;
      }

      delay(25);
    }

    if (!fixStabilised)
    {
      if (firstFixDetected)
      {
        colorPrint("[SEQUENCE] Using initial GPS fix (not fully stabilised)", ANSI_YELLOW);
      }
      else
      {
        colorPrint("[SEQUENCE] GPS fix timeout, proceeding without valid fix", ANSI_YELLOW);
      }
    }

    gpsSleep();
  }
  else
  {
    gpsSleep();
    colorPrint("[SEQUENCE] Home beacon detected, skipping GPS", ANSI_BRIGHT_GREEN);
  }

  craftLoraPacket();
  colorPrint("[SEQUENCE] Transmission sequence complete", ANSI_MAGENTA);
}

void printStatusReport()
{
  colorPrint("-------------------- STATUS REPORT --------------------", ANSI_BOLD);
  Serial.print("  Uptime: ");
  Serial.print(millis() / 1000UL);
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
  Serial.println(gps.satellites.isValid() ? String(gps.satellites.value()) : "Invalid");
  Serial.print("  HDOP: ");
  Serial.println(gps.hdop.isValid() ? String(gps.hdop.value() / 100.0) : "Invalid");
  Serial.print("  Last Rx Msg: ");
  Serial.println(loraRxMsg.length() > 0 ? loraRxMsg : "None");
  Serial.print("  Free Heap: ");
  Serial.println("N/A on nRF52");
  colorPrint("-------------------------------------------------------", ANSI_BOLD);
}

String cardinalDirection(double bearing)
{
  static const char *directions[] = {
      "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

  bearing = fmod(bearing + 360.0, 360.0);
  int index = (int)round(bearing / 22.5) % 16;
  return String(directions[index]);
}

void setup()
{
  delay(100);
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
  {
    delay(10);
  }

  colorPrint("[BOOT] Initialising CAT TRACKER TX for XIAO nRF52840...", ANSI_BRIGHT_CYAN);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (STATUS_BUTTON_PIN >= 0)
  {
    pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP);
  }
  pinMode(GPS_SLEEP_WAKE, OUTPUT);
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);
  digitalWrite(GPS_SLEEP_WAKE, HIGH);
  gpsIsAwake = true;

  pinMode(LORA_RF_SW, OUTPUT);
  digitalWrite(LORA_RF_SW, HIGH);

  String sender = SENDER_ID;
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
    DEVICE_ID_HEX = 0xFFFF;
    colorPrint(String("[WARN] Unknown SENDER_ID: ") + sender + ". Using default hex ID.", ANSI_YELLOW);
  }
  colorPrint(String("[INIT] Device ID set to: 0x") + String(DEVICE_ID_HEX, HEX), ANSI_BLUE);

#if defined(ARDUINO_SEEED_XIAO_NRF52840) || defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE)
  Serial1.setPins(GPS_RX, GPS_TX);
#endif
  Serial1.begin(GPS_BAUD);
  delay(100);
  colorPrint("[GPS] Checking initial serial activity...", ANSI_YELLOW);
  delay(1000);
  if (Serial1.available() > 0)
  {
    colorPrint("[GPS] Serial data detected!", ANSI_BRIGHT_GREEN);
    while (Serial1.available() > 0)
    {
      Serial1.read();
    }
  }
  else
  {
    colorPrint("[GPS] No serial data detected after 1s. Verify wiring/power.", ANSI_BRIGHT_RED);
  }

  colorPrint("[GPS] Warming up GPS (60s window)...", ANSI_YELLOW);
  unsigned long gpsWarmupStart = millis();
  bool fixStabilised = false;
  bool firstFixDetected = false;
  unsigned long firstFixTimestamp = 0;

  while (millis() - gpsWarmupStart < 60000UL)
  {
    processGps();

    if (!firstFixDetected && gps.location.isValid() && gps.location.age() < 5000)
    {
      firstFixDetected = true;
      firstFixTimestamp = millis();
      colorPrint("[GPS] Initial fix obtained, waiting 10s for stability...", ANSI_BRIGHT_GREEN);
    }

    if (firstFixDetected && (millis() - firstFixTimestamp >= 10000UL))
    {
      colorPrint("[GPS] 10s stabilisation complete during warmup.", ANSI_BRIGHT_GREEN);
      fixStabilised = true;
      break;
    }

    delay(25);
  }

  if (fixStabilised)
  {
    colorPrint("[GPS] Warmup complete with stabilised fix.", ANSI_GREEN);
  }
  else if (firstFixDetected)
  {
    colorPrint("[GPS] Warmup ended with early fix (not fully stabilised).", ANSI_YELLOW);
  }
  else
  {
    colorPrint("[GPS] Warmup ended without valid fix.", ANSI_RED);
  }

  gpsSleep();

  SPI.begin();
  int initState = lora.begin(915.0);
  if (initState != RADIOLIB_ERR_NONE)
  {
    colorPrint(String("[ERROR] LoRa init failed. Code: ") + initState, ANSI_RED);
    flickerLong();
    while (true)
    {
      delay(1000);
    }
  }

  lora.setOutputPower(18);
  lora.setSpreadingFactor(8);
  lora.setBandwidth(250.0);
  lora.setCodingRate(5);
  lora.setCRC(true);
  lora.setPreambleLength(8);
  lora.setDio1Action(handleLoraReception);
  colorPrint("[INIT] LoRa initialised and configured", ANSI_BLUE);

  flickerShort();
  colorPrint("════════════════════════════════════════", ANSI_BOLD);
  colorPrint("🚀 SETUP COMPLETE - Entering initial sleep cycle 😴", ANSI_BOLD);
  colorPrint("════════════════════════════════════════", ANSI_BOLD);

  performTransmissionSequence();
  goToLightSleep();
}

void loop()
{
  handleWakeupReason();
  goToLightSleep();
}
