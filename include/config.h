/*
  ┌─────────────────────────────────────────────────────────────┐
  │  CAT TRACKER - DEVICE CONFIGURATION                        │
  │  Shared between TX nodes and RX base station               │
  │  Keep this file IDENTICAL on both devices!                 │
  └─────────────────────────────────────────────────────────────┘
*/

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────
// FIXED LoRa Parameters (NEVER CHANGE VIA REMOTE)
// ─────────────────────────────────────────────
// These must match between ALL devices to maintain communication
// Changing these requires physical reprogramming of all nodes

#define LORA_FREQ_MHZ 915.0 // US 915MHz (EU: 868.0)
#define LORA_SF 8           // Spreading Factor (7-12)
#define LORA_BW_KHZ 250.0   // Bandwidth (kHz)
#define LORA_CR 5           // Coding Rate 4/5
#define LORA_PREAMBLE 16    // Preamble length
#define LORA_USE_CRC 1      // Enable CRC
#define LORA_SYNC_WORD 0x12 // Private network sync word

// LBT (Listen Before Talk) - Collision avoidance
#define LBT_ENABLED true
#define LBT_MAX_RETRIES 5
#define LBT_RETRY_DELAY_MIN_MS 50
#define LBT_RETRY_DELAY_MAX_MS 500

// ─────────────────────────────────────────────
// GPS & BLE Configuration
// ─────────────────────────────────────────────
#define GPS_COLD_START_TIMEOUT 60000 // 60s for initial cold start
#define GPS_WARM_START_TIMEOUT 20000 // 20s for subsequent warm starts
#define GPS_VALID_COUNT_REQUIRED 5   // Consecutive valid fixes needed
#define GPS_STABILISE_MS 15000       // Stabilization period (15s)

#define BLE_INITIAL_SCAN_S 10 // Initial BLE scan on wake
#define BLE_SCAN_WINDOW_S 3   // BLE scan window during GPS
#define BEACON_NAME "Home"    // BLE beacon device name
#define HOME_SLEEP_CYCLES 5   // Cycles at home before "BLEHome" TX

// ─────────────────────────────────────────────
// Operating Mode Profiles
// ─────────────────────────────────────────────

struct OperatingMode
{
    const char *name;
    int8_t lora_power_dbm;           // TX power (2-22 dBm)
    uint16_t sleep_interval_s;       // Sleep duration between cycles
    uint8_t led_flash_count;         // LED flashes per TX success
    bool led_beacon_mode;            // Continuous LED beacon while awake
    uint16_t led_beacon_interval_ms; // Interval between beacon flashes
};

// ─────────────────────────────────────────────
// Mode Definitions
// ─────────────────────────────────────────────

// NORMAL - Daily tracking, balanced performance
const OperatingMode MODE_NORMAL = {
    .name = "normal",
    .lora_power_dbm = 19,    // Good range, not max power
    .sleep_interval_s = 300, // 5 minutes (will become 10 min in production)
    .led_flash_count = 5,
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0};

// POWERSAVE - Maximum battery life at home
const OperatingMode MODE_POWERSAVE = {
    .name = "powersave",
    .lora_power_dbm = 10,     // Minimum viable power
    .sleep_interval_s = 1200, // 20 minutes
    .led_flash_count = 5,     // Keep standard flash (negligible power)
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0};

// ACTIVE - Frequent updates for monitoring
const OperatingMode MODE_ACTIVE = {
    .name = "active",
    .lora_power_dbm = 19,   // Same as normal
    .sleep_interval_s = 60, // 1 minute
    .led_flash_count = 5,
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0};

// LOST - Emergency mode with visual beacon
const OperatingMode MODE_LOST = {
    .name = "lost",
    .lora_power_dbm = 22,          // Maximum power for range
    .sleep_interval_s = 30,        // 30 seconds (still need battery conservation!)
    .led_flash_count = 10,         // More flashes on TX
    .led_beacon_mode = true,       // Enable continuous beacon
    .led_beacon_interval_ms = 2000 // Flash every 2 seconds
};

// ─────────────────────────────────────────────
// Lost Mode Safety
// ─────────────────────────────────────────────
#define LOST_MODE_MAX_DURATION_S 7200    // 2 hours (120 minutes)
#define LOST_MODE_FALLBACK_MODE "active" // Revert to active mode after timeout

// ─────────────────────────────────────────────
// Remote Command Protocol
// ─────────────────────────────────────────────

// Command structure (base station → node):
// {"cmd":"mode","profile":"lost"}
// {"cmd":"mode","profile":"normal"}
// {"cmd":"mode","profile":"active"}
// {"cmd":"mode","profile":"powersave"}
// {"cmd":"get_status"}           // Request current mode/battery/GPS status

// Response structure (node → base station):
// {"ack":"mode","profile":"lost","power":22,"sleep":30}
// {"status":"ok","mode":"normal","battery":3.7,"gps":"locked","uptime":3600}

// ─────────────────────────────────────────────
// Helper Function: Get Mode by Name
// ─────────────────────────────────────────────
inline const OperatingMode *getModeByName(const char *name)
{
    if (strcmp(name, "normal") == 0)
        return &MODE_NORMAL;
    if (strcmp(name, "powersave") == 0)
        return &MODE_POWERSAVE;
    if (strcmp(name, "active") == 0)
        return &MODE_ACTIVE;
    if (strcmp(name, "lost") == 0)
        return &MODE_LOST;
    return &MODE_NORMAL; // Default fallback
}

#endif // CONFIG_H
