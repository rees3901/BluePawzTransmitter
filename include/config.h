/*
  ┌─────────────────────────────────────────────────────────────┐
  │  CAT TRACKER - DEVICE CONFIGURATION                        │
  │  Shared between TX nodes and RX base station               │
  │  Keep this file IDENTICAL on both devices!                 │
  └─────────────────────────────────────────────────────────────┘

  This header defines all the shared configuration settings that BOTH the
  transmitter (collar) and receiver (base station) need to agree on.
  If you change anything here, you MUST reflash both TX and RX devices
  or they won't be able to communicate.

  What's in here:
  - LoRa radio parameters (frequency, spreading factor, bandwidth, etc.)
  - GPS and BLE beacon scan settings
  - Operating mode profiles (Normal, PowerSave, Active, Lost)
  - Lost mode safety timeout
  - Helper function to look up modes by name
*/

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────
// FIXED LoRa Parameters (NEVER CHANGE VIA REMOTE)
// ─────────────────────────────────────────────
// These must match between ALL devices to maintain communication.
// Changing these requires physical reprogramming of all nodes.
// If TX and RX have different values here, packets won't be decoded.

#define LORA_FREQ_MHZ 915.0 // Carrier frequency in MHz. US: 915.0, EU: 868.0
#define LORA_SF 8           // Spreading Factor (7-12). Higher = more range but slower. 8 is a good balance.
#define LORA_BW_KHZ 250.0   // Bandwidth in kHz. 250kHz gives decent speed + range tradeoff.
#define LORA_CR 5           // Coding Rate denominator (4/5). Error correction overhead. 5 = minimal, 8 = maximum.
#define LORA_PREAMBLE 16    // Preamble length in symbols. Helps the receiver detect the start of a packet.
#define LORA_USE_CRC 1      // Enable hardware CRC check on LoRa packets. 1 = on, 0 = off.
#define LORA_SYNC_WORD 0x12 // Private network sync word. All devices must use the same value. 0x34 = LoRaWAN public.

// LBT (Listen Before Talk) - Collision avoidance
// Before transmitting, the radio listens to check if the channel is clear.
// This prevents stepping on other transmissions on the same frequency.
#define LBT_ENABLED true           // Whether to use Listen Before Talk
#define LBT_MAX_RETRIES 5         // How many times to retry if channel is busy
#define LBT_RETRY_DELAY_MIN_MS 50  // Minimum random backoff delay between retries (ms)
#define LBT_RETRY_DELAY_MAX_MS 500 // Maximum random backoff delay between retries (ms)

// ─────────────────────────────────────────────
// GPS & BLE Configuration
// ─────────────────────────────────────────────

// GPS fix timeouts - how long to wait for satellite lock
#define GPS_COLD_START_TIMEOUT 60000 // 60 seconds. Used on very first boot when GPS has no almanac data.
#define GPS_WARM_START_TIMEOUT 20000 // 20 seconds. Used after the GPS already had a fix once (has almanac cached).
#define GPS_VALID_COUNT_REQUIRED 5   // Number of consecutive valid GPS readings before we trust the fix.
#define GPS_STABILISE_MS 15000       // After first fix, wait 15s for coordinates to stabilize (stop jumping around).

// BLE home beacon detection settings
// The collar scans for a Bluetooth beacon at home to know if the pet is nearby
// without needing GPS (saves battery).
#define BLE_INITIAL_SCAN_S 10 // How long to scan for the home beacon on each wake cycle (seconds)
#define BLE_SCAN_WINDOW_S 3   // Shorter BLE scan window used during GPS acquisition (seconds)
#define BEACON_NAME "HOME"    // The BLE device name the collar looks for. Must match what the base station advertises.
#define HOME_SLEEP_CYCLES 5   // After this many consecutive "at home" detections, collar sends a "BLEHome" status TX

// ─────────────────────────────────────────────
// Operating Mode Profiles
// ─────────────────────────────────────────────
// Each profile controls how the collar behaves: how often it transmits,
// how much radio power it uses, and whether the LED blinks.
// The base station can remotely switch the collar between these modes.

struct OperatingMode
{
    const char *name;                // Human-readable mode name (used in serial debug output and protocol)
    int8_t lora_power_dbm;           // LoRa transmit power in dBm. Range: 2 to 22. Higher = more range but more battery drain.
    uint16_t sleep_interval_s;       // Seconds to sleep between transmission cycles. Longer = more battery life.
    uint8_t led_flash_count;         // Number of LED flashes after a successful transmission (visual feedback).
    bool led_beacon_mode;            // If true, the LED flashes continuously while awake (for finding a lost pet).
    uint16_t led_beacon_interval_ms; // Milliseconds between beacon flashes (only used when led_beacon_mode is true).
};

// ─────────────────────────────────────────────
// Mode Definitions
// ─────────────────────────────────────────────

// NORMAL - Default everyday tracking mode.
// Balanced between battery life and update frequency.
// Transmits every 5 minutes at moderate power.
const OperatingMode MODE_NORMAL = {
    .name = "normal",
    .lora_power_dbm = 19,    // 19 dBm - good range without maxing out the radio
    .sleep_interval_s = 300, // 5 minutes between updates (will become 10 min in production)
    .led_flash_count = 5,    // 5 quick flashes on successful TX
    .led_beacon_mode = false, // No continuous beacon
    .led_beacon_interval_ms = 0};

// POWERSAVE - Maximum battery conservation mode.
// Used when the pet is known to be at home. Barely transmits.
// Low power radio, long sleep intervals.
const OperatingMode MODE_POWERSAVE = {
    .name = "powersave",
    .lora_power_dbm = 10,     // 10 dBm - minimum viable power (short range is fine at home)
    .sleep_interval_s = 1200, // 20 minutes between updates (pet is at home, no urgency)
    .led_flash_count = 5,     // Still flash on TX (negligible battery cost)
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0};

// ACTIVE - Frequent update mode for active monitoring.
// Used when you want more frequent location updates (e.g., pet is out and about).
const OperatingMode MODE_ACTIVE = {
    .name = "active",
    .lora_power_dbm = 19,   // Same power as normal for good range
    .sleep_interval_s = 60, // 1 minute between updates (much more frequent)
    .led_flash_count = 5,
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0};

// LOST - Emergency mode! Pet is missing.
// Maximum radio power, very frequent updates, LED beacon flashing to help find the pet visually.
// Has a 2-hour safety timeout to prevent total battery drain.
const OperatingMode MODE_LOST = {
    .name = "lost",
    .lora_power_dbm = 22,          // 22 dBm - MAXIMUM power for maximum range
    .sleep_interval_s = 30,        // 30 seconds between updates (aggressive but still preserves some battery)
    .led_flash_count = 10,         // 10 flashes on TX (more visible)
    .led_beacon_mode = true,       // LED flashes continuously so you can spot the collar in the dark
    .led_beacon_interval_ms = 2000 // LED beacon flashes every 2 seconds
};

// ─────────────────────────────────────────────
// Lost Mode Safety
// ─────────────────────────────────────────────
// Lost mode drains the battery fast (max power + frequent TX + LED beacon).
// These settings prevent total battery death if the owner forgets to turn it off.
#define LOST_MODE_MAX_DURATION_S 7200    // Auto-exit lost mode after 2 hours (7200 seconds)
#define LOST_MODE_FALLBACK_MODE "active" // When lost mode times out, drop back to "active" (not normal, to keep tracking)

// ─────────────────────────────────────────────
// Remote Command Protocol
// ─────────────────────────────────────────────
// These are the JSON command formats used in the OLDER protocol.
// The current binary TLV protocol replaced JSON, but these comments
// document the logical command structure for reference.

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
// Looks up an OperatingMode struct by its string name.
// Used when we receive a mode change command and need to find
// the matching profile settings.
// Returns MODE_NORMAL as a safe fallback if the name doesn't match anything.
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
    return &MODE_NORMAL; // Default fallback — if name is unrecognized, play it safe
}

#endif // CONFIG_H
