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

// V3.3.0 range-tuning. SF8/BW250 → SF9/BW125 trades raw data-rate for
// link budget: SF8→SF9 ≈ +2.5 dB RX sensitivity, BW250→BW125 ≈ +3 dB
// (noise floor halves). Combined ≈ +5.5 dB, roughly 1.5–2× range in
// free space (more through clutter). Cost is ~4× airtime per packet
// (a ~120-byte telemetry frame goes from ~50 ms to ~700 ms on air) —
// fine for our slow telemetry cadence, but see the EU duty-cycle note.
//
// Frequency 868.0 MHz = EU/UK ISM band (was 915 = US). REGULATORY FYI:
//   • 868.0–868.6 MHz: +14 dBm (25 mW) ERP max, ≤1% duty cycle.
//   • 869.4–869.65 MHz: +27 dBm (500 mW) ERP, 10% duty cycle — the
//     high-power sub-band. If you want max power *legally*, retune to
//     ~869.525 here. The SX1262 caps at +22 dBm regardless.
//   Our normal(17)/lost(22) dBm levels exceed the 868.0 +14 dBm limit,
//   and lost mode's 30 s cadence at ~0.7 s airtime ≈ 2% duty (over the
//   1% limit on this sub-band). Acceptable for a lost-pet emergency
//   beacon, but switch to the 869.4–869.65 sub-band if you want to be
//   fully compliant at high power/duty.
//
// These MUST match on ALL nodes (freq, SF, BW, sync word, CRC, header
// mode, preamble). CR is carried in the explicit header so RX auto-
// detects it, but we keep it aligned for clarity.
#define LORA_FREQ_MHZ 868.0 // EU/UK 868MHz ISM (was 915 = US)
#define LORA_SF 9           // Spreading Factor (7-12). 9 = range/airtime balance
#define LORA_BW_KHZ 125.0   // Bandwidth (kHz). 125 narrows noise floor for +3dB
#define LORA_CR 5           // Coding Rate 4/5 (auto-detected by RX via header)
#define LORA_PREAMBLE 16    // Preamble length (base RX is always-on, 16 is ample)
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
#define BEACON_NAME "Home"    // BLE beacon device name (case-sensitive match!)
// HOME_SLEEP_CYCLES removed — now per-mode via OperatingMode.home_heartbeat_cycles

// V3: RSSI gate for the "Home" beacon. The beacon advertises at -12 dBm and
// the collar only counts it as "home" when received signal is >= this value.
// A higher (less negative) value means the cat must be physically closer to
// the base station to register as home. Walk-test to tune.
// Field observation: beacons a few metres apart read -70 to -80 dBm,
// so -85 gives reliable detection across the house.
#define HOME_RSSI_THRESHOLD_DBM (-90)

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
    uint8_t home_heartbeat_cycles;   // Home cycles before GPS heartbeat TX
};

// ─────────────────────────────────────────────
// Mode Definitions
// ─────────────────────────────────────────────

// NORMAL - Daily tracking, balanced performance
// V3.3.0: 19 → 17 dBm. With the SF9/BW125 PHY change adding ~+5.5 dB of
// link budget, 17 dBm here still beats the OLD 19 dBm @ SF8/BW250 by
// ~+3.5 dB net — so everyday range IMPROVES while leaving a 5 dB power
// headroom that only emergency 'lost' mode unlocks (22 dBm). Holding
// normal below max also runs the PA cooler and saves battery per TX.
const OperatingMode MODE_NORMAL = {
    .name = "normal",
    .lora_power_dbm = 17,    // Range headroom reserved for lost mode (was 19)
    .sleep_interval_s = 300, // 5 minutes (will become 10 min in production)
    .led_flash_count = 5,
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0,
    .home_heartbeat_cycles = 10};

// POWERSAVE - Maximum battery life at home
const OperatingMode MODE_POWERSAVE = {
    .name = "powersave",
    .lora_power_dbm = 10,     // Minimum viable power
    .sleep_interval_s = 1200, // 20 minutes
    .led_flash_count = 5,     // Keep standard flash (negligible power)
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0,
    .home_heartbeat_cycles = 10};

// ACTIVE - Frequent updates for monitoring
const OperatingMode MODE_ACTIVE = {
    .name = "active",
    .lora_power_dbm = 17,   // Match normal; headroom reserved for lost (was 19)
    .sleep_interval_s = 60, // 1 minute
    .led_flash_count = 5,
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0,
    .home_heartbeat_cycles = 5};

// LOST - Emergency mode with visual beacon
const OperatingMode MODE_LOST = {
    .name = "lost",
    .lora_power_dbm = 22,          // Maximum power for range
    .sleep_interval_s = 30,        // 30 seconds (still need battery conservation!)
    .led_flash_count = 10,         // More flashes on TX
    .led_beacon_mode = true,       // Enable continuous beacon
    .led_beacon_interval_ms = 2000, // Flash every 2 seconds
    .home_heartbeat_cycles = 3
};

// DEVELOPER - Rapid cycle for testing and diagnostics
// V3.3.0: 19 → 14 dBm. Deliberately the lowest of the awake modes so
// bench/field testing happens with a real power margin still in hand —
// if the link holds at 14 dBm it'll be rock-solid once deployed at 17,
// and lost mode's 22 dBm is a full 8 dB above. Bump this back to 17 to
// match 'normal' if you specifically want representative-range testing.
const OperatingMode MODE_DEVELOPER = {
    .name = "developer",
    .lora_power_dbm = 14,   // Lowest awake power — proves the margin (was 19)
    .sleep_interval_s = 30, // 30 seconds for rapid debugging
    .led_flash_count = 3,   // Quick triple flash (visually distinct)
    .led_beacon_mode = false,
    .led_beacon_interval_ms = 0,
    .home_heartbeat_cycles = 3};

// ─────────────────────────────────────────────
// Developer Mode Configuration
// ─────────────────────────────────────────────
#define DEV_MODE_BUTTON_PIN 21       // GPIO21 (user button on SX1262 expansion)
#define DEV_MODE_DOUBLE_PRESS_MS 500 // Max gap between presses for double-press

// ─────────────────────────────────────────────
// Lost Mode Safety
// ─────────────────────────────────────────────
#define LOST_MODE_MAX_DURATION_S 7200    // 2 hours (120 minutes)
#define LOST_MODE_FALLBACK_MODE "normal" // Revert to normal mode after timeout

// ─────────────────────────────────────────────
// Geofence Configuration
// ─────────────────────────────────────────────
#define GEOFENCE_DEFAULT_RADIUS_M 500.0  // Default radius in metres
#define GEOFENCE_ESCALATE_MODE "active"  // Auto-escalate to this mode when outside fence
#define GEOFENCE_HYSTERESIS_M 20.0       // Must re-enter by this margin to avoid flapping

// ─────────────────────────────────────────────
// Remote Command Protocol
// ─────────────────────────────────────────────

// Command structure (base station → node):
// {"cmd":"mode","profile":"lost"}
// {"cmd":"mode","profile":"normal"}
// {"cmd":"mode","profile":"active"}
// {"cmd":"mode","profile":"powersave"}
// {"cmd":"mode","profile":"developer"}
// {"cmd":"get_status"}           // Request current mode/battery/GPS status
// {"cmd":"ping"}                 // Presence check — collar replies with pong + link stats
// {"cmd":"set_geofence","device_id":N,"lat":XX.X,"lon":YY.Y,"radius_m":500}
// {"cmd":"set_geofence","device_id":N,"enabled":false}  // Disable geofence

// Response structure (node → base station):
// {"ack":"mode","profile":"lost","power":22,"sleep":30}
// {"status":"ok","mode":"normal","battery":3.7,"gps":"locked","uptime":3600}
// {"ack":"set_geofence","ok":true,"lat":XX.X,"lon":YY.Y,"radius_m":500}
// {"pong":true,"device":"Podge","rssi":-45,"snr":8.5,"uptime_ms":3200}

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
    if (strcmp(name, "developer") == 0)
        return &MODE_DEVELOPER;
    return &MODE_NORMAL; // Default fallback
}

#endif // CONFIG_H
