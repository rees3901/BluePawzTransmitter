/*
  ┌─────────────────────────────────────────────────────────────┐
  │  CAT TRACKER - BINARY TELEMETRY PROTOCOL v1                 │
  │  Shared between TX nodes and RX base station                │
  │  Keep this file IDENTICAL on both devices!                  │
  └─────────────────────────────────────────────────────────────┘

  This file defines the binary packet format used for all communication
  between collar (TX) and base station (RX) over LoRa.

  Replaces the old JSON format with a compact TLV-based binary format
  to minimize airtime (less time on air = less battery + less interference).

  PACKET STRUCTURE:
  ┌──────────────────┬──────────────────┬────────────┐
  │  Fixed Header    │  TLV Extensions  │  CRC-16    │
  │  (36 bytes)      │  (0-28 bytes)    │  (2 bytes) │
  └──────────────────┴──────────────────┴────────────┘

  Min packet: 38 bytes (header + CRC, no TLV)
  Max packet: 66 bytes (header + max TLV + CRC)

  HEADER BYTE MAP (36 bytes):
  Offset  Size  Field
  ──────  ────  ─────
  0       1     Protocol version (always 1)
  1       2     Device ID (uint16, little-endian)
  3       4     Message sequence number (uint32)
  7       4     Unix timestamp from GPS (uint32)
  11      1     Status byte (bp_status_t enum)
  12      2     Flags bitfield (uint16: bits 0-3 = packet type, bits 4-15 = boolean flags)
  14      4     Latitude (int32, degrees * 1e7)
  18      4     Longitude (int32, degrees * 1e7)
  22      2     Battery voltage in millivolts (uint16)
  24      2     GPS accuracy in meters (uint16)
  26      2     GPS fix age in seconds (uint16)
  28      2     Speed in cm/s (uint16)
  30      2     Distance to home in meters (uint16)
  32      2     Bearing to home in degrees (uint16)
  34      1     TLV section length in bytes (uint8)
  35      1     Reserved / padding
*/

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>

// ═══════════════════════════════════════════════
// Constants — Packet size limits
// ═══════════════════════════════════════════════
#define BP_PROTOCOL_VERSION 1   // Current protocol version. Increment if format changes.
#define BP_HEADER_SIZE 36       // Fixed header is always 36 bytes
#define BP_MAX_TLV_SIZE 28      // Maximum bytes allowed for TLV extensions
#define BP_CRC_SIZE 2           // CRC-16 checksum appended to every packet
#define BP_MIN_PACKET_SIZE (BP_HEADER_SIZE + BP_CRC_SIZE)                   // 38 bytes minimum (no TLV data)
#define BP_MAX_PACKET_SIZE (BP_HEADER_SIZE + BP_MAX_TLV_SIZE + BP_CRC_SIZE) // 66 bytes maximum (full TLV)

// ═══════════════════════════════════════════════
// Status Enum (u8) — Describes the collar's current state
// ═══════════════════════════════════════════════
// This is sent in every telemetry packet so the base station knows
// what the collar is doing right now.
enum bp_status_t : uint8_t
{
    STATUS_UNKNOWN = 0x00,       // Shouldn't happen — indicates uninitialized state
    STATUS_OUT_AND_ABOUT = 0x01, // Pet is outdoors with a valid GPS fix
    STATUS_BLE_HOME = 0x02,      // Pet is at home (detected via BLE beacon or GPS proximity)
    STATUS_INVALID_GPS = 0x03,   // GPS couldn't get a fix (timed out / no satellites)
    STATUS_OK = 0x04,            // General positive response (used in status/ack packets)
    STATUS_LOST_TIMEOUT = 0x05,  // Lost mode auto-timed-out after 2 hours (alert packet)
};

// ═══════════════════════════════════════════════
// Flags Bitfield (u16)
// ═══════════════════════════════════════════════
// The flags field (2 bytes at offset 12) serves double duty:
//   - Bits 0-3: Packet type (what kind of message this is)
//   - Bits 4-15: Boolean flags (GPS valid, BLE home, warm start, etc.)
// This lets us pack the packet type and boolean state into a single u16.
#define PKT_TYPE_MASK 0x000F // Mask to extract just the packet type from flags

// Packet types — determines what this packet means and how to interpret it
enum bp_pkt_type_t : uint16_t
{
    PKT_TELEMETRY = 0x0001,   // TX→RX: Regular position/status update (the main packet type)
    PKT_MODE_ACK = 0x0002,    // TX→RX: "I received your mode change command and applied it"
    PKT_STATUS_RESP = 0x0003, // TX→RX: Response to a status query (battery, mode, GPS info, etc.)
    PKT_ALERT = 0x0004,       // TX→RX: Alert notification (e.g., lost mode timed out)
    PKT_CMD_MODE = 0x0005,    // RX→TX: Command to change operating mode (e.g., switch to "lost")
    PKT_CMD_STATUS = 0x0006,  // RX→TX: Request the collar to send back its current status
};

// Boolean flags — OR'd into the flags field alongside the packet type
#define FLAG_HAS_GPS 0x0010  // Set if this packet contains valid GPS coordinates (lat/lon are meaningful)
#define FLAG_BLE_HOME 0x0020 // Set if the BLE home beacon was detected during this cycle
#define FLAG_GPS_WARM 0x0040 // Set if the GPS module is in warm-start state (faster fixes)

// ═══════════════════════════════════════════════
// Profile Enum (u8) — Operating mode identifier
// ═══════════════════════════════════════════════
// These map to the OperatingMode structs defined in config.h.
// Sent in TLV fields so the base station knows which mode the collar is in,
// or to command the collar to switch to a different mode.
enum bp_profile_t : uint8_t
{
    PROFILE_UNKNOWN = 0x00,   // Invalid/unrecognized profile
    PROFILE_NORMAL = 0x01,    // Normal tracking (5 min interval, 19 dBm)
    PROFILE_POWERSAVE = 0x02, // Power save (20 min interval, 10 dBm)
    PROFILE_ACTIVE = 0x03,    // Active tracking (1 min interval, 19 dBm)
    PROFILE_LOST = 0x04,      // Lost/emergency (30s interval, 22 dBm, LED beacon)
};

// ═══════════════════════════════════════════════
// TLV Type IDs (u8)
// ═══════════════════════════════════════════════
// TLV = Type-Length-Value. This is how we attach optional/variable data to packets.
// Each TLV entry is: [type: 1 byte] [length: 1 byte] [value: 'length' bytes]
//
// Example: To send profile=ACTIVE (0x03):
//   Type=0x01, Length=0x01, Value=0x03  → 3 bytes total
//
// TLVs are appended after the 36-byte header and before the 2-byte CRC.
// The total TLV section length is stored at header byte 34.
enum bp_tlv_type_t : uint8_t
{
    TLV_PROFILE = 0x01,        // u8  — Current operating profile (bp_profile_t). Sent in most packets.
    TLV_TX_POWER = 0x02,       // i8  — Current LoRa TX power in dBm (signed, can be 2-22)
    TLV_SLEEP_INTERVAL = 0x03, // u16 — Current sleep duration between cycles in seconds
    TLV_GPS_WARM = 0x04,       // u8  — GPS warm start state: 0=cold (no prior fix), 1=warm (has prior fix)
    TLV_HOME_CYCLES = 0x05,    // u8  — How many consecutive BLE home detections in a row
    TLV_LOG_INFO = 0x06,       // u16+u16 — Log entries count + log size in KB (4 value bytes total)
    TLV_LOST_MODE_S = 0x08,    // u32 — Seconds elapsed since lost mode was activated
    TLV_NEW_MODE = 0x09,       // u8  — bp_profile_t of the mode we're reverting to (used in timeout alerts)
    TLV_DURATION_S = 0x0A,     // u32 — Total duration in seconds (used in timeout alerts)
    TLV_CMD_MSG_ID = 0x0B,     // u32 — The msg_seq_id of the command this packet is acknowledging
};

// ═══════════════════════════════════════════════
// Device Registry — Special device IDs
// ═══════════════════════════════════════════════
#define DEVICE_ID_BASE 0x0000      // Reserved ID for the base station (receiver)
#define DEVICE_ID_BROADCAST 0xFFFF // Send to all collars at once (not collar-specific)

// Static buffer for formatting device name strings like "Device_0001".
// Must be static so the returned pointer from getDeviceName() stays valid
// after the function returns.
static char _bp_dev_name_buf[16];

// Convert a device ID number to a human-readable name string.
// Base station (0x0000) → "BaseStation"
// Any collar → "Device_XXXX" where XXXX is the hex ID (e.g., Device_0001)
static inline const char *getDeviceName(uint16_t id)
{
    if (id == DEVICE_ID_BASE)
        return "BaseStation";
    snprintf(_bp_dev_name_buf, sizeof(_bp_dev_name_buf), "Device_%04X", id);
    return _bp_dev_name_buf;
}

// Reverse lookup: convert a device name string back to its numeric ID.
// "BaseStation" → 0x0000
// "broadcast" → 0xFFFF
// "Device_0001" → 0x0001
// Returns 0 if the name can't be parsed (0 = base station, so be careful).
static inline uint16_t getDeviceIdByName(const char *name)
{
    if (strcmp(name, "BaseStation") == 0)
        return DEVICE_ID_BASE;
    if (strcmp(name, "broadcast") == 0)
        return DEVICE_ID_BROADCAST;
    // Parse "Device_XXXX" format — extract the hex digits after "Device_"
    if (strncmp(name, "Device_", 7) == 0)
    {
        unsigned int id = 0;
        if (sscanf(name + 7, "%x", &id) == 1 && id > 0 && id <= 0xFFFE)
            return (uint16_t)id;
    }
    return 0; // Invalid name — couldn't parse it
}

// ═══════════════════════════════════════════════
// Profile Helpers — Convert between profile enum and string names
// ═══════════════════════════════════════════════

// Convert a profile name string (e.g., "lost") to its enum value (e.g., PROFILE_LOST).
// Returns PROFILE_UNKNOWN if the name doesn't match any known profile.
static inline bp_profile_t profileFromName(const char *name)
{
    if (strcmp(name, "normal") == 0)
        return PROFILE_NORMAL;
    if (strcmp(name, "powersave") == 0)
        return PROFILE_POWERSAVE;
    if (strcmp(name, "active") == 0)
        return PROFILE_ACTIVE;
    if (strcmp(name, "lost") == 0)
        return PROFILE_LOST;
    return PROFILE_UNKNOWN;
}

// Convert a profile enum value back to its string name.
// Used for serial debug output and display purposes.
static inline const char *profileToName(bp_profile_t p)
{
    switch (p)
    {
    case PROFILE_NORMAL:
        return "normal";
    case PROFILE_POWERSAVE:
        return "powersave";
    case PROFILE_ACTIVE:
        return "active";
    case PROFILE_LOST:
        return "lost";
    default:
        return "unknown";
    }
}

// Convert a status enum to a short display string for the base station UI.
// Maps device states to simple words the user can understand at a glance.
static inline const char *statusToDisplayString(bp_status_t s)
{
    switch (s)
    {
    case STATUS_OUT_AND_ABOUT:
        return "Out";    // Pet is outside
    case STATUS_BLE_HOME:
        return "Home";   // Pet is at home
    case STATUS_INVALID_GPS:
        return "Error";  // GPS couldn't lock
    case STATUS_OK:
        return "Out";    // General OK (used in responses)
    case STATUS_LOST_TIMEOUT:
        return "Error";  // Lost mode expired
    default:
        return "Error";  // Unknown = treat as error
    }
}

// ═══════════════════════════════════════════════
// CRC-16/CCITT-FALSE — Packet integrity check
// ═══════════════════════════════════════════════
// Every packet has a 2-byte CRC appended at the end.
// The sender computes the CRC over the header+TLV, appends it.
// The receiver recomputes the CRC and compares. If they don't match,
// the packet was corrupted in transit and gets dropped.
//
// Algorithm: CRC-16/CCITT-FALSE
// Polynomial: 0x1021
// Initial value: 0xFFFF
// No final XOR
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF; // Start with all bits set
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8; // XOR next byte into high byte of CRC
        for (uint8_t j = 0; j < 8; j++) // Process each bit
        {
            // If high bit is set, shift left and XOR with polynomial; otherwise just shift
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

// ═══════════════════════════════════════════════
// GPS Time to Unix Epoch Conversion
// ═══════════════════════════════════════════════
// GPS modules output time as year/month/day/hour/minute/second.
// We need a Unix timestamp (seconds since Jan 1, 1970) for the packet header.
// This function does the conversion without needing any time library.
static inline uint32_t gpsToUnixTime(uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t minute, uint8_t second)
{
    // Cumulative days at the start of each month (non-leap year)
    static const uint16_t mdays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

    uint32_t y = year;
    uint32_t days = (y - 1970) * 365; // Base days (365 per year since epoch)
    days += (y - 1969) / 4;           // Add leap year days (one every 4 years)
    days -= (y - 1901) / 100;         // Subtract century years (not leap years)
    days += (y - 1601) / 400;         // Add back 400-year leap years

    days += mdays[month - 1]; // Add days for completed months this year
    // If we're past February in a leap year, add one more day
    if (month > 2 && (y % 4 == 0) && ((y % 100 != 0) || (y % 400 == 0)))
        days++;

    days += day - 1; // Add days within the current month (day 1 = 0 extra days)
    // Convert everything to seconds: days→seconds + hours→seconds + minutes→seconds + seconds
    return days * 86400UL + hour * 3600UL + minute * 60UL + second;
}

// ═══════════════════════════════════════════════
// Packet Builders — Functions to construct outgoing packets
// ═══════════════════════════════════════════════

// Initialize a packet buffer with the common header fields.
// Zeroes the entire buffer first, then fills in:
//   - Protocol version at byte 0
//   - Device ID at bytes 1-2
//   - Message sequence at bytes 3-6
//   - Unix timestamp at bytes 7-10
//   - Status byte at byte 11
//   - Flags (packet type + boolean flags) at bytes 12-13
static inline void pkt_init(uint8_t *buf, uint16_t device_id,
                            uint32_t msg_seq, uint32_t time_unix,
                            uint8_t status, uint16_t flags)
{
    memset(buf, 0, BP_MAX_PACKET_SIZE); // Zero everything first
    buf[0] = BP_PROTOCOL_VERSION;       // Version byte (always 1)
    memcpy(&buf[1], &device_id, 2);     // Device ID (little-endian u16)
    memcpy(&buf[3], &msg_seq, 4);       // Message sequence number (u32)
    memcpy(&buf[7], &time_unix, 4);     // GPS-derived Unix timestamp (u32)
    buf[11] = status;                   // Status enum value (u8)
    memcpy(&buf[12], &flags, 2);        // Flags bitfield (u16)
}

// Set the GPS-related fields in the fixed header.
// These go at specific byte offsets within the 36-byte header.
//   - lat_e7/lon_e7: GPS coordinates scaled by 1e7 (e.g., 51.8737° → 518737000)
//   - dist_home_m: Distance to home location in meters
//   - bearing_deg: Compass bearing from current position to home (0-359°)
static inline void pkt_set_gps(uint8_t *buf, int32_t lat_e7, int32_t lon_e7,
                               uint16_t dist_home_m, uint16_t bearing_deg)
{
    memcpy(&buf[14], &lat_e7, 4);        // Latitude at bytes 14-17
    memcpy(&buf[18], &lon_e7, 4);        // Longitude at bytes 18-21
    memcpy(&buf[30], &dist_home_m, 2);   // Distance to home at bytes 30-31
    memcpy(&buf[32], &bearing_deg, 2);   // Bearing to home at bytes 32-33
}

// ── TLV Appenders ──
// These functions add optional data entries to the TLV section of the packet.
// The TLV section starts right after the 36-byte header.
// Byte 34 of the header tracks the current TLV section length.
//
// Each TLV entry is: [type: 1 byte] [length: 1 byte] [value: N bytes]
//
// All appenders check that adding the entry won't exceed BP_MAX_TLV_SIZE (28 bytes).
// Returns true on success, false if there's no room left.

// Append a 1-byte unsigned value as TLV. Total entry size: 3 bytes (type + len + val).
static inline bool pkt_add_tlv_u8(uint8_t *buf, uint8_t type, uint8_t val)
{
    uint8_t off = BP_HEADER_SIZE + buf[34]; // Calculate write offset (header + current TLV length)
    if (buf[34] + 3 > BP_MAX_TLV_SIZE)     // Check if 3 more bytes would overflow
        return false;
    buf[off] = type;       // TLV type ID
    buf[off + 1] = 1;     // Value length = 1 byte
    buf[off + 2] = val;   // The actual value
    buf[34] += 3;         // Update TLV section length
    return true;
}

// Append a 1-byte signed value. Same as u8 internally (just casts the sign away).
static inline bool pkt_add_tlv_i8(uint8_t *buf, uint8_t type, int8_t val)
{
    return pkt_add_tlv_u8(buf, type, (uint8_t)val);
}

// Append a 2-byte unsigned value as TLV. Total entry size: 4 bytes.
static inline bool pkt_add_tlv_u16(uint8_t *buf, uint8_t type, uint16_t val)
{
    uint8_t off = BP_HEADER_SIZE + buf[34]; // Write offset
    if (buf[34] + 4 > BP_MAX_TLV_SIZE)     // Need 4 bytes: type + len + 2-byte value
        return false;
    buf[off] = type;                   // TLV type ID
    buf[off + 1] = 2;                 // Value length = 2 bytes
    memcpy(&buf[off + 2], &val, 2);   // Copy value (little-endian)
    buf[34] += 4;                     // Update TLV section length
    return true;
}

// Append a 4-byte unsigned value as TLV. Total entry size: 6 bytes.
static inline bool pkt_add_tlv_u32(uint8_t *buf, uint8_t type, uint32_t val)
{
    uint8_t off = BP_HEADER_SIZE + buf[34];
    if (buf[34] + 6 > BP_MAX_TLV_SIZE)     // Need 6 bytes: type + len + 4-byte value
        return false;
    buf[off] = type;                   // TLV type ID
    buf[off + 1] = 4;                 // Value length = 4 bytes
    memcpy(&buf[off + 2], &val, 4);   // Copy value (little-endian)
    buf[34] += 6;                     // Update TLV section length
    return true;
}

// Append a combined log info TLV: two u16 values packed together.
// Used for sending log statistics (number of entries + total size in KB).
// Total entry size: 6 bytes (type + len + 2-byte entries + 2-byte size_kb).
static inline bool pkt_add_tlv_log_info(uint8_t *buf, uint16_t entries, uint16_t size_kb)
{
    uint8_t off = BP_HEADER_SIZE + buf[34];
    if (buf[34] + 6 > BP_MAX_TLV_SIZE)
        return false;
    buf[off] = TLV_LOG_INFO;              // Type = log info
    buf[off + 1] = 4;                     // Value length = 4 bytes (two u16s)
    memcpy(&buf[off + 2], &entries, 2);   // First u16: number of log entries
    memcpy(&buf[off + 4], &size_kb, 2);   // Second u16: log size in KB
    buf[34] += 6;
    return true;
}

// Finalize the packet: compute CRC over the header+TLV, append 2-byte CRC at the end.
// Returns the total packet length (header + TLV + CRC) — this is what you pass to LoRa transmit.
static inline uint8_t pkt_finalize(uint8_t *buf)
{
    uint8_t payload_len = BP_HEADER_SIZE + buf[34]; // Header + TLV data (no CRC yet)
    uint16_t crc = crc16_ccitt(buf, payload_len);   // Compute CRC over everything so far
    memcpy(&buf[payload_len], &crc, 2);             // Append CRC at the end
    return payload_len + BP_CRC_SIZE;               // Return total length
}

// ═══════════════════════════════════════════════
// Packet Parsers — Functions to decode incoming packets
// ═══════════════════════════════════════════════

// Validate that a received packet has a correct CRC.
// Call this FIRST on any received data before reading any fields.
// Returns true if the CRC matches (packet is intact), false if corrupted.
static inline bool pkt_validate_crc(const uint8_t *buf, uint8_t total_len)
{
    if (total_len < BP_MIN_PACKET_SIZE)  // Too short to even be a valid packet
        return false;
    uint8_t tlv_len = buf[34];           // Read TLV length from header
    uint8_t expected_len = BP_HEADER_SIZE + tlv_len + BP_CRC_SIZE;
    if (total_len < expected_len)        // Packet is shorter than it claims to be
        return false;

    uint8_t payload_len = BP_HEADER_SIZE + tlv_len;      // Everything before the CRC
    uint16_t computed = crc16_ccitt(buf, payload_len);    // Recompute CRC
    uint16_t received;
    memcpy(&received, &buf[payload_len], 2);              // Read the CRC that was sent
    return computed == received;                           // Do they match?
}

// ── Header Field Accessors ──
// Convenience functions to read specific fields from a received packet buffer.
// Each one reads from the correct byte offset and handles endianness via memcpy.
// These avoid direct struct casting which could break on different architectures.

static inline uint8_t pkt_version(const uint8_t *b) { return b[0]; } // Protocol version (byte 0)
static inline uint16_t pkt_device_id(const uint8_t *b)  // Who sent this packet (bytes 1-2)
{
    uint16_t v;
    memcpy(&v, &b[1], 2);
    return v;
}
static inline uint32_t pkt_msg_seq(const uint8_t *b) // Message sequence number (bytes 3-6)
{
    uint32_t v;
    memcpy(&v, &b[3], 4);
    return v;
}
static inline uint32_t pkt_time_unix(const uint8_t *b) // Unix timestamp (bytes 7-10)
{
    uint32_t v;
    memcpy(&v, &b[7], 4);
    return v;
}
static inline uint8_t pkt_status(const uint8_t *b) { return b[11]; } // Status byte (byte 11)
static inline uint16_t pkt_flags(const uint8_t *b) // Raw flags field (bytes 12-13)
{
    uint16_t v;
    memcpy(&v, &b[12], 2);
    return v;
}
static inline int32_t pkt_lat_e7(const uint8_t *b) // Latitude * 1e7 (bytes 14-17)
{
    int32_t v;
    memcpy(&v, &b[14], 4);
    return v;
}
static inline int32_t pkt_lon_e7(const uint8_t *b) // Longitude * 1e7 (bytes 18-21)
{
    int32_t v;
    memcpy(&v, &b[18], 4);
    return v;
}
static inline uint16_t pkt_batt_mV(const uint8_t *b) // Battery voltage in mV (bytes 22-23)
{
    uint16_t v;
    memcpy(&v, &b[22], 2);
    return v;
}
static inline uint16_t pkt_acc_m(const uint8_t *b) // GPS accuracy in meters (bytes 24-25)
{
    uint16_t v;
    memcpy(&v, &b[24], 2);
    return v;
}
static inline uint16_t pkt_fix_age_s(const uint8_t *b) // How old the GPS fix is in seconds (bytes 26-27)
{
    uint16_t v;
    memcpy(&v, &b[26], 2);
    return v;
}
static inline uint16_t pkt_speed_cms(const uint8_t *b) // Speed in cm/s (bytes 28-29)
{
    uint16_t v;
    memcpy(&v, &b[28], 2);
    return v;
}
static inline uint16_t pkt_dist_home_m(const uint8_t *b) // Distance to home in meters (bytes 30-31)
{
    uint16_t v;
    memcpy(&v, &b[30], 2);
    return v;
}
static inline uint16_t pkt_bearing_deg(const uint8_t *b) // Bearing to home in degrees (bytes 32-33)
{
    uint16_t v;
    memcpy(&v, &b[32], 2);
    return v;
}
static inline uint8_t pkt_tlv_len(const uint8_t *b) { return b[34]; }                      // TLV section length (byte 34)
static inline uint16_t pkt_pkt_type(const uint8_t *b) { return pkt_flags(b) & PKT_TYPE_MASK; } // Extract just the packet type from flags

// ── TLV Search ──
// Walk through the TLV section to find a specific TLV type.
// TLVs are stored sequentially: [type][length][value...][type][length][value...]...
// Returns true if found, with *value pointing to the value bytes and *vlen set to value length.
// Returns false if the type wasn't found in this packet.
static inline bool pkt_tlv_find(const uint8_t *buf, uint8_t tlv_type,
                                const uint8_t **value, uint8_t *vlen)
{
    uint8_t tlen = buf[34];                    // Total TLV section length
    uint8_t pos = 0;                           // Current position within TLV section
    const uint8_t *tlv = &buf[BP_HEADER_SIZE]; // Start of TLV section
    while (pos + 2 <= tlen)                    // Need at least 2 bytes for type+length
    {
        uint8_t t = tlv[pos];     // This entry's type
        uint8_t l = tlv[pos + 1]; // This entry's value length
        if (pos + 2 + l > tlen)
            break; // Value would extend past TLV section — malformed packet
        if (t == tlv_type) // Found the type we're looking for!
        {
            *value = &tlv[pos + 2]; // Point to the start of the value bytes
            *vlen = l;              // Tell caller how many value bytes there are
            return true;
        }
        pos += 2 + l; // Skip to next TLV entry (type + length + value)
    }
    return false; // Type not found in this packet
}

// ── Typed TLV Extractors ──
// Convenience wrappers around pkt_tlv_find() that extract and return typed values.
// Each one finds the TLV, checks the minimum length, and copies the value out.

// Extract a uint8_t TLV value. Returns false if not found or too short.
static inline bool pkt_tlv_get_u8(const uint8_t *buf, uint8_t type, uint8_t *out)
{
    const uint8_t *v;
    uint8_t l;
    if (!pkt_tlv_find(buf, type, &v, &l) || l < 1)
        return false;
    *out = v[0];
    return true;
}

// Extract an int8_t TLV value (signed byte). Same as u8 under the hood.
static inline bool pkt_tlv_get_i8(const uint8_t *buf, uint8_t type, int8_t *out)
{
    return pkt_tlv_get_u8(buf, type, (uint8_t *)out);
}

// Extract a uint16_t TLV value (2 bytes, little-endian).
static inline bool pkt_tlv_get_u16(const uint8_t *buf, uint8_t type, uint16_t *out)
{
    const uint8_t *v;
    uint8_t l;
    if (!pkt_tlv_find(buf, type, &v, &l) || l < 2)
        return false;
    memcpy(out, v, 2);
    return true;
}

// Extract a uint32_t TLV value (4 bytes, little-endian).
static inline bool pkt_tlv_get_u32(const uint8_t *buf, uint8_t type, uint32_t *out)
{
    const uint8_t *v;
    uint8_t l;
    if (!pkt_tlv_find(buf, type, &v, &l) || l < 4)
        return false;
    memcpy(out, v, 4);
    return true;
}

// Extract combined log info: two u16 values (entries count + size in KB).
static inline bool pkt_tlv_get_log_info(const uint8_t *buf, uint16_t *entries, uint16_t *size_kb)
{
    const uint8_t *v;
    uint8_t l;
    if (!pkt_tlv_find(buf, TLV_LOG_INFO, &v, &l) || l < 4)
        return false;
    memcpy(entries, v, 2);       // First 2 bytes = entry count
    memcpy(size_kb, v + 2, 2);  // Next 2 bytes = size in KB
    return true;
}

// ═══════════════════════════════════════════════
// Debug: Hex dump a packet to Serial
// ═══════════════════════════════════════════════
// Prints every byte of a packet in hexadecimal for debugging.
// Only compiled when building for Arduino (not for unit tests on desktop).
// Example output: [PKT] 42 bytes: 01 01 00 05 00 00 00 ...
#ifdef ARDUINO
#include <Arduino.h>
static inline void pkt_print_hex(const uint8_t *buf, uint8_t len)
{
    Serial.printf("[PKT] %d bytes: ", len);
    for (uint8_t i = 0; i < len; i++)
    {
        Serial.printf("%02X ", buf[i]); // Print each byte as 2-digit hex with space
    }
    Serial.println(); // Newline at the end
}
#endif

#endif // PROTOCOL_H
