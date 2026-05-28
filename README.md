# BluePawz Transmitter 🛰️

The collar half of the BluePawz cat tracker. Runs on a
**Seeeduino XIAO ESP32-S3** strapped to a cat with a:

- Semtech SX1262 LoRa radio,
- GPS module (NEO-6M tested; UC6580 also fine),
- BLE radio (built-in to the S3) for "am I at home?" detection,
- LiPo battery + deep sleep so it lasts more than a day.

This repo is the collar firmware. The base-station firmware lives at
[**BluePawzReceiver**](https://github.com/rees3901/BluePawzReceiver) —
the two need to be in sync.

For the system-level design (wire format, downlink timing, mode profiles)
see [`ARCHITECTURE.md`](./ARCHITECTURE.md).

---

## What's V3?

V3 is the JSON-only protocol generation, ready for first multi-cat rollout.
The repo briefly hosted a binary TLV migration — that work is preserved on
the [`wip/binary-migration`](https://github.com/rees3901/BluePawzTransmitter/tree/wip/binary-migration)
branch and not on `main`/`esp32s3RTOS`.

### Headline V3 features

- **Class-A LoRaWAN-style RX window** — after every telemetry TX, the
  collar holds the radio in RX for 5 s (extended 3 s per incoming
  command). The receiver sees the telemetry, knows the collar is
  awake right now, and pushes any queued command immediately.
- **Runtime-configurable collar name** — `g_senderName` lives in NVS
  and defaults to `"Device-N"` on first flash. The receiver's UI sends
  a `set_name` command targeted by immutable numeric `device_id` to
  rename without USB.
- **Device targeting** — the collar checks `device` (name) or
  `device_id` (number) on incoming commands and ignores anything not
  for it. Stops broadcast collisions with 5 collars listening at once.
- **Dynamic home location** lives at the receiver — collars no longer
  compute distance or carry hardcoded HOME_LAT/HOME_LON. They send raw
  lat/lon; the receiver does haversine.
- **Lost-mode 2 h auto-revert that actually works** — the old code
  stored a `millis()` timestamp which reset on every deep-sleep wake,
  so the 7200 s timer never fired correctly. V3 replaces it with an
  RTC-persisted accumulator (`g_lostModeAccumS`).
- **BLE home detection with RSSI threshold** — `HOME_RSSI_THRESHOLD_DBM`
  in `config.h` (default `-65`). A faint distant beacon no longer
  registers as "the cat is home"; only beacons strong enough to suggest
  the collar is genuinely indoors count.
- **Silent lost-mode revert** — when the 2 h timer fires the collar just
  saves `g_currentMode = "active"`. No special alert packet (the old one
  had no `status` field and corrupted the receiver's UI). The next
  routine telemetry packet carries `mode: "active"` and the change shows
  up naturally.

---

## Hardware

| Component | Detail |
|---|---|
| Board | Seeeduino XIAO ESP32-S3 |
| MCU | ESP32-S3 (8 MB flash, PSRAM) |
| LoRa radio | Semtech SX1262 module (B2B-mounted) |
| GPS | NEO-6M or compatible NMEA, UART1 @ 9600 |
| BLE | ESP32-S3 onboard (scanner only) |
| Power | LiPo battery, deep-sleep cycles |

### Pin map

```
SX1262 LoRa (HSPI):
  NSS  = 41    SCK  = 7     MOSI = 9     MISO = 8
  RST  = 42    BUSY = 40    DIO1 = 39

GPS (UART1 @9600):
  RX  = D7     TX  = D6
  EN  = D2/GPIO 1  (HIGH = GPS power on)

LED (status indicator)   = GPIO 48
Debug serial (UART2 TX)  = D5 / GPIO 6   (for monitoring on battery)
```

### LoRa radio settings (must match the receiver)

| Setting | Value |
|---|---|
| Frequency | 915.0 MHz (US) / 868.0 MHz (EU) — set in `config.h` |
| Spreading factor | SF8 |
| Bandwidth | 250 kHz |
| Coding rate | 4/5 |
| Preamble | 16 symbols |
| Sync word | 0x12 (private network) |
| LBT | enabled with random backoff |

---

## Operating modes

Four profiles defined in `config.h`. Switched remotely via the receiver's
web UI — no USB needed.

| Mode | TX power | Sleep interval | LED behaviour | Typical use |
|---|---|---|---|---|
| `normal` | 19 dBm | 5 min | 5 flashes per wake | Default daily tracking |
| `powersave` | 10 dBm | 20 min | 5 flashes | Cat is reliably indoors |
| `active` | 19 dBm | 1 min | 5 flashes | Recent activity / actively watching |
| `lost` | 22 dBm (max) | 30 s | continuous beacon | Emergency: cat missing |

`lost` mode auto-reverts to `active` after **`LOST_MODE_MAX_DURATION_S`**
(default 7200 s = 2 h) to protect the battery from a forgotten command.

---

## Wake cycle (the "Power" task)

```
                    ┌─────────────────┐
                    │  Deep sleep     │
                    │  (RTC timer)    │
                    └────────┬────────┘
                             ▼
                       WAKE → setup()
                             ▼
        ┌─────────────────────────────────┐
        │  Phase 1: BLE + LoRa RX (10 s)  │ ← scans for "Home" beacon
        │                                 │   and inbound LoRa commands
        └────────────┬────────────────────┘
                     │
              ┌──────┴──────┐
              ▼             ▼
        Home found?    No home → enable GPS
              │             │
              │             ▼
              │   Acquire fix (cold ≤60 s, warm ≤20 s)
              │             │
              ▼             ▼
        ┌──────────────────────────┐
        │  Build JSON telemetry    │
        │  (id, lat, lon, status,  │
        │   mode, msg_id, …)       │
        └────────────┬─────────────┘
                     ▼
              LoRa TX (LBT-gated)
                     ▼
        ┌──────────────────────────┐
        │  POST-TX RX window: 5 s  │ ← receiver pushes any queued
        │  (+3 s per command)      │   command into this window
        └────────────┬─────────────┘
                     ▼
                Deep sleep
```

---

## JSON wire formats

### Telemetry (collar → base, every wake)

```jsonc
{
  "msg_id":   42,            // monotonic per-collar (RTC-persisted)
  "device_id": 4,            // immutable numeric identity (never changes)
  "id":       "Podge",       // runtime name (NVS, set via set_name)
  "status":   "outanabout",  // or "BLEHome" / "invalidGPSLoc"
  "mode":     "normal",      // current operating profile
  "lat":      51.873782,     // raw fix (receiver does haversine)
  "lon":     -2.239428,
  "time":     "2026-05-28 14:31:02"   // optional, when GPS time valid
}
```

### Mode command (base → collar)

```jsonc
{ "cmd":"mode", "profile":"lost", "device":"Podge", "msg_id":42 }
```

### Status request (base → collar)

```jsonc
{ "cmd":"get_status", "device":"Podge", "msg_id":43 }
```

### Rename (base → collar) — targets by immutable id

```jsonc
{ "cmd":"set_name", "device_id":4, "name":"Whiskers", "msg_id":44 }
```

### ACKs (collar → base)

```jsonc
{ "ack":"mode",     "profile":"lost", "power":22, "sleep":30,
  "device":"Podge", "msg_id":42 }

{ "ack":"set_name", "ok":true,
  "device":"Whiskers", "id":"Whiskers", "device_id":4, "msg_id":44 }
```

---

## Building & flashing

### Per-collar configuration

The only thing hardcoded per flash is the **immutable numeric ID** in
`src/main.cpp`:

```cpp
#define DEVICE_ID_INT 4    // unique per flash — never changes remotely
```

Set it to 1, 2, 3, 4, 5 (or whatever) before flashing each collar.
The friendly name is **not** baked in any more — it lives in NVS and
defaults to `"Device-<DEVICE_ID_INT>"` on first boot.

### Compile & upload

```bash
cd BluePawzTransmitter
pio run -e seeed_xiao_esp32s3 -t upload
pio device monitor -b 115200
```

Hold the **BOOT** button + tap **RESET** if PIO can't see the chip.

### Naming the collar after flashing

1. Power on the collar. It joins the receiver as `"Device-4"` (or whatever).
2. Open the receiver web UI → side panel → Command & Control.
3. Click the ✏️ next to the new collar → type the cat's name → save.
4. Within ~2 wake cycles the card relabels.

Configuration of the collar's name is now over-the-air. USB cable
only required for the very first flash.

---

## File / directory layout

```
BluePawzTransmitter/
├── README.md                  ← you are here
├── ARCHITECTURE.md            ← system-wide design notes
├── platformio.ini             ← build config (XIAO ESP32-S3 + libs)
├── partitions_8MB_bigfs.csv   ← partition table with big LittleFS
├── create_partition.py        ← helper to regenerate the partition CSV
├── include/
│   └── config.h               ← LoRa params, mode profiles, BLE knobs
├── src/
│   └── main.cpp               ← FreeRTOS tasks: GPS, BLE, LoRa, Power
├── lib/                       ← project-local libs (rarely used)
└── test/                      ← test scaffolding (mostly empty)
```

### State persistence map (what lives where)

| Lives in | Survives | Used for |
|---|---|---|
| `RTC_DATA_ATTR` (RTC memory) | deep sleep only — lost on reset | `g_msgCounter`, `g_currentMode`, `g_lostModeAccumS`, `g_homeBeaconCycles`, `g_gpsWarmedUp` |
| `Preferences` (NVS, flash) | full reset, USB unplug | `g_senderName`, persistent backup of `msg_id` (every 10 msgs) |
| `LittleFS` (flash) | full reset, USB unplug | `/track_log.csv` (3 MB cap) |

---

## Tuning the BLE home threshold

`config.h`:

```cpp
#define HOME_RSSI_THRESHOLD_DBM (-65)
```

How to set this:

1. Flash a collar and put it on a cat (or just carry it).
2. Walk around the house while watching the serial monitor.
3. Note RSSI values where you want the boundary (e.g. -55 dBm by the
   sofa, -80 dBm at the bottom of the garden).
4. Pick a value that's clearly above the garden reading and at-or-below
   the indoor reading.
5. Adjust `HOME_RSSI_THRESHOLD_DBM`, reflash. The threshold is also
   surfaced in `get_status` responses so you can verify without USB.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Collar dies in minutes | Battery health, or sleep_interval too short — check current mode |
| No GPS fix ever | Antenna shielded by collar housing, indoors, or GPS_EN not toggled HIGH |
| Receiver never sees this collar | LoRa params mismatch (freq, SF, BW, CR, sync word) — check `config.h` matches receiver |
| Mode command applied by every collar | Device targeting bypassed — make sure receiver sends `device` field |
| Lost mode never reverts | Was a real bug pre-V3. If still seeing it, the collar firmware is older than this branch |
| Status reported as "Home" outdoors | BLE TX power on receiver too high, or `HOME_RSSI_THRESHOLD_DBM` too low |

### Capturing logs without USB

The debug UART is wired to **D5 / GPIO 6** at 115200. Connect a USB-Serial
adapter's RX to that pin (GND too) and you can watch the collar on battery.

The collar also writes telemetry to `/track_log.csv` in LittleFS — capped
at 3 MB. Dump via `pio run -t uploadfs` (extract) or pull off the chip
with esptool's flash-read on partition 1.

---

## Related repositories

- 🏠 **Receiver (base-station) firmware**: https://github.com/rees3901/BluePawzReceiver
- 🌳 **Branch graveyard / WIP binary**: https://github.com/rees3901/BluePawzTransmitter/tree/wip/binary-migration

The receiver repo carries the same `config.h`. Keep the LoRa radio
parameters in lockstep across both repos.
