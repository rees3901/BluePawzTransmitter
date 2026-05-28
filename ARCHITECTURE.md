# BluePawz V3 — System Architecture

Same document lives in both
[BluePawzReceiver](https://github.com/rees3901/BluePawzReceiver/blob/main/ARCHITECTURE.md)
and
[BluePawzTransmitter](https://github.com/rees3901/BluePawzTransmitter/blob/main/ARCHITECTURE.md).
When you change one, change the other.

---

## 1. The big picture

```
                    ┌──────────────────────────────────────┐
                    │        Heltec Wireless Tracker V2    │
                    │            (BluePawzReceiver)        │
                    │                                      │
                    │  ┌──────┐  ┌──────┐  ┌─────────────┐ │
                    │  │ LoRa │  │ WiFi │  │ BLE beacon  │ │
                    │  │ SX1262│ │ +UI  │  │ "Home" -12dBm│ │
                    │  └──┬───┘  └──┬───┘  └──────┬──────┘ │
                    └─────┼─────────┼─────────────┼────────┘
                          │         │             │
                LoRa 915MHz│   WiFi/HTTP/WS    BLE
                  SF8/250k │   (port 80, 81)  (short range)
                          │         │             │
                          │         ▼             │
                          │   ┌───────────┐       │
                          │   │  Browser  │       │
                          │   │  (map UI) │       │
                          │   └───────────┘       │
                          │                       │
                  ┌───────┴────────┐              │
                  ▼                ▼              │
            ┌─────────┐      ┌─────────┐          │
            │ Collar 1│ ...  │ Collar 5│ ◀────────┘ scanned by collars
            │ (XIAO   │      │ (XIAO   │           to detect "I'm home"
            │  ESP32S3)│     │  ESP32S3)│
            └─────────┘      └─────────┘
              ▲                 ▲
              │                 │
            GPS               GPS
        (NEO-6M / UC6580)
```

Five wearable collars send GPS positions over LoRa to one mains-powered
base station. The base station serves a Wi-Fi web UI showing all
positions on a map and lets the user push commands back to individual
collars. The base station also runs a low-power BLE beacon named
`"Home"` that the collars use to detect "I'm indoors, no need to TX".

---

## 2. LoRa link parameters (RX and TX must match exactly)

| Setting | Value |
|---|---|
| Frequency | 915.0 MHz (US) / 868.0 MHz (EU) — see `config.h` |
| Spreading factor | SF8 |
| Bandwidth | 250 kHz |
| Coding rate | 4/5 |
| Preamble | 16 symbols |
| Sync word | 0x12 (private network) |
| CRC | enabled |
| Listen-Before-Talk | enabled, with random backoff on collision |

These numbers are duplicated in **both** `config.h` files. Mismatch in
any of them = no link. Sync word in particular: 0x12 is a private
network; LoRaWAN public networks use 0x34.

---

## 3. Wire format (JSON)

Everything on the radio is human-readable JSON. We chose this over a
binary TLV protocol on purpose for the V3 rollout — debuggability and
flexibility were worth more than the airtime savings, given that even
five collars × ~12 packets/hour barely tickles the duty cycle.

The binary work is preserved on `wip/binary-migration` branches in
both repos for future revival if power becomes a constraint.

### 3.1 Telemetry (collar → base)

Three variants depending on the wake outcome.

**`outanabout`** — normal GPS fix:

```jsonc
{
  "msg_id":   42,            // monotonic per-collar (RTC-persisted)
  "device_id": 4,            // immutable numeric identity
  "id":       "Podge",       // current friendly name
  "status":   "outanabout",
  "mode":     "normal",      // current operating profile
  "lat":      51.873782,
  "lon":     -2.239428,
  "time":     "2026-05-28 14:31:02"   // optional, GPS time
}
```

**`BLEHome`** — BLE beacon detected, GPS not acquired:

```jsonc
{
  "msg_id":   43,
  "device_id": 4,
  "id":       "Podge",
  "status":   "BLEHome",
  "mode":     "normal"
  // no lat/lon — receiver shows the cat at the home marker
}
```

**`invalidGPSLoc`** — GPS timed out:

```jsonc
{
  "msg_id":   44,
  "device_id": 4,
  "id":       "Podge",
  "status":   "invalidGPSLoc",
  "mode":     "normal"
}
```

The receiver's JSON normaliser maps these to four UI states:
`Home` / `Out` / `Offline` / `Error`.

### 3.2 Commands (base → collar)

Targeting rule: a command with a `device` field that doesn't match the
collar's current name (or `"broadcast"`) is ignored. A command with a
`device_id` field that doesn't match the collar's immutable numeric id
is also ignored. Commands with neither field are accepted for
backward-compat but logged.

**Mode change**:

```jsonc
{ "cmd":"mode", "profile":"lost", "device":"Podge", "msg_id":42 }
```

**Status request** (collar responds with full state including BLE RSSI
threshold, lost-mode elapsed time, mode, etc.):

```jsonc
{ "cmd":"get_status", "device":"Podge", "msg_id":43 }
```

**Rename** — targets by immutable id because the current name may be
the default `Device-N`:

```jsonc
{ "cmd":"set_name", "device_id":4, "name":"Whiskers", "msg_id":44 }
```

### 3.3 ACKs (collar → base)

```jsonc
{ "ack":"mode",     "profile":"lost", "power":22, "sleep":30,
  "device":"Podge", "msg_id":42 }

{ "ack":"set_name", "ok":true,
  "device":"Whiskers", "id":"Whiskers", "device_id":4, "msg_id":44 }

// get_status response
{ "status":"ok", "device":"Podge", "mode":"lost", "power":22, "sleep":30,
  "msg_id":12345, "gps_warm":true, "home_cycles":0,
  "home_rssi_threshold":-65, "lost_mode_s":1832,
  "log":"127 entries, 42 KB" }
```

---

## 4. Downlink timing — Class-A LoRaWAN pattern

This is the most subtle bit of V3 and worth understanding deeply.

### The problem

Collars wake on their own schedule (every 1–20 minutes depending on
mode). The receiver doesn't know when. If the receiver pushes a
queued command "blindly" it almost certainly goes out while the target
collar is in deep sleep — packet lost forever.

### The fix

Two pieces, mirror images of each other:

**On the collar (transmitter `main.cpp`, `loop()`)**:

```cpp
// After EV_TXDONE, hold TaskLoRa alive for 5 s of RX before sleeping.
// Each EV_LORA_CMD received during the window extends the deadline
// by 3 s, so a burst of commands in one cycle all land.
xEventGroupClearBits(evBits, EV_LORA_CMD);  // ignore pre-TX commands

uint32_t deadline = millis() + POST_TX_LISTEN_MS;
while (millis() < deadline) {
    if (xEventGroupGetBits(evBits) & EV_LORA_CMD) {
        xEventGroupClearBits(evBits, EV_LORA_CMD);
        deadline = millis() + POST_TX_EXTEND_MS;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}
// then deep-sleep
```

**On the receiver (`handleLoRaPacketJSON`)**:

```cpp
// Telemetry just arrived → the collar is in its 5 s RX window NOW.
// Fire any matching command immediately, bypassing the 3 s safety gate.
String reporting = doc["id"].as<String>();
transmitCommandForDevice(reporting);
```

### Timing budget

```
Collar TX ends ────────────────────────────────► t = 0
Collar radio TX → RX                              t ≈ 5 ms
Collar RX window OPEN, 5000 ms budget             t ≈ 5 ms
Receiver decodes last symbol                      t ≈ 10 ms
Receiver runs JSON handler                        t ≈ 30-60 ms
Receiver lora.transmit() begins                   t ≈ 35-65 ms
                                                    ▲
                                          ~75× headroom inside the 5 s window
```

The receiver's processing is dominated by JSON parsing + WebSocket
broadcast + (rarely) a LittleFS flush. Even pessimistic worst case
(~200 ms) leaves >24× headroom.

### What about a busy receiver?

If the receiver was *already* transmitting a different command to a
different collar when the new telemetry arrived, the inbound packet
would be missed (radio in TX state). With 5 collars on 5-minute
cycles the odds of overlap are ~1 in a few thousand cycles. Not worth
mitigating for the V3 rollout. If we ever scale to 20 collars we'd
revisit.

---

## 5. Operating modes

Defined in `config.h` on both sides as an `OperatingMode` struct array.
Switching is via a mode command; the new mode is persisted to NVS on
the collar and survives a reset.

| Mode | TX dBm | Sleep | LED | Purpose |
|---|---|---|---|---|
| `normal` | 19 | 5 min | 5 flashes | Default |
| `powersave` | 10 | 20 min | 5 flashes | Cat is reliably indoors |
| `active` | 19 | 1 min | 5 flashes | Recent activity / actively watching |
| `lost` | 22 | 30 s | continuous beacon | Cat missing |

### Lost-mode auto-revert (the subtle bug we fixed)

The original code stored `g_lostModeStartTime = millis() / 1000` and
checked `millis()/1000 - g_lostModeStartTime >= 7200`. But **`millis()`
resets to 0 on every deep-sleep wake**, so the subtraction underflowed
or didn't accumulate — the 2 h timer never fired correctly.

V3 replaces the timestamp with an accumulator:

```cpp
RTC_DATA_ATTR uint32_t g_lostModeAccumS = 0;

// Before each deep_sleep_start (only when in lost mode):
g_lostModeAccumS += (millis() / 1000) + upcomingSleepS;

// At the top of each wake (in setup):
if (g_lostModeAccumS >= LOST_MODE_MAX_DURATION_S) {
    saveOperatingMode("active");
    g_lostModeAccumS = 0;
}
```

When the revert fires, the collar silently saves the new mode. **No
special alert packet** — the previous version sent one with no
`status` field, which the receiver's JSON normaliser tagged as
`"Error"` and the cat dropped off the map at the exact moment recovery
should have happened. Now the next routine telemetry packet carries
`mode: "active"` and the change shows up naturally.

---

## 6. BLE "Home" detection

The receiver advertises a non-connectable BLE beacon named `"Home"` at
**-12 dBm TX power** (intentionally weak, short range only). Each
collar runs an active BLE scan during its initial 10 s wake window and
during GPS acquisition.

A scan hit only counts as "home" if **all three** conditions match:

1. The advertised name equals `"Home"` (case-sensitive — pre-V3 the
   receiver advertised `"HOME"` and the collar checked `"Home"`, so
   home detection was silently broken).
2. The beacon includes an RSSI reading (`haveRSSI()` true).
3. RSSI ≥ `HOME_RSSI_THRESHOLD_DBM` (default `-65`).

The combination — beacon at -12 dBm, threshold at -65 dBm — keeps
"home" reliably confined to indoors. Walk-test to tune; the threshold
is reported in `get_status` for tuning visibility.

After `HOME_SLEEP_CYCLES` (default 5) consecutive home-detected wakes
the collar sends a `"BLEHome"` status packet. This is throttled because
otherwise an indoor cat would TX every wake interval for nothing.

---

## 7. State persistence map (transmitter)

The collar has three persistence tiers:

| Tier | Survives | Used for |
|---|---|---|
| `RTC_DATA_ATTR` (RTC memory) | deep sleep only — lost on full reset / USB unplug | `g_msgCounter`, `g_currentMode`, `g_lostModeAccumS`, `g_homeBeaconCycles`, `g_gpsWarmedUp` |
| `Preferences` (NVS, in flash) | everything except erase-flash | `g_senderName` (set via `set_name`), backup `msg_id` every 10 packets |
| `LittleFS` (in flash) | as NVS | `/track_log.csv` (3 MB capped, rotated) |

The receiver has two tiers:

| Tier | Survives | Used for |
|---|---|---|
| In-memory `std::map<String, NodeState>` | reboot wipes it | per-collar state for the UI |
| `LittleFS` | reboots | `/home_location.json`, `/messages.json` (circular log, 500 entries cap) |

---

## 8. Identity model

Every collar has:

- **`DEVICE_ID_INT`** — immutable numeric (1–255), set at flash time,
  used for command targeting. Think of it as the MAC address.
- **`g_senderName`** — friendly label (`"Podge"`, `"Macy"`, `"Device-4"`),
  lives in NVS, changes any time via `set_name`. Pure UX, no
  load-bearing role in the protocol.

Commands target by **name** for `mode` / `get_status` and by
**device_id** for `set_name` (because the name may be unknown/default).
Telemetry always carries both. The receiver's `NodeState` tracks both
and detects renames by spotting "same device_id, different name" and
dropping the stale name's NodeState entry.

---

## 9. Where things compile to / live

```
TRANSMITTER REPO            RECEIVER REPO
─────────────────────       ───────────────────────────
include/config.h    ◄─────► include/config.h    (must match)
include/protocol.h    ←     include/protocol.h  (both parked on
   (deleted on V3)              (V3 #if 0)       wip/binary-migration)
src/main.cpp                src/main.cpp
                            data/
                              index.html        (web UI)
                              Leaflet2_minimal.js
                              icons/
```

The `config.h` files are **expected to be byte-identical** at the LoRa
parameters section. If they diverge by even one value (e.g. sync word),
the radio link silently fails. Worth a script-based sync check in a
future cleanup.

---

## 10. The "binary TLV" branch — what & why

A binary TLV protocol was prototyped in `protocol.h` (parked in both
repos behind `#if 0` and on the `wip/binary-migration` branches). It
would have shrunk telemetry packets from ~150 bytes JSON to ~50 bytes
binary, with CRC-16 and TLV-encoded fields.

We chose JSON for V3 because:

- 5 collars × ~12 packets/hour × 150 bytes is trivial airtime
- JSON is debuggable from any serial monitor
- The binary protocol introduced subtle interop bugs (CRC mismatches,
  enum mismatches between receiver and transmitter)
- We'd rather get reliable real-world data first and optimise later

The binary work is preserved verbatim on the WIP branches and can be
revived if/when one of these changes:

- Scaling to 20+ collars puts pressure on duty cycle
- Battery life becomes the dominant constraint and we need to cut TX
  airtime by 60%
- We want to use payload encryption (TLV is friendlier to libsodium
  than JSON-in-the-clear)

---

## 11. Versioning

Semantic versioning, `MAJOR.MINOR.PATCH`.

| Level | Bump when… | Examples |
|---|---|---|
| **MAJOR** | the wire format, hardware target, or anything that breaks compatibility between receiver and any collar in the fleet | V3 itself (JSON protocol + Heltec V2); a future V4 if we move to binary or encrypt the link |
| **MINOR** | adding a new user-visible feature, backwards-compatible | adding battery telemetry, geofence alerts, breadcrumb persistence |
| **PATCH** | bug fixes, polish, refactors with no functional change | the BLE name case fix, the lost-mode accumulator fix |

Versions are owned per-repo:

- **Receiver** version lives in
  [`BluePawzReceiver/include/version.h`](https://github.com/rees3901/BluePawzReceiver/blob/main/include/version.h)
  as `#define BLUEPAWZ_VERSION`. Surfaced on the TFT, on the web UI
  title, and at `GET /version`.
- **Transmitter** version is currently uncoupled from the receiver's —
  collars don't yet send their version in telemetry. (Adding a
  `"fw":"3.0.0"` field to the telemetry JSON is a likely MINOR bump.)

The two MAJORs should always agree (a V3 receiver must only talk to V3
collars). MINOR / PATCH can drift between halves — receivers are usually
ahead because we OTA them more often.

### When to bump in practice

- Commits that don't change behaviour (typos, comments, doc) — no bump.
- Commits that change behaviour but fix something — bump PATCH.
- Commits that expose new functionality to the user — bump MINOR.
- Commits that break compatibility with a deployed collar — bump MAJOR
  and write a migration plan.

## 12. Glossary

- **LBT** — Listen Before Talk. The radio scans the channel for an
  in-progress transmission before keying up. Random backoff on collision.
- **NVS** — ESP32 Non-Volatile Storage (key-value KV in flash).
  Accessed via `Preferences`.
- **RTC memory** — fast SRAM that survives deep sleep (but not reset).
  Accessed via `RTC_DATA_ATTR`.
- **Class A** — LoRaWAN device class where the device speaks first and
  opens an RX window after. The pattern V3 uses for downlink.
- **TLV** — Type-Length-Value. The binary encoding scheme used in the
  parked binary protocol.
- **Vext** — External voltage rail on Heltec boards, switches power
  to the GPS + TFT. Active LOW.
