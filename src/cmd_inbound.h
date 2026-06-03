#pragma once
// ─────────────────────────────────────────────────────────────────────────
// Shared, hardware-independent inbound-command core for the COLLAR.
//
// This is the receive→parse→target→dispatch→apply→build-ACK logic that the
// firmware runs when a LoRa command arrives from the base station. It is the
// SAME translation unit (cmd_inbound.cpp) compiled BOTH by the ESP32 firmware
// (via handleModeCommand) AND by the native test harness (BluePawzSim/native),
// so the off-device tests exercise the exact code that flashes — no drift.
//
// It owns the commands most relevant to the rename/presence bug hunt:
//   • parse           — deserialize the JSON payload (ArduinoJson 7)
//   • targeting       — STRICT immutable-UID match (device_id) or broadcast
//   • "ping"          — build the pong reply
//   • "set_name"      — validate + apply (via name_store) + build the ACK
//
// mode / get_status / set_geofence are NOT owned here: bpHandleInbound returns
// BP_OTHER for them and the firmware dispatches them from its own inline code.
// (Those paths mutate a lot of collar-specific state — operating mode, geofence
//  — and are not under investigation for the rename/ping bug.)
//
// Dependencies are only ArduinoJson + INvs (name_store.h) — no Arduino.h, no
// Serial, no FreeRTOS — so it compiles unmodified on the host.
// ─────────────────────────────────────────────────────────────────────────
#include <stddef.h>
#include <stdint.h>
#include "name_store.h" // INvs, SENDER_NAME_MAX_LEN, bpSaveSenderName

// Broadcast UID: a command addressed here is acted on by every collar.
#ifndef BP_BROADCAST_ID
#define BP_BROADCAST_ID 65535
#endif

// Outcome of handing one raw LoRa payload to the inbound core.
enum BpInboundKind
{
  BP_PARSE_ERROR,  // JSON did not parse
  BP_NO_DEVICE_ID, // missing / non-integer device_id → rejected (UID required)
  BP_NOT_FOR_ME,   // device_id present but addressed to a different collar
  BP_NO_CMD,       // missing "cmd" field
  BP_PING,         // ping → pong written to ackOut
  BP_SET_NAME_OK,  // set_name applied → success ACK written to ackOut
  BP_SET_NAME_BAD, // set_name rejected (invalid name) → failure ACK in ackOut
  BP_OTHER         // parsed + targeted OK, but cmd is not owned here
                   // (mode / get_status / set_geofence — firmware handles it)
};

// Everything the core needs from the live collar to handle a command.
struct BpInboundCtx
{
  int myDeviceId;         // DEVICE_ID_INT — this collar's immutable UID
  char *nameBuf;          // live g_senderName (mutated on successful set_name)
  size_t nameBufSz;       // sizeof(g_senderName)
  INvs *nvs;              // persistence backend (PrefsNvs device / MockNvs test)
  int16_t rxRssi;         // RSSI of the inbound packet (echoed in pong)
  float rxSnr;            // SNR of the inbound packet (echoed in pong)
  unsigned long uptimeMs; // millis() at receipt (echoed in pong)
};

struct BpInboundResult
{
  BpInboundKind kind;
  int targetId;     // parsed device_id (valid for NOT_FOR_ME/PING/SET_NAME/OTHER)
  bool nameChanged; // true only when SET_NAME_OK actually changed the stored name
};

// Parse + target + dispatch one inbound command payload. For ping/set_name the
// reply JSON is written NUL-terminated into ackOut (empty string otherwise).
// For BP_OTHER the caller dispatches mode/get_status/set_geofence itself.
BpInboundResult bpHandleInbound(const char *json, const BpInboundCtx &ctx,
                                char *ackOut, size_t ackOutSz);
