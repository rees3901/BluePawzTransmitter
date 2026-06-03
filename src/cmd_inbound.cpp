// ─────────────────────────────────────────────────────────────────────────
// Shared inbound-command core (collar side). See cmd_inbound.h.
//
// The logic below is a faithful lift of the firmware's handleModeCommand()
// parse → targeting → ping → set_name branches. It is intentionally written
// against only ArduinoJson + INvs so the native harness can compile and run
// the IDENTICAL code that flashes to the ESP32.
// ─────────────────────────────────────────────────────────────────────────
#include "cmd_inbound.h"
#include <ArduinoJson.h>

BpInboundResult bpHandleInbound(const char *json, const BpInboundCtx &ctx,
                                char *ackOut, size_t ackOutSz)
{
  BpInboundResult r{};
  r.kind = BP_PARSE_ERROR;
  r.targetId = -1;
  r.nameChanged = false;
  if (ackOut && ackOutSz)
    ackOut[0] = '\0';

  // ── Parse ──────────────────────────────────────────────────────────────
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    r.kind = BP_PARSE_ERROR;
    return r;
  }

  // ── Targeting (immutable-UID) ────────────────────────────────────────────
  // Commands are addressed STRICTLY by the numeric device_id (UID). Accepted:
  //   "device_id":<myDeviceId>  -> only this collar acts
  //   "device_id":65535         -> broadcast: every collar acts
  // A command with no integer device_id is rejected (no legacy name routing).
  if (!doc["device_id"].is<int>())
  {
    r.kind = BP_NO_DEVICE_ID;
    return r;
  }
  {
    int tgt = doc["device_id"].as<int>();
    r.targetId = tgt;
    if (tgt != ctx.myDeviceId && tgt != (int)BP_BROADCAST_ID)
    {
      r.kind = BP_NOT_FOR_ME;
      return r;
    }
  }

  // ── Command dispatch ─────────────────────────────────────────────────────
  const char *cmd = doc["cmd"];
  if (!cmd)
  {
    r.kind = BP_NO_CMD;
    return r;
  }

  // Lightweight presence check — minimal packet, fast response.
  if (strcmp(cmd, "ping") == 0)
  {
    JsonDocument pong;
    pong["pong"] = true;
    pong["device_id"] = ctx.myDeviceId;          // UID (identity)
    pong["name"] = (const char *)ctx.nameBuf;     // editable label
    pong["rssi"] = ctx.rxRssi;
    pong["snr"] = ctx.rxSnr;
    pong["uptime_ms"] = ctx.uptimeMs;
    if (doc["msg_id"].is<uint32_t>())
      pong["msg_id"] = doc["msg_id"].as<uint32_t>();
    serializeJson(pong, ackOut, ackOutSz);
    r.kind = BP_PING;
    return r;
  }

  // Rename the collar. Wire format:
  //   {"cmd":"set_name","device_id":N,"name":"Podge","msg_id":N}
  // device_id is REQUIRED and must match THIS collar EXACTLY (no broadcast
  // rename) so a missing/loose field can't rename every collar at once.
  if (strcmp(cmd, "set_name") == 0)
  {
    if (!doc["device_id"].is<int>() || doc["device_id"].as<int>() != ctx.myDeviceId)
    {
      // Addressed by broadcast or to another collar — refuse silently (no ACK),
      // exactly as the firmware did ("set_name without matching device_id").
      r.kind = BP_NOT_FOR_ME;
      return r;
    }
    if (!doc["name"].is<const char *>())
    {
      // Missing/non-string name: refuse, no ACK (mirrors firmware return false).
      r.kind = BP_SET_NAME_BAD;
      return r;
    }
    const char *newName = doc["name"];
    if (!bpSaveSenderName(newName, ctx.nameBuf, ctx.nameBufSz, *ctx.nvs))
    {
      // Invalid name (length / forbidden chars). ACK with ok:false so the UI
      // sees a definitive response and the stored name stays unchanged.
      JsonDocument ack;
      ack["ack"] = "set_name";
      ack["ok"] = false;
      ack["device_id"] = ctx.myDeviceId;
      ack["name"] = (const char *)ctx.nameBuf; // current (unchanged) label
      if (doc["msg_id"].is<uint32_t>())
        ack["msg_id"] = doc["msg_id"].as<uint32_t>();
      serializeJson(ack, ackOut, ackOutSz);
      r.kind = BP_SET_NAME_BAD;
      return r;
    }

    // Applied. ACK with the new name so the receiver UI can confirm at once.
    r.nameChanged = true;
    JsonDocument ack;
    ack["ack"] = "set_name";
    ack["ok"] = true;
    ack["device_id"] = ctx.myDeviceId;        // UID (identity)
    ack["name"] = (const char *)ctx.nameBuf;   // the new editable label
    if (doc["msg_id"].is<uint32_t>())
      ack["msg_id"] = doc["msg_id"].as<uint32_t>();
    serializeJson(ack, ackOut, ackOutSz);
    r.kind = BP_SET_NAME_OK;
    return r;
  }

  // mode / get_status / set_geofence are dispatched by the firmware itself.
  r.kind = BP_OTHER;
  return r;
}
