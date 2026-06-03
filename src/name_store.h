#pragma once
// ─────────────────────────────────────────────────────────────────────────
// name_store — the collar's friendly-name persistence logic, extracted as
// hardware-independent C++ so the EXACT SAME code runs on the ESP32 and in
// the native sandbox harness (BluePawzSim/native). No Arduino / RadioLib /
// Preferences includes here — persistence is reached through the INvs
// interface, which the firmware backs with Preferences (NVS) and the harness
// backs with an in-memory map that survives a simulated deep-sleep reset.
//
// This is the SOURCE OF TRUTH for load/save/validate of g_senderName. The
// firmware's loadSenderName()/saveSenderName() are now thin adapters around
// these functions, so a test of bpLoadSenderName/bpSaveSenderName is a test
// of the real flashed logic — including bug #8 (name reverting to default
// across a wake), which only manifests when the name is reloaded from NVS
// after the RAM copy is wiped by deep sleep.
// ─────────────────────────────────────────────────────────────────────────
#include <stddef.h>

#ifndef SENDER_NAME_MAX_LEN
#define SENDER_NAME_MAX_LEN 15 // 15 chars + NUL; matches the firmware
#endif

// Abstraction over non-volatile storage. On the ESP32 this is Preferences
// (NVS); in tests it's a std::map that persists across simulated resets.
struct INvs {
  // Read key into out (NUL-terminated, ≤ outsz). Returns true iff a
  // non-empty value existed.
  virtual bool nvsGetString(const char *key, char *out, size_t outsz) = 0;
  // Persist key=val. Returns true on success (commit included).
  virtual bool nvsPutString(const char *key, const char *val) = 0;
  virtual ~INvs() {}
};

// Validation — identical rules to the firmware's saveSenderName():
// 1..SENDER_NAME_MAX_LEN chars, no control chars / comma / quote / backslash.
bool bpValidSenderName(const char *name);

// Load the friendly name into `out`. If NVS has a valid stored name, use it;
// otherwise derive the default "Device-<deviceId>". Mirrors loadSenderName().
void bpLoadSenderName(char *out, size_t outsz, int deviceId, INvs &nvs);

// Validate + persist newName, and (on success) copy it into `outCurrent`.
// Returns false (and leaves outCurrent untouched) if invalid or the write
// failed. Mirrors saveSenderName().
bool bpSaveSenderName(const char *newName, char *outCurrent, size_t outsz, INvs &nvs);
