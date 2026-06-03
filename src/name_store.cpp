// ─────────────────────────────────────────────────────────────────────────
// name_store implementation — see name_store.h. Pure C++/C-stdlib, so it
// compiles unchanged on the ESP32 (xtensa) and on a host compiler in the
// native sandbox. This is the literal logic that was inlined in the
// transmitter's loadSenderName()/saveSenderName(); they now call here.
// ─────────────────────────────────────────────────────────────────────────
#include "name_store.h"
#include <string.h>
#include <stdio.h>

bool bpValidSenderName(const char *n) {
  if (!n) return false;
  size_t len = strnlen(n, SENDER_NAME_MAX_LEN + 2);
  if (len == 0 || len > SENDER_NAME_MAX_LEN) return false;
  for (size_t i = 0; i < len; i++) {
    unsigned char c = (unsigned char)n[i];
    if (c < 0x20 || c == ',' || c == '"' || c == '\\') return false;
  }
  return true;
}

void bpLoadSenderName(char *out, size_t outsz, int deviceId, INvs &nvs) {
  char stored[SENDER_NAME_MAX_LEN + 1] = {0};
  if (nvs.nvsGetString("name", stored, sizeof(stored))) {
    size_t len = strnlen(stored, SENDER_NAME_MAX_LEN + 1);
    if (len > 0 && len <= SENDER_NAME_MAX_LEN) {
      strncpy(out, stored, outsz - 1);
      out[outsz - 1] = '\0';
      return;
    }
  }
  // Empty / unset / oversized → derive the default.
  snprintf(out, outsz, "Device-%d", deviceId);
}

bool bpSaveSenderName(const char *newName, char *outCurrent, size_t outsz, INvs &nvs) {
  if (!bpValidSenderName(newName)) return false;
  if (!nvs.nvsPutString("name", newName)) return false;
  strncpy(outCurrent, newName, outsz - 1);
  outCurrent[outsz - 1] = '\0';
  return true;
}
