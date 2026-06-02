"""
PlatformIO pre-build hook: prune unused RadioLib source files so PIO
doesn't compile every radio driver + digital-mode encoder in the
library every build.

Why this is needed (RADIOLIB_EXCLUDE_* isn't enough):
   RADIOLIB_EXCLUDE_* macros in platformio.ini make those modules'
   implementations EMPTY at preprocessing time, but PIO still walks
   every .cpp in the library and compiles each one to a near-empty .o
   file. That's why clean builds were showing dozens of
   `Compiling RadioLib/.../CC1101.cpp.o`, `LR11x0/...`, `APRS.cpp.o`
   despite the EXCLUDE flags being set.

Approach: physically rename the unused .cpp files to .cpp.skip in the
   installed RadioLib copy under .pio/libdeps/. PIO compiles only *.cpp;
   the .cpp.skip files are invisible to it. Idempotent (already-renamed
   = no-op), recovers if RadioLib is reinstalled. The .h files are NOT
   touched so any neighbouring code that #include's a skipped module's
   header still compiles cleanly (we just don't link an implementation).

Why not also use SCons SRC_FILTER?
   Tried lb.env.Replace(SRC_FILTER=...) on the receiver -- it broke the
   LibBuilder's compiler config (`'CC' is not recognized` errors). The
   rename approach is blunter but bulletproof.

Why no git-hash logic here?
   The transmitter's FIRMWARE_VERSION is a static -D build flag the
   user edits manually in platformio.ini. Changing it already triggers
   a full rebuild (platformio.ini changes always do), so we don't need
   the per-file CPPDEFINES gymnastics the receiver uses. Just the
   RadioLib prune is enough.
"""

from pathlib import Path

Import("env")  # noqa: F821  (provided by PlatformIO's SCons env)

PROJECT_DIR = Path(env["PROJECT_DIR"])  # noqa: F821

# Keep ONLY: modules/SX126x (our SX1262 family) + base infrastructure
# (Hal, Module, ArduinoHal -- those are at the lib's root, untouched).
# Verified against the RadioLib 6.6.0 layout in .pio/libdeps/.
RADIOLIB_SKIP_DIRS = [
    # Other radio driver families
    "modules/CC1101", "modules/LLCC68", "modules/LR11x0",
    "modules/RF69", "modules/RFM2x", "modules/SX123x",
    "modules/SX127x", "modules/SX128x", "modules/Si443x",
    "modules/nRF24",
    # Digital-mode encoders we never use. Note: in 6.6.0 APRS's HEADER
    # depends on AX25 (and AX25 on AFSK), but renaming the .cpp does NOT
    # remove the header -- it only removes the implementation .o from
    # the link. Headers remain parseable for any code that #include's
    # RadioLib.h. Since main.cpp never instantiates these classes, the
    # missing implementations are harmless.
    "protocols/ADSB", "protocols/AFSK", "protocols/AX25",
    "protocols/APRS", "protocols/BellModem", "protocols/ExternalRadio",
    "protocols/FSK4", "protocols/Hellschreiber", "protocols/LoRaWAN",
    "protocols/Morse", "protocols/Pager", "protocols/RTTY",
    "protocols/SSTV",
]

pioenv = env["PIOENV"]  # noqa: F821  e.g. seeed_xiao_esp32s3
radiolib_src = PROJECT_DIR / ".pio" / "libdeps" / pioenv / "RadioLib" / "src"

renamed_count = 0
if radiolib_src.is_dir():
    for sub in RADIOLIB_SKIP_DIRS:
        sub_path = radiolib_src / sub
        if not sub_path.is_dir():
            continue
        for cpp in sub_path.rglob("*.cpp"):
            cpp.rename(cpp.with_suffix(".cpp.skip"))
            renamed_count += 1
    if renamed_count:
        print(f"[prune_radiolib] renamed {renamed_count} unused .cpp -> .cpp.skip")
    else:
        print("[prune_radiolib] RadioLib already pruned (nothing to rename)")
else:
    # Lib not installed yet; PIO will fetch on first build, this script
    # runs again next time and applies the rename. Non-fatal.
    print(f"[prune_radiolib] RadioLib not installed yet at {radiolib_src} -- skip applied on next build")
