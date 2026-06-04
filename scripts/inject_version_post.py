"""
PlatformIO POST-build hook (transmitter). Counterpart to the pre script
prune_radiolib.py, which resolves the live git short-hash and stashes it on
env as TX_FIRMWARE_VERSION. Here we append `-DFIRMWARE_VERSION="<hash>"` to
PROJENV only, so the collar's telemetry `fw` field reports the ACTUAL flashed
commit instead of a frozen, hand-edited string.

Why projenv (not the global env):
   projenv is PIO's project-sources build env (src/*.cpp). Scoping the macro
   here means only main.cpp.o picks it up; the framework + library .o caches
   keep their recorded compile commands, so flipping the hash on each commit
   does NOT invalidate them. Appending to the GLOBAL env instead embeds the
   hash in every .o's compile command and turns every "upload" into a 15-20
   min full rebuild (the exact mistake the receiver's split fixed). projenv is
   only exposed to post scripts — that's why this is a post hook.

main.cpp keeps `#ifndef FIRMWARE_VERSION #define FIRMWARE_VERSION "unknown"`
as a fallback for IDE/intellisense builds that don't run this hook.
"""

Import("env", "projenv")  # noqa: F821

version = env.get("TX_FIRMWARE_VERSION", "unknown")  # noqa: F821
projenv.Append(  # noqa: F821
    CPPDEFINES=[("FIRMWARE_VERSION", projenv.StringifyMacro(version))]  # noqa: F821
)
print(f"[inject_version_post] applied FIRMWARE_VERSION={version} to projenv")
