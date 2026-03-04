"""
═══════════════════════════════════════════════════════════
Create Custom Partition Table — 8MB Flash (Alternative Layout)
═══════════════════════════════════════════════════════════

This PlatformIO build script generates an ALTERNATIVE partition table
(custom_8MB.csv) with a different layout than partitions_8MB_bigfs.csv:

  - Single app partition (2MB) instead of dual OTA partitions
  - Much larger LittleFS filesystem (5.9375 MB) for extensive data logging
  - No OTA update support (trade-off for more storage)

This script runs during the PlatformIO build process (via Import("env")).
It writes the partition CSV file to the project root directory.

NOTE: This generates custom_8MB.csv, but the project currently uses
partitions_8MB_bigfs.csv (set in platformio.ini). To use this layout
instead, change board_build.partitions in platformio.ini.

Layout:
  0x9000  - 0xDFFF  : NVS (20KB) — Non-Volatile Storage for settings
  0xE000  - 0xFFFF  : OTA data (8KB) — OTA partition tracking (kept for compatibility)
  0x10000 - 0x20FFFF : App partition (2MB) — firmware (single, no OTA swap)
  0x210000 - 0x7FFFFF : LittleFS (5.9375 MB) — large filesystem for data logging
"""

Import("env")  # PlatformIO build system hook — gives access to build environment variables
import os

# Define the partition table as a CSV string.
# Format: Name, Type, SubType, Offset, Size, Flags
# The offsets and sizes must not overlap and must fit within 8MB (0x800000).
partition_table = """# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x200000,
littlefs, data, spiffs,  0x210000,0x5F0000,
"""

# Write the partition table CSV file to the project root directory.
# env.subst("$PROJECT_DIR") resolves to the project's root path.
with open(os.path.join(env.subst("$PROJECT_DIR"), "custom_8MB.csv"), "w") as f:
    f.write(partition_table)

print("Created custom partition table for 8MB flash")
