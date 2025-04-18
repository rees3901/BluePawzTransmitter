Import("env")
import os

# Create a custom partition table for 8MB flash
# The total flash size is 8MB (0x800000 bytes)
partition_table = """# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x200000,
littlefs, data, spiffs,  0x210000,0x5F0000,
"""

# Write the partition table to a file
with open(os.path.join(env.subst("$PROJECT_DIR"), "custom_8MB.csv"), "w") as f:
    f.write(partition_table)

print("Created custom partition table for 8MB flash")