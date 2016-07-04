#!/usr/bin/env sh

set -eu

# Uncomment this line if you're missing the board definitions. 
# arduino --install-boards "arduino:samd" || true

# Build, and generate asm file.
arduino \
    --board arduino:samd:arduino_zero_native \
    --verify Mechaduino/Mechaduino.ino \
    --preserve-temp-files \
    --verbose \
    | grep -o "\"/tmp/.*\.elf\"" \
    | head -n1 \
    | xargs -i mv {} `pwd`

avr-objdump -m avr2 -S Mechaduino.ino.elf > Mechaduino.asm
