#!/usr/bin/env sh

# If you're not using platformio to build, you can set 'verbose output' 
# in the Arduino IDE to find where the .elf is output.
platformio run
avr-objdump -m avr2 -S .pioenvs/zeroUSB/firmware.elf > Mechaduino.asm
