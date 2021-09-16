#!/bin/sh -xe

set -e

#arduino-cli lib install PinChangeInterrupt
arduino-cli compile --fqbn aceduino:avr:m168xt4m AceGrid
# arduino-cli upload -p /dev/ttyUSB0 --fqbn aceduino:avr:m168xt4m AceGrid
arduino-cli upload -P avrispmk2 --fqbn aceduino:avr:m168xt4m AceGrid
