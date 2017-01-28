#!/bin/bash

# Check arguments
if [ "$#" -ne 1 ]; then
	echo "Usage: flash.sh [on/off]"
fi

# Get arguement
SWITCH=$1

pushd MSPFLASHER_1.3.11

if [ "$SWITCH" = "on" ]; then
	./MSP430Flasher -w ../GCC/proj/LOS/g2553/hex_files/blink.hex -v -z [VCC]
elif [ "$SWITCH" = "off" ]; then
	./MSP430Flasher -z [RESET]
fi

popd
