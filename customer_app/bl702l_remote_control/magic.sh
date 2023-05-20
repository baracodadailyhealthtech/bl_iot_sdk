#!/bin/bash

make CONFIG_BLUETOOTH=1 CONFIG_PDS_ENABLE=1 CONFIG_BTBLECONTROLLER_LIB=m1s1p -j
bflb-iot-tool --chipname=bl702l --xtal=32M --port=/dev/cu.usbmodem1411202 --baudrate=2000000 --firmware=build_out/bl702l_remote_control.bin 
python3 /Users/will/Development/Kaeretch/sensor-platform/tools/miniterm.py /dev/cu.usbmodem1411202 2000000
