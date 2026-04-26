#!/bin/bash
modprobe can
modprobe can_raw

ip link set can0 down
ip link set can0 up type can bitrate 250000

echo "CAN interface configured"

chmod a+rw /dev/i2c-1
