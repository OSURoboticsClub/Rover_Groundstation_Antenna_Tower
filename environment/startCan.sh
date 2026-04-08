#!/bin/bash
modprobe can
modprobe can_raw

ip link set can0 down
ip link set can0 up type can bitrate 1000000

echo "CAN interface configured"
