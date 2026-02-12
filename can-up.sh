#!/bin/bash
sudo su -c 'ip link set can0 down && sleep 1 && ip link set can0 up type can bitrate 500000'
sleep 1
ip link show can0

echo "candump -ta -d -e -x can0"

