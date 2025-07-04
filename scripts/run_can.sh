#!/bin/bash

# Note: Run the following commands after reboot
sudo ip link set can0 up type can bitrate 500000
sudo ifconfig can0 txqueuelen 65536
dmesg | grep spi