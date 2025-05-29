#!/bin/bash

# Append CAN configuration to /boot/firmware/config.txt
echo "Configuring CAN interfaces..."
sudo tee -a /boot/firmware/config.txt <<EOF
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=25
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
dtoverlay=spi-bcm2835-overlay
EOF

# Reboot to apply changes
echo "Rebooting to apply CAN configuration..."
sudo reboot

# Note: Run the following commands manually after reboot
# sudo ip link set can0 up type can bitrate 500000
# sudo ip link set can1 up type can bitrate 250000
# sudo ifconfig can0 txqueuelen 65536
# sudo ifconfig can1 txqueuelen 65536
# dmesg | grep spi