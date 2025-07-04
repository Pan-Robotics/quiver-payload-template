#!/bin/bash

# Append CAN configuration to /boot/firmware/config.txt
echo "Configuring CAN interfaces..."
sudo tee -a /boot/firmware/config.txt <<EOF
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
dtoverlay=spi-bcm2835-overlay
EOF

# Reboot to apply changes
echo "Rebooting to apply CAN configuration..."
sudo reboot