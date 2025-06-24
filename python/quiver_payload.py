#!/usr/bin/env python3
"""Quiver UAS payload configuration module for Raspberry Pi."""

# GPIO pin definitions (BCM numbering)
PIN_ETHERNET_TX_PLUS = 17
PIN_ETHERNET_TX_MINUS = 18
PIN_ETHERNET_RX_PLUS = 27
PIN_ETHERNET_RX_MINUS = 22
PIN_CAN_H = 23
PIN_CAN_L = 24
PIN_ANALOG_IO = 25
PIN_DIGITAL_IO = 16
PIN_POWER_12V = 12
PIN_GROUND = 6

# Configuration constants
CAN_BAUDRATE = 1000000
ETHERNET_UDP_PORT = 14550
MAX_CURRENT_DRAW = 2.0