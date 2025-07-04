#ifndef QUIVER_PAYLOAD_H
#define QUIVER_PAYLOAD_H

#ifdef __arm__ // Raspberry Pi
#define PIN_ETHERNET_TX_PLUS  17
#define PIN_ETHERNET_TX_MINUS 18
#define PIN_ETHERNET_RX_PLUS  27
#define PIN_ETHERNET_RX_MINUS 22
#define PIN_CAN_H             23
#define PIN_CAN_L             24
#define PIN_ANALOG_IO         25
#define PIN_DIGITAL_IO        16
#define PIN_POWER_12V         12
#define PIN_GROUND            6
#else // ESP32, STM32
#define PIN_ETHERNET_TX_PLUS  17
#define PIN_ETHERNET_TX_MINUS 18
#define PIN_ETHERNET_RX_PLUS  19
#define PIN_ETHERNET_RX_MINUS 23
#define PIN_CAN_H             5
#define PIN_CAN_L             4
#define PIN_ANALOG_IO         34
#define PIN_DIGITAL_IO        15
#define PIN_POWER_12V         12
#define PIN_GROUND            0
#endif

// Configuration constants
#define CAN_BAUDRATE          1000000
#define ETHERNET_UDP_PORT     14550
#define MAX_CURRENT_DRAW      2.0

#endif