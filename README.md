# Quiver Payload Firmware Template

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![GitHub Issues](https://img.shields.io/github/issues/Arrow-air/quiver-payload-template)](https://github.com/Arrow-air/quiver-payload-template/issues)
[![GitHub Stars](https://img.shields.io/github/stars/Arrow-air/quiver-payload-template)](https://github.com/Arrow-air/quiver-payload-template/stargazers)

A template repository for developing payload firmware for the Quiver UAS, supporting **C++** (Raspberry Pi, ESP32, STM32) and **Python** (Raspberry Pi) with **MAVLink** and **DroneCAN** protocols. It integrates with the Quiver's Pixhawk and Ardupilot ecosystem via a 10-pin Molex Mini-Fit Jr. connector.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Setup Instructions](#setup-instructions)
  - [C++ Setup (Raspberry Pi, ESP32, STM32)](#c-setup-raspberry-pi-esp32-stm32)
  - [Python Setup (Raspberry Pi)](#python-setup-raspberry-pi)
- [Hardware Connections](#hardware-connections)
- [Testing](#testing)
- [Contributing](#contributing)
- [License](#license)
- [Support](#support)

## Features
- **Protocols**: MAVLink and DroneCAN for robust communication.
- **Platforms**: C++ for Raspberry Pi, ESP32, STM32; Python for Raspberry Pi.
- **Hardware Compatibility**: 10-pin Molex Mini-Fit Jr. connector (12V, 2.0A, CAN, Ethernet, analog/digital I/O).
- **Example Code**: Heartbeats, sensor data, actuator control.
- **CAN Support**: MCP2515 for Raspberry Pi, native CAN for ESP32/STM32.

## Prerequisites

### Hardware
- **Raspberry Pi**: Pi 4 with Raspbian OS, MCP2515 CAN module, MCP3008 ADC (optional).
- **Microcontrollers**: ESP32 or STM32 with CAN support.
- **Connector**: 10-pin Molex Mini-Fit Jr. (Part #2077601281, mating #2045231201).

### Software
- **C++**: PlatformIO IDE with Arduino framework.
- **Python**: Python 3.7+ with `pymavlink`, `python-can`, `RPi.GPIO`.
- **Ground Control**: Mission Planner 1.3.77 or later.

## Setup Instructions

### C++ Setup (Raspberry Pi, ESP32, STM32)
1. Clone the repository:
   ```bash
   git clone https://github.com/Arrow-air/quiver-payload-template.git
   cd quiver-payload-template
   ```
2. Install PlatformIO and dependencies:
   ```bash
   pio lib install "mavlink/c_library_v2" "uavcan/libcanard" "wiringpi/wiringpi" "espressif/esp32-can" "STM32duino/CAN"
   ```
3. Edit `platformio.ini` to select your target environment (`rpi`, `esp32`, or `stm32`).
4. Build and upload:
   ```bash
   pio run -t upload
   ```
5. For Raspberry Pi CAN, configure the MCP2515 module:
   ```bash
   sudo bash scripts/setup_can.sh
   ```
6. Connect the payload to the Quiver UAS and verify with Mission Planner.

### Python Setup (Raspberry Pi)
1. Clone the repository (if not already done).
2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Configure CAN interfaces:
   ```bash
   sudo bash scripts/setup_can.sh
   ```
   After reboot, run:
   ```bash
   sudo ip link set can0 up type can bitrate 500000
   sudo ip link set can1 up type can bitrate 250000
   sudo ifconfig can0 txqueuelen 65536
   sudo ifconfig can1 txqueuelen 65536
   dmesg | grep spi
   ```
4. Run the Python script:
   ```bash
   python src/main.py
   ```
5. Verify communication with Mission Planner.

## Hardware Connections

| Platform       | Details                                                                 |
|----------------|-------------------------------------------------------------------------|
| **Raspberry Pi** | GPIO pins per `quiver_payload.h`/`quiver_payload.py`. MCP2515 CAN module: SPI0, GPIO 23 (CAN_H), 24 (CAN_L). MCP3008 ADC: GPIO 25. |
| **ESP32/STM32** | Native CAN pins (e.g., GPIO 5/4 for ESP32). ADC: GPIO 34 (ESP32).       |

- **Power**: Ensure draw does not exceed 2.0A at 12V.
- **Connector Pinout**: See [Quiver Integration Guide](#) for details.

## Testing
- Use Mission Planner to verify MAVLink/DroneCAN communication (e.g., heartbeats, sensor data).
- Test digital I/O with `MAV_CMD_DO_SET_ACTUATOR` commands.
- Monitor CAN bus on Raspberry Pi:
  ```bash
  candump can0
  ```

## Contributing
Contributions are welcome! Please:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/your-feature`).
3. Commit changes (`git commit -m 'Add your feature'`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Support
- Email: [support@arrow-air.com](mailto:support@arrow-air.com)
- Community: Join the [Arrow-air Discord](https://discord.com/invite/arrow-air) for help and collaboration.

---

**Star this repository** to support the project! 🌟

*Maintained by Arrow-air*