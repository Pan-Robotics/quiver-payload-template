
# Quiver Payload Integration Guide

This guide provides a comprehensive framework for manufacturers and developers to design and integrate quick-release payload attachments and peripherals for the Quiver UAS, with potential compatibility for future Arrow-air UASs. It outlines the hardware, electrical, and software standards necessary for seamless integration with Quiver’s custom hardware and its Pixhawk and Ardupilot ecosystem, including support for both MAVLink and DroneCAN protocols. The guide draws inspiration from the [uAvionix Ping Integration Guide](https://uavionix.com/downloads/integration/uAvionix%20Ping%20Integration%20Guide.pdf).

## Table of Contents

-   Connector Hardware Standard
-   Electrical Interface Standard
-   [CAN and Ethernet Interface Protocol Options
-   MAVLink and DroneCAN Firmware
-   Ground Control Plugin Configuration
-   Example Potential Payload Attachments
-   Additional Guidelines
-   Support and Resources

## Connector Hardware Standard

Payloads must use standardized mechanical and electrical connectors to ensure secure and reliable integration with the Quiver UAS.

-   **Quick-Release Mechanism**:
    
    -   **Component**: Quick-Release Clip Plate Clamp (comparable to [Alibaba Quick-Release Plate](https://www.alibaba.com/product-detail/Quick-Release-Clip-Plate-Clamp-Quick_1600982145247.html)).
    -   **Specifications**:
        -   Material: Aluminum or high-strength polymer for durability.
        -   Locking Mechanism: Spring-loaded or lever-based for toolless operation.
        -   Maximum Payload Weight: 10 kg to ensure UAS stability.
-   **Electrical Connector**:
    
    -   **Connector**: Molex Mini-Fit Jr. Connector, 12 circuits, Part Number: [2077601281](https://www.molex.com/en-us/products/part-detail/2077601281).
    -   **Mating Connector**: Molex Mini-Fit Jr. Receptacle, 12 circuits, Part Number: [2045231201](https://www.molex.com/en-us/products/part-detail/2045231201).
    -   **Durability**: Rated for 100 mating cycles, IP65 weather resistance when mated.
    -   **Pin Configuration**: See the Electrical Interface Standard for details.

## Electrical Interface Standard

The electrical interface ensures power delivery and signal communication between the Quiver Main PCB and payload attachments via a 10-pin Molex Mini-Fit Jr. connector.

-   **Power Distribution**:
    
    -   **Voltage**: 12V DC nominal.
    -   **Current Limit**: 2.0A maximum draw per payload to prevent overloading.
    -   **Protection**: Overcurrent and short-circuit protection on the Main PCB.
    -   **Switched Output**: Single 12V analog output for payload-specific power control (e.g., sensors, actuators).
-   **Signal Interfaces**:
    
    -   **Ethernet**: Full-duplex, 100 Mbps, RJ45 standard (pins 1–4).
    -   **CAN Bus**: 3.3V CAN H and CAN L for DroneCAN/UAVCAN (pins 5–6).
    -   **Analog**: Single analog I/O (0–5V) for sensor data or control (pin 7).
    -   **Digital I/O**: Single digital I/O connected to FMU_CH1 (3.3V logic, pin 8).
    -   **Power and Ground**: 12V power (pin 9), Ground (pin 10).
-   **Connector Pinout**:
The pin assignments for the 10-pin Molex Mini-Fit Jr. connector are as follows:  

  | Pin | Function            | Description                              |
  |-----|---------------------|------------------------------------------|
  | 1   | Ethernet TX+        | Transmit positive for Ethernet           |
  | 2   | Ethernet TX-        | Transmit negative for Ethernet           |
  | 3   | Ethernet RX+        | Receive positive for Ethernet            |
  | 4   | Ethernet RX-        | Receive negative for Ethernet            |
  | 5   | CAN H               | CAN bus high signal (3.3V)              |
  | 6   | CAN L               | CAN bus low signal (3.3V)               |
  | 7   | Analog I/O          | Analog input/output (0–5V)              |
  | 8   | Digital I/O         | FMU_CH1 digital I/O (3.3V logic)        |
  | 9   | 12V Power           | 12V DC power supply (2.0A max)          |
  | 10  | Ground              | System ground                           |

  *Note*: Refer to the [Connector Schematic](https://canada1.discourse-cdn.com/flex004/uploads/arrow1/original/1X/98c6f7d7084deef3248806bab84a657a67b17e6e.png) for visual reference.

## CAN and Ethernet Interface Protocol Options

Payloads can communicate using **DroneCAN/UAVCAN** or **MAVLink-over-Ethernet**, depending on the application.

-   **DroneCAN/UAVCAN**:
    
    -   **Description**: Lightweight, robust protocol for CAN bus, ideal for real-time sensor/actuator control.
    -   **Implementation**:
        -   **C++**: Use [Libcanard](https://github.com/UAVCAN/libcanard) for Raspberry Pi, ESP32, STM32.
        -   **Python**: Use [python3-can](https://python-can.readthedocs.io/) with MCP2515 CAN module on Raspberry Pi.
        -   Configure node ID and message types per UAVCAN v1.0 specification.
        -   Example Messages: `uavcan.equipment.sensor.RawSensor`, `uavcan.equipment.actuator.Command`.
    -   **Bitrate**: 1 Mbps (CAN 2.0B standard).
    -   **Termination**: 120Ω resistor required at the payload if it’s the last node in the CAN chain.
    -   **Raspberry Pi CAN Setup**:
        
        ```bash
        sudo bash scripts/setup_can.sh
        sudo ip link set can0 up type can bitrate 500000
        sudo ip link set can1 up type can bitrate 250000
        sudo ifconfig can0 txqueuelen 65536
        sudo ifconfig can1 txqueuelen 65536
        dmesg | grep spi
        
        ```
        
-   **MAVLink-over-Ethernet**:
    
    -   **Description**: MAVLink protocol over UDP for high-bandwidth applications (e.g., video streaming, telemetry).
    -   **Implementation**:
        -   **C++**: Use [MAVLink C Library](https://github.com/mavlink/c_library_v2).
        -   **Python**: Use [pymavlink](https://github.com/ArduPilot/pymavlink).
        -   Default UDP port: 14550 (configurable).
        -   Example Messages: `DATA_STREAM`, `CAMERA_IMAGE_CAPTURED`, custom payload messages.
    -   **IP Configuration**: Payloads must support DHCP (default subnet: 192.168.1.0/24).

## MAVLink and DroneCAN Firmware

A forkable GitHub repository provides templates for developing payload firmware in C++ (Raspberry Pi, ESP32, STM32) and Python (Raspberry Pi).

-   **Repository**: [https://github.com/Arrow-air/quiver-payload-template](https://github.com/Arrow-air/quiver-payload-template)
-   **Structure**:
    
    ```
    quiver-payload-template/
    ├── src/
    │   ├── main.cpp
    │   ├── quiver_payload.h
    │   ├── main.py
    │   ├── quiver_payload.py
    ├── scripts/
    │   ├── setup_can.sh
    ├── platformio.ini
    ├── requirements.txt
    ├── README.md
    ├── LICENSE
    
    ```
    
-   **C++ Firmware**:
    -   Supports Raspberry Pi (`wiringPi`), ESP32, STM32 with `libcanard` (DroneCAN) and MAVLink C library.
    -   Example: Sending heartbeats, sensor data, handling actuator commands.
    -   Build with PlatformIO: `pio run -t upload`.
-   **Python Firmware**:
    -   Supports Raspberry Pi with `pymavlink` (MAVLink) and `python3-can` (DroneCAN).
    -   Run with: `python src/main.py`.
-   **CAN Setup** (Raspberry Pi):
    -   Requires MCP2515 CAN module.
    -   Configure using `scripts/setup_can.sh` (see CAN and Ethernet Interface Protocol Options.

## Ground Control Plugin Configuration

Payload configuration going to be supported via a custom Mission Planner plugin.

-   **Plugin Overview**:
    
    -   **Repository**: [https://github.com/Arrow-air/quiver-mission-planner-plugin](https://github.com/Arrow-air/quiver-mission-planner-plugin)
    -   **Compatibility**: Mission Planner 1.3.77 or later.
    -   Features: Real-time telemetry, parameter configuration, control of 12V switched output and digital I/O.
-   **Setup Instructions**:
    
    1.  Download and install the plugin from the repository.
    2.  In Mission Planner, go to **CONFIG > Plugins** and load the Quiver plugin.
    3.  Configure payload parameters (e.g., sensor gain, output triggers) via the plugin UI.
-   **Example Configuration**:
    
    -   **Parameter**: `PAYLOAD_SENSOR_GAIN` (adjust sensor sensitivity).
    -   **Control**: Toggle `PAYLOAD_OUTPUT_12V` (enable/disable 12V output).

## Example Potential Payload Attachments

Based on the [Possible Attachment List](https://github.com/DowFisherKBM/project-quiver/blob/main/task-grant-bounty/equipment/attachment/0001-possible_attachment_list/information-note.md), example payloads include:

-   **Environmental Sensor**:
    
    -   Function: Measures temperature, humidity, pressure.
    -   Interface: CAN (DroneCAN) for data, 12V power for heater.
    -   Example Message: `SCALED_PRESSURE` (MAVLink), `uavcan.equipment.sensor.RawSensor` (DroneCAN).
-   **Camera**:
    
    -   Function: Captures high-resolution images/video.
    -   Interface: Ethernet for streaming, digital I/O for triggering.
    -   Example Message: `CAMERA_IMAGE_CAPTURED` (MAVLink).
-   **LIDAR**:
    
    -   Function: 3D mapping, obstacle detection.
    -   Interface: Ethernet for high-bandwidth data, 12V power.
    -   Example Message: `DISTANCE_SENSOR` (MAVLink).
-   **Agricultural Sprayer**:
    
    -   Function: Dispenses liquid for precision agriculture.
    -   Interface: Analog I/O for pump control, digital I/O for activation.
    -   Example Message: `COMMAND_LONG` (MAVLink).

## Additional Guidelines

-   **Safety**:
    
    -   Payloads must not exceed 10 kg to maintain UAS stability.
    -   Shield for electromagnetic interference (EMI) to avoid disrupting avionics.
    -   Test payloads in a controlled environment before deployment.
-   **Testing and Validation**:
    
    -   Use the provided firmware template and Mission Planner plugin to verify communication.
    -   Simulate payload operation with a bench setup using the 10-pin connector.
    -   Ensure power draw stays within 2.0A under maximum load.
-   **Documentation**:
    
    -   Provide wiring diagrams, firmware setup, and Mission Planner configuration details.
    -   Submit payload specifications to Arrow-air for certification.

## Support and Resources

-   **Technical Support**: Contact [support@arrow-air.com](mailto:support@arrow-air.com).
-   **Community**: Join the [Arrow-air Discord](https://discord.com/invite/arrow-air).
-   **Documentation**:
    -   [Ardupilot Documentation](https://ardupilot.org/)
    -   [MAVLink Developer Guide](https://mavlink.io/)
    -   [UAVCAN Specification](https://uavcan.org/)
