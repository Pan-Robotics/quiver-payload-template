#include <Arduino.h>
#ifdef __arm__ // Raspberry Pi
#include <wiringPi.h>
#else // ESP32, STM32
#include <SPI.h>
#endif
#include <mavlink.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include "quiver_payload.h"

// MAVLink system and component IDs
#define SYSTEM_ID 100
#define COMPONENT_ID 200

// Serial port for MAVLink (adjust for platform)
#ifdef __arm__
#define MAVLINK_SERIAL Serial1
#else
#define MAVLINK_SERIAL Serial
#endif

// CAN configuration
#ifdef __arm__
#include <linux/can.h>
#include <linux/can/raw.h>
#else
#include <CAN.h>
#endif

// DroneCAN node
uavcan::Node<0> *node;

// Global MAVLink system configuration
mavlink_system_t mavlink_system = { SYSTEM_ID, COMPONENT_ID };

// Send MAVLink heartbeat message
void send_mavlink_heartbeat() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID,
        MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MAVLINK_SERIAL.write(buf, len);
}

// Send MAVLink sensor data (e.g., analog voltage)
void send_mavlink_sensor_data(float value) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_scaled_pressure_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        millis(), value, 0.0, 0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MAVLINK_SERIAL.write(buf, len);
}

// Send DroneCAN node status
void send_dronecan_status() {
    uavcan::protocol::NodeStatus status;
    status.uptime_sec = millis() / 1000;
    status.health = uavcan::protocol::NodeStatus::HEALTH_OK;
    status.mode = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
    node->broadcast(status);
}

// Setup function for hardware and communication
void setup() {
    // Initialize platform-specific GPIO
#ifdef __arm__
    wiringPiSetupGpio();
#else
    pinMode(PIN_ANALOG_IO, INPUT);
    pinMode(PIN_DIGITAL_IO, OUTPUT);
#endif
    pinMode(PIN_DIGITAL_IO, OUTPUT);
    digitalWrite(PIN_DIGITAL_IO, LOW);

    // Initialize serial for MAVLink
    MAVLINK_SERIAL.begin(115200);

    // Initialize CAN
#ifdef __arm__
    // Assume MCP2515 configured via setup_can.sh
#else
    CAN.begin(CAN_BAUDRATE);
#endif

    // Initialize DroneCAN
    node = new uavcan::Node<0>(CAN_BAUDRATE);
    node->setNodeID(100);
    node->start();
}

// Main loop for periodic tasks and message handling
void loop() {
    // Send MAVLink heartbeat every 1 second
    static uint32_t last_mavlink_heartbeat = 0;
    if (millis() - last_mavlink_heartbeat >= 1000) {
        send_mavlink_heartbeat();
        last_mavlink_heartbeat = millis();
    }

    // Send sensor data (MAVLink) every 500ms
    static uint32_t last_sensor = 0;
    if (millis() - last_sensor >= 500) {
        float sensor_value = analogRead(PIN_ANALOG_IO) * (3.3 / 1023.0); // Adjust for ADC
        send_mavlink_sensor_data(sensor_value);
        last_sensor = millis();
    }

    // Send DroneCAN status every 1 second
    static uint32_t last_dronecan = 0;
    if (millis() - last_dronecan >= 1000) {
        send_dronecan_status();
        last_dronecan = millis();
    }

    // Handle incoming MAVLink messages
    mavlink_message_t msg;
    mavlink_status_t status;
    while (MAVLINK_SERIAL.available()) {
        uint8_t c = MAVLINK_SERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);
                if (cmd.command == MAV_CMD_DO_SET_ACTUATOR) {
                    digitalWrite(PIN_DIGITAL_IO, cmd.param1 > 0 ? HIGH : LOW);
                }
            }
        }
    }

    // Handle DroneCAN messages
    node->spinOnce();
}