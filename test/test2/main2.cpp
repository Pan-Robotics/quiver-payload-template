#include <Arduino.h>
#ifdef __arm__
#include <wiringPi.h>
#else
#include <SPI.h>
#endif
#include <common/mavlink.h>
#include <canard.h>
#include "quiver_payload.h"

#define SYSTEM_ID 100
#define COMPONENT_ID 200

#ifdef __arm__
#define MAVLINK_SERIAL Serial1
#else
#define MAVLINK_SERIAL Serial
#endif

#ifdef __arm__
#include <linux/can.h>
#include <linux/can/raw.h>
#else
#include <CAN.h>
#endif

#define DRONECAN_NODE_ID 100
#define CANARD_TX_QUEUE_CAPACITY 16
#define CANARD_TX_QUEUE_ITEM_SIZE sizeof(CanardTxQueueItem)

static CanardTxQueue tx_queue;
static CanardInstance canard;

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
        millis(), value, 0.0, 0, 0
    );
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MAVLINK_SERIAL.write(buf, len);
}

// Send DroneCAN node status using libcanard v1 API
void send_dronecan_status() {
    uint8_t payload[7] = {0};
    uint32_t uptime_sec = millis() / 1000;
    payload[0] = uptime_sec & 0xFF;
    payload[1] = (uptime_sec >> 8) & 0xFF;
    payload[2] = (uptime_sec >> 16) & 0xFF;
    payload[3] = (uptime_sec >> 24) & 0xFF;
    payload[4] = 0; // health: OK
    payload[5] = 0; // mode: OPERATIONAL
    payload[6] = 0; // sub-mode

    static uint8_t transfer_id = 0;
    CanardTransferMetadata meta = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = 341,
        .remote_node_id = CANARD_NODE_ID_UNSET,
        .transfer_id    = transfer_id++,
    };

    // Construct CanardPayload and push to tx_queue
    CanardPayload canard_payload;
    canard_payload.data = payload;
    canard_payload.size = sizeof(payload);
    canardTxPush(&tx_queue, &canard, micros(), &meta, canard_payload, micros(), NULL);

}

void setup() {
#ifdef __arm__
    wiringPiSetupGpio();
#else
    pinMode(PIN_ANALOG_IO, INPUT);
    pinMode(PIN_DIGITAL_IO, OUTPUT);
#endif
    pinMode(PIN_DIGITAL_IO, OUTPUT);
    digitalWrite(PIN_DIGITAL_IO, LOW);

    MAVLINK_SERIAL.begin(115200);

#ifndef __arm__
    CAN.begin(CAN_BAUDRATE);
#endif
CanardMemoryResource memory_resource = {
    .user_reference = NULL,  // User-defined reference, can be used for custom memory management
};
    // Initialize the CAN bus
    // Use NULL for CanardMemoryResource (default allocator)
    canard = canardInit(memory_resource);
    canard.node_id = DRONECAN_NODE_ID;
    tx_queue = canardTxInit(CANARD_TX_QUEUE_CAPACITY, CANARD_TX_QUEUE_ITEM_SIZE, memory_resource);
}

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
        float sensor_value = analogRead(PIN_ANALOG_IO) * (3.3 / 1023.0);
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

     // Transmit DroneCAN messages from the queue
    for (CanardTxQueueItem* item = canardTxPeek(&tx_queue);
         item != NULL;
         item = canardTxPeek(&tx_queue)) {
#ifndef __arm__
        CAN.beginPacket(item->frame.extended_can_id ? item->frame.extended_can_id : (item->frame.extended_can_id & 0x7FF));
        // Cast void* to uint8_t* for byte access
        const uint8_t* payload_data = static_cast<const uint8_t*>(item->frame.payload.data);
        for (size_t i = 0; i < item->frame.payload.size; ++i) {
            CAN.write(payload_data[i]);
        }
        CAN.endPacket();
#endif
        canardTxPop(&tx_queue, item);
    }
}