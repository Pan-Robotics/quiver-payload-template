#include <Arduino.h>
#include <ETH.h>
#include <common/mavlink.h>
#include "quiver_payload.h"

// MAVLink configuration
#define SYSTEM_ID 100
#define COMPONENT_ID 200
#define MAVLINK_UDP_PORT 14550
#define MAVLINK_BUFFER_SIZE 2048

// Ethernet configuration
#define ETH_PHY_ADDR 1
#define ETH_PHY_POWER 12
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

// Global variables
static WiFiUDP udp;
static IPAddress ground_station_ip;
static uint16_t ground_station_port;
mavlink_system_t mavlink_system = { SYSTEM_ID, COMPONENT_ID };
static uint8_t mavlink_buffer[MAVLINK_BUFFER_SIZE];

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
    udp.beginPacket(ground_station_ip, ground_station_port);
    udp.write(buf, len);
    udp.endPacket();
}

// Send MAVLink sensor data
void send_mavlink_sensor_data(float value) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_scaled_pressure_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        millis(), value, 0.0, 0, 0
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    udp.beginPacket(ground_station_ip, ground_station_port);
    udp.write(buf, len);
    udp.endPacket();
}

// Ethernet event handler
void ethernet_event(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("ETH Started");
            ETH.setHostname("quiver-payload");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("ETH IP: ");
            Serial.println(ETH.localIP());
            // Start UDP
            udp.begin(MAVLINK_UDP_PORT);
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Disconnected");
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            break;
        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize Ethernet
    WiFi.onEvent(ethernet_event);
    ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
    
    // Set default ground station address
    ground_station_ip.fromString("192.168.1.2"); // Change to your GCS IP
    ground_station_port = 14550;
}

void loop() {
    // Send MAVLink heartbeat every 1 second
    static uint32_t last_heartbeat = 0;
    if (millis() - last_heartbeat >= 1000) {
        send_mavlink_heartbeat();
        last_heartbeat = millis();
    }
    
    // Send sensor data every 500ms
    static uint32_t last_sensor = 0;
    if (millis() - last_sensor >= 500) {
        float sensor_value = analogRead(PIN_ANALOG_IO) * (3.3 / 1023.0);
        send_mavlink_sensor_data(sensor_value);
        last_sensor = millis();
    }
    
    // Handle incoming MAVLink messages
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(mavlink_buffer, MAVLINK_BUFFER_SIZE);
        
        mavlink_message_t msg;
        mavlink_status_t status;
        
        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, mavlink_buffer[i], &msg, &status)) {
                // Handle MAVLink message
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_COMMAND_LONG: {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);
                        // Handle command
                        break;
                    }
                    // Add other message handlers as needed
                }
            }
        }
        
        // Update ground station address if different
        if (udp.remoteIP() != ground_station_ip || udp.remotePort() != ground_station_port) {
            ground_station_ip = udp.remoteIP();
            ground_station_port = udp.remotePort();
        }
    }
}