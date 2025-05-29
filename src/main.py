#!/usr/bin/env python3
import time
from pymavlink import mavutil
import can
import RPi.GPIO as GPIO
from quiver_payload import PIN_ANALOG_IO, PIN_DIGITAL_IO, ETHERNET_UDP_PORT, CAN_BAUDRATE

# MAVLink system and component IDs
SYSTEM_ID = 100
COMPONENT_ID = 200

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_ANALOG_IO, GPIO.IN)  # Requires external ADC
GPIO.setup(PIN_DIGITAL_IO, GPIO.OUT)
GPIO.output(PIN_DIGITAL_IO, GPIO.LOW)

# Initialize MAVLink (UDP)
conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{ETHERNET_UDP_PORT}', source_system=SYSTEM_ID, source_component=COMPONENT_ID)

# Initialize CAN (MCP2515)
can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=CAN_BAUDRATE)

def send_mavlink_heartbeat():
    """Send MAVLink heartbeat message."""
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_MODE_MANUAL_ARMED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_mavlink_sensor_data(value):
    """Send MAVLink sensor data (e.g., pressure)."""
    conn.mav.scaled_pressure_send(
        int(time.time() * 1000),
        value,
        0.0,
        0
    )

def send_dronecan_status():
    """Send DroneCAN node status message."""
    msg = can.Message(
        arbitration_id=0x0C2,  # NodeStatus message ID
        data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00],  # Uptime, health, mode
        is_extended_id=True
    )
    can_bus.send(msg)

# Main loop
last_mavlink_heartbeat = 0
last_sensor = 0
last_dronecan = 0
while True:
    # Send MAVLink heartbeat every 1 second
    if time.time() - last_mavlink_heartbeat >= 1:
        send_mavlink_heartbeat()
        last_mavlink_heartbeat = time.time()

    # Send sensor data every 500ms
    if time.time() - last_sensor >= 0.5:
        sensor_value = 0.0  # Placeholder: Replace with ADC reading
        send_mavlink_sensor_data(sensor_value)
        last_sensor = time.time()

    # Send DroneCAN status every 1 second
    if time.time() - last_dronecan >= 1:
        send_dronecan_status()
        last_dronecan = time.time()

    # Handle MAVLink messages
    msg = conn.recv_match(blocking=False)
    if msg and msg.get_type() == 'COMMAND_LONG':
        if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR:
            GPIO.output(PIN_DIGITAL_IO, GPIO.HIGH if msg.param1 > 0 else GPIO.LOW)

    # Handle DroneCAN messages
    can_msg = can_bus.recv(timeout=0.01)
    if can_msg:
        # Process DroneCAN messages (e.g., actuator commands)
        pass

    time.sleep(0.01)