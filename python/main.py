#!/usr/bin/env python3
"""
Quiver payload simulator: DroneCAN-v0 rangefinder + MAVLink mirror

Structured exactly like the official PyDroneCAN sim_rangefinder example:
  https://github.com/dronecan/pydronecan/blob/master/examples/sim_rangefinder.py
"""

import dronecan, time, math
from argparse import ArgumentParser
from pymavlink import mavutil
import RPi.GPIO as GPIO

from quiver_payload import PIN_DIGITAL_IO, CAN_BAUDRATE, PIXHAWK_IP, PIXHAWK_PORT

# ────────────────────────────────────────────────────────────────────────────────
# 1) CLI arguments
# ────────────────────────────────────────────────────────────────────────────────
parser = ArgumentParser(description='Quiver payload: DroneCAN rangefinder + MAVLink mirror')
parser.add_argument("--node-id",  default=42,     type=int,   help="CAN node ID")
parser.add_argument("--uri",      default="can0", type=str,   help="SocketCAN interface")
parser.add_argument("--rate",     default=20.0,   type=float, help="range broadcast rate (Hz)")
parser.add_argument("--debug",    action="store_true",      help="enable debug prints")
parser.add_argument("--gcs-ip",   default=PIXHAWK_IP,       help="Ground station IP for MAVLink")
parser.add_argument("--gcs-port", default=PIXHAWK_PORT, type=int,   help="Ground station UDP port")
args = parser.parse_args()

# ────────────────────────────────────────────────────────────────────────────────
# 2) GPIO setup for digital IO control
# ────────────────────────────────────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_DIGITAL_IO, GPIO.OUT)
GPIO.output(PIN_DIGITAL_IO, GPIO.LOW)

# ────────────────────────────────────────────────────────────────────────────────
# 3) Initialize DroneCAN-v0 node
# ────────────────────────────────────────────────────────────────────────────────
node = dronecan.make_node(
    args.uri,
    node_id=args.node_id,
    bitrate=CAN_BAUDRATE
)

# ────────────────────────────────────────────────────────────────────────────────
# 4) MAVLink setup (UDP out)
# ────────────────────────────────────────────────────────────────────────────────
mav = mavutil.mavlink_connection(
    f"udpout:{args.gcs_ip}:{args.gcs_port}",
    source_system=args.node_id,
    source_component=1
)

def send_mavlink_heartbeat():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_MODE_MANUAL_ARMED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_mavlink_distance(cm):
    mav.mav.distance_sensor_send(
        int(time.time() * 1000),  # time_boot_ms
        20,    # min_distance_cm
        400,   # max_distance_cm
        cm,    # current_distance_cm
        2,     # sensor type = LIDAR
        0,     # sensor id
        mavutil.mavlink.MAV_DISTANCE_SENSOR_VALID,
        0,     # orientation forward
        0      # covariance
    )

# ────────────────────────────────────────────────────────────────────────────────
# 5) DSDL‐generated message classes via dronecan.uavcan namespace
# ────────────────────────────────────────────────────────────────────────────────
RangeMeasurement = dronecan.uavcan.equipment.range_sensor.Measurement
NodeStatus       = dronecan.uavcan.protocol.NodeStatus

# ────────────────────────────────────────────────────────────────────────────────
# 6a) Periodic NodeStatus (heartbeat) at 1 Hz
# ────────────────────────────────────────────────────────────────────────────────
def publish_node_status(_):
    msg = NodeStatus(
        uptime_sec=int(time.time()),
        health=NodeStatus.HEALTH_OK,
        mode=NodeStatus.MODE_OPERATIONAL
    )
    node.broadcast(msg)
    if args.debug:
        print("CAN Heartbeat →", msg)

node.periodic(1.0, publish_node_status)

# ────────────────────────────────────────────────────────────────────────────────
# 6b) Periodic range measurement + MAVLink mirror at args.rate Hz
# ────────────────────────────────────────────────────────────────────────────────
def publish_range(_):
    # build the DroneCAN range measurement
    msg = RangeMeasurement()
    msg.sensor_id     = 1
    msg.field_of_view = math.radians(30)                   # 30° FOV
    msg.sensor_type   = RangeMeasurement.SENSOR_TYPE_LASER
    msg.reading_type  = RangeMeasurement.READING_TYPE_VALID
    msg.range         = 2.5 + 2.0 * math.sin(time.time() * 2 * math.pi * 0.2)

    node.broadcast(msg)
    if args.debug:
        print("CAN Range →", msg)

    # mirror it over MAVLink
    cm = int(msg.range * 100)
    send_mavlink_heartbeat()
    send_mavlink_distance(cm)
    if args.debug:
        print(f"MAVLink Distance → {cm} cm")

node.periodic(1.0 / args.rate, publish_range)

# ────────────────────────────────────────────────────────────────────────────────
# 7) Spin the node (exactly like the sim_rangefinder example)
# ────────────────────────────────────────────────────────────────────────────────
try:
    print("Starting Quiver payload node; press CTRL-C to stop.")
    node.spin()
except KeyboardInterrupt:
    pass
finally:
    node.close()
    print("Shutdown complete.")
# ────────────────────────────────────────────────────────────────────────────────
'''
UAV Ground Station Configuration
Mission Planner: Connect to Pixhawk USB + UDP 14550.

Parameters:

CAN_P1_DRIVER = 1

UAVCAN_ENABLE = 1

RNGFND1_TYPE = 24 (DroneCAN)

RNGFND2_TYPE = 10 (MAVLink)

Confirm uavcan.protocol.NodeStatus and uavcan.equipment.range_sensor.Measurement are received.
'''
