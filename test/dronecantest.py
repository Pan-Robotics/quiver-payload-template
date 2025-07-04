#!/usr/bin/env python3
"""
simulate DroneCAN-v0 rangefinder + mirror via MAVLink

Follows the structure of:
https://github.com/dronecan/pydronecan/blob/master/examples/sim_rangefinder.py
"""

import dronecan, time, math
from argparse import ArgumentParser
from pymavlink import mavutil

# 1) CLI arguments (exactly like the upstream example, plus MAVLink)
parser = ArgumentParser(description='simulate DroneCAN rangefinder + MAVLink mirror')
parser.add_argument("--node-id",  default=120,    type=int,   help="CAN node ID")
parser.add_argument("--uri",      default="can0", type=str,   help="SocketCAN URI")
parser.add_argument("--bitrate",  default=1000000, type=int,   help="CAN bitrate")
parser.add_argument("--rate",     default=20.0,    type=float, help="range broadcast rate (Hz)")
parser.add_argument("--debug",    action="store_true",      help="enable debug prints")
parser.add_argument("--gcs-ip",   default="127.0.0.1",      help="Ground station IP for MAVLink")
parser.add_argument("--gcs-port", default=14550,   type=int,   help="Ground station UDP port")
args = parser.parse_args()

# 2) Initialize DroneCAN node (exactly as upstream)
node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=args.bitrate)

# 3) MAVLink setup (UDP out)
mav = mavutil.mavlink_connection(
    f"udpout:{args.gcs_ip}:{args.gcs_port}",
    source_system=250,       # pick a unique system ID
    source_component=1
)

def send_mavlink_heartbeat():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_MODE_MANUAL_DISARMED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_mavlink_distance(cm):
    mav.mav.distance_sensor_send(
        int(time.time() * 1000),  # time_boot_ms
        20,   # min distance cm
        400,  # max distance cm
        cm,   # current distance cm
        2,    # sensor type = LIDAR
        0,    # sensor id
        mavutil.mavlink.MAV_DISTANCE_SENSOR_VALID,
        0,    # orientation forward
        0     # covariance
    )

# 4) Pull in the DSDL-generated classes via the dronecan.uavcan namespace
RangeMeasurement = dronecan.uavcan.equipment.range_sensor.Measurement
NodeStatus       = dronecan.uavcan.protocol.NodeStatus

# 5a) Periodic NodeStatus (v0 heartbeat) at 1 Hz
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

# 5b) Periodic range measurement + MAVLink mirror at args.rate Hz
def publish_range(_):
    # build the DroneCAN range measurement
    msg = RangeMeasurement()
    msg.sensor_id     = 1
    msg.field_of_view = math.radians(30)                 # 30° FOV
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

# 6) Spin the node (same as the example)
try:
    print("Starting DroneCAN+MAVLink simulator; CTRL-C to stop.")
    node.spin()
except KeyboardInterrupt:
    pass
finally:
    node.close()
    print("Shutdown complete.")

# Note: The above code is a complete script that simulates a DroneCAN rangefinder
# and mirrors the data over MAVLink. It includes periodic broadcasts of NodeStatus
# and range measurements, with optional debug output. The MAVLink connection is set up
# to send data to a specified ground station IP and port. The range measurement is
# generated using a sine wave function to simulate changing distances, and the
# DroneCAN node is set up to run at a specified rate. The script can be run
# with command-line arguments to customize the node ID, CAN interface, bitrate,
# broadcast rate, and debug mode. The MAVLink messages are sent using the pymavlink
# library, which is compatible with ArduPilot and other MAVLink-compatible systems.
# The script is designed to be run in a Python environment with the necessary libraries installed.