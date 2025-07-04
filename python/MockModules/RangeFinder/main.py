#!/usr/bin/env python3
"""
Quiver payload simulator: DroneCAN-v0 rangefinder only

Structured like the official PyDroneCAN sim_rangefinder example:
  https://github.com/dronecan/pydronecan/blob/master/examples/sim_rangefinder.py
"""

import dronecan, time, math
from argparse import ArgumentParser
import RPi.GPIO as GPIO

from quiver_payload import PIN_DIGITAL_IO, CAN_BAUDRATE

# ────────────────────────────────────────────────────────────────────────────────
# 1) CLI arguments
# ────────────────────────────────────────────────────────────────────────────────
parser = ArgumentParser(description='Quiver payload: DroneCAN rangefinder')
parser.add_argument("--node-id",  default=42,     type=int,   help="CAN node ID")
parser.add_argument("--uri",      default="can0", type=str,   help="SocketCAN interface")
parser.add_argument("--rate",     default=20.0,   type=float, help="range broadcast rate (Hz)")
parser.add_argument("--debug",    action="store_true",      help="enable debug prints")
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
# 4) DSDL‐generated message classes via dronecan.uavcan namespace
# ────────────────────────────────────────────────────────────────────────────────
RangeMeasurement = dronecan.uavcan.equipment.range_sensor.Measurement
NodeStatus       = dronecan.uavcan.protocol.NodeStatus

# ────────────────────────────────────────────────────────────────────────────────
# 5a) Periodic NodeStatus (heartbeat) at 1 Hz
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
# 5b) Periodic range measurement at args.rate Hz
# ────────────────────────────────────────────────────────────────────────────────
def publish_range(_):
    msg = RangeMeasurement()
    msg.sensor_id     = 1
    msg.field_of_view = math.radians(30)                   # 30° FOV
    msg.sensor_type   = RangeMeasurement.SENSOR_TYPE_LASER
    msg.reading_type  = RangeMeasurement.READING_TYPE_VALID
    msg.range         = 2.5 + 2.0 * math.sin(time.time() * 2 * math.pi * 0.2)

    node.broadcast(msg)
    if args.debug:
        print("CAN Range →", msg)

node.periodic(1.0 / args.rate, publish_range)

# ────────────────────────────────────────────────────────────────────────────────
# 6) Spin the node (exactly like the sim_rangefinder example)
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
Mission Planner: Connect to Pixhawk USB

Parameters:

CAN_P1_DRIVER = 1

UAVCAN_ENABLE = 1

RNGFND1_TYPE = 24 (DroneCAN)

Confirm uavcan.protocol.NodeStatus and uavcan.equipment.range_sensor.Measurement are received.
'''
