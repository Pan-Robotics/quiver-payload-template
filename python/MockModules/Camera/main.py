#!/usr/bin/env python3
"""
Quiver payload simulator: MAVLink distance sensor mirror only
"""

import time, math
from argparse import ArgumentParser
from pymavlink import mavutil
import RPi.GPIO as GPIO

from quiver_payload import PIN_DIGITAL_IO, PIXHAWK_IP, PIXHAWK_PORT

# ────────────────────────────────────────────────────────────────────────────────
# 1) CLI arguments
# ────────────────────────────────────────────────────────────────────────────────
parser = ArgumentParser(description='Quiver payload: MAVLink distance sensor mirror')
parser.add_argument("--rate",     default=20.0,   type=float, help="distance broadcast rate (Hz)")
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
# 3) MAVLink setup (UDP out)
# ────────────────────────────────────────────────────────────────────────────────
mav = mavutil.mavlink_connection(
    f"udpout:{args.gcs_ip}:{args.gcs_port}",
    source_system=42,
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
# 4) Periodic distance measurement + MAVLink mirror at args.rate Hz
# ────────────────────────────────────────────────────────────────────────────────
def publish_distance():
    # Simulate a range measurement (in meters)
    rng = 2.5 + 2.0 * math.sin(time.time() * 2 * math.pi * 0.2)
    cm = int(rng * 100)
    send_mavlink_heartbeat()
    send_mavlink_distance(cm)
    if args.debug:
        print(f"MAVLink Distance → {cm} cm")

# ────────────────────────────────────────────────────────────────────────────────
# 5) Main loop
# ────────────────────────────────────────────────────────────────────────────────
try:
    print("Starting Quiver MAVLink distance node; press CTRL-C to stop.")
    period = 1.0 / args.rate
    while True:
        publish_distance()
        time.sleep(period)
except KeyboardInterrupt:
    pass
finally:
    print("Shutdown complete.")