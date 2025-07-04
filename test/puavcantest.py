#!/usr/bin/env python3
import os
import asyncio
import time

# Tell PyCyphal where to find the DroneCAN v0 DSDL:
# You can also export CYPHAL_PATH in your shell instead of doing this here.
os.environ.setdefault('CYPHAL_PATH', os.path.abspath('dronecan_dSDL/uavcan'))

import pycyphal                                      # installs the DSDL import hook
from pycyphal.transport.can import CANTransport
from pycyphal.transport.can.media.pythoncan import PythonCANMedia
from pycyphal.presentation import Presentation
from pycyphal.application import make_node, NodeInfo

# After the import hook, these imports trigger on‐the‐fly DSDL compilation:
import uavcan.node
import uavcan.equipment.range_sensor

from pymavlink import mavutil

# === Configuration ===
CAN_IFACE      = 'can0'
CAN_BITRATE    = 1_000_000
LOCAL_NODE_ID  = 42
MAV_SYS_ID     = 100
MAV_COMP_ID    = 200
PIXHAWK_IP     = '192.168.1.10'
PIXHAWK_UDP    = 14550

# === MAVLink setup ===
conn = mavutil.mavlink_connection(
    f'udpout:{PIXHAWK_IP}:{PIXHAWK_UDP}',
    source_system=MAV_SYS_ID,
    source_component=MAV_COMP_ID
)

def send_mavlink_heartbeat():
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_MODE_MANUAL_ARMED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_mavlink_distance(cm):
    conn.mav.distance_sensor_send(
        int(time.time() * 1000),  # time_boot_ms
        20,                       # min_distance_cm
        400,                      # max_distance_cm
        cm,                       # current_distance_cm
        0,                        # type (laser)
        0,                        # id
        mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,
        0,                        # orientation forward
        0                         # covariance
    )

# === Cyphal (DroneCAN v0) node setup ===
# 1) CAN transport
can_media     = PythonCANMedia(CAN_IFACE, CAN_BITRATE)
can_transport = CANTransport(can_media, local_node_id=LOCAL_NODE_ID)

# 2) Presentation layer
presentation = Presentation(can_transport)

# 3) Build and start the node
node_info = NodeInfo(name='org.quiver.payload')
node = make_node(node_info, presentation=presentation)
node.start()

# 4) Publishers
hb_pub = presentation.make_publisher(uavcan.node.Heartbeat_1_0)
rs_pub = presentation.make_publisher(uavcan.equipment.range_sensor.Measurement_1_0)

async def publish_cyphal():
    """Publish a Heartbeat (1 Hz) and a mock RangeSensor (2 Hz)."""
    last_hb = time.time()
    last_rs = time.time()
    while True:
        now = time.time()
        if now - last_hb >= 1.0:
            hb = uavcan.node.Heartbeat_1_0(
                uptime=int(now),
                health=uavcan.node.Heartbeat_1_0.HEALTH_NOMINAL,
                mode=uavcan.node.Heartbeat_1_0.MODE_OPERATIONAL
            )
            await hb_pub.publish(hb)
            last_hb = now
        if now - last_rs >= 0.5:
            mock_m = 1.23
            rs = uavcan.equipment.range_sensor.Measurement_1_0(
                sensor_id=0,
                reading=mock_m
            )
            await rs_pub.publish(rs)
            last_rs = now
        await asyncio.sleep(0.01)

async def publish_mavlink():
    """Mirror the same data over MAVLink for ArduPilot."""
    last_hb = time.time()
    last_rs = time.time()
    while True:
        now = time.time()
        if now - last_hb >= 1.0:
            send_mavlink_heartbeat()
            last_hb = now
        if now - last_rs >= 0.5:
            send_mavlink_distance(int(1.23 * 100))  # cm
            last_rs = now
        await asyncio.sleep(0.01)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.create_task(publish_cyphal())
    loop.create_task(publish_mavlink())
    loop.run_forever()
