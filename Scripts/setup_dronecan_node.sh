#!/usr/bin/env bash
# ============================================
# setup_quiver_pydronecan.sh
# PyDroneCAN-v0 + MAVLink example setup
# ============================================

set -euo pipefail

echo "🔹 Updating system packages..."
sudo apt-get update

echo "🔹 Installing CAN utilities and Python essentials..."
sudo apt-get install -y \
    python3-pip python3-can can-utils git

echo "🔹 Installing Python packages: PyDroneCAN, MAVLink support..."
pip3 install --upgrade \
    dronecan \
    pymavlink \
    python-can

echo "🔹 Cloning DroneCAN-v0 DSDL definitions (if not present)..."
[ -d dronecan_dSDL ] || git clone https://github.com/dronecan/DSDL.git dronecan_dSDL

echo ""
echo "✅ ✅ ✅ Setup complete!"
echo ""
echo "👉 Next steps:"
echo ""
echo "1) Bring up your CAN interface:"
echo "     sudo ip link set can0 up type can bitrate 1000000"
echo ""
echo "2) Run the simulator:"
echo "     python3 dronecantest.py --uri can0 --node-id 42 --rate 20 --gcs-ip 192.168.1.10 --gcs-port 14550 [--debug]"
echo ""
echo "Your node will now:"
echo "  • Broadcast NodeStatus_1_0 at 1 Hz"
echo "  • Broadcast a simulated rangefinder at your chosen rate"
echo "  • Mirror both heartbeat+distance over MAVLink to your Pixhawk"
echo "============================================"
