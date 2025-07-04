#!/usr/bin/env bash

# ===============================
# setup_quiver_payload.sh
# Modern PyCyphal + MAVLink payload dev setup
# ===============================

echo "🔹 Updating system packages..."
sudo apt-get update

echo "🔹 Installing OpenBLAS, CAN utils, and Python essentials..."
sudo apt-get install -y \
    libopenblas-base libopenblas-dev \
    python3-pip python3-can can-utils

echo "🔹 Installing Python packages: PyCyphal, CAN transport, MAVLink support..."
pip3 install --upgrade \
    pycyphal \
    python-can \
    pymavlink \
    numpy

echo "🔹 Cloning DroneCAN DSDL definitions (if not present)..."
if [ ! -d dronecan_dSDL ]; then
    git clone https://github.com/dronecan/DSDL.git dronecan_dSDL
fi

echo "=============================="
echo "✅ ✅ ✅ All done!"
echo ""
echo "👉 Next steps:"
echo ""
echo "1) Bring up your CAN interface:"
echo "     sudo ip link set can0 up type can bitrate 1000000"
echo ""
echo "2) Point PyCyphal at the DroneCAN v0 DSDL directory:"
echo "     export CYPHAL_PATH=\$PWD/dronecan_dSDL/uavcan"
echo ""
echo "3) Run your payload node:"
echo "     python3 quiver_payload_node.py"
echo ""
echo "Your node will now auto-compile and import both:"
echo "  • uavcan.node.Heartbeat_1_0"
echo "  • uavcan.equipment.range_sensor.Measurement_1_0"
echo "and mirror the same data via MAVLink to your Pixhawk."
echo "=============================="
