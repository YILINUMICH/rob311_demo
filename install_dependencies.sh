#!/bin/bash

# ROB 311 - Robot Dependencies Installation Script
# This script installs the required Python packages for the robot

echo "Installing dependencies for robot..."
echo "================================================"

# Check if pip is available
if ! command -v pip3 &> /dev/null
then
    echo "Error: pip3 not found. Please install Python 3 and pip first."
    exit 1
fi

# Install Python packages with --break-system-packages flag
# This flag is needed on newer Debian/Ubuntu systems with externally-managed Python

echo "Installing numpy..."
pip3 install --break-system-packages numpy

echo "Installing scipy..."
pip3 install --break-system-packages scipy

echo "Installing lcm (Lightweight Communications and Marshalling)..."
pip3 install --break-system-packages lcm

echo ""
echo "================================================"
echo "Installation complete!"
echo ""
echo "Installed packages:"
echo "  - numpy: Numerical computing library (used by control algorithms)"
echo "  - scipy: Scientific computing library (for LQR controller)"
echo "  - lcm: Lightweight Communications and Marshalling (robot messaging)"
echo ""
echo "Additional requirements:"
echo "  - mbot_lcm_msgs: Custom LCM message types (should be pre-installed)"
echo "  - utils/DataLogger.py: Data logging utilities (included in repo)"
echo "  - utils/ps4_controller_api.py: PS4 controller interface (included in repo)"
echo ""
echo "Note: Real-time plotting is done via network connection to your laptop."
echo "On your laptop, install matplotlib to use imu_viewer.py:"
echo "  pip install matplotlib"
echo "================================================"
