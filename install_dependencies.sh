#!/bin/bash

# ROB 311 - Real-time Plotting Dependencies Installation Script
# This script installs the required Python packages for real-time plotting

echo "Installing dependencies for real-time plotting..."
echo "================================================"

# Check if pip is available
if ! command -v pip3 &> /dev/null
then
    echo "Error: pip3 not found. Please install Python 3 and pip first."
    exit 1
fi

# Install PyQtGraph and its dependencies with --break-system-packages flag
# This flag is needed on newer Debian/Ubuntu systems with externally-managed Python
echo "Installing PyQtGraph..."
pip3 install --break-system-packages pyqtgraph

echo "Installing PyQt5 (GUI backend)..."
pip3 install --break-system-packages PyQt5

echo "Installing numpy (if not already installed)..."
pip3 install --break-system-packages numpy

echo ""
echo "================================================"
echo "Installation complete!"
echo ""
echo "Installed packages:"
echo "  - pyqtgraph: High-performance real-time plotting"
echo "  - PyQt5: GUI backend for PyQtGraph"
echo "  - numpy: Numerical computing library"
echo ""
echo "You can now run test_motors.py with real-time plotting capability."
echo "================================================"
