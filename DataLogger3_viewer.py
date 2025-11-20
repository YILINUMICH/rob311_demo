#!/usr/bin/env python3
"""
DataLogger3 Viewer - Standalone real-time plot viewer

Run this ON YOUR LAPTOP to view data from robot running DataLogger3.

Usage:
    python DataLogger3_viewer.py <robot_ip> [port]

Examples:
    python DataLogger3_viewer.py 67.194.46.111
    python DataLogger3_viewer.py mbot 5557
"""

import sys
import socket
import json
import subprocess


def check_dependencies():
    """Check and optionally install PyQtGraph dependencies"""
    missing = []

    try:
        import numpy as np
    except ImportError:
        missing.append('numpy')

    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except ImportError:
        missing.extend(['pyqtgraph', 'PyQt5'])

    if missing:
        print(f"Missing packages: {', '.join(missing)}")
        response = input("Install now? (y/n): ").strip().lower()
        if response == 'y':
            try:
                subprocess.check_call([sys.executable, '-m', 'pip', 'install'] + missing)
                print("✓ Installation complete! Please restart the viewer.")
                sys.exit(0)
            except:
                print("✗ Installation failed. Install manually:")
                print(f"  pip3 install {' '.join(missing)}")
                sys.exit(1)
        else:
            sys.exit(1)
    return True


# Check dependencies before importing
check_dependencies()

import numpy as np
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


class DataLogger3Viewer:
    """Real-time viewer for DataLogger3 streams"""

    def __init__(self, host='localhost', port=5557):
        self.host = host
        self.port = port
        self.sock = None
        self.buffer = ""
        self.headers = None

        # Data buffers (5 seconds at 200 Hz = 1000 points)
        self.max_points = 1000
        self.data = {}

        # Create Qt application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)

        # Connect to server
        self.connect_to_server()

        # Setup will happen after receiving headers
        self.win = None
        self.plots = {}
        self.curves = {}

        # Timer for reading data
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.read_and_update)
        self.timer.start(20)  # 50 Hz update rate

    def connect_to_server(self):
        """Connect to DataLogger3 TCP server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.setblocking(False)
            print(f"✓ Connected to {self.host}:{self.port}")
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            print(f"Make sure your script is running with DataLogger3")
            sys.exit(1)

    def setup_ui(self):
        """Create plot window after receiving headers"""
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.win.resize(1600, 900)
        self.win.setWindowTitle(f'DataLogger3 Viewer - {self.host}:{self.port}')

        # Create plots for each data column (skip iteration counter)
        num_plots = len(self.headers) - 1  # Skip first column (iteration)
        cols = 2
        rows = (num_plots + cols - 1) // cols

        for idx, header in enumerate(self.headers[1:], start=1):
            row = (idx - 1) // cols
            col = (idx - 1) % cols

            plot = self.win.addPlot(title=header, row=row, col=col)
            plot.setLabel('left', header)
            plot.setLabel('bottom', 'Time', units='s')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setDownsampling(mode='peak')
            plot.setClipToView(True)

            curve = plot.plot(pen=pg.mkPen(color=(100, 200, 255), width=2))

            self.plots[header] = plot
            self.curves[header] = curve

        print(f"✓ Created plots for {len(self.headers)-1} channels")

    def read_and_update(self):
        """Read data from socket and update plots"""
        if not self.sock:
            return

        try:
            chunk = self.sock.recv(8192).decode('utf-8')
            self.buffer += chunk

            # Process complete lines
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                try:
                    msg = json.loads(line)
                    self.process_message(msg)
                except json.JSONDecodeError:
                    continue
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Read error: {e}")
            self.sock = None
            return

        # Update plots if we have UI
        if self.win and self.data:
            time_key = self.headers[1] if len(self.headers) > 1 else 'time'
            if time_key in self.data and len(self.data[time_key]) > 0:
                time_data = np.array(self.data[time_key])

                for header in self.headers[1:]:
                    if header in self.curves and header in self.data:
                        y_data = np.array(self.data[header])
                        if len(y_data) > 0:
                            self.curves[header].setData(time_data, y_data)

    def process_message(self, msg):
        """Process incoming message"""
        if msg['type'] == 'header':
            self.headers = msg['headers']
            print(f"✓ Received headers: {self.headers}")

            # Initialize data buffers
            for header in self.headers:
                self.data[header] = deque(maxlen=self.max_points)

            # Create UI if not exists
            if not self.win:
                self.setup_ui()

        elif msg['type'] == 'data' and self.headers:
            values = msg['values']
            for header, value in zip(self.headers, values):
                if header in self.data:
                    try:
                        self.data[header].append(float(value))
                    except (ValueError, TypeError):
                        pass  # Skip non-numeric values

    def run(self):
        """Start the viewer event loop"""
        sys.exit(self.app.exec_())


def main():
    # Default robot IP - change this to your robot's IP
    DEFAULT_ROBOT_IP = "67.194.46.111"
    DEFAULT_PORT = 5557

    if len(sys.argv) < 2:
        print(f"No IP specified, using default: {DEFAULT_ROBOT_IP}")
        host = DEFAULT_ROBOT_IP
        port = DEFAULT_PORT
    else:
        host = sys.argv[1]
        port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT

    print(f"Starting DataLogger3 Viewer...")
    print(f"Connecting to {host}:{port}...")

    viewer = DataLogger3Viewer(host=host, port=port)
    viewer.run()


if __name__ == '__main__':
    main()
