#!/usr/bin/env python3
"""
DataLogger3 Viewer - Standalone real-time plot viewer

This is a SELF-CONTAINED script that checks and installs its own dependencies.
Just download and run!

Run this ON YOUR LAPTOP to view data from robot running DataLogger3.

Usage:
    python DataLogger3_viewer.py <robot_ip> [port]

Examples:
    python DataLogger3_viewer.py 67.194.46.111
    python DataLogger3_viewer.py mbot 5557

Features:
    - Auto-detects and installs dependencies (numpy, pyqtgraph, PyQt5)
    - Two display modes: Diagnosis (all plots) and PID Tuning (error & control)
    - Real-time streaming up to 200 Hz
    - 5-second rolling window (1000 points)

Requirements:
    - Python 3.6+
    - Internet connection (for first-time dependency installation)
"""

__version__ = "3.0"
__author__ = "ROB 311 Course Staff"

import sys
import socket
import json
import subprocess


def check_python_version():
    """Ensure minimum Python version"""
    if sys.version_info < (3, 6):
        print(f"✗ Python 3.6+ required (you have {sys.version_info.major}.{sys.version_info.minor})")
        print("Please upgrade Python: https://www.python.org/downloads/")
        sys.exit(1)


def check_dependencies():
    """Check and optionally install PyQtGraph dependencies"""
    missing = []
    
    print("Checking dependencies...")

    try:
        import numpy as np
        print("  ✓ numpy")
    except ImportError:
        print("  ✗ numpy (missing)")
        missing.append('numpy')

    try:
        import pyqtgraph as pg
        print("  ✓ pyqtgraph")
    except ImportError:
        print("  ✗ pyqtgraph (missing)")
        missing.append('pyqtgraph')
    
    try:
        from pyqtgraph.Qt import QtCore, QtWidgets
        print("  ✓ PyQt5")
    except ImportError:
        print("  ✗ PyQt5 (missing)")
        missing.append('PyQt5')

    if missing:
        print(f"\n⚠ Missing packages: {', '.join(missing)}")
        print("\nOptions:")
        print("  1. Auto-install (recommended)")
        print("  2. Show manual installation command")
        print("  3. Exit")
        
        response = input("\nChoose option (1/2/3): ").strip()
        
        if response == '1':
            print(f"\nInstalling: {' '.join(missing)}...")
            try:
                subprocess.check_call(
                    [sys.executable, '-m', 'pip', 'install'] + missing,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                print("✓ Installation complete! Please restart the viewer.")
                sys.exit(0)
            except subprocess.CalledProcessError as e:
                print("✗ Installation failed.")
                print(f"\nError: {e}")
                print(f"\nTry manually: pip3 install {' '.join(missing)}")
                sys.exit(1)
        elif response == '2':
            print(f"\nManual installation command:")
            print(f"  pip3 install {' '.join(missing)}")
            print(f"\nOr using conda:")
            print(f"  conda install {' '.join(missing)}")
            sys.exit(1)
        else:
            print("Exiting.")
            sys.exit(1)
    else:
        print("✓ All dependencies satisfied!\n")
    
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
        
        # Display mode: 'diagnosis' or 'pid_tuning'
        self.display_mode = 'diagnosis'

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
        # Main window with vertical layout
        self.win = QtWidgets.QWidget()
        self.win.resize(1600, 900)
        self.win.setWindowTitle(f'DataLogger3 Viewer - {self.host}:{self.port}')
        
        layout = QtWidgets.QVBoxLayout()
        self.win.setLayout(layout)
        
        # Control panel at top
        control_panel = QtWidgets.QHBoxLayout()
        
        # Mode selection
        mode_label = QtWidgets.QLabel("Display Mode:")
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(['Diagnosis (All Plots)', 'PID Tuning (Error & Control)'])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        
        control_panel.addWidget(mode_label)
        control_panel.addWidget(self.mode_combo)
        control_panel.addStretch()
        
        layout.addLayout(control_panel)
        
        # Graphics layout for plots
        self.graphics_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_widget)
        
        self.win.show()
        
        # Create initial plots
        self.create_plots()
        
        print(f"✓ UI created with {len(self.curves)} plots")

    def create_plots(self):
        """Create plots based on current display mode"""
        # Clear existing plots
        self.graphics_widget.clear()
        self.plots = {}
        self.curves = {}
        
        if self.display_mode == 'diagnosis':
            self.create_diagnosis_plots()
        else:
            self.create_pid_tuning_plots()
    
    def create_diagnosis_plots(self):
        """Create all plots for diagnosis mode"""
        # Create plots for each data column (skip iteration counter)
        num_plots = len(self.headers) - 1  # Skip first column (iteration)
        cols = 2
        rows = (num_plots + cols - 1) // cols

        for idx, header in enumerate(self.headers[1:], start=1):
            row = (idx - 1) // cols
            col = (idx - 1) % cols

            plot = self.graphics_widget.addPlot(title=header, row=row, col=col)
            plot.setLabel('left', header)
            plot.setLabel('bottom', 'Time', units='s')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setDownsampling(mode='peak')
            plot.setClipToView(True)

            curve = plot.plot(pen=pg.mkPen(color=(100, 200, 255), width=2))

            self.plots[header] = plot
            self.curves[header] = curve
    
    def create_pid_tuning_plots(self):
        """Create focused plots for PID tuning mode"""
        # Ballbot-specific signal patterns:
        # - theta_x, theta_y = body angles (errors for balance control)
        # - Tx, Ty = control torques
        # - u1, u2, u3 = motor control outputs
        # - psi = wheel angles, dpsi = wheel velocities
        
        x_error_signals = []
        x_control_signals = []
        y_error_signals = []
        y_control_signals = []
        other_signals = []
        
        print(f"  Analyzing {len(self.headers)-1} signals for PID tuning mode...")
        
        # Categorize signals based on ballbot conventions
        for header in self.headers[1:]:  # Skip iteration counter
            header_lower = header.lower()
            
            # X-axis signals
            if header_lower == 'theta_x':
                x_error_signals.append(header)
                print(f"    ✓ {header} → X-axis error (body angle)")
            elif header_lower in ['tx', 'u1']:
                x_control_signals.append(header)
                print(f"    ✓ {header} → X-axis control")
            
            # Y-axis signals
            elif header_lower == 'theta_y':
                y_error_signals.append(header)
                print(f"    ✓ {header} → Y-axis error (body angle)")
            elif header_lower in ['ty', 'u2']:
                y_control_signals.append(header)
                print(f"    ✓ {header} → Y-axis control")
            
            # Generic error signals (wheel positions/velocities for inner loop)
            elif 'psi' in header_lower or 'dpsi' in header_lower:
                print(f"    ~ {header} → Wheel state (excluded for outer loop tuning)")
            
            # Everything else
            else:
                print(f"    ✗ {header} → Excluded from PID mode")
        
        # If no axis-specific signals found, fall back to generic error/control grouping
        if not (x_error_signals or x_control_signals or y_error_signals or y_control_signals):
            print("⚠ No axis-specific signals detected, using generic grouping")
            self.create_generic_pid_plots()
            return
        
        # Create 2x2 grid: X-axis and Y-axis, each with error and control plots
        row = 0
        
        # X-AXIS ERROR (top-left)
        if x_error_signals:
            x_error_plot = self.graphics_widget.addPlot(title='X-Axis Error', row=0, col=0)
            x_error_plot.setLabel('left', 'Error')
            x_error_plot.setLabel('bottom', 'Time', units='s')
            x_error_plot.showGrid(x=True, y=True, alpha=0.3)
            x_error_plot.setDownsampling(mode='peak')
            x_error_plot.setClipToView(True)
            x_error_plot.addLegend()
            
            # Add reference line at zero
            x_error_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=1, style=2))
            
            colors = [(255, 100, 100), (255, 150, 100), (255, 200, 100)]
            for idx, signal in enumerate(x_error_signals):
                color = colors[idx % len(colors)]
                curve = x_error_plot.plot(pen=pg.mkPen(color=color, width=2), name=signal)
                self.plots[signal] = x_error_plot
                self.curves[signal] = curve
        
        # X-AXIS CONTROL (bottom-left)
        if x_control_signals:
            x_control_plot = self.graphics_widget.addPlot(title='X-Axis Control Effort', row=1, col=0)
            x_control_plot.setLabel('left', 'Control')
            x_control_plot.setLabel('bottom', 'Time', units='s')
            x_control_plot.showGrid(x=True, y=True, alpha=0.3)
            x_control_plot.setDownsampling(mode='peak')
            x_control_plot.setClipToView(True)
            x_control_plot.addLegend()
            
            # Add reference line at zero
            x_control_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=1, style=2))
            
            colors = [(255, 150, 50), (255, 100, 0), (200, 100, 50)]
            for idx, signal in enumerate(x_control_signals):
                color = colors[idx % len(colors)]
                curve = x_control_plot.plot(pen=pg.mkPen(color=color, width=2), name=signal)
                self.plots[signal] = x_control_plot
                self.curves[signal] = curve
        
        # Y-AXIS ERROR (top-right)
        if y_error_signals:
            y_error_plot = self.graphics_widget.addPlot(title='Y-Axis Error', row=0, col=1)
            y_error_plot.setLabel('left', 'Error')
            y_error_plot.setLabel('bottom', 'Time', units='s')
            y_error_plot.showGrid(x=True, y=True, alpha=0.3)
            y_error_plot.setDownsampling(mode='peak')
            y_error_plot.setClipToView(True)
            y_error_plot.addLegend()
            
            # Add reference line at zero
            y_error_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=1, style=2))
            
            colors = [(100, 255, 100), (100, 255, 150), (100, 255, 200)]
            for idx, signal in enumerate(y_error_signals):
                color = colors[idx % len(colors)]
                curve = y_error_plot.plot(pen=pg.mkPen(color=color, width=2), name=signal)
                self.plots[signal] = y_error_plot
                self.curves[signal] = curve
        
        # Y-AXIS CONTROL (bottom-right)
        if y_control_signals:
            y_control_plot = self.graphics_widget.addPlot(title='Y-Axis Control Effort', row=1, col=1)
            y_control_plot.setLabel('left', 'Control')
            y_control_plot.setLabel('bottom', 'Time', units='s')
            y_control_plot.showGrid(x=True, y=True, alpha=0.3)
            y_control_plot.setDownsampling(mode='peak')
            y_control_plot.setClipToView(True)
            y_control_plot.addLegend()
            
            # Add reference line at zero
            y_control_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=1, style=2))
            
            colors = [(50, 150, 255), (0, 100, 255), (50, 100, 200)]
            for idx, signal in enumerate(y_control_signals):
                color = colors[idx % len(colors)]
                curve = y_control_plot.plot(pen=pg.mkPen(color=color, width=2), name=signal)
                self.plots[signal] = y_control_plot
                self.curves[signal] = curve
        
        print(f"✓ PID Tuning mode:")
        print(f"  X-axis: {len(x_error_signals)} error, {len(x_control_signals)} control")
        print(f"  Y-axis: {len(y_error_signals)} error, {len(y_control_signals)} control")
        print(f"  Total curves created: {len(self.curves)}")
        print(f"  Excluded signals: {len(self.headers) - 1 - len(self.curves)}")
    
    def create_generic_pid_plots(self):
        """Fallback: Create generic error/control plots without axis separation"""
        error_keywords = ['error', 'err', 'setpoint', 'reference', 'target']
        control_keywords = ['control', 'effort', 'output', 'cmd', 'command', 'torque', 'voltage', 'pwm']
        
        error_signals = []
        control_signals = []
        
        for header in self.headers[1:]:
            header_lower = header.lower()
            if any(kw in header_lower for kw in error_keywords):
                error_signals.append(header)
            elif any(kw in header_lower for kw in control_keywords):
                control_signals.append(header)
        
        if not error_signals and not control_signals:
            print("⚠ No error/control signals detected, showing all signals")
            self.create_diagnosis_plots()
            return
        
        # Create error plot
        if error_signals:
            error_plot = self.graphics_widget.addPlot(title='Error Signals', row=0, col=0)
            error_plot.setLabel('left', 'Error')
            error_plot.setLabel('bottom', 'Time', units='s')
            error_plot.showGrid(x=True, y=True, alpha=0.3)
            error_plot.setDownsampling(mode='peak')
            error_plot.setClipToView(True)
            error_plot.addLegend()
            error_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=1, style=2))
            
            colors = [(255, 100, 100), (100, 255, 100), (100, 100, 255), 
                     (255, 255, 100), (255, 100, 255), (100, 255, 255)]
            
            for idx, signal in enumerate(error_signals):
                color = colors[idx % len(colors)]
                curve = error_plot.plot(pen=pg.mkPen(color=color, width=2), name=signal)
                self.plots[signal] = error_plot
                self.curves[signal] = curve
        
        # Create control plot
        if control_signals:
            control_plot = self.graphics_widget.addPlot(title='Control Effort', row=1, col=0)
            control_plot.setLabel('left', 'Control')
            control_plot.setLabel('bottom', 'Time', units='s')
            control_plot.showGrid(x=True, y=True, alpha=0.3)
            control_plot.setDownsampling(mode='peak')
            control_plot.setClipToView(True)
            control_plot.addLegend()
            control_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=1, style=2))
            
            colors = [(255, 150, 50), (50, 150, 255), (150, 255, 50),
                     (255, 50, 150), (150, 50, 255), (50, 255, 150)]
            
            for idx, signal in enumerate(control_signals):
                color = colors[idx % len(colors)]
                curve = control_plot.plot(pen=pg.mkPen(color=color, width=2), name=signal)
                self.plots[signal] = control_plot
                self.curves[signal] = curve
        
        print(f"✓ PID Tuning mode (generic): {len(error_signals)} error, {len(control_signals)} control")
    
    def on_mode_changed(self, index):
        """Handle mode selection change"""
        if index == 0:
            self.display_mode = 'diagnosis'
        else:
            self.display_mode = 'pid_tuning'
        
        print(f"Switching to {self.display_mode} mode...")
        self.create_plots()

    def read_and_update(self):
        """Read data from socket and update plots"""
        if not self.sock:
            return

        try:
            chunk = self.sock.recv(8192).decode('utf-8')
            if chunk:
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
        except ConnectionResetError:
            print("✗ Connection lost to robot")
            self.sock = None
            return
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
            
            # Debug: print first few data points
            if len(self.data.get(self.headers[1] if len(self.headers) > 1 else 'time', [])) <= 3:
                print(f"✓ Receiving data: {len(values)} values")

    def run(self):
        """Start the viewer event loop"""
        sys.exit(self.app.exec_())


def main():
    # Check Python version first
    check_python_version()
    
    print("=" * 60)
    print("  DataLogger3 Viewer - Standalone Real-time Plot Viewer")
    print(f"  Version {__version__}")
    print("=" * 60)
    print()
    
    # Default robot IP - change this to your robot's IP
    DEFAULT_ROBOT_IP = "67.194.46.111"
    DEFAULT_PORT = 5557

    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <robot_ip> [port]")
        print(f"\nNo IP specified, using default: {DEFAULT_ROBOT_IP}")
        host = DEFAULT_ROBOT_IP
        port = DEFAULT_PORT
    else:
        host = sys.argv[1]
        port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT

    print(f"\nConnecting to {host}:{port}...")

    try:
        viewer = DataLogger3Viewer(host=host, port=port)
        viewer.run()
    except KeyboardInterrupt:
        print("\n\n✓ Viewer closed by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()