#!/usr/bin/env python3
"""
ROB 311 - Fall 2025
Author: GSI Yilin Ma & IA Eeshwar Krishnan
University of Michigan

DataLogger3 Viewer - Standalone real-time plot viewer

This is a SELF-CONTAINED script that checks and installs its own dependencies.
Just download and run!

Run this ON YOUR LAPTOP to view data from robot running DataLogger3.

Usage:
    python DataLogger3_viewer.py <robot_ip> [port]
    Datastructure is expected to be JSON with a "header" (list of column names) and "data" (list of values).
    data = ["i t_now Tx Ty Tz u1 u2 u3 theta_x theta_y theta_z psi_1 psi_2 psi_3 dpsi_1 dpsi_2 dpsi_3"]

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

# ============================================================================
# CONFIGURATION - Change these values for your robot
# ============================================================================
DEFAULT_ROBOT_IP = "67.194.46.111"  # Robot IP address (change to your robot's IP)
DEFAULT_PORT = 5557                  # TCP port for DataLogger3 streaming
# ============================================================================

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
    """
    Real-time viewer for DataLogger3 streams
    
    Architecture:
    1. TCP socket receives JSON messages from robot
    2. Messages parsed into 'header' (column names) and 'data' (values)
    3. Data buffered in deques (fixed-size rolling windows)
    4. PyQtGraph updates plots at 50 Hz
    
    Data Flow:
    Robot -> TCP -> JSON parser -> Data buffers -> PyQtGraph plots
    """

    def __init__(self, host='localhost', port=5557):
        """Initialize viewer with connection to robot's DataLogger3 server"""
        self.host = host  # Robot IP address
        self.port = port  # TCP port (default 5557)
        self.sock = None  # TCP socket connection
        self.buffer = ""  # Buffer for incomplete JSON messages
        self.headers = None  # Column names (e.g., ["time", "theta_x", ...])
        self.connection_status = "Not connected"

        # Data buffers: rolling windows for each column
        # Example: self.data = {"time": deque([0.0, 0.005, ...]), "theta_x": deque([0.1, 0.12, ...])}
        self.max_points = 1000  # 5 seconds at 200 Hz
        self.data = {}
        
        # Display mode: 'diagnosis' or 'pid_tuning' or 'imu'
        self.display_mode = 'diagnosis'
        self.imu_mode_active = False

        # Create Qt application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)

        # Connect to server
        self.connect_to_server()

        # Setup will happen immediately (not waiting for headers)
        self.win = None
        self.plots = {}
        self.curves = {}
        
        # Show UI immediately with "waiting" message
        self.setup_ui_shell()

        # Timer for reading data
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.read_and_update)
        self.timer.start(20)  # 50 Hz update rate
        
        # Timer for reconnection attempts if not connected
        self.reconnect_timer = QtCore.QTimer()
        self.reconnect_timer.timeout.connect(self.try_reconnect)
        self.reconnect_timer.start(2000)  # Try every 2 seconds

    def connect_to_server(self):
        """Connect to DataLogger3 TCP server on robot
        
        Connection Process:
        1. Create TCP socket (IPv4, stream-based)
        2. Connect to robot at (host, port)
        3. Switch to non-blocking mode for reading (prevents UI freezing)
        
        Non-blocking mode is crucial for real-time plotting:
        - With blocking: recv() waits indefinitely, freezes UI
        - With non-blocking: recv() returns immediately, raises BlockingIOError if no data
        
        If connection fails, self.sock is set to None, triggering auto-reconnect attempts
        by try_reconnect() which runs every 2 seconds.
        """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket
            self.sock.connect((self.host, self.port))  # Connect to robot
            self.sock.setblocking(False)  # Non-blocking mode for recv()
            print(f"✓ Connected to {self.host}:{self.port}")
            self.connection_status = "Connected"
        except Exception as e:
            print(f"⚠ Connection failed: {e}")
            print(f"Waiting for data... Make sure DataLogger3 is running on {self.host}:{self.port}")
            self.sock = None  # Trigger auto-reconnect
            self.connection_status = "Waiting for connection..."
    
    def try_reconnect(self):
        """Attempt automatic reconnection if socket is closed
        
        Called periodically by reconnect_timer (every 2 seconds).
        Only attempts reconnection if self.sock is None (no active connection).
        
        Auto-reconnection is useful when:
        - Robot reboots and DataLogger3 restarts
        - Network temporarily drops connection
        - User stops/restarts control script on robot
        
        Failures are silent to avoid spamming console with error messages.
        """
        if self.sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket
                self.sock.connect((self.host, self.port))  # Connect to robot
                self.sock.setblocking(False)  # Non-blocking mode
                print(f"✓ Connected to {self.host}:{self.port}")
                self.connection_status = "Connected"
                # Update UI status to show successful reconnection (green indicator)
                if hasattr(self, 'status_label'):
                    self.status_label.setText(f"Status: {self.connection_status}")
                    self.status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
            except:
                pass  # Silently fail and try again in 2 seconds

    def setup_ui_shell(self):
        """Create UI window structure before data arrives
        
        UI Components:
        1. Top control panel:
           - Display mode selector (diagnosis/PID tuning/IMU)
           - Connection status indicator (changes color: orange → green)
        2. Main graphics widget:
           - Empty PyQtGraph container (plots created after headers arrive)
           - "Waiting for data..." placeholder message
        
        The window is shown immediately, even without data, so users can see
        the connection status. Actual plots are created by setup_plots_from_headers()
        after the first header message arrives.
        """
        # Main window with vertical layout
        self.win = QtWidgets.QWidget()
        self.win.resize(1600, 900)  # Wide window for multiple plots
        self.win.setWindowTitle(f'DataLogger3 Viewer - {self.host}:{self.port}')
        
        layout = QtWidgets.QVBoxLayout()
        self.win.setLayout(layout)
        
        # TOP PANEL: Control panel at top
        control_panel = QtWidgets.QHBoxLayout()
        
        # Display mode selector dropdown
        mode_label = QtWidgets.QLabel("Display Mode:")
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(['Diagnosis (All Plots)', 'PID Tuning (Error & Control)', 'IMU (2D Level)'])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)  # Callback when changed
        
        # Connection status indicator (changes color based on state)
        self.status_label = QtWidgets.QLabel(f"Status: {self.connection_status}")
        self.status_label.setStyleSheet("QLabel { color: orange; font-weight: bold; }")
        
        control_panel.addWidget(mode_label)
        control_panel.addWidget(self.mode_combo)
        control_panel.addStretch()
        control_panel.addWidget(self.status_label)
        
        layout.addLayout(control_panel)
        
        # Graphics layout for plots
        self.graphics_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_widget)
        
        # Add "Waiting for data..." message
        self.waiting_text = pg.TextItem("Waiting for data...\n\nMake sure DataLogger3 is running", 
                                        anchor=(0.5, 0.5), 
                                        color=(200, 200, 200))
        self.graphics_widget.addItem(self.waiting_text)
        self.waiting_text.setPos(0, 0)
        
        self.win.show()
        
        print(f"✓ UI created")
    
    def setup_plots_from_headers(self):
        """Create plots after receiving headers from DataLogger3
        
        Called once when the first header message arrives.
        Removes the "Waiting for data..." placeholder and creates the actual plots.
        """
        # Remove waiting message (no longer needed)
        if hasattr(self, 'waiting_text'):
            self.graphics_widget.removeItem(self.waiting_text)
        
        # Create initial plots based on selected mode
        self.create_plots()
        
        print(f"✓ Plots created with {len(self.curves)} curves")

    def create_plots(self):
        """Create plots based on current display mode
        
        Three display modes:
        1. 'diagnosis': Show all signals in separate subplots (good for debugging)
        2. 'pid_tuning': Show grouped X/Y error and control signals (good for tuning)
        3. 'imu': Show 2D bubble level display (theta_x vs theta_y)
        
        Called when:
        - Headers first arrive (setup_plots_from_headers)
        - User changes display mode (on_mode_changed)
        """
        # Clear existing plots and curves
        self.graphics_widget.clear()
        self.plots = {}  # Maps header name -> PyQtGraph PlotItem
        self.curves = {}  # Maps header name -> PyQtGraph PlotDataItem (curve)
        self.imu_mode_active = False  # Flag for special IMU update logic
        
        # Dispatch to mode-specific plot creation
        if self.display_mode == 'diagnosis':
            self.create_diagnosis_plots()  # All signals in grid
        elif self.display_mode == 'pid_tuning':
            self.create_pid_tuning_plots()  # Grouped error/control plots
        else:
            self.create_imu_plot()  # 2D bubble level
    
    def create_diagnosis_plots(self):
        """Create all plots for diagnosis mode (one subplot per signal)
        
        Layout: 2-column grid with all signals displayed
        - Row/column calculated from signal index
        - Each signal gets its own time-series plot
        - Useful for debugging: see all signals simultaneously
        
        Example with 8 signals:
        ┌──────────┬──────────┐
        │ Signal 1 │ Signal 2 │
        ├──────────┼──────────┤
        │ Signal 3 │ Signal 4 │
        ├──────────┼──────────┤
        │ Signal 5 │ Signal 6 │
        ├──────────┼──────────┤
        │ Signal 7 │ Signal 8 │
        └──────────┴──────────┘
        """
        # Create plots for each data column (skip iteration counter)
        num_plots = len(self.headers) - 1  # Skip first column (iteration counter 'i')
        cols = 2  # Fixed 2-column layout
        rows = (num_plots + cols - 1) // cols  # Calculate needed rows (ceiling division)

        for idx, header in enumerate(self.headers[1:], start=1):  # Skip 'i' column
            # Calculate grid position
            row = (idx - 1) // cols  # Integer division for row index
            col = (idx - 1) % cols   # Modulo for column index (0 or 1)

            # Create plot widget in grid
            plot = self.graphics_widget.addPlot(title=header, row=row, col=col)
            plot.setLabel('left', header)  # Y-axis label
            plot.setLabel('bottom', 'Time', units='s')  # X-axis label
            plot.showGrid(x=True, y=True, alpha=0.3)  # Gridlines for readability
            plot.setDownsampling(mode='peak')  # Downsample for performance (keeps peaks visible)
            plot.setClipToView(True)  # Only render visible data (faster)

            # Create curve (line plot) with blue color
            curve = plot.plot(pen=pg.mkPen(color=(100, 200, 255), width=2))

            # Store references for updating in read_and_update()
            self.plots[header] = plot
            self.curves[header] = curve
    
    def create_pid_tuning_plots(self):
        """Create focused plots for PID tuning mode (grouped error and control signals)
        
        Layout: 2x2 grid separating X and Y axes, error and control
        ┌─────────────────┬─────────────────┐
        │ X-Axis Error    │ Y-Axis Error    │
        │ (theta_x)       │ (theta_y)       │
        ├─────────────────┼─────────────────┤
        │ X-Axis Control  │ Y-Axis Control  │
        │ (Tx, u1)        │ (Ty, u2)        │
        └─────────────────┴─────────────────┘
        
        Signal Categorization (ballbot-specific):
        - theta_x, theta_y: body tilt angles → error signals
        - Tx, Ty: control torques → control signals
        - u1, u2, u3: motor PWM outputs → control signals
        - psi, dpsi: wheel angles/velocities → excluded (inner loop)
        
        Benefits for PID tuning:
        - See error and control effort side-by-side
        - Compare X and Y axis responses
        - Identify oscillations, overshoot, steady-state error
        - Zero reference line helps assess tracking performance
        """
        # Ballbot-specific signal patterns (documented for reference)
        # - theta_x, theta_y = body angles (errors for balance control)
        # - Tx, Ty = control torques
        # - u1, u2, u3 = motor control outputs
        # - psi = wheel angles, dpsi = wheel velocities
        
        # Signal category lists
        x_error_signals = []
        x_control_signals = []
        y_error_signals = []
        y_control_signals = []
        other_signals = []
        
        print(f"  Analyzing {len(self.headers)-1} signals for PID tuning mode...")
        
        # Categorize signals based on ballbot naming conventions
        for header in self.headers[1:]:  # Skip iteration counter 'i'
            header_lower = header.lower()
            
            # X-axis signals (forward/backward)
            if header_lower == 'theta_x':
                x_error_signals.append(header)
                print(f"    ✓ {header} → X-axis error (body angle)")
            elif header_lower in ['tx', 'u1']:
                x_control_signals.append(header)
                print(f"    ✓ {header} → X-axis control")
            
            # Y-axis signals (left/right)
            elif header_lower == 'theta_y':
                y_error_signals.append(header)
                print(f"    ✓ {header} → Y-axis error (body angle)")
            elif header_lower in ['ty', 'u2']:
                y_control_signals.append(header)
                print(f"    ✓ {header} → Y-axis control")
            
            # Wheel state signals (excluded from outer loop tuning view)
            elif 'psi' in header_lower or 'dpsi' in header_lower:
                print(f"    ~ {header} → Wheel state (excluded for outer loop tuning)")
            
            # Everything else (excluded)
            else:
                print(f"    ✗ {header} → Excluded from PID mode")
        
        # Fallback: if no ballbot-specific signals, use generic error/control keywords
        if not (x_error_signals or x_control_signals or y_error_signals or y_control_signals):
            print("⚠ No axis-specific signals detected, using generic grouping")
            self.create_generic_pid_plots()
            return
        
        # Create 2x2 grid layout
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
        """Fallback: Create generic error/control plots without axis separation
        
        Used when ballbot-specific signals (theta_x, theta_y, Tx, Ty) are not detected.
        Falls back to keyword-based signal categorization:
        
        Error keywords: 'error', 'err', 'setpoint', 'reference', 'target'
        Control keywords: 'control', 'effort', 'output', 'cmd', 'command', 'torque', 'voltage', 'pwm'
        
        Layout: 2 stacked plots
        ┌─────────────────┐
        │ Error Signals   │
        │ (all errors)    │
        ├─────────────────┤
        │ Control Signals │
        │ (all controls)  │
        └─────────────────┘
        
        If no error/control keywords found, falls back to diagnosis mode (all signals).
        """
        # Keyword lists for signal categorization
        error_keywords = ['error', 'err', 'setpoint', 'reference', 'target']
        control_keywords = ['control', 'effort', 'output', 'cmd', 'command', 'torque', 'voltage', 'pwm']
        
        error_signals = []
        control_signals = []
        
        # Categorize signals by keywords in header name
        for header in self.headers[1:]:  # Skip iteration counter
            header_lower = header.lower()
            if any(kw in header_lower for kw in error_keywords):
                error_signals.append(header)
            elif any(kw in header_lower for kw in control_keywords):
                control_signals.append(header)
        
        # If no categorization possible, fall back to diagnosis mode
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
    
    def create_imu_plot(self):
        """Create 2D bubble level plot for IMU visualization
        
        Displays robot orientation as a bubble level:
        - X-axis: theta_x (forward/backward tilt)
        - Y-axis: theta_y (left/right tilt)
        - Bubble (green dot): current orientation
        - Center (0, 0): perfectly balanced/upright
        
        Visual elements:
        - Concentric circles: tilt angle thresholds (30°, 60°, 90°, etc.)
        - Crosshairs: X/Y axes
        - Diagonal lines: 45° reference lines
        
        This is a 2D phase plot (not time-series), showing instantaneous state.
        Useful for:
        - Quickly assessing if robot is balanced
        - Visualizing tilt magnitude and direction
        - Monitoring IMU calibration
        """
        print("  Creating IMU 2D level display...")
        
        # Create single centered plot (no time-series, just 2D state space)
        self.imu_plot = self.graphics_widget.addPlot(title='IMU 2D Level (theta_x, theta_y)', row=0, col=0)
        self.imu_plot.setLabel('left', 'theta_y', units='rad')  # Left/right tilt
        self.imu_plot.setLabel('bottom', 'theta_x', units='rad')  # Forward/backward tilt
        
        # Set fixed range from -π to π (covers full tilt range)
        import math
        self.imu_plot.setXRange(-math.pi, math.pi)
        self.imu_plot.setYRange(-math.pi, math.pi)
        self.imu_plot.setAspectLocked(True)  # Equal scaling for X and Y (circle looks circular)
        
        # Add grid for readability
        self.imu_plot.showGrid(x=True, y=True, alpha=0.3)
        
        # Draw concentric circles at different tilt angles (like physical bubble level)
        # Circles at 30°, 60°, 90°, 120°, 150° (π/6, π/3, π/2, 2π/3, 5π/6)
        angles = [math.pi/6, math.pi/3, math.pi/2, 2*math.pi/3, 5*math.pi/6]
        for angle in angles:
            # QGraphicsEllipseItem(x, y, width, height) - centered at (0,0)
            circle = pg.QtWidgets.QGraphicsEllipseItem(-angle, -angle, 2*angle, 2*angle)
            circle.setPen(pg.mkPen(color=(100, 100, 100), width=1))
            self.imu_plot.addItem(circle)
        
        # Add crosshairs at center (X and Y axes)
        self.imu_plot.addLine(x=0, pen=pg.mkPen(color=(150, 150, 150), width=2))  # Vertical line
        self.imu_plot.addLine(y=0, pen=pg.mkPen(color=(150, 150, 150), width=2))  # Horizontal line
        
        # Add diagonal reference lines (45° and 135° angles)
        line_len = math.pi
        self.imu_plot.plot([-line_len, line_len], [-line_len, line_len],  # \ diagonal
                          pen=pg.mkPen(color=(100, 100, 100), width=1, style=2))
        self.imu_plot.plot([-line_len, line_len], [line_len, -line_len],  # / diagonal
                          pen=pg.mkPen(color=(100, 100, 100), width=1, style=2))
        
        # Create the "bubble" - a green dot that moves to show current orientation
        self.imu_bubble = pg.ScatterPlotItem(
            size=30,  # Pixel size
            pen=pg.mkPen(color=(0, 0, 0), width=2),  # Black border
            brush=pg.mkBrush(color=(100, 255, 100, 200))  # Green fill (semi-transparent)
        )
        self.imu_plot.addItem(self.imu_bubble)
        
        # Initialize bubble at center (upright position)
        self.imu_bubble.setData([0], [0])
        
        # Track that we're in IMU mode (uses different update logic in read_and_update)
        # Instead of updating curves with time-series data, we move the bubble position
        self.imu_mode_active = True
        
        print("✓ IMU mode: 2D level display created")
    
    def on_mode_changed(self, index):
        """Handle display mode selection change from dropdown
        
        Dropdown options:
        0: 'Diagnosis (All Plots)' → show all signals in grid
        1: 'PID Tuning (Error & Control)' → grouped X/Y error/control plots
        2: 'IMU (2D Level)' → 2D bubble level display
        
        When user changes mode:
        1. Update self.display_mode string
        2. Call create_plots() to rebuild UI with new layout
        3. Existing data in self.data is preserved (only UI changes)
        """
        # Map dropdown index to internal mode string
        if index == 0:
            self.display_mode = 'diagnosis'
        elif index == 1:
            self.display_mode = 'pid_tuning'
        else:
            self.display_mode = 'imu'
        
        print(f"Switching to {self.display_mode} mode...")
        self.create_plots()  # Rebuild UI with new layout

    def read_and_update(self):
        """Read data from socket and update plots
        
        This is the heart of the real-time plotting system, called every 20ms by QTimer.
        
        Data Flow:
        1. Read raw bytes from TCP socket (non-blocking)
        2. Decode UTF-8 and append to buffer
        3. Split buffer into complete JSON lines (separated by \n)
        4. Parse each JSON line and extract header/data messages
        5. Update internal data buffers (deques)
        6. Refresh PyQtGraph plots with new data
        
        The socket is non-blocking, so recv() returns immediately even if no data is available.
        Incomplete JSON messages stay in self.buffer until the next call.
        """
        if not self.sock:
            return

        try:
            # Read up to 8192 bytes from socket (non-blocking)
            chunk = self.sock.recv(8192).decode('utf-8')
            if chunk:
                self.buffer += chunk  # Append to buffer (may contain partial JSON)

            # Process complete lines (each line is a JSON message)
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)  # Split on first newline
                try:
                    msg = json.loads(line)  # Parse JSON message
                    self.process_message(msg)  # Handle header or data message
                except json.JSONDecodeError:
                    continue  # Skip malformed JSON
        except BlockingIOError:
            pass  # No data available right now (expected for non-blocking socket)
        except ConnectionResetError:
            print("✗ Connection lost to robot")
            self.sock = None
            return
        except Exception as e:
            print(f"Read error: {e}")
            self.sock = None
            return

        # Update plots if we have UI and data
        if self.win and self.data:
            if self.imu_mode_active:
                # IMU mode: Update bubble position in 2D level display
                # Shows current orientation as (theta_x, theta_y) point
                if 'theta_x' in self.data and 'theta_y' in self.data:
                    if len(self.data['theta_x']) > 0 and len(self.data['theta_y']) > 0:
                        theta_x = self.data['theta_x'][-1]  # Get most recent value
                        theta_y = self.data['theta_y'][-1]
                        self.imu_bubble.setData([theta_x], [theta_y])  # Move bubble
            else:
                # Normal time-series plotting mode: update all curves
                time_key = self.headers[1] if len(self.headers) > 1 else 'time'  # Usually 't_now'
                if time_key in self.data and len(self.data[time_key]) > 0:
                    time_data = np.array(self.data[time_key])  # X-axis (time)

                    # Update each curve with new data
                    for header in self.headers[1:]:
                        if header in self.curves and header in self.data:
                            y_data = np.array(self.data[header])  # Y-axis (signal)
                            if len(y_data) > 0:
                                self.curves[header].setData(time_data, y_data)  # Redraw curve

    def process_message(self, msg):
        """Process incoming JSON message from DataLogger3
        
        Message Types:
        1. Header message (sent once at start):
           {"type": "header", "headers": ["i", "t_now", "theta_x", "theta_y", ...]}
           - Defines column names for data
           - Triggers plot creation based on headers
           
        2. Data message (sent continuously at 200 Hz):
           {"type": "data", "values": [123, 0.615, 0.05, -0.03, ...]}
           - Contains numeric values matching header order
           - Appended to self.data deques for plotting
        
        The viewer must receive headers before it can process data.
        """
        if msg['type'] == 'header':
            # First message: defines column names
            self.headers = msg['headers']
            print(f"✓ Received headers: {self.headers}")

            # Initialize data buffers (one deque per column)
            # maxlen=1000 → keep last 1000 points (5 seconds at 200 Hz)
            for header in self.headers:
                self.data[header] = deque(maxlen=self.max_points)

            # Create plots now that we know what columns exist
            self.setup_plots_from_headers()
            
            # Update status indicator to green
            self.connection_status = "Connected - receiving data"
            if hasattr(self, 'status_label'):
                self.status_label.setText(f"Status: {self.connection_status}")
                self.status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")

        elif msg['type'] == 'data' and self.headers:
            # Data message: list of values matching header order
            values = msg['values']
            for header, value in zip(self.headers, values):
                if header in self.data:
                    try:
                        self.data[header].append(float(value))  # Add to deque
                    except (ValueError, TypeError):
                        pass  # Skip non-numeric values (shouldn't happen)
            
            # Debug: print confirmation for first few data points
            if len(self.data.get(self.headers[1] if len(self.headers) > 1 else 'time', [])) <= 3:
                print(f"✓ Receiving data: {len(values)} values")

    def run(self):
        """Start the Qt event loop (blocks until window closed)
        
        Qt Event Loop:
        - Processes UI events (mouse clicks, keyboard input, window resizing)
        - Executes timer callbacks (read_and_update every 20ms, try_reconnect every 2s)
        - Renders plots and handles redraws
        
        This call blocks until the user closes the window. When the window is closed,
        app.exec_() returns, and sys.exit() terminates the Python process cleanly.
        """
        sys.exit(self.app.exec_())


def main():
    """Main entry point for DataLogger3 viewer
    
    Command-line usage:
        python DataLogger3_viewer.py <robot_ip> [port]
    
    Examples:
        python DataLogger3_viewer.py 67.194.46.111           # Connect to robot, port 5557
        python DataLogger3_viewer.py 192.168.1.10 5558      # Custom port
        python DataLogger3_viewer.py                        # Use default IP (67.194.46.111)
    
    The viewer will continuously attempt to reconnect if the robot is not available.
    """
    # Check Python version first (requires 3.6+)
    check_python_version()
    
    print("=" * 60)
    print("  DataLogger3 Viewer - Standalone Real-time Plot Viewer")
    print(f"  Version {__version__}")
    print("=" * 60)
    print()
    
    # Parse command-line arguments
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <robot_ip> [port]")
        print(f"\nNo IP specified, using default: {DEFAULT_ROBOT_IP}")
        host = DEFAULT_ROBOT_IP
        port = DEFAULT_PORT
    else:
        host = sys.argv[1]  # First argument: robot IP address
        port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT  # Optional second argument: port

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