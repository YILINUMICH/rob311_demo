"""
Ballbot PID Tuning Viewer (Laptop Side)

This script runs ON YOUR LAPTOP to display real-time plots from the robot.
The robot streams data over TCP socket, avoiding web server/firewall issues.

Usage:
    python ballbot_pid_viewer.py <robot_hostname_or_ip>
    
Example:
    python ballbot_pid_viewer.py mbot
    python ballbot_pid_viewer.py 192.168.1.100
"""

import sys
import socket
import json
import subprocess

# Check and auto-install dependencies
def check_and_install_dependencies():
    """Check for required packages and install if missing"""
    missing_packages = []
    
    # Check numpy
    try:
        import numpy as np
    except ImportError:
        missing_packages.append('numpy')
    
    # Check pyqtgraph and PyQt5
    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except ImportError:
        missing_packages.extend(['pyqtgraph', 'PyQt5'])
    
    # Install missing packages
    if missing_packages:
        print(f"Missing packages detected: {', '.join(missing_packages)}")
        print("Installing dependencies...")
        try:
            subprocess.check_call([sys.executable, '-m', 'pip', 'install'] + missing_packages)
            print("✓ Installation complete! Please restart the script.")
            sys.exit(0)
        except subprocess.CalledProcessError as e:
            print(f"✗ Installation failed: {e}")
            print("Please install manually with:")
            print(f"  pip3 install {' '.join(missing_packages)}")
            sys.exit(1)

# Run dependency check
check_and_install_dependencies()

# Import after verification
import numpy as np
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

PORT = 5556  # Different from IMU test to avoid conflicts
DEFAULT_ROBOT_HOST = "67.194.46.111"  # Default robot IP address

class BallbotPIDViewer:
    """Real-time viewer for ballbot PID tuning data"""
    
    def __init__(self, robot_host, port=PORT):
        self.robot_host = robot_host
        self.port = port
        self.sock = None
        self.buffer = ""
        
        # Data buffers (5 seconds at 200 Hz = 1000 points)
        self.max_points = 1000
        self.data = {
            'time': deque(maxlen=self.max_points),
            # Angles
            'theta_x': deque(maxlen=self.max_points),
            'theta_y': deque(maxlen=self.max_points),
            'theta_d_x': deque(maxlen=self.max_points),
            'theta_d_y': deque(maxlen=self.max_points),
            # Errors
            'error_x': deque(maxlen=self.max_points),
            'error_y': deque(maxlen=self.max_points),
            # Control effort
            'Tx': deque(maxlen=self.max_points),
            'Ty': deque(maxlen=self.max_points),
            # Gyro rates
            'dtheta_x': deque(maxlen=self.max_points),
            'dtheta_y': deque(maxlen=self.max_points),
        }
        
        # Create Qt application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)
        
        # Setup UI
        self.setup_ui()
        
        # Setup socket connection
        self.connect_to_robot()
        
        # Timer for reading data
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.read_and_update)
        self.timer.start(20)  # 50 Hz update rate
        
    def setup_ui(self):
        """Create the plot window"""
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.win.resize(1600, 1000)
        self.win.setWindowTitle(f'Ballbot PID Tuning - Connected to {self.robot_host}')
        
        # Row 1: Angle tracking
        self.plot_theta_x = self.win.addPlot(title="X-Axis Angle Tracking", row=0, col=0)
        self.plot_theta_x.setLabel('left', 'Angle', units='deg')
        self.plot_theta_x.setLabel('bottom', 'Time', units='s')
        self.plot_theta_x.addLegend()
        self.plot_theta_x.showGrid(x=True, y=True, alpha=0.3)
        
        self.plot_theta_y = self.win.addPlot(title="Y-Axis Angle Tracking", row=0, col=1)
        self.plot_theta_y.setLabel('left', 'Angle', units='deg')
        self.plot_theta_y.setLabel('bottom', 'Time', units='s')
        self.plot_theta_y.addLegend()
        self.plot_theta_y.showGrid(x=True, y=True, alpha=0.3)
        
        # Row 2: Error signals
        self.plot_error_x = self.win.addPlot(title="X-Axis Error", row=1, col=0)
        self.plot_error_x.setLabel('left', 'Error', units='deg')
        self.plot_error_x.setLabel('bottom', 'Time', units='s')
        self.plot_error_x.addLegend()
        self.plot_error_x.showGrid(x=True, y=True, alpha=0.3)
        
        self.plot_error_y = self.win.addPlot(title="Y-Axis Error", row=1, col=1)
        self.plot_error_y.setLabel('left', 'Error', units='deg')
        self.plot_error_y.setLabel('bottom', 'Time', units='s')
        self.plot_error_y.addLegend()
        self.plot_error_y.showGrid(x=True, y=True, alpha=0.3)
        
        # Row 3: Control effort
        self.plot_Tx = self.win.addPlot(title="X-Axis Control Effort", row=2, col=0)
        self.plot_Tx.setLabel('left', 'Control (Tx)', units='PWM')
        self.plot_Tx.setLabel('bottom', 'Time', units='s')
        self.plot_Tx.addLegend()
        self.plot_Tx.showGrid(x=True, y=True, alpha=0.3)
        
        self.plot_Ty = self.win.addPlot(title="Y-Axis Control Effort", row=2, col=1)
        self.plot_Ty.setLabel('left', 'Control (Ty)', units='PWM')
        self.plot_Ty.setLabel('bottom', 'Time', units='s')
        self.plot_Ty.addLegend()
        self.plot_Ty.showGrid(x=True, y=True, alpha=0.3)
        
        # Row 4: Phase plots (Control vs Error)
        self.plot_phase_x = self.win.addPlot(title="X-Axis Phase Plot", row=3, col=0)
        self.plot_phase_x.setLabel('left', 'Control (Tx)', units='PWM')
        self.plot_phase_x.setLabel('bottom', 'Error', units='deg')
        self.plot_phase_x.addLegend()
        self.plot_phase_x.showGrid(x=True, y=True, alpha=0.3)
        
        self.plot_phase_y = self.win.addPlot(title="Y-Axis Phase Plot", row=3, col=1)
        self.plot_phase_y.setLabel('left', 'Control (Ty)', units='PWM')
        self.plot_phase_y.setLabel('bottom', 'Error', units='deg')
        self.plot_phase_y.addLegend()
        self.plot_phase_y.showGrid(x=True, y=True, alpha=0.3)
        
        # Create plot curves
        # Angle tracking
        self.curve_theta_x = self.plot_theta_x.plot(pen=pg.mkPen('b', width=2), name='Actual')
        self.curve_theta_d_x = self.plot_theta_x.plot(pen=pg.mkPen('r', width=2, style=QtCore.Qt.DashLine), name='Desired')
        
        self.curve_theta_y = self.plot_theta_y.plot(pen=pg.mkPen('b', width=2), name='Actual')
        self.curve_theta_d_y = self.plot_theta_y.plot(pen=pg.mkPen('r', width=2, style=QtCore.Qt.DashLine), name='Desired')
        
        # Error signals
        self.curve_error_x = self.plot_error_x.plot(pen=pg.mkPen('r', width=2), name='Error X')
        self.curve_error_y = self.plot_error_y.plot(pen=pg.mkPen('r', width=2), name='Error Y')
        
        # Zero reference lines for errors
        self.plot_error_x.addLine(y=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        self.plot_error_y.addLine(y=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        
        # Control effort
        self.curve_Tx = self.plot_Tx.plot(pen=pg.mkPen('g', width=2), name='Tx')
        self.curve_Ty = self.plot_Ty.plot(pen=pg.mkPen('g', width=2), name='Ty')
        
        # Saturation lines
        self.plot_Tx.addLine(y=1.0, pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine))
        self.plot_Tx.addLine(y=-1.0, pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine))
        self.plot_Ty.addLine(y=1.0, pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine))
        self.plot_Ty.addLine(y=-1.0, pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine))
        
        # Phase plots
        self.curve_phase_x = self.plot_phase_x.plot(pen=None, symbol='o', symbolSize=3, 
                                                     symbolPen=None, symbolBrush=(0, 0, 255, 100), name='Trajectory')
        self.curve_phase_y = self.plot_phase_y.plot(pen=None, symbol='o', symbolSize=3,
                                                     symbolPen=None, symbolBrush=(0, 0, 255, 100), name='Trajectory')
        
        # Zero reference lines for phase plots
        self.plot_phase_x.addLine(x=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        self.plot_phase_x.addLine(y=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        self.plot_phase_y.addLine(x=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        self.plot_phase_y.addLine(y=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        
    def connect_to_robot(self):
        """Connect to robot's TCP server"""
        print(f"\nConnecting to robot at {self.robot_host}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.robot_host, self.port))
            self.sock.setblocking(False)
            print(f"✓ Connected successfully!")
            print("Waiting for data...\n")
        except socket.timeout:
            print(f"✗ Connection timeout - is the robot script running?")
            sys.exit(1)
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            sys.exit(1)
    
    def read_and_update(self):
        """Read data from socket and update plots"""
        try:
            # Read available data
            data = self.sock.recv(4096)
            if data:
                self.buffer += data.decode('utf-8')
                
                # Process complete JSON lines
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    try:
                        packet = json.loads(line)
                        self.process_packet(packet)
                    except json.JSONDecodeError:
                        pass  # Incomplete packet, will be completed next time
                        
        except BlockingIOError:
            pass  # No data available
        except Exception as e:
            print(f"Error reading data: {e}")
        
        # Update plots
        self.update_plots()
    
    def process_packet(self, packet):
        """Process incoming data packet"""
        self.data['time'].append(packet.get('t', 0))
        
        # Angles (already in degrees from robot)
        self.data['theta_x'].append(packet.get('theta_x_deg', 0))
        self.data['theta_y'].append(packet.get('theta_y_deg', 0))
        self.data['theta_d_x'].append(packet.get('theta_d_x_deg', 0))
        self.data['theta_d_y'].append(packet.get('theta_d_y_deg', 0))
        
        # Errors
        self.data['error_x'].append(packet.get('error_x_deg', 0))
        self.data['error_y'].append(packet.get('error_y_deg', 0))
        
        # Control effort
        self.data['Tx'].append(packet.get('Tx', 0))
        self.data['Ty'].append(packet.get('Ty', 0))
        
        # Gyro rates (for debugging)
        self.data['dtheta_x'].append(packet.get('dtheta_x_dps', 0))
        self.data['dtheta_y'].append(packet.get('dtheta_y_dps', 0))
    
    def update_plots(self):
        """Update all plot curves"""
        if len(self.data['time']) < 2:
            return
        
        t = np.array(self.data['time'])
        
        # Angle tracking
        self.curve_theta_x.setData(t, np.array(self.data['theta_x']))
        self.curve_theta_d_x.setData(t, np.array(self.data['theta_d_x']))
        
        self.curve_theta_y.setData(t, np.array(self.data['theta_y']))
        self.curve_theta_d_y.setData(t, np.array(self.data['theta_d_y']))
        
        # Errors
        self.curve_error_x.setData(t, np.array(self.data['error_x']))
        self.curve_error_y.setData(t, np.array(self.data['error_y']))
        
        # Control effort
        self.curve_Tx.setData(t, np.array(self.data['Tx']))
        self.curve_Ty.setData(t, np.array(self.data['Ty']))
        
        # Phase plots (error vs control)
        error_x = np.array(self.data['error_x'])
        error_y = np.array(self.data['error_y'])
        Tx = np.array(self.data['Tx'])
        Ty = np.array(self.data['Ty'])
        
        self.curve_phase_x.setData(error_x, Tx)
        self.curve_phase_y.setData(error_y, Ty)
    
    def run(self):
        """Start the application"""
        self.app.exec_()
        if self.sock:
            self.sock.close()


def main():
    # Use default robot host if not provided
    if len(sys.argv) < 2:
        robot_host = DEFAULT_ROBOT_HOST
        print(f"No hostname provided, using default: {robot_host}")
    else:
        robot_host = sys.argv[1]
    
    print("="*80)
    print("BALLBOT PID TUNING VIEWER")
    print("="*80)
    print(f"\nRobot: {robot_host}")
    print(f"Port: {PORT}")
    print("\n💡 Make sure the ballbot control script is running on the robot first!")
    print("="*80)
    
    try:
        viewer = BallbotPIDViewer(robot_host, PORT)
        viewer.run()
    except KeyboardInterrupt:
        print("\n\nViewer stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
