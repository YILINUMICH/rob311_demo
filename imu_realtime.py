"""
ROB 311 - Fall 2025
Author: Prof. Greg Formosa
University of Michigan

Script is meant to test MBot Control Board IMU & LCM communications, without additional overhead.
Will store data as a "test_IMU_[#].txt" file that can be parsed in Matlab to show IMU angle data.
Run script, choose a test number to store data as (will overwrite if file already exists), and then
rotate board along multiple axes to take IMU readings. Press Ctrl+C to stop script at any time.

Note: IMU readings are not zeroed at each run; they are zeroed from calibration when Pico was originally booted up.

LCM data type for ROB 311 ball-bots:
struct mbot_balbot_feedback_t
{
    int64_t utime;
    int32_t enc_ticks[3];       // absolute postional ticks
    int32_t enc_delta_ticks[3]; // number of ticks since last step
    int32_t enc_delta_time;     // [usec]
    float imu_angles_rpy[3];    // [radian]
    float volts[4];             // volts
}

"""

import time
import lcm
import threading
import numpy as np
import sys
import os
import socket
import json
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t
from mbot_lcm_msgs.mbot_balbot_feedback_t import mbot_balbot_feedback_t

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.DataLogger import dataLogger

# Set Qt platform to offscreen if no display is available
# This prevents crashes when running via SSH or headless
if 'DISPLAY' not in os.environ or not os.environ['DISPLAY']:
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    PLOTTING_AVAILABLE = True
except ImportError:
    print("Warning: PyQtGraph not available. Real-time plotting disabled.")
    PLOTTING_AVAILABLE = False

# Constants for the control loop
FREQ = 200  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds
JOYSTICK_SCALE = 32767  # Scale factor for normalizing joystick values
N_GEARBOX = 70 # Motor gearbox ratio
N_ENC = 64 # Ticks per revolution of encoder

# Global flags to control the listening thread & msg data
listening = False
msg = mbot_balbot_feedback_t()

# Global data buffers for plotting
plot_data = {
    'time': [],
    'roll': [],   # theta_x
    'pitch': [],  # theta_y
    'yaw': []     # theta_z
}
MAX_PLOT_POINTS = 1000  # Maximum number of points to display

def feedback_handler(channel, data):
    """Callback function to handle received mbot_balbot_feedback_t messages"""
    global msg
    msg = mbot_balbot_feedback_t.decode(data)

def lcm_listener(lc):
    """Function to continuously listen for LCM messages in a separate thread"""
    global listening
    while listening:
        try:
            lc.handle_timeout(100)  # 100ms timeout
        except Exception as e:
            print(f"LCM listening error: {e}")
            break


class RealTimePlotter:
    """Class to handle real-time plotting of IMU angle data"""
    
    def __init__(self):
        # Create the application and main window
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)
        
        self.win = pg.GraphicsLayoutWidget(show=True, title="IMU Test Real-Time Data")
        self.win.resize(1200, 800)
        self.win.setWindowTitle('ROB 311 - IMU Test Real-Time Plotting')
        
        # Create plots for IMU angles
        self.roll_plot = self.win.addPlot(title="IMU Roll (θx)", row=0, col=0)
        self.roll_plot.setLabel('left', 'Angle', units='rad')
        self.roll_plot.setLabel('bottom', 'Time', units='s')
        self.roll_plot.addLegend()
        self.roll_plot.showGrid(x=True, y=True, alpha=0.3)
        
        self.pitch_plot = self.win.addPlot(title="IMU Pitch (θy)", row=1, col=0)
        self.pitch_plot.setLabel('left', 'Angle', units='rad')
        self.pitch_plot.setLabel('bottom', 'Time', units='s')
        self.pitch_plot.addLegend()
        self.pitch_plot.showGrid(x=True, y=True, alpha=0.3)
        
        self.yaw_plot = self.win.addPlot(title="IMU Yaw (θz)", row=2, col=0)
        self.yaw_plot.setLabel('left', 'Angle', units='rad')
        self.yaw_plot.setLabel('bottom', 'Time', units='s')
        self.yaw_plot.addLegend()
        self.yaw_plot.showGrid(x=True, y=True, alpha=0.3)
        
        # Create plot curves
        self.roll_curve = self.roll_plot.plot(pen=pg.mkPen('r', width=2), name='Roll')
        self.pitch_curve = self.pitch_plot.plot(pen=pg.mkPen('g', width=2), name='Pitch')
        self.yaw_curve = self.yaw_plot.plot(pen=pg.mkPen('b', width=2), name='Yaw')
        
        # Timer for updating plots
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)  # Update every 50ms (20 Hz update rate for display)
        
    def update_plots(self):
        """Update the plot data"""
        global plot_data
        
        if len(plot_data['time']) > 0:
            # Limit data points for performance
            if len(plot_data['time']) > MAX_PLOT_POINTS:
                for key in plot_data:
                    plot_data[key] = plot_data[key][-MAX_PLOT_POINTS:]
            
            # Update IMU angle plots
            self.roll_curve.setData(plot_data['time'], plot_data['roll'])
            self.pitch_curve.setData(plot_data['time'], plot_data['pitch'])
            self.yaw_curve.setData(plot_data['time'], plot_data['yaw'])
    
    def process_events(self):
        """Process Qt events to keep GUI responsive"""
        self.app.processEvents()


def main():
    # === Data Logging Initialization ===
    # Prompt user for trial number and create a data logger
    trial_num = int(input("Test Number? "))
    filename = f"test_IMU_{trial_num}.txt"
    dl = dataLogger(filename)
    
    # === Network Plotting Initialization ===
    enable_network_plot = False
    plot_socket = None
    plot_client = None
    
    user_input = input("Enable network plotting (view on laptop)? (y/n): ").lower().strip()
    enable_network_plot = user_input == 'y'
    
    if enable_network_plot:
        PORT = 5555
        plot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plot_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        plot_socket.bind(('0.0.0.0', PORT))
        plot_socket.listen(1)
        plot_socket.settimeout(10.0)
        
        print(f"\n{'='*80}")
        print("WAITING FOR LAPTOP CONNECTION")
        print(f"{'='*80}")
        print(f"On your laptop, run:")
        print(f"  python imu_viewer.py {socket.gethostname()}")
        print(f"  or")
        print(f"  python imu_viewer.py <robot_ip_address>")
        print(f"{'='*80}\n")
        
        try:
            plot_client, addr = plot_socket.accept()
            print(f"✓ Laptop connected from {addr[0]}:{addr[1]}")
            print("Starting data collection...\n")
        except socket.timeout:
            print("✗ No connection received within 10 seconds. Continuing without plotting...")
            enable_network_plot = False
            plot_socket.close()
            plot_socket = None
    
    # === Real-Time Plotting Initialization (fallback for local display) ===
    enable_plotting = False
    plotter = None
    
    if not enable_network_plot and PLOTTING_AVAILABLE:
        user_input = input("Enable local real-time plotting? (y/n): ").lower().strip()
        enable_plotting = user_input == 'y'
        
        if enable_plotting:
            try:
                print("Initializing real-time plotter...")
                plotter = RealTimePlotter()
                print("Real-time plotting enabled!")
            except Exception as e:
                print(f"Warning: Could not initialize plotter: {e}")
                print("Continuing without real-time plotting...")
                enable_plotting = False
    elif not enable_network_plot:
        print("Real-time plotting not available (PyQtGraph not installed or no display).")
    
    # === LCM Messaging Initialization ===
    # Initialize the serial communication protocol
    global listening
    global msg
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    subscription = lc.subscribe("MBOT_BALBOT_FEEDBACK", feedback_handler)
    # Start a separate thread for reading LCM data
    listening = True
    listener_thread = threading.Thread(target=lcm_listener, args=(lc,), daemon=True)
    listener_thread.start()
    print("Started continuous LCM listener...")

    try:
        command = mbot_motor_pwm_t()
        # === Main Control Loop ===
        print("Starting steering control loop...")
        i = 0  # Iteration counter
        t_start = time.time()
        enc_pos_1_start = msg.enc_ticks[0]
        enc_pos_2_start = msg.enc_ticks[1]
        enc_pos_3_start = msg.enc_ticks[2]
        data = ["i t_now theta_x theta_y theta_z"]
        dl.appendData(data)

        while True:
            time.sleep(DT)
            t_now = time.time() - t_start  # Elapsed time
            i += 1

            try:
                # Pull sensor data
                theta_x = msg.imu_angles_rpy[0]
                theta_y = msg.imu_angles_rpy[1]
                theta_z = msg.imu_angles_rpy[2]
                
                # Store and printout data
                data = [i, t_now, theta_x, theta_y, theta_z]
                dl.appendData(data)
                print(
                    f"Time: {t_now:.3f}s | "
                    f"Theta X: {theta_x:.4f} rad ({np.degrees(theta_x):.2f}°) | "
                    f"Theta Y: {theta_y:.4f} rad ({np.degrees(theta_y):.2f}°) | "
                    f"Theta Z: {theta_z:.4f} rad ({np.degrees(theta_z):.2f}°)"
                )
                
                # Send data to laptop if network plotting is enabled
                if enable_network_plot and plot_client:
                    try:
                        data_packet = {
                            'time': t_now,
                            'theta_x_rad': theta_x,
                            'theta_y_rad': theta_y,
                            'theta_z_rad': theta_z,
                            'theta_x_deg': np.degrees(theta_x),
                            'theta_y_deg': np.degrees(theta_y),
                            'theta_z_deg': np.degrees(theta_z)
                        }
                        # Send as JSON with newline delimiter
                        json_str = json.dumps(data_packet) + '\n'
                        plot_client.sendall(json_str.encode('utf-8'))
                    except (BrokenPipeError, ConnectionResetError):
                        print("✗ Laptop disconnected")
                        enable_network_plot = False
                        plot_client.close()
                        plot_client = None
                    except Exception as e:
                        print(f"Error sending plot data: {e}")
                
                # Update plot data if local plotting is enabled
                if enable_plotting:
                    plot_data['time'].append(t_now)
                    plot_data['roll'].append(theta_x)
                    plot_data['pitch'].append(theta_y)
                    plot_data['yaw'].append(theta_z)
                    plotter.process_events()
            
            except KeyError:
                print("Waiting for sensor data...")
                if enable_plotting:
                    plotter.process_events()

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Stopping all commands...")
        # Emergency stop
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
    
    finally:
        # Save/log data
        print(f"Saving data as {filename}...")
        dl.writeOut()  # Write logged data to the file
        
        # Close network plotting connection
        if plot_client:
            print("Closing network plot connection...")
            plot_client.close()
        if plot_socket:
            plot_socket.close()
        
        # Stop the listener thread
        listening = False
        print("Stopping LCM listener...")
        listener_thread.join(timeout=1)  # Wait up to 1 second for thread to finish
        
        # Keep plot window open if plotting was enabled
        if enable_plotting:
            print("Test complete! Close the plot window to exit.")
            plotter.app.exec_()

if __name__ == "__main__":
    main()