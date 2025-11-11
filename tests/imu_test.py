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
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t
from mbot_lcm_msgs.mbot_balbot_feedback_t import mbot_balbot_feedback_t

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.DataLogger import dataLogger

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

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
    
    # === Real-Time Plotting Initialization ===
    enable_plotting = input("Enable real-time plotting? (y/n): ").lower().strip() == 'y'
    plotter = None
    if enable_plotting:
        print("Initializing real-time plotter...")
        plotter = RealTimePlotter()
        print("Real-time plotting enabled!")
    
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
        data = ["i t_now Tx Ty Tz u1 u2 u3 theta_x theta_y theta_z psi_1 psi_2 psi_3 dpsi_1 dpsi_2 dpsi_3"]
        dl.appendData(data)
        Tx = 0
        Ty = 0
        Tz = 0
        u1 = 0
        u2 = 0
        u3 = 0

        while True:
            time.sleep(DT)
            t_now = time.time() - t_start  # Elapsed time
            i += 1

            try:
                # Pull sensor data
                theta_x = msg.imu_angles_rpy[0]
                theta_y = msg.imu_angles_rpy[1]
                theta_z = msg.imu_angles_rpy[2]
                psi_1 = 0
                psi_2 = 0
                psi_3 = 0
                dpsi_1 = 0
                dpsi_2 = 0
                dpsi_3 = 0
                
                # Store and printout data
                data = [i, t_now, Tx, Ty, Tz, u1, u2, u3, theta_x, theta_y, theta_z, psi_1, psi_2, psi_3, dpsi_1, dpsi_2, dpsi_3]
                dl.appendData(data)
                print(
                    f"Time: {t_now:.3f}s | Tx: {Tx:.2f}, Ty: {Ty:.2f}, Tz: {Tz:.2f} | "
                    f"u1: {u1:.2f}, u2: {u2:.2f}, u3: {u3:.2f} | "
                    f"Theta X: {theta_x:.2f}, Theta Y: {theta_y:.2f}, Theta Z: {theta_z:.2f} | "
                    f"Psi 1: {psi_1:.1f}, Psi 2: {psi_2:.1f}, Psi 3: {psi_3:.1f} | "
                    f"dPsi 1: {dpsi_1:.2f}, dPsi 2: {dpsi_2:.2f}, dPsi 3: {dpsi_3:.2f} | "
                )
                
                # Update plot data if plotting is enabled
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