"""
===============================================================================
BALL-BOT CONTROL TEMPLATE (Student Starter File)
===============================================================================
This file is a BEGINNER-FRIENDLY scaffold for implementing ball-bot control.
You will gradually fill in the TODOs across labs (LAB-07, LAB-08, etc.).

KEY IDEAS THIS TEMPLATE REINFORCES:
    1. Separation of concerns (constants, helpers, I/O, main loop, shutdown)
    2. Reading sensor data via LCM and using callbacks
    3. Sending motor PWM commands over LCM
    4. Organizing computation into helper functions you will implement later
    5. Adding DEBUG prints safely without cluttering output

MESSAGE STRUCTURE (mbot_balbot_feedback_t):
------------------------------------------
struct mbot_balbot_feedback_t {
        int64_t utime;              # message timestamp (microseconds)
        int32_t enc_ticks[3];       # absolute positional ticks per motor
        int32_t enc_delta_ticks[3]; # ticks since last feedback update
        int32_t enc_delta_time;     # time between encoder updates [usec]
        float imu_angles_rpy[3];    # roll, pitch, yaw (radians)
        float volts[4];             # battery / supply voltages
}

HOW TO USE THIS FILE:
---------------------
1. Run it first and observe printed data changing.
2. Fill in the TODO functions one at a time (start with calc_enc2rad()).
3. Add control logic to produce Tx, Ty, Tz torques (LAB-07).
4. Convert torques to motor commands (u1,u2,u3).
5. Convert encoder ticks to wheel angles and ball kinematics (LAB-08).

DEBUGGING STRATEGY:
-------------------
We include a DEBUG flag and a dbg() helper. You can set DEBUG = False to
silence internal prints while keeping the user-facing summary output.

EMERGENCY STOP (ADVANCED):
--------------------------
You may later add an e-stop (L3+R3) similar to the reference demos. For now,
this template focuses on structure and readability.

IMPORTANT: Keep commits incremental—only add features you fully understand.
===============================================================================
"""

import time
import lcm
import threading
import numpy as np
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t
from mbot_lcm_msgs.mbot_balbot_feedback_t import mbot_balbot_feedback_t
from DataLogger import dataLogger
from ps4_controller_api import PS4InputHandler

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
# These values configure timing and physical parameters. Adjust ONLY if you
# understand their effect. They are used by your later computations.

# Constants for the control loop
FREQ = 200            # Control loop frequency [Hz]
DT = 1 / FREQ         # Time step for each iteration [sec]
PWM_MAX = 0.98        # Max motor effort (keep <= 1 for hardware safety)
N_GEARBOX = 70        # Motor gearbox ratio (motor revs : wheel rev)
N_ENC = 64            # Encoder ticks per motor shaft revolution
R_W = 0.048           # Omni-wheel radius [m]
R_K = 0.121           # Ball radius [m]

# Debug flag – set to False to silence dbg() output (keeps template clean)
DEBUG = True

def dbg(msg: str):
    """Conditional debug print to avoid cluttering console."""
    if DEBUG:
        print(f"[DEBUG] {msg}")

# ============================================================================
# GLOBAL STATE (LCM feedback handling)
# ============================================================================
# These globals hold the most recent feedback message and status flags used by
# the listener thread. You could wrap them in a class later for cleaner design.
listening = False
msg = mbot_balbot_feedback_t()
last_time = 0
last_seen = {"MBOT_BALBOT_FEEDBACK": 0}

def feedback_handler(channel, data):
    """LCM CALLBACK: Decodes incoming feedback message and updates globals.

    This function runs inside the LCM listener thread context whenever a
    message on the subscribed channel arrives.
    """
    global msg
    global last_seen
    global last_time
    last_time = time.time()
    last_seen[channel] = time.time()
    msg = mbot_balbot_feedback_t.decode(data)

def lcm_listener(lc):
    """THREAD LOOP: Polls LCM for messages and monitors activity.

    Uses a timeout to periodically check if messages have stopped arriving.
    Helpful for detecting disconnects or stalled publishers.
    """
    global listening
    while listening:
        try:
            lc.handle_timeout(100)  # 100ms timeout
            if time.time() - last_time > 2.0:
                print("LCM Publisher seems inactive...")
            elif time.time() - last_seen["MBOT_BALBOT_FEEDBACK"] > 2.0:
                print("LCM MBOT_BALBOT_FEEDBACK node seems inactive...")
        except Exception as e:
            print(f"LCM listening error: {e}")
            break


def calc_enc2rad(ticks: int) -> float:
    """Convert encoder ticks to wheel angle [rad].

    TODO [LAB-08]: Implement:
        motor_revs = ticks / N_ENC
        wheel_revs = motor_revs / N_GEARBOX
        angle_rad  = wheel_revs * 2*pi
    Return angle_rad
    """
    rad = 0.0  # Placeholder until implemented
    return rad

def calc_torque_conv(Tx: float, Ty: float, Tz: float):
    """Map body-frame torques (Tx,Ty,Tz) to individual motor efforts.

    TODO [LAB-07]: Derive transformation matrix from geometry.
    Example (pseudo): [u1,u2,u3]^T = M * [Tx,Ty,Tz]^T
    Return (u1,u2,u3)
    """
    u1 = 0.0
    u2 = 0.0
    u3 = 0.0
    return u1, u2, u3

def calc_kinematic_conv(psi1: float, psi2: float, psi3: float):
    """Convert wheel angles to ball angular displacement (phix,phiy,phiz).

    TODO [LAB-08]: Use ball/wheel geometry and rolling constraints.
    Return (phix, phiy, phiz)
    """
    phix = 0.0
    phiy = 0.0
    phiz = 0.0
    return phix, phiy, phiz

def func_clip(x: float, lim_lo: float, lim_hi: float) -> float:
    """Clamp value into [lim_lo, lim_hi] without modifying arguments."""
    if x > lim_hi:
        return lim_hi
    if x < lim_lo:
        return lim_lo
    return x


def main():
    # ========================================================================
    # DATA LOGGING SETUP
    # ========================================================================
    # Ask user for a test number so each run produces a unique output file.
    trial_num = int(input("Test Number? "))
    filename = f"ballbot_control_{trial_num}.txt"
    dl = dataLogger(filename)
    
    # ========================================================================
    # LCM INITIALIZATION (Messaging Backbone)
    # ========================================================================
    # Create LCM instance and subscribe to feedback channel. A separate
    # listener thread will keep updating global 'msg'.
    global listening
    global msg
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    subscription = lc.subscribe("MBOT_BALBOT_FEEDBACK", feedback_handler)
    # Start a separate thread for reading LCM data
    listening = True
    listener_thread = threading.Thread(target=lcm_listener, args=(lc,), daemon=True)
    listener_thread.start()
    print("Started continuous LCM listener...")

    # ========================================================================
    # CONTROLLER INITIALIZATION (User Input)
    # ========================================================================
    # The PS4 controller provides steering / balancing inputs. You can expand
    # usage (e.g., gain tuning) as you progress through labs.
    controller = PS4InputHandler(interface="/dev/input/js0",connecting_using_ds4drv=False)
    # Start a separate thread to listen for controller inputs
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True  # Ensures the thread stops with the main program
    controller_thread.start()
    print("PS4 Controller is active...")

    try:
        command = mbot_motor_pwm_t()
        # ====================================================================
        # MAIN CONTROL LOOP INTRO
        # ====================================================================
        print("Starting steering control loop...")
        time.sleep(1.0)
        # Store variable names as header to data logged, for easier parsing in Matlab
        # TODO [IF DESIRED]: Update data header variables names to match actual data logged (at end of loop)
        data = ["i t_now Tx Ty Tz u1 u2 u3 theta_x theta_y theta_z psi_1 psi_2 psi_3 dpsi_1 dpsi_2 dpsi_3"]
        dl.appendData(data)
        i = 0  # Iteration counter
        t_start = time.time()
        t_now = 0
        enc_pos_1_start = msg.enc_ticks[0]
        enc_pos_2_start = msg.enc_ticks[1]
        enc_pos_3_start = msg.enc_ticks[2]
        u1 = 0
        u2 = 0
        u3 = 0
        theta_x_0 = msg.imu_angles_rpy[0]
        theta_y_0 = msg.imu_angles_rpy[1]
        theta_z_0 = msg.imu_angles_rpy[2]

        # ====================================================================
        # CONTROL LOOP
        # ====================================================================
        # Each iteration:
        #   1) Sleep until next tick
        #   2) Read controller + sensor data
        #   3) Compute desired torques (Tx,Ty,Tz)
        #   4) Convert to motor efforts (u1,u2,u3)
        #   5) Publish PWM
        #   6) Log and optionally print
        while True:
            time.sleep(DT)                # 1) Timing
            t_now = time.time() - t_start # Elapsed time
            i += 1                        # Iteration counter

            try:
                # 2) CONTROLLER INPUTS
                # Retrieve dictionary of button/analog signals from handler.
                bt_signals = controller.get_signals()
                # parse out individual buttons you want data from
                js_R_x = bt_signals["js_R_x"]   # steering bot (XY) with js_R
                js_R_y = bt_signals["js_R_y"]
                trigger_L2 = bt_signals["trigger_L2"]   # spinning bot (Z) with L2/R2 triggers
                trigger_R2 = bt_signals["trigger_R2"]

                # Pull sensor data
                # 3) SENSOR DATA (IMU & ENCODERS)
                theta_x = msg.imu_angles_rpy[0] - theta_x_0  # roll
                theta_y = msg.imu_angles_rpy[1] - theta_y_0  # pitch
                theta_z = msg.imu_angles_rpy[2] - theta_z_0  # yaw
                enc_pos_1 = msg.enc_ticks[0] - enc_pos_1_start
                enc_pos_2 = msg.enc_ticks[1] - enc_pos_2_start
                enc_pos_3 = msg.enc_ticks[2] - enc_pos_3_start
                enc_dtick_1 = msg.enc_delta_ticks[0]
                enc_dtick_2 = msg.enc_delta_ticks[1]
                enc_dtick_3 = msg.enc_delta_ticks[2]
                enc_dt = msg.enc_delta_time


                # Calculate motor angles from encoder ticks
                # TODO [LAB-08]: Call method to calculate motor angles & speeds from measured encoder values
                psi_1 = 0.0
                psi_2 = 0.0
                psi_3 = 0.0
                dpsi_1 = 0.0
                dpsi_2 = 0.0
                dpsi_3 = 0.0

                # Calculate ball's roll and translation through kinematic conversions of wheel data
                # TODO [LAB-08]: Call method to calculate kinematic conversion of encoder-angles to ball-translations


                # Set x-y-z bot commands
                # TODO [LAB-07 & LAB-08]: Choose how Tx,Ty,Tz are set
                Tx = 0.0
                Ty = 0.0
                Tz = 0.0

                # Calculate motor effort/commands from desired Tx,Ty,Tz motion
                # TODO [LAB-07]: Call method to calculate motor commands (u1,u2,u3) from axis torques (Tx,Ty,Tz)



                
                # Send individual motor commands
                # Clip motor efforts for safety before sending
                u1 = func_clip(u1,-PWM_MAX,PWM_MAX)
                u2 = func_clip(u2,-PWM_MAX,PWM_MAX)
                u3 = func_clip(u3,-PWM_MAX,PWM_MAX)
                cmd_utime = int(time.time() * 1e6)
                command.utime = cmd_utime
                command.pwm[0] = u1
                command.pwm[1] = u2
                command.pwm[2] = u3
                lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                
                # Store data in data logger
                # TODO [IF DESIRED]: Update variables to match data header names for logging
                data = [i, t_now, Tx, Ty, Tz, u1, u2, u3, theta_x, theta_y, theta_z, psi_1, psi_2, psi_3, dpsi_1, dpsi_2, dpsi_3]
                dl.appendData(data)
                
                # NOTE: err_x / err_y referenced previously were undefined.
                # If you implement a PID later, track previous errors here:
                # err_x_prev = err_x
                # err_y_prev = err_y

                # Print out data in terminal
                # TODO: [IF DESIRED]: Update for what info you want to see in terminal (note: this is only printed data, not logged!)
                print(
                    f"Time: {t_now:.3f}s | Tx: {Tx:.2f}, Ty: {Ty:.2f}, Tz: {Tz:.2f} | "
                    f"u1: {u1:.2f}, u2: {u2:.2f}, u3: {u3:.2f} | "
                    f"Theta X: {theta_x:.2f}, Theta Y: {theta_y:.2f}, Theta Z: {theta_z:.2f} | "
                    f"Psi 1: {psi_1:.1f}, Psi 2: {psi_2:.1f}, Psi 3: {psi_3:.1f} | "
                    f"dPsi 1: {dpsi_1:.2f}, dPsi 2: {dpsi_2:.2f}, dPsi 3: {dpsi_3:.2f}"
                )
            
            except KeyError:
                print("Waiting for sensor data...")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Stopping motors...")
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
        # Stop Bluetooth thread
        controller_thread.join(timeout=1)  # Wait up to 1 second for thread to finish
        # Avoid forcing sys.exit from controller API here; threads are daemon.
        # Stop motors
        print("Shutting down motors...\n")
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())

if __name__ == "__main__":
    main()