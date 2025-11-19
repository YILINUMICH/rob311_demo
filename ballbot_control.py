"""
ROB 311 - Fall 2025
Author: Prof. Greg Formosa & GSI Yilin Ma
University of Michigan

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

# Control how often to print status lines to the console (every N iterations)
PRINT_EVERY = 10      # With FREQ=200, prints ~20 lines/sec / 10 = 20 Hz

# Debug flag – set to False to silence dbg() output (keeps template clean)
DEBUG = True

def dbg(msg: str):
    """Conditional debug print to avoid cluttering console."""
    if DEBUG:
        print(f"[DEBUG] {msg}")

# ============================================================================
# GLOBAL VARIABLES FOR LCM COMMUNICATION
# ============================================================================
listening = False
msg = mbot_balbot_feedback_t()
last_time = 0
last_seen = {"MBOT_BALBOT_FEEDBACK": 0}

# ============================================================================
# LCM CALLBACK AND LISTENER
# ============================================================================
def feedback_handler(channel, data):
    """
    Callback function invoked when sensor feedback is received.
    
    Updates global variables with latest sensor data including:
    - IMU angles (roll, pitch, yaw)
    - Encoder positions and velocities
    - Battery voltage
    
    Troubleshooting:
    - If this doesn't execute, check LCM network configuration
    - Verify that the feedback channel name matches the publisher's channel
    """
    global msg
    global last_seen
    global last_time
    last_time = time.time()
    last_seen[channel] = time.time()
    msg = mbot_balbot_feedback_t.decode(data)

def lcm_listener(lc):
    """
    Continuously listens for LCM messages in a separate thread.
    
    Monitors connection health and warns if publishers become inactive.
    
    Troubleshooting:
    - "LCM Publisher seems inactive" = No messages received for 2+ seconds
    - Check physical connection to robot
    - Verify LCM network settings match robot configuration
    """
    global listening

    while listening:
        try:
            lc.handle_timeout(100)  # Wait up to 100ms for messages
            
            # Connection health monitoring
            if time.time() - last_time > 2.0:
                print("WARNING: LCM Publisher seems inactive...")
            elif time.time() - last_seen["MBOT_BALBOT_FEEDBACK"] > 2.0:
                print("WARNING: MBOT_BALBOT_FEEDBACK node seems inactive...")
                
        except Exception as e:
            print(f"ERROR: LCM listening error: {e}")
            break

# ============================================================================
# KINEMATIC AND DYNAMIC CONVERSION FUNCTIONS
# ============================================================================

# ============================================================================
# STUDENT TODO [LAB-08]: ENCODER -> RADIANS
# ============================================================================
def calc_enc2rad(ticks: int) -> float:
    """
    Args:
        ticks (int): Raw encoder count from the motor-side encoder.

    Returns:
        float: Wheel angular position [rad]. If you pass delta ticks / delta time
               you can compute angular velocity [rad/s] similarly.

    Notes:
        - Accounts for gearbox reduction and encoder resolution.
        - Angle sign convention should match your kinematic model.
    """
    rad = 0.0  # Placeholder until implemented
    return rad
# ============================================================================

# ============================================================================
# STUDENT TODO [LAB-07]: TORQUES -> MOTOR COMMANDS (u1,u2,u3)
# ============================================================================
def calc_torque_conv(Tx: float, Ty: float, Tz: float):
    """
    Args:
        Tx (float): Desired torque/effort along robot X (roll) axis in robot's frame.
        Ty (float): Desired torque/effort along robot Y (pitch) axis in robot's frame.
        Tz (float): Desired yaw torque about Z (rotation) axis in robot's frame.
        
    Returns:
        u1, u2, u3: Motor command signals for wheels 1, 2, 3
        
    Troubleshooting:
        - If you use trig, pi is np.pi, cos is np.cos, sin is np.sin.
    """
    u1 = 0.0    # Placeholder until implemented
    u2 = 0.0    # Placeholder until implemented
    u3 = 0.0    # Placeholder until implemented
    return u1, u2, u3
# ============================================================================

# ============================================================================
# STUDENT TODO [LAB-08]: ENCODER ODOMETRY -> BALL ANGLES (phix, phiy, phiz)
# ============================================================================
def calc_kinematic_conv(psi1: float, psi2: float, psi3: float):
    """
    Args:
        psi1 (float): Wheel 1 angle [rad]
        psi2 (float): Wheel 2 angle [rad]
        psi3 (float): Wheel 3 angle [rad]
        
    Returns:
        phix: Ball rotation about X-axis [rad]
        phiy: Ball rotation about Y-axis [rad]
        phiz: Ball rotation about Z-axis [rad]
        
    Notes:
        - Multiply by R_K to get linear displacement (x = phix * R_K).
        - Ensure your wheel-angle sign conventions match your geometry.
    """
    phix = 0.0  # Placeholder until implemented
    phiy = 0.0  # Placeholder until implemented
    phiz = 0.0  # Placeholder until implemented
    return phix, phiy, phiz
# ============================================================================

def func_clip(x: float, lim_lo: float, lim_hi: float) -> float:
    """
    Saturate a value to within [lim_lo, lim_hi].

    Why this matters:
        - Safety: prevents commanding the motors beyond allowed effort.
        - Control: helps avoid integrator windup in PID implementations.

    Args:
        x (float): Input value to clamp.
        lim_lo (float): Lower bound (inclusive).
        lim_hi (float): Upper bound (inclusive).

    Returns:
        float: Clipped value within [lim_lo, lim_hi].
    """
    if x > lim_hi:
        return lim_hi
    if x < lim_lo:
        return lim_lo
    return x

def apply_deadzone(x: float, deadzone: float) -> float:
    """
    Apply a deadzone filter to sensor or joystick readings.

    Values within ±deadzone are set to zero to filter out small noise.
    This prevents constant micro-corrections from tiny fluctuations.

    Args:
        x (float): Input value (sensor reading or joystick axis).
        deadzone (float): Deadzone threshold (positive value).

    Returns:
        float: 0.0 if |x| < deadzone, else the original value.
    """
    if abs(x) < deadzone:
        return 0.0
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
        # Store variable names as header to data logged, for easier parsing in MATLAB
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
                # Parse out individual inputs you want to use.
                # Joystick/trigger ranges are normalized to [-1, 1] or [0, 1].
                js_R_x = bt_signals["js_R_x"]   # steer X with right stick (left/right)
                js_R_y = bt_signals["js_R_y"]   # steer Y with right stick (up/down)
                trigger_L2 = bt_signals["trigger_L2"]   # yaw negative (e.g., CCW)
                trigger_R2 = bt_signals["trigger_R2"]   # yaw positive (e.g., CW)

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
                # ============================================================================
                # TODO [LAB-08]: Call method to calculate motor angles & speeds from measured encoder values
                # ============================================================================
                psi_1 = 0.0
                psi_2 = 0.0
                psi_3 = 0.0
                
                dpsi_1 = 0.0
                dpsi_2 = 0.0
                dpsi_3 = 0.0

                # Calculate ball's roll and translation through kinematic conversions of wheel data
                # ============================================================================
                # TODO [LAB-08]: Call method to calculate kinematic conversion of encoder-angles to ball-translations
                # ============================================================================
                # phi_x, phi_y, phi_z = calc_kinematic_conv(psi_1,psi_2,psi_3)

                # ============================================================================
                # STUDENT TODO [LAB-07 & 8]: Set x-y-z bot commands
                # ============================================================================
                # Choose how Tx,Ty,Tz are set
                Tx = 0.0
                Ty = 0.0
                Tz = 0.0

                # ============================================================================
                # TODO [LAB-07]: Call method to calculate motor commands (u1,u2,u3) from axis torques (Tx,Ty,Tz)
                # ============================================================================


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
                data = [
                    "i",           # Iteration counter
                    "t_now",       # Elapsed time [s]
                    "Tx", "Ty", "Tz",  # Commanded torques
                    "u1", "u2", "u3",  # Motor PWM commands
                    "theta_x", "theta_y", "theta_z",  # IMU angles [rad]
                    "psi_1", "psi_2", "psi_3",  # Wheel angles [rad]
                    "dpsi_1", "dpsi_2", "dpsi_3",  # Wheel velocities [rad/s]
            ]
                dl.appendData(data)
                
                # NOTE: err_x / err_y referenced previously were undefined.
                # If you implement a PID later, track previous errors here:
                # err_x_prev = err_x
                # err_y_prev = err_y

                # ============================================================
                # TERMINAL OUTPUT
                # ============================================================
                if i % PRINT_EVERY == 0:
                    print(
                        f"Time: {t_now:.3f}s | Tx: {Tx:.2f}, Ty: {Ty:.2f}, Tz: {Tz:.2f} | "
                        f"u1: {u1:.2f}, u2: {u2:.2f}, u3: {u3:.2f} | "
                        f"Theta X: {theta_x:.2f}, Theta Y: {theta_y:.2f}, Theta Z: {theta_z:.2f} | "
                        f"Psi 1: {psi_1:.1f}, Psi 2: {psi_2:.1f}, Psi 3: {psi_3:.1f} | "
                        f"dPsi 1: {dpsi_1:.2f}, dPsi 2: {dpsi_2:.2f}, dPsi 3: {dpsi_3:.2f}"
                    )
            
            except KeyError:
                print("Waiting for sensor data...")

    # ========================================================================
    # EXCEPTION HANDLING AND SHUTDOWN
    # ========================================================================
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
        # ====================================================================
        # CLEANUP AND DATA SAVING
        # ====================================================================
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