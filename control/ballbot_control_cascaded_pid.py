"""
Ball-Bot Cascaded PID Control System

This script implements a two-level cascaded PID control architecture for a ball-balancing robot:
- Inner Loop (Fast): Attitude stabilization using IMU feedback
- Outer Loop (Slow): Position/velocity control using encoder odometry
- Yaw Control: Independent heading control for steering

Hardware Interface:
- LCM communication for sensor feedback (mbot_balbot_feedback_t)
- PS4 controller for manual operation
- Three omniwheel motors controlled via PWM

Coordinate System:
- X-axis: Forward/backward motion
- Y-axis: Left/right motion  
- Z-axis: Rotational motion (yaw)

Author: [Your Name]
Date: [Date]
"""

import time
import lcm
import threading
import numpy as np
import sys
import os
import json
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t
from mbot_lcm_msgs.mbot_balbot_feedback_t import mbot_balbot_feedback_t

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from DataLogger import dataLogger
from ps4_controller_api import PS4InputHandler

# ============================================================================
# SYSTEM CONSTANTS
# ============================================================================

# Control Loop Timing
FREQ = 200  # Main control loop frequency [Hz] - critical for stability
DT = 1 / FREQ  # Time step [sec]

# Motor and PWM Limits
PWM_MAX = 1.0  # Maximum PWM signal (0 to 1 range)
PWM_MIN = -1.0  # Minimum PWM signal

# Mechanical Parameters
N_GEARBOX = 70  # Motor gearbox reduction ratio
N_ENC = 64  # Encoder ticks per motor shaft revolution
R_W = 0.048  # Omniwheel radius [m]
R_K = 0.121  # Basketball radius [m]

# Control Mode Selection (can be changed dynamically with PS4 controller)
# 0 = Open-loop test sequence (Square button)
# 1 = Bluetooth controller (Cross button)
# 2 = Balance PID (Circle button)
# 3 = Autonomous balance with cascaded PID (Triangle button)
CONTROL_MODE = 1  # Start in manual mode for safety

# Safety Limits
THETA_MAX = np.radians(5)  # Maximum allowed lean angle [rad] (5 degrees)
VELOCITY_MAX = 1.0  # Maximum commanded velocity [m/s]
YAW_RATE_MAX = 2.0  # Maximum yaw rate [rad/s]

# Sensor Filtering
IMU_DEADZONE = np.radians(0.05)  # IMU angle deadzone [rad] (0.05 degrees) - ignore small angles to reduce noise
IMU_LP_CUTOFF_HZ = 12.0  # Low-pass cutoff frequency for IMU angles [Hz]
# Precompute low-pass filter alpha using RC filter: alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
IMU_LP_ALPHA = (DT / ((1.0 / (2.0 * np.pi * IMU_LP_CUTOFF_HZ)) + DT))

# ============================================================================
# GLOBAL VARIABLES FOR LCM COMMUNICATION
# ============================================================================

listening = False  # Flag to control the LCM listener thread
msg = mbot_balbot_feedback_t()  # Latest sensor feedback message
last_time = 0  # Timestamp of last received message
last_seen = {"MBOT_BALBOT_FEEDBACK": 0}  # Track channel activity


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
    - Verify MBOT_BALBOT_FEEDBACK publisher is active
    """
    global msg, last_seen, last_time
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
    global listening, last_time, last_seen
    
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

def calc_enc2rad(ticks):
    """
    Convert encoder ticks to wheel angular position in radians.
    
    Args:
        ticks: Raw encoder count (can be position or velocity)
        
    Returns:
        Angular position or velocity [rad] or [rad/s]
        
    Note: Accounts for gearbox reduction and encoder resolution
    """
    rad = ticks / (N_GEARBOX * N_ENC) * 2 * np.pi
    return rad


def calc_torque_conv(Tx, Ty, Tz):
    """
    Convert desired Cartesian torques to individual motor commands.
    
    This implements the inverse kinematic mapping for a three-omnidirectional-wheel
    configuration arranged at 120° intervals.
    
    Args:
        Tx: Desired torque/force in X direction (forward/back)
        Ty: Desired torque/force in Y direction (left/right)
        Tz: Desired torque about Z axis (rotation)
        
    Returns:
        u1, u2, u3: Motor command signals for wheels 1, 2, 3
        
    Troubleshooting:
    - If script directly crashed, check for TYPOs
        - pi should be np.pi
        - cos should be np.cos 
    """
    # Geometric transformation matrix for 120° spaced omniwheels
    u1 = (1/3) * (Tz - (1/np.cos(np.pi/4)) * (2*Ty))
    u2 = (1/3) * (Tz + (1/np.cos(np.pi/4)) * (-np.sqrt(3)*Tx + Ty))
    u3 = (1/3) * (Tz + (1/np.cos(np.pi/4)) * (np.sqrt(3)*Tx + Ty))
    
    return u1, u2, u3


def calc_kinematic_conv(psi1, psi2, psi3):
    """
    Calculate ball angular position from wheel encoder odometry.
    
    This is the forward kinematic mapping that estimates how the ball has rotated
    based on the wheel rotations. Used for position/velocity estimation.
    
    Args:
        psi1, psi2, psi3: Individual wheel angles [rad]
        
    Returns:
        phix: Ball rotation about X-axis [rad]
        phiy: Ball rotation about Y-axis [rad]
        phiz: Ball rotation about Z-axis [rad]
        
    Note: Multiply by R_K to get linear displacement (x = phix * R_K)
    """
    # Forward kinematics for omnidirectional base
    phix = (R_W / R_K) * np.sqrt(2/3) * (psi2 - psi3)
    phiy = (R_W / R_K) * np.sqrt(2)/3 * (-2*psi1 + psi2 + psi3)
    phiz = (R_W / R_K) * np.sqrt(2)/3 * (psi1 + psi2 + psi3)
    
    return phix, phiy, phiz


def func_clip(x, lim_lo, lim_hi):
    """
    Saturate a value to within specified limits.
    
    Critical for safety and preventing integrator windup.
    
    Args:
        x: Input value
        lim_lo: Lower limit
        lim_hi: Upper limit
        
    Returns:
        Clipped value

    Make sure you implement this function correctly to prevent over powering the motors
    """
    if x > lim_hi:
        return lim_hi
    elif x < lim_lo:
        return lim_lo
    return x


def apply_deadzone(x, deadzone):
    """
    Apply a deadzone filter to sensor readings.
    
    Values within ±deadzone are set to zero to filter out noise.
    This prevents constant small corrections due to sensor noise.
    
    Args:
        x: Input value
        deadzone: Deadzone threshold (positive value)
        
    Returns:
        Filtered value (0 if within deadzone, original value otherwise)
    """
    if abs(x) < deadzone:
        return 0.0
    return x


# ============================================================================
# PID CONTROLLER CLASS
# ============================================================================

class PIDController:
    """
    Generic PID controller with anti-windup and derivative filtering.
    
    Implements the control law:
        u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
        
    Features:
    - Integral clamping to prevent windup
    - Option to use direct derivative measurement (e.g., from gyro)
    - Automatic reset functionality
    """
    
    def __init__(self, Kp, Ki, Kd, dt, integral_limit=None, output_limit=None):
        """
        Initialize PID controller with specified gains.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            dt: Time step for integration/differentiation
            integral_limit: Optional limit for integral term (anti-windup)
            output_limit: Optional limit for total output (tuple: min, max)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        
        # Internal state
        self.integral = 0.0
        self.prev_error = None
        
    def update(self, setpoint, measurement, derivative_measurement=None):
        """
        Compute control output based on current error.
        
        Args:
            setpoint: Desired value
            measurement: Current measured value
            derivative_measurement: Optional direct measurement of derivative
                                   (e.g., gyro rate instead of differentiating angle)
                                   
        Returns:
            Control output
            
        Troubleshooting:
        - Oscillation = Kp or Kd too high, or Ki causing instability
        - Steady-state error = Increase Ki (but watch for overshoot)
        - Slow response = Increase Kp
        - Overshoot = Increase Kd or decrease Kp
        """
        # Calculate error
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.Kp * error
        
        # Integral term with anti-windup clamping
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = func_clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.Ki * self.integral
        
        # Derivative term
        if derivative_measurement is not None:
            # Use direct measurement (preferred for IMU data)
            # Negative sign because we want rate of error decrease
            d_term = -self.Kd * derivative_measurement
        elif self.prev_error is not None:
            # Finite difference approximation
            derivative = (error - self.prev_error) / self.dt
            d_term = self.Kd * derivative
        else:
            # First iteration, no derivative available
            d_term = 0.0
        
        self.prev_error = error
        
        # Compute total output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limit is not None:
            output = func_clip(output, self.output_limit[0], self.output_limit[1])
        
        return output
    
    def reset(self):
        """
        Reset controller state (useful when switching modes).
        """
        self.integral = 0.0
        self.prev_error = None


# ============================================================================
# PID GAINS SAVE/LOAD FUNCTIONS
# ============================================================================

def save_pid_gains(inner_x, inner_y, outer_x, outer_y, yaw, filename='pid_gains.json'):
    """
    Save PID gains to a JSON file for persistence across runs.
    
    Args:
        inner_x: Inner loop X-axis PIDController
        inner_y: Inner loop Y-axis PIDController
        outer_x: Outer loop X-axis PIDController
        outer_y: Outer loop Y-axis PIDController
        yaw: Yaw PIDController
        filename: Path to save gains file
    """
    gains = {
        'inner_x': {'Kp': inner_x.Kp, 'Ki': inner_x.Ki, 'Kd': inner_x.Kd},
        'inner_y': {'Kp': inner_y.Kp, 'Ki': inner_y.Ki, 'Kd': inner_y.Kd},
        'outer_x': {'Kp': outer_x.Kp, 'Ki': outer_x.Ki, 'Kd': outer_x.Kd},
        'outer_y': {'Kp': outer_y.Kp, 'Ki': outer_y.Ki, 'Kd': outer_y.Kd},
        'yaw': {'Kp': yaw.Kp, 'Ki': yaw.Ki, 'Kd': yaw.Kd}
    }
    
    try:
        with open(filename, 'w') as f:
            json.dump(gains, f, indent=4)
        print(f"✓ PID gains saved to {filename}")
    except Exception as e:
        print(f"✗ Failed to save PID gains: {e}")


def load_pid_gains(filename='pid_gains.json'):
    """
    Load PID gains from a JSON file, or return defaults if file doesn't exist.
    
    Args:
        filename: Path to gains file
        
    Returns:
        Dictionary containing PID gains for all controllers
    """
    # Default gains (used if file doesn't exist)
    defaults = {
        'inner_x': {'Kp': 12.0, 'Ki': 0.0, 'Kd': 1.0},
        'inner_y': {'Kp': 12.0, 'Ki': 0.0, 'Kd': 1.0},
        'outer_x': {'Kp': 0.3, 'Ki': 0.05, 'Kd': 0.8},
        'outer_y': {'Kp': 0.3, 'Ki': 0.05, 'Kd': 0.8},
        'yaw': {'Kp': 0.5, 'Ki': 0.1, 'Kd': 0.05}
    }
    
    try:
        with open(filename, 'r') as f:
            gains = json.load(f)
        print(f"✓ PID gains loaded from {filename}")
        return gains
    except FileNotFoundError:
        print(f"ℹ No saved gains found, using defaults")
        return defaults
    except Exception as e:
        print(f"✗ Failed to load PID gains: {e}, using defaults")
        return defaults


# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

def main():
    """
    Main execution function implementing the cascaded PID control system.
    
    Control Architecture:
    1. Outer Loop (slow): Position/velocity → Desired pitch/roll angle
    2. Inner Loop (fast): Pitch/roll angle → Motor torque commands
    3. Yaw Loop: Independent heading control
    
    Execution Flow:
    - Initialize hardware interfaces (LCM, PS4 controller)
    - Set up data logging
    - Run control loop at 200 Hz
    - Handle graceful shutdown on interrupt
    """
    
    # ========================================================================
    # DATA LOGGING INITIALIZATION
    # ========================================================================
    
    trial_num = int(input("Test Number? "))
    filename = f"ballbot_control_{trial_num}.txt"
    dl = dataLogger(filename)
    print(f"Data will be saved to: {filename}")
    
    # ========================================================================
    # LCM COMMUNICATION INITIALIZATION
    # ========================================================================
    
    global listening, msg
    
    # Initialize LCM with multicast configuration
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    subscription = lc.subscribe("MBOT_BALBOT_FEEDBACK", feedback_handler)
    
    # Start listener thread for asynchronous message handling
    listening = True
    listener_thread = threading.Thread(target=lcm_listener, args=(lc,), daemon=True)
    listener_thread.start()
    print("✓ LCM listener started")
    
    # ========================================================================
    # PS4 CONTROLLER INITIALIZATION
    # ========================================================================
    
    controller = PS4InputHandler(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True
    controller_thread.start()
    print("✓ PS4 Controller initialized")
    
    # ========================================================================
    # ========================================================================
    # CONTROLLER INITIALIZATION
    # ========================================================================
    
    # Load PID gains from file (or use defaults if file doesn't exist)
    gains = load_pid_gains('pid_gains.json')
    
    # Inner loop controllers (attitude stabilization) - FAST
    # These run at full 200 Hz and directly control motor torques
    # PURPOSE: Stabilize lean angle (theta) to prevent falling
    # INPUT: Desired lean angle (theta_d) vs actual lean angle (theta)
    # OUTPUT: Motor torque commands (Tx, Ty)
    # 
    # Tuning notes: 
    # - X and Y axes may have different dynamics due to robot geometry
    # - Small angle errors should produce gentle corrections
    # - Increase Kp gradually if response is too slow
    # - Add Kd if oscillations occur
    
    # X-axis (Forward/Backward - Pitch)
    # Note: Sign may need adjustment based on IMU mounting orientation
    inner_pid_x = PIDController(
        Kp=gains['inner_x']['Kp'],
        Ki=gains['inner_x']['Ki'],
        Kd=gains['inner_x']['Kd'],
        dt=DT, 
        integral_limit=0.5,
        output_limit=(-PWM_MAX, PWM_MAX)
    )
    
    # Y-axis (Left/Right - Roll)
    # Note: Sign may need adjustment based on IMU mounting orientation
    inner_pid_y = PIDController(
        Kp=gains['inner_y']['Kp'],
        Ki=gains['inner_y']['Ki'],
        Kd=gains['inner_y']['Kd'],
        dt=DT,
        integral_limit=0.5,
        output_limit=(-PWM_MAX, PWM_MAX)
    )
    # Outer loop controllers (position/velocity control) - SLOW  
    # These run at reduced rate (e.g., 50 Hz) and command desired lean angles
    # PURPOSE: Control position/velocity by commanding lean angles to inner loop
    # INPUT: Desired velocity (dx_d, dy_d) vs actual velocity (dx, dy)
    # OUTPUT: Desired lean angle (theta_d_x, theta_d_y) → fed to inner loop
    #
    # Tuning notes:
    # - X and Y axes use same gains for simplicity
    # - Lower gains = smoother but slower position tracking
    # - Higher gains = faster tracking but may cause oscillation
    
    OUTER_LOOP_DECIMATION = 4  # Run outer loop every N iterations (50 Hz at 200 Hz main loop)
    
    # X-axis (Forward/Backward velocity control)
    outer_pid_x = PIDController(
        Kp=0.30, Ki=0.05, Kd=0.80,
        dt=DT * OUTER_LOOP_DECIMATION,
        integral_limit=2.0,
        output_limit=(-THETA_MAX, THETA_MAX)
    )
    
    # Y-axis (Left/Right velocity control)
    outer_pid_y = PIDController(
        Kp=0.30, Ki=0.05, Kd=0.80,
        dt=DT * OUTER_LOOP_DECIMATION,
        integral_limit=2.0,
        output_limit=(-THETA_MAX, THETA_MAX)
    )
    
    # Yaw controller (heading control) - INDEPENDENT
    yaw_pid = PIDController(
        Kp=0.5, Ki=0.1, Kd=0.05,
        dt=DT,
        integral_limit=1.0,
        output_limit=(-PWM_MAX, PWM_MAX)
    )
    
    print("✓ PID controllers initialized")
    print("\nStarting control loop in 0.5 seconds...")
    time.sleep(0.5)
    
    # ========================================================================
    # DATA LOGGING HEADER
    # ========================================================================
    
    # Define column headers for logged data
    data_header = [
        "i",           # Iteration counter
        "t_now",       # Elapsed time [s]
        "control_mode",  # Control mode (0=cascaded, 1=manual, 2=balance)
        "Tx", "Ty", "Tz",  # Commanded torques
        "u1", "u2", "u3",  # Motor PWM commands
        "theta_x", "theta_y", "theta_z",  # IMU angles [rad]
        "psi_1", "psi_2", "psi_3",  # Wheel angles [rad]
        "dpsi_1", "dpsi_2", "dpsi_3",  # Wheel velocities [rad/s]
        "phi_x", "phi_y",  # Ball position (odometry) [rad]
        "x", "y",  # Linear position [m]
        "dx", "dy",  # Linear velocity [m/s]
        "theta_d_x", "theta_d_y",  # Desired lean angles [rad]
        "inner_x_Kp", "inner_x_Ki", "inner_x_Kd",  # Inner X PID gains
        "inner_y_Kp", "inner_y_Ki", "inner_y_Kd",  # Inner Y PID gains
        "outer_x_Kp", "outer_x_Ki", "outer_x_Kd",  # Outer X PID gains
        "outer_y_Kp", "outer_y_Ki", "outer_y_Kd"   # Outer Y PID gains
    ]
    dl.appendData([" ".join(data_header)])
    
    # ========================================================================
    # CONTROL LOOP INITIALIZATION
    # ========================================================================
    
    try:
        command = mbot_motor_pwm_t()
        
        # Control loop counters and timing
        i = 0
        t_start = time.time()
        t_now = 0
        
        # Store initial encoder positions for relative measurement
        enc_pos_1_start = msg.enc_ticks[0]
        enc_pos_2_start = msg.enc_ticks[1]
        enc_pos_3_start = msg.enc_ticks[2]
        
        # Store initial IMU orientation as reference frame
        theta_x_0 = msg.imu_angles_rpy[0]
        theta_y_0 = msg.imu_angles_rpy[1]
        theta_z_0 = msg.imu_angles_rpy[2]
        
        # Initialize control variables
        Tx = Ty = Tz = 0.0
        u1 = u2 = u3 = 0.0
        theta_d_x = theta_d_y = 0.0
        
        # Previous position for velocity estimation
        prev_x = prev_y = 0.0
        
        # IMU low-pass filter state
        imu_filt_initialized = False
        theta_x_f = 0.0
        theta_y_f = 0.0
        theta_z_f = 0.0
        
        # Mode switching state tracking
        control_mode = CONTROL_MODE  # Local variable for dynamic mode switching
        prev_but_tri = 0  # Previous state of triangle button
        prev_but_sq = 0   # Previous state of square button
        prev_but_cir = 0  # Previous state of circle button
        prev_but_x = 0    # Previous state of cross button
        mode_start_time = t_start  # Track when current mode was entered
        
        # Gain tuning state tracking (for D-pad in Mode 2)
        prev_dpad_up = 0
        prev_dpad_down = 0
        prev_dpad_right = 0
        prev_dpad_left = 0
        last_gain_change_time = 0  # Timestamp of last gain change
        GAIN_CHANGE_COOLDOWN = 0.12  # Minimum time between gain changes [seconds]
        
        # Gain selection: 0 = P, 1 = I, 2 = D
        gain_sel = 0
        # Increments for each gain type
        GAIN_INC = {0: 0.1, 1: 0.1, 2: 0.01}
        
        print("\n" + "="*80)
        print("CONTROL LOOP ACTIVE - Press Ctrl+C to stop")
        print("="*80)
        print("\nCONTROL MODE SWITCHING:")
        print("  Square:   Mode 0 - Cascaded PID (outer + inner loops with manual tuned gains)")
        print("  Cross:    Mode 1 - Manual control (direct controller to motors)")
        print("  Circle:   Mode 2 - Balance PID (use D-pad to tune P/I/D gains)")
        print("  Triangle: Mode 2 - Balance PID (alias)")
        print("\nADDITIONAL CONTROLS:")
        print("  R1:       Reset IMU reference (set current position as upright)")
        print("\nMODE 2 GAIN TUNING:")
        print("  D-Pad Left/Right: Select parameter (P <-> I <-> D)")
        print("  D-Pad Up/Down:    Increase/Decrease selected gain (increment: 0.1)")
        print(f"\nStarting in Mode {control_mode}")
        print("="*80 + "\n")
        
        # ====================================================================
        # MAIN CONTROL LOOP
        # ====================================================================
        
        while True:
            time.sleep(DT)
            t_now = time.time() - t_start
            i += 1
            
            try:
                # ============================================================
                # SENSOR DATA ACQUISITION
                # ============================================================
                
                # IMU angles (relative to initial orientation)
                theta_x_raw = msg.imu_angles_rpy[0] - theta_x_0  # Pitch
                theta_y_raw = msg.imu_angles_rpy[1] - theta_y_0  # Roll
                theta_z_raw = msg.imu_angles_rpy[2] - theta_z_0  # Yaw
                
                # Low-pass filter IMU angles, then apply deadzone
                if not imu_filt_initialized:
                    theta_x_f = theta_x_raw
                    theta_y_f = theta_y_raw
                    theta_z_f = theta_z_raw
                    imu_filt_initialized = True
                else:
                    theta_x_f = theta_x_f + IMU_LP_ALPHA * (theta_x_raw - theta_x_f)
                    theta_y_f = theta_y_f + IMU_LP_ALPHA * (theta_y_raw - theta_y_f)
                    theta_z_f = theta_z_f + IMU_LP_ALPHA * (theta_z_raw - theta_z_f)

                # Apply deadzone filter to reduce noise and prevent constant small corrections
                theta_x = apply_deadzone(theta_x_f, IMU_DEADZONE)
                theta_y = apply_deadzone(theta_y_f, IMU_DEADZONE)
                theta_z = apply_deadzone(theta_z_f, IMU_DEADZONE)
                
                # IMU angular velocities (gyroscope readings)
                # WARNING: The message struct only has angles, not rates!
                # Using angles as rate will give incorrect derivative term
                # Better to let PID calculate derivative from angle changes
                # Setting these to None to force PID to compute derivative
                dtheta_x = None  # Will be computed by PID from angle changes
                dtheta_y = None  # Will be computed by PID from angle changes
                dtheta_z = None  # Will be computed by PID from angle changes
                
                # Encoder positions (relative to start)
                enc_pos_1 = msg.enc_ticks[0] - enc_pos_1_start
                enc_pos_2 = msg.enc_ticks[1] - enc_pos_2_start
                enc_pos_3 = msg.enc_ticks[2] - enc_pos_3_start
                
                # Encoder velocities
                enc_dtick_1 = msg.enc_delta_ticks[0]
                enc_dtick_2 = msg.enc_delta_ticks[1]
                enc_dtick_3 = msg.enc_delta_ticks[2]
                enc_dt = msg.enc_delta_time  # Microseconds
                
                # ============================================================
                # KINEMATIC CALCULATIONS
                # ============================================================
                
                # Convert encoder ticks to wheel angles
                psi_1 = calc_enc2rad(enc_pos_1)
                psi_2 = calc_enc2rad(enc_pos_2)
                psi_3 = calc_enc2rad(enc_pos_3)
                
                # Convert encoder velocity to wheel angular velocities [rad/s]
                # Safety check: avoid division by zero if enc_dt is 0
                if enc_dt > 0:
                    dpsi_1 = calc_enc2rad(enc_dtick_1 / enc_dt * 1e6)
                    dpsi_2 = calc_enc2rad(enc_dtick_2 / enc_dt * 1e6)
                    dpsi_3 = calc_enc2rad(enc_dtick_3 / enc_dt * 1e6)
                else:
                    dpsi_1 = dpsi_2 = dpsi_3 = 0.0  # No movement if dt is zero
                
                # Forward kinematics: wheel angles → ball rotation angles
                phi_x, phi_y, phi_z = calc_kinematic_conv(psi_1, psi_2, psi_3)
                
                # Convert ball rotation to linear displacement
                x = phi_x * R_K
                y = phi_y * R_K
                
                # Estimate velocity using finite difference
                dx = (x - prev_x) / DT
                dy = (y - prev_y) / DT
                prev_x = x
                prev_y = y
                
                # ============================================================
                # BLUETOOTH CONTROLLER INPUT
                # ============================================================
                
                bt_signals = controller.get_signals()
                js_R_x = bt_signals["js_R_x"]      # Right stick X (left/right)
                js_R_y = bt_signals["js_R_y"]      # Right stick Y (forward/back)
                trigger_L2 = bt_signals["trigger_L2"]  # Left trigger (rotate CCW)
                trigger_R2 = bt_signals["trigger_R2"]  # Right trigger (rotate CW)
                
                # Face buttons for mode switching
                but_tri = bt_signals["but_tri"]    # Triangle
                but_sq = bt_signals["but_sq"]      # Square
                but_cir = bt_signals["but_cir"]    # Circle
                but_x = bt_signals["but_x"]        # Cross
                
                # Shoulder buttons
                shoulder_R1 = bt_signals["shoulder_R1"]  # R1 for reference reset
                
                # Joystick buttons (L3/R3) for emergency stop
                but_L3 = bt_signals.get("but_L3", 0)  # Left joystick press
                but_R3 = bt_signals.get("but_R3", 0)  # Right joystick press
                
                # D-pad buttons for gain tuning
                dpad_up = bt_signals["dir_U"]      # Increase gain
                dpad_down = bt_signals["dir_D"]    # Decrease gain
                dpad_right = bt_signals["dir_R"]  # Next parameter (P->I->D->P)
                dpad_left = bt_signals["dir_L"]    # Previous parameter (D->I->P->D)
                
                # ============================================================
                # REFERENCE RESET (R1 button)
                # ============================================================
                
                # Reset IMU reference orientation when R1 is pressed
                if shoulder_R1 == 1:
                    theta_x_0 = msg.imu_angles_rpy[0]
                    theta_y_0 = msg.imu_angles_rpy[1]
                    theta_z_0 = msg.imu_angles_rpy[2]
                    print("\n>>> IMU REFERENCE RESET - Current position set as upright <<<\n")
                
                # ============================================================
                # EMERGENCY STOP (L3 + R3 pressed simultaneously)
                # ============================================================
                
                if but_L3 == 1 and but_R3 == 1:
                    print("\n\n" + "="*80)
                    print("!!! EMERGENCY STOP TRIGGERED (L3+R3) !!!")
                    print("="*80)
                    command = mbot_motor_pwm_t()
                    command.utime = int(time.time() * 1e6)
                    command.pwm[0] = 0.0
                    command.pwm[1] = 0.0
                    command.pwm[2] = 0.0
                    lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                    print("✓ Motors stopped - exiting control loop\n")
                    break
                
                # ============================================================
                # D-PAD GAIN TUNING (for Mode 2 Balance PID)
                # ============================================================
                
                # Only allow gain tuning in Mode 2 (Balance PID)
                if control_mode == 2:
                    current_time = time.time()
                    gain_changed = False
                    selection_changed = False
                    
                    # Terminal highlight helpers
                    H_START = '\033[1;33m'  # bold yellow
                    H_END = '\033[0m'
                    
                    # Check cooldown to prevent rapid changes
                    if current_time - last_gain_change_time > GAIN_CHANGE_COOLDOWN:
                        # Selection: Left/Right to cycle through P, I, D
                        if dpad_left == 1 and prev_dpad_left == 0:
                            gain_sel = (gain_sel - 1) % 3
                            selection_changed = True
                            last_gain_change_time = current_time
                        elif dpad_right == 1 and prev_dpad_right == 0:
                            gain_sel = (gain_sel + 1) % 3
                            selection_changed = True
                            last_gain_change_time = current_time
                        # Adjust selected gain with Up/Down
                        elif dpad_up == 1 and prev_dpad_up == 0:
                            inc = GAIN_INC.get(gain_sel, 0.1)
                            if gain_sel == 0:
                                inner_pid_x.Kp += inc
                                inner_pid_y.Kp += inc
                            elif gain_sel == 1:
                                inner_pid_x.Ki += inc
                                inner_pid_y.Ki += inc
                            elif gain_sel == 2:
                                inner_pid_x.Kd += inc
                                inner_pid_y.Kd += inc
                            gain_changed = True
                            last_gain_change_time = current_time
                        elif dpad_down == 1 and prev_dpad_down == 0:
                            inc = GAIN_INC.get(gain_sel, 0.1)
                            if gain_sel == 0:
                                inner_pid_x.Kp = max(0.0, inner_pid_x.Kp - inc)
                                inner_pid_y.Kp = max(0.0, inner_pid_y.Kp - inc)
                            elif gain_sel == 1:
                                inner_pid_x.Ki = max(0.0, inner_pid_x.Ki - inc)
                                inner_pid_y.Ki = max(0.0, inner_pid_y.Ki - inc)
                            elif gain_sel == 2:
                                inner_pid_x.Kd = max(0.0, inner_pid_x.Kd - inc)
                                inner_pid_y.Kd = max(0.0, inner_pid_y.Kd - inc)
                            gain_changed = True
                            last_gain_change_time = current_time
                    
                    # Update previous D-pad states
                    prev_dpad_up = dpad_up
                    prev_dpad_down = dpad_down
                    prev_dpad_right = dpad_right
                    prev_dpad_left = dpad_left
                    
                    # Print selection or gain update message
                    if selection_changed:
                        sel_name = ['P', 'I', 'D'][gain_sel]
                        kp_s = f"{inner_pid_x.Kp:.2f}"
                        ki_s = f"{inner_pid_x.Ki:.2f}"
                        kd_s = f"{inner_pid_x.Kd:.2f}"
                        if gain_sel == 0:
                            kp_s = H_START + kp_s + H_END
                        elif gain_sel == 1:
                            ki_s = H_START + ki_s + H_END
                        elif gain_sel == 2:
                            kd_s = H_START + kd_s + H_END
                        print(f"\n[SELECTED: {H_START}{sel_name}{H_END}] Kp={kp_s}, Ki={ki_s}, Kd={kd_s}\n")
                    elif gain_changed:
                        kp_s = f"{inner_pid_x.Kp:.2f}"
                        ki_s = f"{inner_pid_x.Ki:.2f}"
                        kd_s = f"{inner_pid_x.Kd:.2f}"
                        if gain_sel == 0:
                            kp_s = H_START + kp_s + H_END
                        elif gain_sel == 1:
                            ki_s = H_START + ki_s + H_END
                        elif gain_sel == 2:
                            kd_s = H_START + kd_s + H_END
                        print(f"\n>>> GAINS UPDATED: Kp={kp_s}, Ki={ki_s}, Kd={kd_s} <<<\n")
                        # Save updated gains to file
                        save_pid_gains(inner_pid_x, inner_pid_y, outer_pid_x, outer_pid_y, yaw_pid, 'pid_gains.json')
                
                # ============================================================
                # CONTROL MODE SWITCHING (with debouncing)
                # ============================================================
                # Detect button press (rising edge) and switch modes
                mode_changed = False
                
                if but_sq == 1 and prev_but_sq == 0:
                    control_mode = 0  # Square → Open-loop test
                    mode_changed = True
                elif but_x == 1 and prev_but_x == 0:
                    control_mode = 1  # Cross → Bluetooth control
                    mode_changed = True
                elif but_cir == 1 and prev_but_cir == 0:
                    control_mode = 2  # Circle → Balance PID
                    mode_changed = True
                elif but_tri == 1 and prev_but_tri == 0:
                    control_mode = 2  # Triangle → Balance PID (alias)
                    mode_changed = True
                
                # Update previous button states
                prev_but_tri = but_tri
                prev_but_sq = but_sq
                prev_but_cir = but_cir
                prev_but_x = but_x
                
                # Reset controllers when mode changes to prevent transients
                if mode_changed:
                    inner_pid_x.reset()
                    inner_pid_y.reset()
                    outer_pid_x.reset()
                    outer_pid_y.reset()
                    yaw_pid.reset()
                    theta_d_x = theta_d_y = 0.0
                    mode_start_time = time.time()  # Reset timer for new mode
                    print(f"\n>>> MODE CHANGED TO {control_mode} <<<\n")
                
                # ============================================================
                # CONTROL MODE SELECTION
                # ============================================================
                
                if control_mode == 0:
                    # --------------------------------------------------------
                    # Mode 0: Cascaded PID (outer + inner loops with manual tuned gains)
                    # --------------------------------------------------------
                    # Full cascaded control with position/velocity feedback
                    # Uses manually tuned inner gains (separate for X/Y) and outer gains (same for X/Y)
                    
                    # Apply manual tuned inner gains (different for X and Y axes)
                    inner_pid_x.Kp = 17.0
                    inner_pid_x.Ki = 1.5
                    inner_pid_x.Kd = 0.30
                    
                    inner_pid_y.Kp = 18.5
                    inner_pid_y.Ki = 1.5
                    inner_pid_y.Kd = 0.35
                    
                    # Apply manual tuned outer gains (same for both axes)
                    outer_pid_x.Kp = 0.30
                    outer_pid_x.Ki = 0.05
                    outer_pid_x.Kd = 0.80
                    
                    outer_pid_y.Kp = 0.30
                    outer_pid_y.Ki = 0.05
                    outer_pid_y.Kd = 0.80
                    
                    # Outer loop update (runs at reduced frequency)
                    if i % OUTER_LOOP_DECIMATION == 0:
                        # Set desired velocity from controller or autonomous planner
                        dx_d = js_R_y * VELOCITY_MAX
                        dy_d = js_R_x * VELOCITY_MAX
                        
                        # Outer loop: velocity error → desired lean angle
                        theta_d_x = outer_pid_x.update(dx_d, dx)
                        theta_d_y = outer_pid_y.update(dy_d, dy)
                        
                        # Safety: clamp lean angle setpoint
                        theta_d_x = func_clip(theta_d_x, -THETA_MAX, THETA_MAX)
                        theta_d_y = func_clip(theta_d_y, -THETA_MAX, THETA_MAX)
                    
                    # Inner loop update (runs every iteration)
                    # Attitude control: lean angle error → motor torque
                    # Note: theta_x/theta_y from IMU may be swapped relative to robot frame
                    # Swapping the mapping: theta_x → Ty, theta_y → Tx
                    Ty = inner_pid_x.update(theta_d_x, theta_x, derivative_measurement=dtheta_x)
                    Tx = inner_pid_y.update(theta_d_y, theta_y, derivative_measurement=dtheta_y)
                    
                    # Yaw control (independent) - only control if triggers are pressed
                    if abs(trigger_R2 - trigger_L2) > 0.05:  # Deadzone for triggers
                        dpsi_d = (trigger_R2 - trigger_L2) * YAW_RATE_MAX
                        Tz = yaw_pid.update(dpsi_d, dtheta_z)
                    else:
                        Tz = 0.0  # No yaw control if triggers not pressed
                        
                elif control_mode == 1:
                    # --------------------------------------------------------
                    # Mode 1: Manual control (direct controller to motors)
                    # --------------------------------------------------------
                    # Convert controller signals directly to Tx, Ty, Tz
                    # This bypasses PID and gives direct control
                    
                    Tx = js_R_y  # Forward/backward
                    Ty = js_R_x  # Left/right
                    Tz = trigger_R2 - trigger_L2  # Rotation
                    
                elif control_mode == 2:
                    # --------------------------------------------------------
                    # Mode 2: Balance PID (attitude stabilization only)
                    # --------------------------------------------------------
                    # Simple balance control without position feedback
                    # Robot will balance in place but may drift
                    
                    # Target: keep robot upright (theta_d = 0)
                    # Allow small manual lean commands from joystick for movement
                    # Apply deadzone to joystick to prevent drift
                    js_R_y_filtered = 0.0 if abs(js_R_y) < 0.1 else js_R_y
                    js_R_x_filtered = 0.0 if abs(js_R_x) < 0.1 else js_R_x
                    
                    theta_d_x = js_R_y_filtered * 0.1  # Small lean angle command
                    theta_d_y = js_R_x_filtered * 0.1



                    # --------------------------------------------------------
                    # --------------------------------------------------------
                    # Inner loop: lean angle error → motor torque
                    # Note: theta_x/theta_y from IMU may be swapped relative to robot frame
                    # Swapping the mapping: theta_x → Ty, theta_y → Tx
                    Ty = inner_pid_x.update(theta_d_x, theta_x, derivative_measurement=dtheta_x)
                    Tx = inner_pid_y.update(theta_d_y, theta_y, derivative_measurement=dtheta_y)
                    # --------------------------------------------------------
                    # --------------------------------------------------------




                    # Yaw control (independent) - only control if triggers are pressed
                    if abs(trigger_R2 - trigger_L2) > 0.05:  # Deadzone for triggers
                        dpsi_d = (trigger_R2 - trigger_L2) * YAW_RATE_MAX
                        Tz = yaw_pid.update(dpsi_d, dtheta_z)
                    else:
                        Tz = 0.0  # No yaw control if triggers not pressed
                    
                else:
                    print(f"ERROR: Invalid control mode {control_mode}")
                    Tx = Ty = Tz = 0
                
                # ============================================================
                # MOTOR COMMAND CALCULATION
                # ============================================================
                
                # Convert Cartesian torques to individual motor commands
                u1, u2, u3 = calc_torque_conv(Tx, Ty, Tz)
                
                # Apply safety limits
                u1 = func_clip(u1, PWM_MIN, PWM_MAX)
                u2 = func_clip(u2, PWM_MIN, PWM_MAX)
                u3 = func_clip(u3, PWM_MIN, PWM_MAX)
                
                # ============================================================
                # MOTOR COMMAND TRANSMISSION
                # ============================================================
                
                cmd_utime = int(time.time() * 1e6)
                command.utime = cmd_utime
                command.pwm[0] = u1
                command.pwm[1] = u2
                command.pwm[2] = u3
                lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                
                # ============================================================
                # DATA LOGGING
                # ============================================================
                
                data = [
                    i, t_now,
                    control_mode,
                    Tx, Ty, Tz,
                    u1, u2, u3,
                    theta_x, theta_y, theta_z,
                    psi_1, psi_2, psi_3,
                    dpsi_1, dpsi_2, dpsi_3,
                    phi_x, phi_y,
                    x, y,
                    dx, dy,
                    theta_d_x, theta_d_y,
                    inner_pid_x.Kp, inner_pid_x.Ki, inner_pid_x.Kd,
                    inner_pid_y.Kp, inner_pid_y.Ki, inner_pid_y.Kd,
                    outer_pid_x.Kp, outer_pid_x.Ki, outer_pid_x.Kd,
                    outer_pid_y.Kp, outer_pid_y.Ki, outer_pid_y.Kd
                ]
                dl.appendData(data)
                
                # ============================================================
                # TERMINAL OUTPUT (every 10 iterations for readability)
                # ============================================================
                
                if i % 10 == 0:
                    mode_names = ["Cascaded", "Manual", "Balance"]
                    print(
                        f"[{mode_names[control_mode]}] "
                        f"t={t_now:.2f}s | "
                        f"u=[{u1:.2f}, {u2:.2f}, {u3:.2f}] | "
                        f"θ=[{np.degrees(theta_x):.1f}°, {np.degrees(theta_y):.1f}°] | "
                        f"PID(Kp={inner_pid_x.Kp:.2f}, Ki={inner_pid_x.Ki:.2f}, Kd={inner_pid_x.Kd:.2f})"
                    )
            
            except KeyError as e:
                print(f"WARNING: Waiting for sensor data... (missing key: {e})")
                continue
            except Exception as e:
                print(f"ERROR in control loop: {e}")
                continue
    
    # ========================================================================
    # EXCEPTION HANDLING AND SHUTDOWN
    # ========================================================================
    
    except KeyboardInterrupt:
        print("\n\n" + "="*80)
        print("KEYBOARD INTERRUPT DETECTED - INITIATING EMERGENCY STOP")
        print("="*80)
        
        # Emergency stop: zero all motor commands immediately
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        print("✓ Motors stopped")
    
    finally:
        # ====================================================================
        # CLEANUP AND DATA SAVING
        # ====================================================================
        
        print("\n" + "="*80)
        print("SHUTTING DOWN")
        print("="*80)
        
        # Save logged data
        print(f"\n→ Saving data to {filename}...")
        dl.writeOut()
        print("✓ Data saved successfully")
        
        # Save PID gains
        print("\n→ Saving PID gains...")
        save_pid_gains(inner_pid_x, inner_pid_y, outer_pid_x, outer_pid_y, yaw_pid, 'pid_gains.json')
        
        # Stop LCM listener thread
        print("\n→ Stopping LCM listener...")
        listener_thread.join(timeout=1)
        if listener_thread.is_alive():
            print("WARNING: LCM listener thread did not stop cleanly")
        else:
            print("✓ LCM listener stopped")
        
        # Stop controller thread
        print("\n→ Stopping controller...")
        controller_thread.join(timeout=1)
        controller.on_options_press()
        if controller_thread.is_alive():
            print("WARNING: Controller thread did not stop cleanly")
        else:
            print("✓ Controller stopped")
        
        # Final motor shutdown (redundant safety measure)
        print("\n→ Final motor shutdown...")
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        print("✓ Motors confirmed off")
        
        print("\n" + "="*80)
        print("SHUTDOWN COMPLETE")
        print("="*80 + "\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    """
    TROUBLESHOOTING GUIDE:
    
    Problem: Robot doesn't respond
    - Check LCM network configuration (udpm://239.255.76.67:7667?ttl=0)
    - Verify robot is powered on and publishing feedback
    - Check physical connections (USB, power)
    
    Problem: Robot falls over immediately
    - Inner loop gains may be too low (increase Kp_inner)
    - Check IMU calibration and mounting orientation
    - Verify encoder directions match motor directions
    - Reduce outer loop gains or disable outer loop initially
    
    Problem: Oscillation/instability
    - Inner loop: Reduce Kp_inner, increase Kd_inner
    - Outer loop: Reduce all outer loop gains by 50%
    - Check for mechanical issues (loose wheels, friction)
    
    Problem: Drift in one direction
    - Increase Ki terms (but watch for overshoot)
    - Check for IMU bias or encoder calibration issues
    - Verify motor directions are correct
    
    Problem: Poor position tracking
    - Increase outer loop Kp and Kd
    - Check that outer loop is running (not too decimated)
    - Verify encoder odometry is accurate (check calc_kinematic_conv)
    
    Problem: PS4 controller not working
    - Check device path: ls /dev/input/js*
    - Try connecting_using_ds4drv=True if using ds4drv
    - Test controller separately: jstest /dev/input/js0
    
    Problem: Data not logging correctly
    - Check write permissions in current directory
    - Verify DataLogger class is imported correctly
    - Check that dl.writeOut() is called in finally block
    """
    
    main()
