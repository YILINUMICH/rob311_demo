"""
Ballbot PID Control with Live D-pad Tuning and Network Plotting

This script combines:
1. Real-time PID gain adjustment via D-pad
2. Network plotting to visualize tuning effects instantly
3. Visual feedback showing which gain is selected

D-pad Controls:
- LEFT/RIGHT: Cycle through gains (Kp → Ki → Kd → Kp)
- UP/DOWN: Increase/decrease selected gain
- The selected gain is highlighted in terminal

Usage:
    1. On robot: python ballbot_control_live_tuning.py
    2. On laptop: ballbot_pid_viewer('robot_ip') in Matlab
    3. Use D-pad to tune while watching plots update in real-time!

ROB 311 - Fall 2025
University of Michigan
"""

import time
import lcm
import threading
import numpy as np
import sys
import os
import json
import socket
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t
from mbot_lcm_msgs.mbot_balbot_feedback_t import mbot_balbot_feedback_t
from mbot_lcm_msgs.mbot_imu_t import mbot_imu_t

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from DataLogger import dataLogger
from ps4_controller_api import PS4InputHandler

# ============================================================================
# TERMINAL HIGHLIGHTING
# ============================================================================

H_START = '\033[7m'  # Reverse video (highlight)
H_END = '\033[0m'    # Reset to normal

# ============================================================================
# SYSTEM CONSTANTS
# ============================================================================

FREQ = 200
DT = 1 / FREQ

PWM_MAX = 1.0
PWM_MIN = -1.0

N_GEARBOX = 70
N_ENC = 64
R_W = 0.048
R_K = 0.121

CONTROL_MODE = 1

THETA_MAX = np.radians(5)
VELOCITY_MAX = 1.0
YAW_RATE_MAX = 2.0

IMU_DEADZONE = np.radians(0.05)
IMU_LP_CUTOFF_HZ = 15.0
IMU_LP_ALPHA = (DT / ((1.0 / (2.0 * np.pi * IMU_LP_CUTOFF_HZ)) + DT))

GYRO_LP_CUTOFF_HZ = 20.0
GYRO_LP_ALPHA = (DT / ((1.0 / (2.0 * np.pi * GYRO_LP_CUTOFF_HZ)) + DT))

PLOT_PORT = 5556

# ============================================================================
# GAIN TUNING PARAMETERS
# ============================================================================

GAIN_INCREMENT_KP = 0.5   # Kp adjustment step
GAIN_INCREMENT_KI = 0.01  # Ki adjustment step
GAIN_INCREMENT_KD = 0.05  # Kd adjustment step

GAIN_MIN = 0.0
GAIN_MAX = 50.0

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================

listening = False
msg = mbot_balbot_feedback_t()
imu_msg = mbot_imu_t()
last_time = 0
last_seen = {"MBOT_BALBOT_FEEDBACK": 0, "MBOT_IMU": 0}

# ============================================================================
# LCM CALLBACKS
# ============================================================================

def feedback_handler(channel, data):
    global msg, last_seen, last_time
    last_time = time.time()
    last_seen[channel] = time.time()
    msg = mbot_balbot_feedback_t.decode(data)

def imu_handler(channel, data):
    global imu_msg, last_seen
    last_seen[channel] = time.time()
    imu_msg = mbot_imu_t.decode(data)

def lcm_listener(lc):
    global listening, last_time, last_seen
    while listening:
        try:
            lc.handle_timeout(100)
            if time.time() - last_time > 2.0:
                print("WARNING: LCM Publisher seems inactive...")
            elif time.time() - last_seen["MBOT_BALBOT_FEEDBACK"] > 2.0:
                print("WARNING: MBOT_BALBOT_FEEDBACK node seems inactive...")
            elif time.time() - last_seen["MBOT_IMU"] > 2.0:
                print("WARNING: MBOT_IMU node seems inactive...")
        except Exception as e:
            print(f"ERROR: LCM listening error: {e}")
            break

# ============================================================================
# KINEMATIC FUNCTIONS
# ============================================================================

def calc_enc2rad(ticks):
    rad = ticks / (N_GEARBOX * N_ENC) * 2 * np.pi
    return rad

def calc_torque_conv(Tx, Ty, Tz):
    u1 = (1/3) * (Tz - (1/np.cos(np.pi/4)) * (2*Ty))
    u2 = (1/3) * (Tz + (1/np.cos(np.pi/4)) * (-np.sqrt(3)*Tx + Ty))
    u3 = (1/3) * (Tz + (1/np.cos(np.pi/4)) * (np.sqrt(3)*Tx + Ty))
    return u1, u2, u3

def calc_kinematic_conv(psi1, psi2, psi3):
    phix = (R_W / R_K) * np.sqrt(2/3) * (psi2 - psi3)
    phiy = (R_W / R_K) * np.sqrt(2)/3 * (-2*psi1 + psi2 + psi3)
    phiz = (R_W / R_K) * np.sqrt(2)/3 * (psi1 + psi2 + psi3)
    return phix, phiy, phiz

def func_clip(x, lim_lo, lim_hi):
    if x > lim_hi:
        return lim_hi
    elif x < lim_lo:
        return lim_lo
    return x

def apply_deadzone(x, deadzone):
    if abs(x) < deadzone:
        return 0.0
    return x

# ============================================================================
# PID CONTROLLER
# ============================================================================

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, integral_limit=None, output_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = None
        
    def update(self, setpoint, measurement, derivative_measurement=None):
        error = setpoint - measurement
        p_term = self.Kp * error
        
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = func_clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.Ki * self.integral
        
        if derivative_measurement is not None:
            d_term = -self.Kd * derivative_measurement
        elif self.prev_error is not None:
            derivative = (error - self.prev_error) / self.dt
            d_term = self.Kd * derivative
        else:
            d_term = 0.0
        
        self.prev_error = error
        output = p_term + i_term + d_term
        
        if self.output_limit is not None:
            output = func_clip(output, self.output_limit[0], self.output_limit[1])
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = None

# ============================================================================
# PID GAINS SAVE/LOAD
# ============================================================================

def save_pid_gains(inner_pid, yaw, filename='pid_gains.json'):
    """Save gains (X and Y share same gains)"""
    gains = {
        'inner': {'Kp': inner_pid.Kp, 'Ki': inner_pid.Ki, 'Kd': inner_pid.Kd},
        'yaw': {'Kp': yaw.Kp, 'Ki': yaw.Ki, 'Kd': yaw.Kd}
    }
    try:
        with open(filename, 'w') as f:
            json.dump(gains, f, indent=4)
        print(f"✓ PID gains saved to {filename}")
    except Exception as e:
        print(f"✗ Failed to save PID gains: {e}")

def load_pid_gains(filename='pid_gains.json'):
    """Load gains (X and Y share same gains)"""
    defaults = {
        'inner': {'Kp': 13.0, 'Ki': 0.1, 'Kd': 0.5},
        'yaw': {'Kp': 0.5, 'Ki': 0.1, 'Kd': 0.05}
    }
    try:
        with open(filename, 'r') as f:
            gains = json.load(f)
        
        # Handle old file format (inner_x, inner_y) vs new format (inner)
        if 'inner' not in gains and 'inner_x' in gains:
            # Convert old format to new format (use inner_x gains for both)
            print(f"ℹ Converting old gain format (using inner_x gains for both axes)")
            gains['inner'] = gains['inner_x']
        
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
    global listening, msg, imu_msg
    
    print("\n" + "="*80)
    print("BALLBOT PID CONTROL - LIVE D-PAD TUNING + NETWORK PLOTTING")
    print("="*80)
    print("\n🎮 D-pad Controls:")
    print("  LEFT/RIGHT: Cycle through gains (Kp → Ki → Kd → Kp)")
    print("  UP/DOWN:    Increase/decrease selected gain")
    print("  R1:         Reset IMU reference")
    print("  L3+R3:      Emergency stop")
    print("\n🎯 Face Button Modes:")
    print("  Cross (X):  Manual control")
    print("  Circle (O): Balance PID mode")
    print("="*80)
    
    trial_num = int(input("\nTest Number? "))
    filename = f"ballbot_control_{trial_num}.txt"
    dl = dataLogger(filename)
    print(f"\n✓ Data will be saved to: {filename}")
    
    # === Network Plotting Setup ===
    enable_network_plot = False
    plot_socket = None
    plot_client = None
    
    user_input = input("Enable network plotting (view on laptop)? (y/n): ").lower().strip()
    enable_network_plot = user_input == 'y'
    
    if enable_network_plot:
        plot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plot_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        plot_socket.bind(('0.0.0.0', PLOT_PORT))
        plot_socket.listen(1)
        plot_socket.settimeout(15.0)
        
        robot_hostname = socket.gethostname()
        robot_ip = socket.gethostbyname(robot_hostname)
        
        print(f"\n{'='*80}")
        print("WAITING FOR LAPTOP CONNECTION")
        print(f"{'='*80}")
        print(f"\n📊 In Matlab on your laptop, run:")
        print(f"\n   ballbot_pid_viewer('{robot_hostname}')")
        print(f"   or")
        print(f"   ballbot_pid_viewer('{robot_ip}')")
        print(f"\n{'='*80}")
        print(f"Waiting for connection (15 second timeout)...")
        
        try:
            plot_client, addr = plot_socket.accept()
            print(f"\n✓ Laptop connected from {addr[0]}:{addr[1]}")
            print("✓ Real-time plotting enabled!")
        except socket.timeout:
            print("\n✗ No connection received within 15 seconds.")
            print("Continuing without network plotting...")
            enable_network_plot = False
            plot_socket.close()
            plot_socket = None
    
    # Initialize LCM
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    lc.subscribe("MBOT_BALBOT_FEEDBACK", feedback_handler)
    lc.subscribe("MBOT_IMU", imu_handler)
    
    listening = True
    listener_thread = threading.Thread(target=lcm_listener, args=(lc,), daemon=True)
    listener_thread.start()
    print("\n✓ LCM listener started")
    
    # Initialize PS4 controller
    controller = PS4InputHandler(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True
    controller_thread.start()
    print("✓ PS4 Controller initialized")
    
    # Initialize PID controllers (X and Y share same gains)
    gains = load_pid_gains('pid_gains.json')
    
    inner_pid = PIDController(
        Kp=gains['inner']['Kp'],
        Ki=gains['inner']['Ki'],
        Kd=gains['inner']['Kd'],
        dt=DT, 
        integral_limit=0.5,
        output_limit=(-PWM_MAX, PWM_MAX)
    )
    
    yaw_pid = PIDController(
        Kp=gains['yaw']['Kp'],
        Ki=gains['yaw']['Ki'],
        Kd=gains['yaw']['Kd'],
        dt=DT,
        integral_limit=1.0,
        output_limit=(-PWM_MAX, PWM_MAX)
    )
    
    print("✓ PID controllers initialized")
    
    # D-pad gain selection state
    gain_sel = 0  # 0=Kp, 1=Ki, 2=Kd
    gain_names = ["Kp", "Ki", "Kd"]
    prev_dpad_up = 0
    prev_dpad_down = 0
    prev_dpad_left = 0
    prev_dpad_right = 0
    
    print("\n" + "="*80)
    print("Starting control loop in 0.5 seconds...")
    print("Use D-pad to tune PID gains while balancing!")
    print("="*80 + "\n")
    time.sleep(0.5)
    
    # Data logging header
    data_header = ["t", "mode", "Tx", "Ty", "Tz", "u1", "u2", "u3",
                   "theta_x_deg", "theta_y_deg", "theta_z_deg",
                   "theta_d_x_deg", "theta_d_y_deg",
                   "error_x_deg", "error_y_deg",
                   "dtheta_x_dps", "dtheta_y_dps", "dtheta_z_dps",
                   "x", "y", "dx", "dy",
                   "Kp", "Ki", "Kd"]
    dl.appendData([" ".join(data_header)])
    
    try:
        command = mbot_motor_pwm_t()
        
        i = 0
        t_start = time.time()
        
        enc_pos_1_start = msg.enc_ticks[0]
        enc_pos_2_start = msg.enc_ticks[1]
        enc_pos_3_start = msg.enc_ticks[2]
        
        theta_x_0 = msg.imu_angles_rpy[0]
        theta_y_0 = msg.imu_angles_rpy[1]
        theta_z_0 = msg.imu_angles_rpy[2]
        
        Tx = Ty = Tz = 0.0
        u1 = u2 = u3 = 0.0
        theta_d_x = theta_d_y = 0.0
        
        prev_x = prev_y = 0.0
        
        imu_filt_initialized = False
        theta_x_f = 0.0
        theta_y_f = 0.0
        theta_z_f = 0.0
        
        gyro_filt_initialized = False
        dtheta_x_f = 0.0
        dtheta_y_f = 0.0
        dtheta_z_f = 0.0
        
        control_mode = CONTROL_MODE
        prev_but_sq = 0
        prev_but_cir = 0
        prev_but_x = 0
        
        while True:
            t_now = time.time() - t_start
            i += 1
            
            try:
                # Sensor data acquisition
                theta_x_raw = msg.imu_angles_rpy[0] - theta_x_0
                theta_y_raw = msg.imu_angles_rpy[1] - theta_y_0
                theta_z_raw = msg.imu_angles_rpy[2] - theta_z_0

                if not imu_filt_initialized:
                    theta_x_f = theta_x_raw
                    theta_y_f = theta_y_raw
                    theta_z_f = theta_z_raw
                    imu_filt_initialized = True
                else:
                    theta_x_f = theta_x_f + IMU_LP_ALPHA * (theta_x_raw - theta_x_f)
                    theta_y_f = theta_y_f + IMU_LP_ALPHA * (theta_y_raw - theta_y_f)
                    theta_z_f = theta_z_f + IMU_LP_ALPHA * (theta_z_raw - theta_z_f)

                theta_x = apply_deadzone(theta_x_f, IMU_DEADZONE)
                theta_y = apply_deadzone(theta_y_f, IMU_DEADZONE)
                theta_z = apply_deadzone(theta_z_f, IMU_DEADZONE)
                
                # Gyro data
                dtheta_x_raw = imu_msg.gyro[0]
                dtheta_y_raw = imu_msg.gyro[1]
                dtheta_z_raw = imu_msg.gyro[2]
                
                if not gyro_filt_initialized:
                    dtheta_x_f = dtheta_x_raw
                    dtheta_y_f = dtheta_y_raw
                    dtheta_z_f = dtheta_z_raw
                    gyro_filt_initialized = True
                    dtheta_x = dtheta_x_raw
                    dtheta_y = dtheta_y_raw
                    dtheta_z = dtheta_z_raw
                else:
                    dtheta_x_f = dtheta_x_f + GYRO_LP_ALPHA * (dtheta_x_raw - dtheta_x_f)
                    dtheta_y_f = dtheta_y_f + GYRO_LP_ALPHA * (dtheta_y_raw - dtheta_y_f)
                    dtheta_z_f = dtheta_z_f + GYRO_LP_ALPHA * (dtheta_z_raw - dtheta_z_f)
                    
                    dtheta_x = dtheta_x_f
                    dtheta_y = dtheta_y_f
                    dtheta_z = dtheta_z_f
                
                # Kinematics
                enc_pos_1 = msg.enc_ticks[0] - enc_pos_1_start
                enc_pos_2 = msg.enc_ticks[1] - enc_pos_2_start
                enc_pos_3 = msg.enc_ticks[2] - enc_pos_3_start
                
                psi_1 = calc_enc2rad(enc_pos_1)
                psi_2 = calc_enc2rad(enc_pos_2)
                psi_3 = calc_enc2rad(enc_pos_3)
                
                phi_x, phi_y, phi_z = calc_kinematic_conv(psi_1, psi_2, psi_3)
                x = phi_x * R_K
                y = phi_y * R_K
                dx = (x - prev_x) / DT
                dy = (y - prev_y) / DT
                prev_x = x
                prev_y = y
                
                # Controller input
                bt_signals = controller.get_signals()
                js_R_x = bt_signals["js_R_x"]
                js_R_y = bt_signals["js_R_y"]
                trigger_L2 = bt_signals["trigger_L2"]
                trigger_R2 = bt_signals["trigger_R2"]
                
                but_sq = bt_signals["but_sq"]
                but_cir = bt_signals["but_cir"]
                but_x = bt_signals["but_x"]
                
                shoulder_R1 = bt_signals["shoulder_R1"]
                but_L3 = bt_signals.get("but_L3", 0)
                but_R3 = bt_signals.get("but_R3", 0)
                
                # D-pad for gain tuning
                dpad_up = bt_signals['dir_U']
                dpad_down = bt_signals['dir_D']
                dpad_left = bt_signals['dir_L']
                dpad_right = bt_signals['dir_R']
                
                # ============================================================
                # D-PAD GAIN TUNING
                # ============================================================
                
                # Cycle selection LEFT
                if dpad_left == 1 and prev_dpad_left == 0:
                    gain_sel = (gain_sel - 1) % 3
                    print(f"\n🎯 Selected: {gain_names[gain_sel]}")
                
                # Cycle selection RIGHT
                if dpad_right == 1 and prev_dpad_right == 0:
                    gain_sel = (gain_sel + 1) % 3
                    print(f"\n🎯 Selected: {gain_names[gain_sel]}")
                
                # Increase gain UP
                if dpad_up == 1 and prev_dpad_up == 0:
                    if gain_sel == 0:  # Kp
                        inner_pid.Kp = min(inner_pid.Kp + GAIN_INCREMENT_KP, GAIN_MAX)
                        print(f"  ↑ Kp = {inner_pid.Kp:.2f}")
                    elif gain_sel == 1:  # Ki
                        inner_pid.Ki = min(inner_pid.Ki + GAIN_INCREMENT_KI, GAIN_MAX)
                        print(f"  ↑ Ki = {inner_pid.Ki:.3f}")
                    elif gain_sel == 2:  # Kd
                        inner_pid.Kd = min(inner_pid.Kd + GAIN_INCREMENT_KD, GAIN_MAX)
                        print(f"  ↑ Kd = {inner_pid.Kd:.2f}")
                    # Reset integral when gains change
                    inner_pid.reset()
                
                # Decrease gain DOWN
                if dpad_down == 1 and prev_dpad_down == 0:
                    if gain_sel == 0:  # Kp
                        inner_pid.Kp = max(inner_pid.Kp - GAIN_INCREMENT_KP, GAIN_MIN)
                        print(f"  ↓ Kp = {inner_pid.Kp:.2f}")
                    elif gain_sel == 1:  # Ki
                        inner_pid.Ki = max(inner_pid.Ki - GAIN_INCREMENT_KI, GAIN_MIN)
                        print(f"  ↓ Ki = {inner_pid.Ki:.3f}")
                    elif gain_sel == 2:  # Kd
                        inner_pid.Kd = max(inner_pid.Kd - GAIN_INCREMENT_KD, GAIN_MIN)
                        print(f"  ↓ Kd = {inner_pid.Kd:.2f}")
                    # Reset integral when gains change
                    inner_pid.reset()
                
                # Store D-pad states
                prev_dpad_up = dpad_up
                prev_dpad_down = dpad_down
                prev_dpad_left = dpad_left
                prev_dpad_right = dpad_right
                
                # ============================================================
                # OTHER BUTTON FUNCTIONS
                # ============================================================
                
                # Reference reset
                if shoulder_R1 == 1:
                    theta_x_0 = msg.imu_angles_rpy[0]
                    theta_y_0 = msg.imu_angles_rpy[1]
                    theta_z_0 = msg.imu_angles_rpy[2]
                    print("\n>>> IMU REFERENCE RESET <<<\n")
                
                # Emergency stop
                if but_L3 == 1 and but_R3 == 1:
                    print("\n!!! EMERGENCY STOP !!!")
                    command = mbot_motor_pwm_t()
                    command.utime = int(time.time() * 1e6)
                    command.pwm[0] = 0.0
                    command.pwm[1] = 0.0
                    command.pwm[2] = 0.0
                    lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                    break
                
                # Mode switching
                mode_changed = False
                
                if but_x == 1 and prev_but_x == 0:
                    control_mode = 1
                    mode_changed = True
                elif but_cir == 1 and prev_but_cir == 0:
                    control_mode = 2
                    mode_changed = True
                
                prev_but_sq = but_sq
                prev_but_cir = but_cir
                prev_but_x = but_x
                
                if mode_changed:
                    inner_pid.reset()
                    yaw_pid.reset()
                    theta_d_x = theta_d_y = 0.0
                    print(f"\n>>> MODE CHANGED TO {control_mode} <<<\n")
                
                # ============================================================
                # CONTROL LOGIC
                # ============================================================
                
                if control_mode == 1:
                    Tx = js_R_y
                    Ty = js_R_x
                    Tz = trigger_R2 - trigger_L2
                    
                elif control_mode == 2:
                    js_R_y_filtered = 0.0 if abs(js_R_y) < 0.1 else js_R_y
                    js_R_x_filtered = 0.0 if abs(js_R_x) < 0.1 else js_R_x
                    
                    theta_d_x = js_R_y_filtered * 0.1
                    theta_d_y = js_R_x_filtered * 0.1
                    
                    # Both X and Y use same PID gains
                    Tx = inner_pid.update(theta_d_y, theta_y, derivative_measurement=dtheta_y)
                    Ty = inner_pid.update(theta_d_x, theta_x, derivative_measurement=dtheta_x)
                    
                    if abs(trigger_R2 - trigger_L2) > 0.05:
                        dpsi_d = (trigger_R2 - trigger_L2) * YAW_RATE_MAX
                        Tz = yaw_pid.update(dpsi_d, dtheta_z)
                    else:
                        Tz = 0.0
                
                else:
                    Tx = Ty = Tz = 0
                
                # Motor commands
                u1, u2, u3 = calc_torque_conv(Tx, Ty, Tz)
                u1 = func_clip(u1, PWM_MIN, PWM_MAX)
                u2 = func_clip(u2, PWM_MIN, PWM_MAX)
                u3 = func_clip(u3, PWM_MIN, PWM_MAX)
                
                cmd_utime = int(time.time() * 1e6)
                command.utime = cmd_utime
                command.pwm[0] = u1
                command.pwm[1] = u2
                command.pwm[2] = u3
                lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                
                # Send data to laptop for plotting
                if enable_network_plot and plot_client:
                    try:
                        data_packet = {
                            't': t_now,
                            'theta_x_deg': np.degrees(theta_x),
                            'theta_y_deg': np.degrees(theta_y),
                            'theta_d_x_deg': np.degrees(theta_d_x),
                            'theta_d_y_deg': np.degrees(theta_d_y),
                            'error_x_deg': np.degrees(theta_d_x - theta_x),
                            'error_y_deg': np.degrees(theta_d_y - theta_y),
                            'Tx': Tx,
                            'Ty': Ty,
                            'dtheta_x_dps': np.degrees(dtheta_x),
                            'dtheta_y_dps': np.degrees(dtheta_y),
                        }
                        json_str = json.dumps(data_packet) + '\n'
                        plot_client.sendall(json_str.encode('utf-8'))
                    except (BrokenPipeError, ConnectionResetError):
                        print("✗ Laptop disconnected")
                        enable_network_plot = False
                        plot_client.close()
                        plot_client = None
                    except Exception as e:
                        pass
                
                # Data logging
                data = [
                    t_now, control_mode,
                    Tx, Ty, Tz,
                    u1, u2, u3,
                    np.degrees(theta_x), np.degrees(theta_y), np.degrees(theta_z),
                    np.degrees(theta_d_x), np.degrees(theta_d_y),
                    np.degrees(theta_d_x - theta_x), np.degrees(theta_d_y - theta_y),
                    np.degrees(dtheta_x), np.degrees(dtheta_y), np.degrees(dtheta_z),
                    x, y, dx, dy,
                    inner_pid.Kp, inner_pid.Ki, inner_pid.Kd,
                ]
                dl.appendData(data)
                
                # ============================================================
                # TERMINAL OUTPUT WITH HIGHLIGHTED SELECTED GAIN
                # ============================================================
                
                if i % 20 == 0:
                    mode_names = {1: "Manual", 2: "Balance"}
                    mode_name = mode_names.get(control_mode, f"Mode{control_mode}")
                    
                    # Format gain values
                    kp_str = f"{inner_pid.Kp:.2f}"
                    ki_str = f"{inner_pid.Ki:.3f}"
                    kd_str = f"{inner_pid.Kd:.2f}"
                    
                    # Highlight selected gain
                    if gain_sel == 0:
                        kp_str = f"{H_START}{kp_str}{H_END}"
                    elif gain_sel == 1:
                        ki_str = f"{H_START}{ki_str}{H_END}"
                    elif gain_sel == 2:
                        kd_str = f"{H_START}{kd_str}{H_END}"
                    
                    print(
                        f"[{mode_name}] "
                        f"t={t_now:.1f}s | "
                        f"err=[{np.degrees(theta_d_x - theta_x):+.1f}°, {np.degrees(theta_d_y - theta_y):+.1f}°] | "
                        f"PID: Kp={kp_str}, Ki={ki_str}, Kd={kd_str}"
                    )
            
            except KeyError as e:
                if i % 100 == 0:  # Only print occasionally
                    print(f"WARNING: Waiting for sensor data... (missing key: {e})")
                continue
            except Exception as e:
                print(f"ERROR in control loop: {e}")
                import traceback
                traceback.print_exc()
                continue
            
            time.sleep(DT)
    
    except KeyboardInterrupt:
        print("\n\nKEYBOARD INTERRUPT - EMERGENCY STOP")
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        print("✓ Motors stopped")
    
    finally:
        print("\n" + "="*80)
        print("SHUTTING DOWN")
        print("="*80)
        
        print(f"\n→ Saving data to {filename}...")
        dl.writeOut()
        print("✓ Data saved successfully")
        
        print("\n→ Saving PID gains...")
        save_pid_gains(inner_pid, yaw_pid)
        
        if plot_client:
            print("\n→ Closing network plot connection...")
            plot_client.close()
        if plot_socket:
            plot_socket.close()
        
        print("\n→ Stopping LCM listener...")
        listener_thread.join(timeout=1)
        
        print("\n→ Stopping controller...")
        controller_thread.join(timeout=1)
        controller.on_options_press()
        
        print("\n→ Final motor shutdown...")
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        print("✓ Motors confirmed off")
        
        print("\n📊 Final PID Gains:")
        print(f"   Kp = {inner_pid.Kp:.2f}")
        print(f"   Ki = {inner_pid.Ki:.3f}")
        print(f"   Kd = {inner_pid.Kd:.2f}")
        
        print("\n" + "="*80)
        print("SHUTDOWN COMPLETE")
        print("="*80 + "\n")

if __name__ == "__main__":
    main()