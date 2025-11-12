"""
D-pad Control Demo for ROB 311
================================

This demo illustrates how to use the PS4 controller D-pad to dynamically adjust
PID gain values with visual feedback in the terminal.

Learning Objectives:
1. Understand how to handle D-pad button events
2. Implement selection cycling through multiple parameters
3. Provide visual feedback using terminal highlighting
4. Update values dynamically based on user input

Hardware Requirements:
- PS4 controller connected via USB or Bluetooth

Key Concepts:
- D-pad Left/Right: Cycle through which gain to adjust (Kp, Ki, Kd)
- D-pad Up/Down: Increase/decrease the selected gain value
- Terminal highlighting shows which parameter is currently selected

Author: ROB 311 Teaching Team
Date: November 2025
"""

import time
import lcm
import threading
import sys
import os
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t
from mbot_lcm_msgs.mbot_balbot_feedback_t import mbot_balbot_feedback_t
from DataLogger import dataLogger
from ps4_controller_api import PS4InputHandler

# ============================================================================
# TERMINAL HIGHLIGHTING SETUP
# ============================================================================

# ANSI escape codes for terminal text highlighting
# These codes make selected values stand out in the terminal output
H_START = '\033[7m'  # Reverse video (swap background/foreground colors)
H_END = '\033[0m'    # Reset to normal text

# Alternative highlighting styles you can try:
# H_START = '\033[1;32m'  # Bold green text
# H_START = '\033[1;33m'  # Bold yellow text
# H_START = '\033[4m'     # Underlined text

# ============================================================================
# SIMULATED PID CLASS (for demonstration purposes)
# ============================================================================

class SimplePID:
    """
    Simple PID controller class to demonstrate gain adjustment.
    In the real ballbot code, this would be the actual PID controller.
    """
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, name="PID"):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.name = name
    
    def __str__(self):
        return f"{self.name}(Kp={self.Kp:.3f}, Ki={self.Ki:.3f}, Kd={self.Kd:.3f})"


# ============================================================================
# GAIN ADJUSTMENT PARAMETERS
# ============================================================================

# How much to increment/decrement gains with each D-pad press
GAIN_INCREMENT_KP = 0.1
GAIN_INCREMENT_KI = 0.01
GAIN_INCREMENT_KD = 0.05

# Minimum and maximum allowed gain values (safety limits)
GAIN_MIN = 0.0
GAIN_MAX = 10.0


# ============================================================================
# MAIN DEMO FUNCTION
# ============================================================================

def main():
    """
    Main demo loop that shows D-pad control with visual feedback.
    """
    
    print("="*80)
    print("D-PAD CONTROL DEMO - ROB 311")
    print("="*80)
    print("\nController Instructions:")
    print("  D-pad LEFT/RIGHT : Cycle through parameters (Kp → Ki → Kd → Kp)")
    print("  D-pad UP         : Increase selected parameter")
    print("  D-pad DOWN       : Decrease selected parameter")
    print("  Ctrl+C           : Exit demo")
    print("\nThe currently selected parameter will be highlighted in the display.")
    print("="*80 + "\n")
    
    # ========================================================================
    # CONTROLLER INITIALIZATION
    # ========================================================================
    
    # Initialize PS4 controller
    print("→ Initializing PS4 controller...")
    try:
        controller = PS4InputHandler(
            interface='/dev/input/js0',
            connecting_using_ds4drv=False
        )
        print("✓ Controller connected successfully\n")
    except Exception as e:
        print(f"ERROR: Could not initialize controller: {e}")
        print("Make sure your PS4 controller is connected!")
        return
    
    # Start controller in separate thread
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True
    controller_thread.start()
    time.sleep(0.5)  # Brief delay to ensure controller is ready
    
    # ========================================================================
    # PID CONTROLLER SETUP
    # ========================================================================
    
    # Create a demo PID controller with initial gains
    # In the real ballbot code, these would be actual controllers
    demo_pid = SimplePID(Kp=1.0, Ki=0.1, Kd=0.5, name="Inner Loop X")
    
    # ========================================================================
    # GAIN SELECTION STATE
    # ========================================================================
    
    # Track which gain is currently selected for adjustment
    # 0 = Kp, 1 = Ki, 2 = Kd
    gain_sel = 0
    gain_names = ["Kp", "Ki", "Kd"]
    
    # Store previous D-pad states to detect new button presses
    # This prevents holding a button from rapidly changing values
    prev_dpad_up = False
    prev_dpad_down = False
    prev_dpad_left = False
    prev_dpad_right = False
    
    print("Demo running! Use D-pad to adjust gains.\n")
    
    # ========================================================================
    # MAIN CONTROL LOOP
    # ========================================================================
    
    try:
        iteration = 0
        running = True
        
        while running:  # Run continuously
            time.sleep(0.05)  # 20 Hz update rate for this demo
            iteration += 1
            
            # ================================================================
            # D-PAD INPUT HANDLING
            # ================================================================
            
            # Get current controller signals
            signals = controller.get_signals()
            
            # Get current D-pad button states (0 = not pressed, 1 = pressed)
            dpad_up = signals['dir_U']
            dpad_down = signals['dir_D']
            dpad_left = signals['dir_L']
            dpad_right = signals['dir_R']
            
            # ----------------------------------------------------------------
            # SELECTION CYCLING (Left/Right D-pad)
            # ----------------------------------------------------------------
            
            # Detect LEFT button press (cycle selection backward)
            if dpad_left == 1 and prev_dpad_left == 0:
                gain_sel = (gain_sel - 1) % 3  # Wrap around: 0→2, 1→0, 2→1
                print(f"\n→ Selected: {gain_names[gain_sel]}")
            
            # Detect RIGHT button press (cycle selection forward)
            if dpad_right == 1 and prev_dpad_right == 0:
                gain_sel = (gain_sel + 1) % 3  # Wrap around: 0→1, 1→2, 2→0
                print(f"\n→ Selected: {gain_names[gain_sel]}")
            
            # ----------------------------------------------------------------
            # VALUE ADJUSTMENT (Up/Down D-pad)
            # ----------------------------------------------------------------
            
            # Detect UP button press (increase selected gain)
            if dpad_up == 1 and prev_dpad_up == 0:
                if gain_sel == 0:  # Kp selected
                    demo_pid.Kp = min(demo_pid.Kp + GAIN_INCREMENT_KP, GAIN_MAX)
                    print(f"  ↑ Increased Kp to {demo_pid.Kp:.3f}")
                elif gain_sel == 1:  # Ki selected
                    demo_pid.Ki = min(demo_pid.Ki + GAIN_INCREMENT_KI, GAIN_MAX)
                    print(f"  ↑ Increased Ki to {demo_pid.Ki:.3f}")
                elif gain_sel == 2:  # Kd selected
                    demo_pid.Kd = min(demo_pid.Kd + GAIN_INCREMENT_KD, GAIN_MAX)
                    print(f"  ↑ Increased Kd to {demo_pid.Kd:.3f}")
            
            # Detect DOWN button press (decrease selected gain)
            if dpad_down == 1 and prev_dpad_down == 0:
                if gain_sel == 0:  # Kp selected
                    demo_pid.Kp = max(demo_pid.Kp - GAIN_INCREMENT_KP, GAIN_MIN)
                    print(f"  ↓ Decreased Kp to {demo_pid.Kp:.3f}")
                elif gain_sel == 1:  # Ki selected
                    demo_pid.Ki = max(demo_pid.Ki - GAIN_INCREMENT_KI, GAIN_MIN)
                    print(f"  ↓ Decreased Ki to {demo_pid.Ki:.3f}")
                elif gain_sel == 2:  # Kd selected
                    demo_pid.Kd = max(demo_pid.Kd - GAIN_INCREMENT_KD, GAIN_MIN)
                    print(f"  ↓ Decreased Kd to {demo_pid.Kd:.3f}")
            
            # Store current button states for next iteration
            prev_dpad_up = dpad_up
            prev_dpad_down = dpad_down
            prev_dpad_left = dpad_left
            prev_dpad_right = dpad_right
            
            # ================================================================
            # VISUAL FEEDBACK DISPLAY
            # ================================================================
            
            # Display status every 10 iterations (0.5 seconds at 20 Hz)
            if iteration % 10 == 0:
                # Prepare gain value strings
                kp_str = f"{demo_pid.Kp:.3f}"
                ki_str = f"{demo_pid.Ki:.3f}"
                kd_str = f"{demo_pid.Kd:.3f}"
                
                # Apply highlighting to the currently selected parameter
                # This is the key feature that provides visual feedback!
                if gain_sel == 0:
                    kp_str = f"{H_START}{kp_str}{H_END}"  # Highlight Kp
                elif gain_sel == 1:
                    ki_str = f"{H_START}{ki_str}{H_END}"  # Highlight Ki
                elif gain_sel == 2:
                    kd_str = f"{H_START}{kd_str}{H_END}"  # Highlight Kd
                
                # Print formatted status line
                # The highlighted value will appear with reversed colors
                print(
                    f"[iter {iteration:4d}] "
                    f"Selected: {gain_names[gain_sel]:2s} | "
                    f"PID Gains → Kp={kp_str}, Ki={ki_str}, Kd={kd_str}"
                )
    
    # ========================================================================
    # EXCEPTION HANDLING AND CLEANUP
    # ========================================================================
    
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt detected!")
    
    finally:
        print("\n" + "="*80)
        print("SHUTTING DOWN DEMO")
        print("="*80)
        
        # Display final gain values
        print(f"\nFinal PID Gains:")
        print(f"  Kp = {demo_pid.Kp:.3f}")
        print(f"  Ki = {demo_pid.Ki:.3f}")
        print(f"  Kd = {demo_pid.Kd:.3f}")
        
        # Stop controller thread
        print("\n→ Stopping controller...")
        controller_thread.join(timeout=1)
        controller.on_options_press()
        if controller_thread.is_alive():
            print("WARNING: Controller thread did not stop cleanly")
        else:
            print("✓ Controller stopped")
        
        print("\n" + "="*80)
        print("DEMO COMPLETE")
        print("="*80 + "\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    """
    TROUBLESHOOTING:
    
    Problem: Controller not detected
    - Check USB connection or Bluetooth pairing
    - Try: ls /dev/input/js*
    - If using ds4drv, set connecting_using_ds4drv=True
    - Test controller: jstest /dev/input/js0
    
    Problem: Highlighting doesn't show up
    - Some terminals don't support ANSI escape codes
    - Try running in a different terminal (xterm, gnome-terminal, etc.)
    - The H_START and H_END codes can be changed to different styles
    
    Problem: Values change too fast when holding button
    - This is intentional! The code detects button "edges" (press events)
    - Only changes value when button transitions from 0→1
    - Does NOT change when button is held at 1
    
    LEARNING EXERCISES:
    
    1. Modify the increment values (GAIN_INCREMENT_*) to see how it affects control
    2. Add a fourth parameter (e.g., setpoint) to the selection cycle
    3. Change the highlighting style (try different H_START codes)
    4. Add bounds checking that prints a warning when limits are reached
    5. Implement a "save gains to file" feature on a button press
    """
    
    main()
