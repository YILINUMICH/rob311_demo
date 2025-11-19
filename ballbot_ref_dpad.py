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
- PS4 controller connected via Bluetooth

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
# 
# BEGINNER'S NOTE - Choosing Increment Values:
# ---------------------------------------------
# These constants control how much each gain changes with each button press.
# 
# Why different increments for different gains?
# - Kp (Proportional) typically needs larger adjustments → 0.1 increment
# - Ki (Integral) is very sensitive, needs small changes → 0.01 increment  
# - Kd (Derivative) is moderately sensitive → 0.05 increment
# 
# You should tune these based on your specific system!
# Start with small increments and increase if tuning takes too long.
# 

# How much to increment/decrement gains with each D-pad press
GAIN_INCREMENT_KP = 0.1   # Proportional gain step size
GAIN_INCREMENT_KI = 0.01  # Integral gain step size (smaller!)
GAIN_INCREMENT_KD = 0.05  # Derivative gain step size

# Minimum and maximum allowed gain values (safety limits)
# These prevent you from accidentally setting gains too high or negative
GAIN_MIN = 0.0   # Never allow negative gains
GAIN_MAX = 10.0  # Cap maximum gain to prevent instability


# ============================================================================
# MAIN DEMO FUNCTION
# ============================================================================

def main():
    """
    Main demo loop that shows D-pad control with visual feedback.
    
    BEGINNER'S GUIDE - How This Works:
    ==================================
    
    1. SETUP PHASE:
       - Connect to PS4 controller
       - Create PID controller objects with initial gain values
       - Initialize variables to track which gain is selected
    
    2. MAIN LOOP (repeats ~20 times per second):
       - Read current button states from controller
       - DETECT button presses (not just "is button down?")
       - Update gains when buttons are pressed
       - Display current values with visual highlighting
    
    3. THE KEY CONCEPT - "Edge Detection":
       Instead of asking "Is the button pressed?", we ask:
       "Did the button JUST get pressed (was it 0, now it's 1)?"
       
       This prevents rapid-fire changes when holding a button!
       
       Example:
         Frame 1: Button is 0 (not pressed), previous was 0 → No change
         Frame 2: Button is 1 (pressed!), previous was 0   → CHANGE VALUE!
         Frame 3: Button is 1 (still pressed), previous was 1 → No change
         Frame 4: Button is 1 (still pressed), previous was 1 → No change
         Frame 5: Button is 0 (released), previous was 1      → No change
       
       The value only changes ONCE when you press, not continuously!
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
    # Think of this like a "cursor" that points to one of three parameters
    # 0 = Kp, 1 = Ki, 2 = Kd
    gain_sel = 0
    gain_names = ["Kp", "Ki", "Kd"]  # Human-readable names for display
    
    # ========================================================================
    # PREVIOUS BUTTON STATES (for Edge Detection)
    # ========================================================================
    # 
    # WHY DO WE NEED THESE?
    # ---------------------
    # To detect a "button press event" (the moment button goes from OFF to ON),
    # we need to remember what the button state was in the PREVIOUS loop iteration.
    # 
    # Think of it like this:
    #   - If button is 1 NOW and was 0 BEFORE → Button JUST got pressed! (rising edge)
    #   - If button is 1 NOW and was 1 BEFORE → Button is being HELD (no action)
    #   - If button is 0 NOW and was 1 BEFORE → Button JUST got released (falling edge)
    #   - If button is 0 NOW and was 0 BEFORE → Button is not pressed (no action)
    # 
    # This technique is called "edge detection" and is fundamental in robotics!
    # 
    prev_dpad_up = 0      # Was D-pad UP pressed in the last loop iteration?
    prev_dpad_down = 0    # Was D-pad DOWN pressed in the last loop iteration?
    prev_dpad_left = 0    # Was D-pad LEFT pressed in the last loop iteration?
    prev_dpad_right = 0   # Was D-pad RIGHT pressed in the last loop iteration?
    
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
            # 
            # STEP 1: Read the current state of the controller
            # -------------------------------------------------
            
            # Get current controller signals dictionary
            # This contains ALL button and joystick values updated in real-time
            signals = controller.get_signals()
            
            # Extract just the D-pad button states we care about
            # Each value is either 0 (not pressed) or 1 (pressed)
            dpad_up = signals['dir_U']      # D-pad UP button (increase value)
            dpad_down = signals['dir_D']    # D-pad DOWN button (decrease value)
            dpad_left = signals['dir_L']    # D-pad LEFT button (cycle backward)
            dpad_right = signals['dir_R']   # D-pad RIGHT button (cycle forward)
            
            # ----------------------------------------------------------------
            # SELECTION CYCLING (Left/Right D-pad)
            # ----------------------------------------------------------------
            # 
            # STEP 2: Check if user wants to change which parameter is selected
            # ------------------------------------------------------------------
            # 
            # The LEFT and RIGHT D-pad buttons cycle through the available
            # parameters (Kp, Ki, Kd). This is like moving a cursor.
            # 
            # BEGINNER NOTE - Understanding the Condition:
            # --------------------------------------------
            #   if dpad_left == 1 and prev_dpad_left == 0:
            #   
            #   This checks TWO things:
            #   1. dpad_left == 1        → Button IS currently pressed
            #   2. prev_dpad_left == 0   → Button WAS NOT pressed before
            #   
            #   Both must be TRUE = This is a NEW button press (rising edge)!
            #   
            #   This runs code ONCE per button press, not continuously.
            # 
            
            # Detect LEFT button press (cycle selection backward)
            if dpad_left == 1 and prev_dpad_left == 0:
                # Use modulo (%) operator to wrap around:
                # If gain_sel is 0, subtract 1 to get -1, then % 3 gives 2
                # If gain_sel is 1, subtract 1 to get 0
                # If gain_sel is 2, subtract 1 to get 1
                # Result: 0→2, 1→0, 2→1 (cycles backward with wrap-around)
                gain_sel = (gain_sel - 1) % 3
                print(f"\n→ Selected: {gain_names[gain_sel]}")
            
            # Detect RIGHT button press (cycle selection forward)
            if dpad_right == 1 and prev_dpad_right == 0:
                # Use modulo (%) operator to wrap around:
                # If gain_sel is 0, add 1 to get 1
                # If gain_sel is 1, add 1 to get 2
                # If gain_sel is 2, add 1 to get 3, then % 3 gives 0
                # Result: 0→1, 1→2, 2→0 (cycles forward with wrap-around)
                gain_sel = (gain_sel + 1) % 3
                print(f"\n→ Selected: {gain_names[gain_sel]}")
            
            # ----------------------------------------------------------------
            # VALUE ADJUSTMENT (Up/Down D-pad)
            # ----------------------------------------------------------------
            # 
            # STEP 3: Check if user wants to increase or decrease the selected gain
            # ----------------------------------------------------------------------
            # 
            # Once a parameter is selected (Kp, Ki, or Kd), we can adjust its value
            # using the UP and DOWN D-pad buttons.
            # 
            # BEGINNER NOTE - Understanding the Update Logic:
            # ------------------------------------------------
            #   demo_pid.Kp = min(demo_pid.Kp + GAIN_INCREMENT_KP, GAIN_MAX)
            #   
            #   Let's break this down:
            #   1. demo_pid.Kp + GAIN_INCREMENT_KP  → Add increment to current value
            #   2. min(..., GAIN_MAX)               → Take the smaller of (new value, max allowed)
            #   
            #   This ensures the value never exceeds GAIN_MAX!
            #   
            #   Example: If Kp = 9.8, increment = 0.1, max = 10.0:
            #     - New value would be 9.8 + 0.1 = 9.9
            #     - min(9.9, 10.0) = 9.9 ✓ (allowed)
            #   
            #   Example: If Kp = 9.95, increment = 0.1, max = 10.0:
            #     - New value would be 9.95 + 0.1 = 10.05
            #     - min(10.05, 10.0) = 10.0 ✓ (clamped to maximum)
            #   
            #   Similarly, max(..., GAIN_MIN) ensures value never goes below minimum.
            # 
            
            # Detect UP button press (increase selected gain)
            if dpad_up == 1 and prev_dpad_up == 0:
                # Check which parameter is currently selected
                if gain_sel == 0:  # Kp selected
                    # Increase Kp by increment, but don't exceed maximum
                    demo_pid.Kp = min(demo_pid.Kp + GAIN_INCREMENT_KP, GAIN_MAX)
                    print(f"  ↑ Increased Kp to {demo_pid.Kp:.3f}")
                    
                elif gain_sel == 1:  # Ki selected
                    # Increase Ki by increment, but don't exceed maximum
                    demo_pid.Ki = min(demo_pid.Ki + GAIN_INCREMENT_KI, GAIN_MAX)
                    print(f"  ↑ Increased Ki to {demo_pid.Ki:.3f}")
                    
                elif gain_sel == 2:  # Kd selected
                    # Increase Kd by increment, but don't exceed maximum
                    demo_pid.Kd = min(demo_pid.Kd + GAIN_INCREMENT_KD, GAIN_MAX)
                    print(f"  ↑ Increased Kd to {demo_pid.Kd:.3f}")
            
            # Detect DOWN button press (decrease selected gain)
            if dpad_down == 1 and prev_dpad_down == 0:
                # Check which parameter is currently selected
                if gain_sel == 0:  # Kp selected
                    # Decrease Kp by increment, but don't go below minimum
                    # max() ensures we never get negative or too-small values
                    demo_pid.Kp = max(demo_pid.Kp - GAIN_INCREMENT_KP, GAIN_MIN)
                    print(f"  ↓ Decreased Kp to {demo_pid.Kp:.3f}")
                    
                elif gain_sel == 1:  # Ki selected
                    # Decrease Ki by increment, but don't go below minimum
                    demo_pid.Ki = max(demo_pid.Ki - GAIN_INCREMENT_KI, GAIN_MIN)
                    print(f"  ↓ Decreased Ki to {demo_pid.Ki:.3f}")
                    
                elif gain_sel == 2:  # Kd selected
                    # Decrease Kd by increment, but don't go below minimum
                    demo_pid.Kd = max(demo_pid.Kd - GAIN_INCREMENT_KD, GAIN_MIN)
                    print(f"  ↓ Decreased Kd to {demo_pid.Kd:.3f}")
            
            # ----------------------------------------------------------------
            # UPDATE PREVIOUS STATES (Critical for Edge Detection!)
            # ----------------------------------------------------------------
            # 
            # STEP 4: Remember current states for next loop iteration
            # --------------------------------------------------------
            # 
            # ⚠️ THIS IS CRUCIAL! ⚠️
            # 
            # At the END of each loop iteration, we save the current button states
            # so they become the "previous" states in the NEXT iteration.
            # 
            # Without this step, edge detection won't work because we'd have no
            # way to know if a button state has CHANGED.
            # 
            # Think of it as: "What's current now will be history next time."
            # 
            # Example timeline:
            #   Loop 1: current=0, prev=0 → No press detected, then prev=0 stored
            #   Loop 2: current=1, prev=0 → PRESS DETECTED!, then prev=1 stored
            #   Loop 3: current=1, prev=1 → No new press (held), then prev=1 stored
            #   Loop 4: current=0, prev=1 → Button released, then prev=0 stored
            # 
            prev_dpad_up = dpad_up        # Current becomes previous for next time
            prev_dpad_down = dpad_down    # Current becomes previous for next time
            prev_dpad_left = dpad_left    # Current becomes previous for next time
            prev_dpad_right = dpad_right  # Current becomes previous for next time
            
            # ================================================================
            # VISUAL FEEDBACK DISPLAY
            # ================================================================
            # 
            # STEP 5: Show the user what's happening
            # ---------------------------------------
            # 
            # Good user interfaces provide clear feedback! Here we print the
            # current parameter values with visual highlighting to show which
            # one is selected.
            # 
            
            # Display status every 10 iterations (0.5 seconds at 20 Hz)
            # We don't print every iteration to avoid cluttering the terminal
            if iteration % 10 == 0:
                # Convert gain values to formatted strings (3 decimal places)
                kp_str = f"{demo_pid.Kp:.3f}"  # e.g., "1.500"
                ki_str = f"{demo_pid.Ki:.3f}"  # e.g., "0.100"
                kd_str = f"{demo_pid.Kd:.3f}"  # e.g., "0.500"
                
                # Apply highlighting to the currently selected parameter
                # This makes it OBVIOUS which parameter will be adjusted
                # when you press UP or DOWN!
                # 
                # The H_START and H_END codes are ANSI escape sequences that
                # tell the terminal to reverse colors (swap foreground/background)
                # 
                if gain_sel == 0:
                    # Kp is selected - wrap its string with highlight codes
                    kp_str = f"{H_START}{kp_str}{H_END}"
                elif gain_sel == 1:
                    # Ki is selected - wrap its string with highlight codes
                    ki_str = f"{H_START}{ki_str}{H_END}"
                elif gain_sel == 2:
                    # Kd is selected - wrap its string with highlight codes
                    kd_str = f"{H_START}{kd_str}{H_END}"
                
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
    
    Problem: Highlighting doesn't show up
    - Some terminals don't support ANSI escape codes
    - Try running in a different terminal (xterm, gnome-terminal, etc.)
    - The H_START and H_END codes can be changed to different styles
    
    Problem: Values change too fast when holding button
    - This is intentional! The code detects button "edges" (press events)
    - Only changes value when button transitions from 0→1
    - Does NOT change when button is held at 1
    
    main()
