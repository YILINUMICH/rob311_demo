"""
ROB 311 - Fall 2025
Author: Prof. Greg Formosa & GSI Yilin Ma
University of Michigan

PS4 Input Handler API - from https://pypi.org/project/pyPS4Controller/
Import this class into your control/test files to interface with your Bluetooth controller.

OVERVIEW:
=========
This module provides the `PS4InputHandler` class to interface with a PS4 DualShock controller.
It captures inputs from various controller elements and processes them into normalized values
suitable for robotics and control systems.

SUPPORTED INPUTS:
=================
- Left and Right Joysticks (L3, R3): Analog 2D axes, normalized to [-1.0, 1.0]
- Triggers (L2, R2): Analog pressure-sensitive, normalized to [0.0, 1.0]
- Shoulder Buttons (L1, R1): Digital buttons, returns 0 or 1
- Face Buttons (X, Circle, Triangle, Square): Digital buttons, returns 0 or 1
- D-Pad (Up, Down, Left, Right): Digital directional buttons, returns 0 or 1
- Joystick Click Buttons (L3, R3): Digital buttons when joysticks are pressed down, returns 0 or 1

NOTE: Touchpad functionality is NOT supported and has been removed from this implementation.

EMERGENCY STOP:
===============
The "Options" button is designated as an emergency stop. Pressing it will immediately
terminate the controller thread and exit the program. Use this for safety purposes.

SIGNAL NORMALIZATION:
=====================
- Joystick axes: Raw values [-32767, 32767] → Normalized [-1.0, 1.0]
- Triggers: Raw values [-32767, 32767] → Normalized [0.0, 1.0]
- Digital buttons: Binary values 0 (released) or 1 (pressed)

All signals are stored in a dictionary accessible via the `get_signals()` method and
updated in real-time as controller events are received.

EXAMPLE USAGE:
==============
    # 1. Initialize the PS4InputHandler with the controller's device interface
    handler = PS4InputHandler(interface="/dev/input/js0")
    
    # 2. Start the controller's event listener in a background thread
    handler.listen(timeout=10)  # Optional timeout in seconds
    
    # 3. In your control loop, retrieve real-time control signals
    while True:
        signals = handler.get_signals()
        
        # Use joystick values for robot control
        forward_speed = signals['js_L_y']    # Left stick Y-axis
        turn_rate = signals['js_R_x']         # Right stick X-axis
        
        # Check if a button is pressed
        if signals['but_x'] == 1:
            print("X button pressed!")
        
        # Emergency stop is automatic via Options button

THREADING NOTE:
===============
The controller runs in a separate background thread started by the listen() method.
All event handlers update the signals dictionary in real-time, so calling get_signals()
always returns the most current controller state.

"""

import sys
import numpy as np
from pyPS4Controller.controller import Controller

# Scale factor for normalizing joystick and trigger values
JOYSTICK_SCALE = 32767

class PS4InputHandler(Controller):
    """
    PS4 Controller Input Handler for Robot Control
    
    This class extends the pyPS4Controller.Controller to provide normalized
    input signals from a PS4 DualShock controller. All analog inputs (joysticks
    and triggers) are normalized to [0.0, 1.0] or [-1.0, 1.0] ranges, while
    digital buttons return binary values (0 or 1).
    
    Args:
        interface (str): The device path for the controller (e.g., "/dev/input/js0")
        **kwargs: Additional arguments passed to the parent Controller class
    """
    
    def __init__(self, interface, **kwargs):
        super().__init__(interface, **kwargs)
        
        # Dictionary storing all controller input states
        # Updated in real-time as controller events are received
        self.signals = {
            # Left Joystick (L3) - Analog inputs normalized to [-1.0, 1.0]
            # Positive X = right, Positive Y = down
            "js_L_x": 0.0,    # Left joystick horizontal axis
            "js_L_y": 0.0,    # Left joystick vertical axis
            
            # Right Joystick (R3) - Analog inputs normalized to [-1.0, 1.0]
            # Positive X = right, Positive Y = down
            "js_R_x": 0.0,    # Right joystick horizontal axis
            "js_R_y": 0.0,    # Right joystick vertical axis
            
            # Triggers - Analog inputs normalized to [0.0, 1.0]
            # 0.0 = not pressed, 1.0 = fully pressed
            "trigger_L2": 0.0,  # Left trigger (L2)
            "trigger_R2": 0.0,  # Right trigger (R2)
            
            # Shoulder Buttons - Digital inputs (0 or 1)
            "shoulder_L1": 0,   # Left shoulder button (L1)
            "shoulder_R1": 0,   # Right shoulder button (R1)
            
            # Face Buttons - Digital inputs (0 or 1)
            "but_x": 0,       # X button (bottom)
            "but_cir": 0,     # Circle button (right)
            "but_tri": 0,     # Triangle button (top)
            "but_sq": 0,      # Square button (left)
            
            # D-Pad Buttons - Digital inputs (0 or 1)
            "dir_L": 0,       # D-pad left
            "dir_R": 0,       # D-pad right
            "dir_U": 0,       # D-pad up
            "dir_D": 0,       # D-pad down
            
            # Joystick Click Buttons - Digital inputs (0 or 1)
            "but_L3": 0,      # Left joystick press (L3 button)
            "but_R3": 0,      # Right joystick press (R3 button)
        }

    # ========================================================================
    # LEFT JOYSTICK (L3) - Analog Stick Controls
    # ========================================================================
    # The left joystick provides continuous 2D control input
    # Values are normalized to [-1.0, 1.0] range for both axes
    # Raw values from controller are in range [-32767, 32767]
    
    def on_L3_left(self, value):
        """Left joystick moved left (negative X direction)"""
        self.signals["js_L_x"] = value / JOYSTICK_SCALE
        
    def on_L3_right(self, value):
        """Left joystick moved right (positive X direction)"""
        self.signals["js_L_x"] = value / JOYSTICK_SCALE
        
    def on_L3_up(self, value):
        """Left joystick moved up (negative Y direction)"""
        self.signals["js_L_y"] = value / JOYSTICK_SCALE
        
    def on_L3_down(self, value):
        """Left joystick moved down (positive Y direction)"""
        self.signals["js_L_y"] = value / JOYSTICK_SCALE
        
    def on_L3_x_at_rest(self):
        """Left joystick X-axis returned to center (released)"""
        self.signals["js_L_x"] = 0.0
        
    def on_L3_y_at_rest(self):
        """Left joystick Y-axis returned to center (released)"""
        self.signals["js_L_y"] = 0.0

    # ========================================================================
    # RIGHT JOYSTICK (R3) - Analog Stick Controls
    # ========================================================================
    # The right joystick provides continuous 2D control input
    # Values are normalized to [-1.0, 1.0] range for both axes
    # Raw values from controller are in range [-32767, 32767]
    
    def on_R3_left(self, value):
        """Right joystick moved left (negative X direction)"""
        self.signals["js_R_x"] = value / JOYSTICK_SCALE
        
    def on_R3_right(self, value):
        """Right joystick moved right (positive X direction)"""
        self.signals["js_R_x"] = value / JOYSTICK_SCALE
        
    def on_R3_up(self, value):
        """Right joystick moved up (negative Y direction)"""
        self.signals["js_R_y"] = value / JOYSTICK_SCALE
        
    def on_R3_down(self, value):
        """Right joystick moved down (positive Y direction)"""
        self.signals["js_R_y"] = value / JOYSTICK_SCALE
        
    def on_R3_x_at_rest(self):
        """Right joystick X-axis returned to center (released)"""
        self.signals["js_R_x"] = 0.0
        
    def on_R3_y_at_rest(self):
        """Right joystick Y-axis returned to center (released)"""
        self.signals["js_R_y"] = 0.0
    
    # ========================================================================
    # JOYSTICK CLICK BUTTONS (L3 and R3)
    # ========================================================================
    # The joysticks can be pressed down like buttons
    # Returns digital values: 1 (pressed) or 0 (released)
    
    def on_L3_press(self):
        """Left joystick pressed down (L3 button)"""
        self.signals["but_L3"] = 1
        
    def on_L3_release(self):
        """Left joystick released (L3 button)"""
        self.signals["but_L3"] = 0
        
    def on_R3_press(self):
        """Right joystick pressed down (R3 button)"""
        self.signals["but_R3"] = 1
        
    def on_R3_release(self):
        """Right joystick released (R3 button)"""
        self.signals["but_R3"] = 0
        
    # ========================================================================
    # TRIGGERS (L2 and R2) - Analog Pressure-Sensitive Buttons
    # ========================================================================
    # Triggers provide analog input from 0.0 (not pressed) to 1.0 (fully pressed)
    # Raw values from controller are in range [-32767, 32767]
    # We normalize to [0.0, 1.0] by adding offset and dividing by 2*JOYSTICK_SCALE
    
    def on_L2_press(self, value):
        """Left trigger (L2) pressed with analog pressure value"""
        self.signals["trigger_L2"] = (value + JOYSTICK_SCALE) / (2 * JOYSTICK_SCALE)
        
    def on_L2_release(self):
        """Left trigger (L2) released"""
        self.signals["trigger_L2"] = 0.0
        
    def on_R2_press(self, value):
        """Right trigger (R2) pressed with analog pressure value"""
        self.signals["trigger_R2"] = (value + JOYSTICK_SCALE) / (2 * JOYSTICK_SCALE)
        
    def on_R2_release(self):
        """Right trigger (R2) released"""
        self.signals["trigger_R2"] = 0.0

    # ========================================================================
    # SHOULDER BUTTONS (L1 and R1) - Digital Buttons
    # ========================================================================
    # Shoulder buttons are digital: 1 (pressed) or 0 (released)
    # Located above the triggers on the controller
    
    def on_L1_press(self):
        """Left shoulder button (L1) pressed"""
        self.signals["shoulder_L1"] = 1
        
    def on_L1_release(self):
        """Left shoulder button (L1) released"""
        self.signals["shoulder_L1"] = 0
        
    def on_R1_press(self):
        """Right shoulder button (R1) pressed"""
        self.signals["shoulder_R1"] = 1
        
    def on_R1_release(self):
        """Right shoulder button (R1) released"""
        self.signals["shoulder_R1"] = 0

    # ========================================================================
    # FACE BUTTONS (X, Circle, Triangle, Square) - Digital Buttons
    # ========================================================================
    # Four primary action buttons on the right side of the controller
    # All are digital: 1 (pressed) or 0 (released)
    # Layout: Triangle (top), Square (left), Circle (right), X (bottom)
    
    def on_x_press(self):
        """X button pressed (bottom face button)"""
        self.signals["but_x"] = 1
        
    def on_x_release(self):
        """X button released"""
        self.signals["but_x"] = 0
        
    def on_triangle_press(self):
        """Triangle button pressed (top face button)"""
        self.signals["but_tri"] = 1
        
    def on_triangle_release(self):
        """Triangle button released"""
        self.signals["but_tri"] = 0
        
    def on_circle_press(self):
        """Circle button pressed (right face button)"""
        self.signals["but_cir"] = 1
        
    def on_circle_release(self):
        """Circle button released"""
        self.signals["but_cir"] = 0
        
    def on_square_press(self):
        """Square button pressed (left face button)"""
        self.signals["but_sq"] = 1
        
    def on_square_release(self):
        """Square button released"""
        self.signals["but_sq"] = 0

    # ========================================================================
    # D-PAD (Directional Pad) - Digital Directional Buttons
    # ========================================================================
    # Four directional buttons on the left side of the controller
    # All are digital: 1 (pressed) or 0 (released)
    # 
    # USAGE NOTE: For PID gain tuning or incremental adjustments, you may want
    # to modify these handlers to increment/decrement values on each press
    # rather than just setting to 1. In that case, you might not need the
    # release handlers (since releasing would reset your incremented values).
    # 
    # Current implementation: Simple binary on/off state
    
    def on_left_arrow_press(self):
        """D-pad left arrow pressed"""
        self.signals["dir_L"] = 1
        
    def on_right_arrow_press(self):
        """D-pad right arrow pressed"""
        self.signals["dir_R"] = 1
        
    def on_left_right_arrow_release(self):
        """D-pad left/right arrows released"""
        self.signals["dir_L"] = 0
        self.signals["dir_R"] = 0
        
    def on_up_arrow_press(self):
        """D-pad up arrow pressed"""
        self.signals["dir_U"] = 1
        
    def on_down_arrow_press(self):
        """D-pad down arrow pressed"""
        self.signals["dir_D"] = 1
        
    def on_up_down_arrow_release(self):
        """D-pad up/down arrows released"""
        self.signals["dir_U"] = 0
        self.signals["dir_D"] = 0

    # === Options Button (Emergency Stop) =====================================================
    # OPTIONS BUTTON - Emergency Stop
    # ========================================================================
    # The Options button (located near the center of the controller) serves
    # as an emergency stop that immediately terminates the controller thread.
    # This is a safety feature to quickly stop all control operations.
    # Note: This will call sys.exit() and terminate the program!
    
    def on_options_press(self):
        """
        Options button pressed - Emergency Stop
        
        Immediately exits the PS4 controller thread and terminates the program.
        Use this as a safety mechanism to stop all robot operations quickly.
        """
        print("⚠️  EMERGENCY STOP: Options button pressed - Exiting PS4 controller thread.")
        sys.exit()

    # ========================================================================
    # PUBLIC API - Signal Access Method
    # ========================================================================
    
    def get_signals(self):
        """
        Get the current state of all controller input signals.
        
        This method returns a dictionary containing real-time values for all
        controller inputs. The dictionary is continuously updated by the
        controller's event handlers running in the background thread.
        
        Returns:
            dict: Dictionary with keys for all controller inputs:
                - Joysticks: 'js_L_x', 'js_L_y', 'js_R_x', 'js_R_y' (float, [-1.0, 1.0])
                - Triggers: 'trigger_L2', 'trigger_R2' (float, [0.0, 1.0])
                - Shoulder buttons: 'shoulder_L1', 'shoulder_R1' (int, 0 or 1)
                - Face buttons: 'but_x', 'but_cir', 'but_tri', 'but_sq' (int, 0 or 1)
                - D-pad: 'dir_L', 'dir_R', 'dir_U', 'dir_D' (int, 0 or 1)
                - Joystick clicks: 'but_L3', 'but_R3' (int, 0 or 1)
        
        Example:
            signals = handler.get_signals()
            forward_speed = signals['js_L_y']  # Use left joystick Y for forward motion
            turn_rate = signals['js_R_x']      # Use right joystick X for turning
        """
        return self.signals