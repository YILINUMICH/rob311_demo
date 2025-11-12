"""
Ball-Bot E-Stop Demo

This is a simplified demonstration showing how the emergency stop (E-Stop) 
functionality works on the ball-balancing robot. This file isolates just 
the e-stop logic to help students understand the safety feature.

E-Stop Triggers:
- L3 + R3: Pressing both joystick buttons simultaneously
- Touchpad: Pressing the PS4 controller touchpad

Safety Behavior:
- When triggered, all motors immediately stop
- Motor 1 is commanded to 20% power to simulate abnormal operation
- When e-stop is triggered, Motor 1 is also stopped

Author: ROB 311 Instructional Team
Date: Fall 2024
"""

import time
import lcm
import threading
import sys
import os

# Import MBot LCM message types
from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.ps4_controller_api import PS4InputHandler

# ============================================================================
# CONFIGURATION
# ============================================================================

# Control loop frequency
FREQ = 50  # Hz (slower than full system since this is just a demo)
DT = 1 / FREQ

# Motor PWM limits
PWM_MAX = 1.0
PWM_MIN = -1.0

# Test motor command (simulates abnormal motor behavior)
MOTOR1_TEST_PWM = 0.2  # Motor 1 runs at 20% to demonstrate e-stop

# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main():
    """
    Main control loop demonstrating emergency stop functionality.
    
    The loop runs continuously, commanding Motor 1 to run at 20% power
    to simulate an abnormal operating condition. Students can then test
    the e-stop by pressing L3 + R3 together or the touchpad.
    """
    
    print("\n" + "="*80)
    print("BALL-BOT E-STOP DEMONSTRATION")
    print("="*80)
    print("\nThis demo shows how the emergency stop works.")
    print("Motor 1 will run at 20% power to simulate abnormal operation.")
    print("\nE-STOP TRIGGERS:")
    print("  • L3 + R3: Press both joystick buttons simultaneously")
    print("  • Touchpad: Press the PS4 controller touchpad")
    print("\nWhen triggered, all motors will immediately stop.")
    print("="*80 + "\n")
    
    # ========================================================================
    # INITIALIZATION
    # ========================================================================
    
    # Initialize LCM for motor commands
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    print("✓ LCM initialized")
    
    # Initialize PS4 controller
    controller = PS4InputHandler()
    controller_thread = threading.Thread(target=controller.listen, daemon=True)
    controller_thread.start()
    print("✓ PS4 controller initialized")
    
    # Wait for controller to be ready
    time.sleep(1)
    print("✓ System ready\n")
    
    # Initialize motor command message
    command = mbot_motor_pwm_t()
    
    # ========================================================================
    # MAIN CONTROL LOOP
    # ========================================================================
    
    try:
        iteration = 0
        start_time = time.time()
        
        print("Starting motor test... Press L3 + R3 to trigger e-stop.\n")
        
        while True:
            loop_start = time.time()
            iteration += 1
            
            # ================================================================
            # READ CONTROLLER STATE
            # ================================================================
            
            # Get controller button states
            buttons = controller.get_button_state()
            
            # E-Stop triggers
            touchpad = buttons['touchpad']
            l3 = buttons['l3']
            r3 = buttons['r3']
            
            # ================================================================
            # E-STOP CHECK
            # ================================================================
            
            # Check for e-stop condition: touchpad OR (L3 AND R3)
            estop_triggered = (touchpad == 1) or (l3 == 1 and r3 == 1)
            
            if estop_triggered:
                print("\n" + "!"*80)
                print("!!! EMERGENCY STOP ACTIVATED !!!")
                if touchpad == 1:
                    print("!!! Triggered by: TOUCHPAD PRESS !!!")
                else:
                    print("!!! Triggered by: L3 + R3 PRESS !!!")
                print("!"*80)
                print("\nAll motors stopped immediately.")
                print("Motor 1 (which was running at 20%) is now OFF.")
                print("\nExiting demo...\n")
                
                # Stop all motors
                command.utime = int(time.time() * 1e6)
                command.pwm[0] = 0.0
                command.pwm[1] = 0.0
                command.pwm[2] = 0.0
                lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                
                # Exit the loop
                break
            
            # ================================================================
            # NORMAL OPERATION (Motor 1 at 20%)
            # ================================================================
            
            # Command Motor 1 to run at 20% power
            # Motors 2 and 3 remain off
            command.utime = int(time.time() * 1e6)
            command.pwm[0] = MOTOR1_TEST_PWM  # Motor 1 at 20%
            command.pwm[1] = 0.0              # Motor 2 off
            command.pwm[2] = 0.0              # Motor 3 off
            lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
            
            # ================================================================
            # STATUS OUTPUT (every 25 iterations = ~0.5 seconds)
            # ================================================================
            
            if iteration % 25 == 0:
                elapsed = time.time() - start_time
                print(f"[{elapsed:.1f}s] Motor 1: {MOTOR1_TEST_PWM*100:.0f}% | "
                      f"Status: Running | "
                      f"Waiting for e-stop...")
            
            # ================================================================
            # TIMING CONTROL
            # ================================================================
            
            # Sleep to maintain desired loop frequency
            loop_time = time.time() - loop_start
            sleep_time = max(0, DT - loop_time)
            time.sleep(sleep_time)
    
    # ========================================================================
    # EXCEPTION HANDLING
    # ========================================================================
    
    except KeyboardInterrupt:
        print("\n\n" + "="*80)
        print("KEYBOARD INTERRUPT DETECTED")
        print("="*80)
        print("\nNote: In a real emergency, use the e-stop button combination!")
        print("Stopping all motors...\n")
        
        # Emergency stop via keyboard interrupt
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
    
    finally:
        # ====================================================================
        # CLEANUP
        # ====================================================================
        
        print("\n" + "="*80)
        print("SHUTTING DOWN")
        print("="*80)
        
        # Final motor shutdown (redundant safety measure)
        print("\n→ Ensuring all motors are off...")
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        print("✓ All motors confirmed OFF")
        
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
        print("="*80)
        print("\nKEY TAKEAWAYS:")
        print("  1. E-stop can be triggered by L3 + R3 or touchpad")
        print("  2. When triggered, all motors stop immediately")
        print("  3. The system exits the control loop safely")
        print("  4. This is a critical safety feature for testing")
        print("\n" + "="*80 + "\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    """
    E-STOP DEMO USAGE:
    
    1. Connect your PS4 controller to the robot
    2. Run this script: python ballbot_estop_demo.py
    3. Motor 1 will start running at 20% power
    4. Test the e-stop by pressing:
       - L3 + R3 buttons together (both joystick clicks), OR
       - Touchpad button
    5. Observe that all motors stop immediately
    
    LEARNING OBJECTIVES:
    - Understand how e-stop conditions are checked
    - See the immediate motor shutdown behavior
    - Learn the importance of safety features in robotics
    
    TROUBLESHOOTING:
    - Controller not responding: Check /dev/input/js* for device
    - Motors not running: Check LCM connection and robot power
    - E-stop not working: Verify button readings with jstest
    """
    
    main()
