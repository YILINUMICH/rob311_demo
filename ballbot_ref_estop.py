"""
Ball-Bot E-Stop Demo (Beginner Friendly)

This demo shows how to trigger an Emergency Stop (E‑Stop) from a PS4 controller
and, most importantly, how the system actually stops BOTH in software and in
hardware by sending zero motor commands.

E‑Stop Triggers in this demo:
- L3 + R3: Press BOTH joystick buttons at the same time (recommended)
    (We do NOT use the PS4 touchpad here.)

What happens on E‑Stop:
1) Hardware stop (motor command): we immediately publish an LCM message with
     zero PWM values on all motors. This cuts the drive command to the motors
     and safely stops the robot's actuation.
2) Software stop (loop exit): we break out of the control loop so no more
     commands are computed or sent.

Why not use the PS4 "Options" button here?
- In our PS4 API, the Options button is mapped to an emergency process exit
    (sys.exit). That is a valid kill switch, but it may terminate the program
    before we get a chance to publish a final zero‑PWM command. For teaching
    purposes, this demo uses L3+R3 so you can SEE both the hardware message
    and the software shutdown happen in order.

Author: ROB 311 Instructional Team
Date: Fall 2025
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
# CONFIGURATION
# ============================================================================

# Control loop frequency (slow for demo clarity)
FREQ = 50  # Hz
DT = 1 / FREQ

# Motor PWM limits (not strictly needed here, but useful context)
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
    print("\nWhen triggered, we will:")
    print("  1) Publish zero PWM to ALL motors (hardware stop)")
    print("  2) Exit the loop so no further commands are sent (software stop)")
    print("="*80 + "\n")
    
    # ========================================================================
    # INITIALIZATION
    # ========================================================================
    
    # LCM for motor commands: this is the "wire" we use to talk to the motor driver
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    print("✓ LCM initialized (channel MBOT_MOTOR_PWM_CMD)")
    
    # Initialize PS4 controller
    controller = PS4InputHandler(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True  # Daemon so it exits when main program exits
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
        
        print("Starting motor test... Press L3 + R3 to trigger e‑stop.\n")
        
        while True:
            loop_start = time.time()
            iteration += 1
            
            # ================================================================
            # READ CONTROLLER STATE
            # ================================================================
            
            # Get controller button states (digital: 0 or 1)
            signals = controller.get_signals()
            l3 = signals['but_L3']  # Left joystick click
            r3 = signals['but_R3']  # Right joystick click
            
            # ================================================================
            # E-STOP CHECK
            # ================================================================
            
            # E-STOP CHECK (Beginner Explanation)
            # We trigger E‑Stop when BOTH L3 and R3 are pressed at the same time.
            # Why both? Reduces accidental stops while still being quick.
            #
            # E‑Stop =
            #   True  → Send zeros to motors NOW (hardware stop) + break loop (software stop)
            #   False → Continue normal operation
            estop_triggered = (l3 == 1 and r3 == 1)
            
            if estop_triggered:
                print("\n" + "!"*80)
                print("!!! EMERGENCY STOP ACTIVATED !!!")
                print("!!! Triggered by: L3 + R3 PRESS !!!")
                print("!"*80)
                
                # ------------------ HARDWARE STOP ------------------
                # Publish zero PWM to every motor. This is what actually
                # cuts power commands to the actuators via the motor driver.
                command.utime = int(time.time() * 1e6)
                command.pwm[0] = 0.0
                command.pwm[1] = 0.0
                command.pwm[2] = 0.0
                lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
                print("→ Hardware stop: published zero PWM to all motors")

                # ------------------ SOFTWARE STOP ------------------
                # Break out of the loop so we stop computing/sending any more commands.
                print("→ Software stop: exiting control loop")
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
                print(f"[{elapsed:.1f}s] Motor 1: {MOTOR1_TEST_PWM*100:.0f}% | Waiting for e‑stop...")
            
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
        print("\nNote: In a real emergency, use the e‑stop button combination!")
        print("Stopping all motors (safe shutdown)...\n")
        
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
        print("\n→ Ensuring all motors are OFF (publishing zeros one last time)...")
        command = mbot_motor_pwm_t()
        command.utime = int(time.time() * 1e6)
        command.pwm[0] = 0.0
        command.pwm[1] = 0.0
        command.pwm[2] = 0.0
        lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        print("✓ All motors confirmed OFF")

        # We avoid calling controller.on_options_press() here because that calls
        # sys.exit() inside the controller thread. Since our controller thread is
        # a daemon, it will terminate automatically when the main program exits.
        print("\n→ Controller thread will terminate automatically on program exit (daemon thread)")
        
        print("\n" + "="*80)
        print("DEMO COMPLETE")
        print("="*80)
        print("\nKEY TAKEAWAYS:")
        print("  1. E‑Stop is BOTH a hardware stop (zero PWM over LCM) and a software stop (exit loop)")
        print("  2. Use a reliable trigger (L3+R3) that you can press quickly but not by accident")
        print("  3. Always publish zero commands on exit, even during normal shutdown")
        print("\n" + "="*80 + "\n")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    """
     E‑STOP DEMO USAGE:

     1. Connect your PS4 controller to the robot
     2. Run this script: python ballbot_ref_estop.py
     3. Motor 1 will start running at 20% power
     4. Trigger the e‑stop by pressing BOTH joystick clicks: L3 + R3
     5. Observe that:
         - We immediately publish zero PWM to all motors (hardware stop)
         - We exit the control loop (software stop)
    """
    
    main()
