#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dynamic open-loop control of the JetTec Rover, for testing"""

import Jetson.GPIO as GPIO  # Motors with ESC, connected to Jetson Orin Nano dev kit
import smbus2  # I2C Bus connected to PCA9685
import time
from sshkeyboard import listen_keyboard  

# --- INIT MOTORS (PWM on GPIO) ---
motor_pin = 15 #CHECK PIN ON JETSON
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor_pin, GPIO.OUT, initial=GPIO.HIGH)

pwm = GPIO.PWM(motor_pin, 350)  # PWM at frequency = 350Hz
pwm.start(50)  # To initialize the ESC, Duty Cycle = 50%
motor_duty = 50.0  # Global for motor speed

# --- INIT SERVO (Control via PCA9685) ---

# I2C config
I2C_BUS = 7 # I2C Bus on Jetson Orin Nano
I2C_ADDR_SERVO = 0x40  # PCA9685 address for servo
LED0_ON_L = 0x06 # Channel base register
MODE1 = 0x00
PRESCALE = 0xFE
f_servo = 50.0 #Hz

# Initialize I2C bus
bus = smbus2.SMBus(I2C_BUS)

def pca9685_init(f):
    """Initializes PCA9685 and sets the PWM frequency to 50 Hz for the servo."""
    bus.write_byte_data(I2C_ADDR_SERVO, MODE1, 0x00) # Normal mode
    time.sleep(0.005)
    prescale_val = int(25000000 / (4096 * f) - 1) # Prescaler calculation for 50 Hz
    bus.write_byte_data(I2C_ADDR_SERVO, MODE1, 0x10) # Enter sleep mode
    bus.write_byte_data(I2C_ADDR_SERVO, PRESCALE, prescale_val)
    bus.write_byte_data(I2C_ADDR_SERVO, MODE1, 0x80) # Restart with Auto-Increment
    time.sleep(0.005)
    print(f"PCA9685 (servo) initialized at {f} Hz")

def set_servo_angle(channel, duty_cycle):
    """
    Sets the PWM signal on the specified channel for the servo.
    duty_cycle must be between 3.5 and 7.5 (%).
    """
    if duty_cycle < 3.5 or duty_cycle > 7.5:
        print("Error: Duty cycle must be between 3.5 and 7.5")
        return

    on_time = 0  # Pulse start
    # Convert duty cycle to "ticks" (out of 4096)
    off_time = int(duty_cycle * 4096 / 100)
    reg_base = LED0_ON_L + 4 * channel
    bus.write_byte_data(I2C_ADDR_SERVO, reg_base, on_time & 0xFF)
    bus.write_byte_data(I2C_ADDR_SERVO, reg_base + 1, on_time >> 8)
    bus.write_byte_data(I2C_ADDR_SERVO, reg_base + 2, off_time & 0xFF)
    bus.write_byte_data(I2C_ADDR_SERVO, reg_base + 3, off_time >> 8)
    print(f"Servo channel {channel} set to {duty_cycle:.2f}% Duty Cycle")

# Predefined servo positions
SERVO_RIGHT = 3.5 # Steer right
SERVO_CENTER = 5.5 # Straight (center)
SERVO_LEFT = 7.5 # Steer left

# Initial servo state (center)
servo_state = "center"
servo_duty = SERVO_CENTER

# Initialize PCA9685 for servo control
pca9685_init(f_servo)
servo_channel = 4
set_servo_angle(servo_channel, servo_duty)

# --- Keyboard controls for servo and motor ---
def on_press(key):
    global servo_state, servo_duty, motor_duty
    # Servo control (discrete positions)
    if key == "left":
        if servo_state == "center":
            # Move from center to left
            servo_state = "left"
            servo_duty = SERVO_LEFT
            print("🔼 Servo: Steering left")
            set_servo_angle(servo_channel, servo_duty)
        elif servo_state == "right":
            # Return from right steer to center
            servo_state = "center"
            servo_duty = SERVO_CENTER
            print("🔽 Servo: Returning to center")
            set_servo_angle(servo_channel, servo_duty)
        else:
            print("⚠️ Already at left steer, press right arrow to return to center.")
    elif key == "right":
        if servo_state == "center":
            # Move from center to right
            servo_state = "right"
            servo_duty = SERVO_RIGHT
            print("🔼 Servo: Steering right")
            set_servo_angle(servo_channel, servo_duty)
        elif servo_state == "left":
            # Return from left steer to center
            servo_state = "center"
            servo_duty = SERVO_CENTER
            print("🔽 Servo: Returning to center")
            set_servo_angle(servo_channel, servo_duty)
        else:
            print("⚠️ Already at right steer, press left arrow to return to center.")
    # Motor control
    elif key == "up":
        motor_duty = 60.0
        pwm.ChangeDutyCycle(motor_duty)
        print(f"🔼 Motor: Speed set to {motor_duty}%")
    elif key == "down":
        motor_duty = 45.0
        pwm.ChangeDutyCycle(motor_duty)
        print(f"🔽 Motor: Speed set to {motor_duty}%")
    elif key == "space":
        motor_duty = 0.0
        pwm.ChangeDutyCycle(motor_duty)
        print("⏹ Motor: Stopped (0%)")
    elif key == "q":
        print("❌ Exiting program")
        exit_program()

def exit_program():
    """Properly stops the program by disabling PWM and closing resources."""
    # Put PCA9685 in sleep mode
    bus.write_byte_data(I2C_ADDR_SERVO, MODE1, 0x10)
    print("😴 PCA9685 (servo) put to sleep.")
    bus.close()
    pwm.stop()
    GPIO.cleanup()
    print("🔌 Cleanup complete. Program exited.")
    exit(0)

print("\n🔄 Control the servo (channel 4) and motor using the keyboard:")
print("➡️ Right arrow  : Steer right or return to center from left")
print("⬅️ Left arrow   : Steer left or return to center from right")
print("⬆️ Up arrow     : Motor speed to 60%")
print("⬇️ Down arrow   : Motor speed to 45%")
print("␣ (Space)      : Stop motor (0%)")
print("❌ 'CTRL+ C' or 'q' key      : Quit\n")

try:
    listen_keyboard(on_press=on_press)
except KeyboardInterrupt:
    print("❌ Program stopped (CTRL+C)")
    exit_program()
