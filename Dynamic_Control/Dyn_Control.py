#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dynamic open-loop control of the JetTec Rover, for testing"""

import smbus2
import time
from sshkeyboard import listen_keyboard

# --- PCA9685 Configuration ---
I2C_BUS = 7            # I²C bus on Orin Nano
I2C_ADDR = 0x40        # PCA9685 adress
LED0_ON_L = 0x06       # base for channels 
MODE1 = 0x00
PRESCALE = 0xFE
FREQ = 100             # PWM (f) : 100 Hz

# Initialization of I²C bus
bus = smbus2.SMBus(I2C_BUS)

def pca9685_init(f):
    """PCA9685 initialization, sets PWM at f (Hz)."""
    bus.write_byte_data(I2C_ADDR, MODE1, 0x00)  # Normal mode
    time.sleep(0.005)
    prescale_val = int(round(25000000.0 / (4096 * f)) - 1)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x10)  # Sleep mode
    bus.write_byte_data(I2C_ADDR, PRESCALE, prescale_val)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x80)  # Restart with auto-increment
    time.sleep(0.005)
    print(f"✅ Initialization of the PCA9685 at {f} Hz (prescale = {prescale_val})")

def set_pwm(channel, duty_cycle):
    """
    Defines the PWM on the selected channel.
    duty cycle (on 100) is converted on 4096 ticks.
    """
    on_time = 0
    off_time = int(duty_cycle * 4096 / 100)
    reg_base = LED0_ON_L + 4 * channel
    bus.write_byte_data(I2C_ADDR, reg_base, on_time & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg_base + 1, on_time >> 8)
    bus.write_byte_data(I2C_ADDR, reg_base + 2, off_time & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg_base + 3, off_time >> 8)
    print(f"🔧 Channel {channel} set at {duty_cycle:.2f}% (ticks = {off_time})")

# --- Servo Config ---
servo_channel = 4
# Servo positions (% duty cycle)
SERVO_RIGHT = 6   # right
SERVO_CENTER = 11 # center
SERVO_LEFT = 15    # left

servo_state = "center"
servo_duty = SERVO_CENTER

def set_servo_angle(duty):
    """
    Settings of the servo
    limits are between 3.5 et 7.5.
    """
    if duty < 6 or duty > 15:
        print("Error : Servo output must be between 3.5 et 7.5")
        return
    set_pwm(servo_channel, duty)

# --- Motor Configuration ---
motor_channel = 0
# motor settings (% duty cycle)
MOTOR_REVERSE = 13.0  # backwards
MOTOR_INIT = 15.0  # neutral (stop)
MOTOR_FORWARD = 20  # forward
MOTOR_NEUTRAL = 0.0

motor_duty = MOTOR_INIT

def set_motor_speed(duty):
    """
    Motor settings via PCA9685.
    """
    set_pwm(motor_channel, duty)

# --- PCA9685 initialization ---
pca9685_init(FREQ)

# Pos initialization 
set_servo_angle(servo_duty)
set_motor_speed(motor_duty)

# --- Keyboard manager ---
def on_press(key):
    global servo_state, servo_duty, motor_duty
    # Servo control
    if key == "left":
        if servo_state == "center":
            servo_state = "left"
            servo_duty = SERVO_LEFT
            print("🔼 Servo : Left")
            set_servo_angle(servo_duty)
        elif servo_state == "right":
            servo_state = "center"
            servo_duty = SERVO_CENTER
            print("🔽 Servo : Return from right")
            set_servo_angle(servo_duty)
        else:
            print("⚠️ Already heading left. Press right key to go back to center position.")
    elif key == "right":
        if servo_state == "center":
            servo_state = "right"
            servo_duty = SERVO_RIGHT
            print("🔼 Servo : right")
            set_servo_angle(servo_duty)
        elif servo_state == "left":
            servo_state = "center"
            servo_duty = SERVO_CENTER
            print("🔽 Servo : Return from left")
            set_servo_angle(servo_duty)
        else:
            print("⚠️ Already heading right. Press left key to go back to center position.")
    # Contrôle du moteur avec flèches et espace
    elif key == "up":
        motor_duty = MOTOR_FORWARD
        print(f"🔼 Moteur : Forward (duty = {motor_duty}%)")
        set_motor_speed(motor_duty)
    elif key == "down":
        motor_duty = MOTOR_REVERSE
        print(f"🔽 Moteur : Backward (duty = {motor_duty}%)")
        set_motor_speed(motor_duty)
    elif key == "space":
        motor_duty = MOTOR_NEUTRAL
        print(f"⏹ Moteur : Neutral (duty = {motor_duty}%)")
        set_motor_speed(motor_duty)
    elif key == "a":
        motor_duty = MOTOR_INIT
        print(f"⏹ Moteur : Emergency stop (duty = {motor_duty}%)")
        set_motor_speed(motor_duty)
    elif key == "q":
        print("❌ Stop the program")
        exit_program()

def exit_program():
    """Properly stops the program, suspending PCA and closing I²C bus."""
    bus.write_byte_data(I2C_ADDR, MODE1, 0x10)
    print("😴 PCA9685 into sleep mode.")
    bus.close()
    print("🔌 Program stopped.")
    exit(0)

print("\n🔄 Commandes clavier :")
print("➡️ Right Key : Right (or going back to center from left)")
print("⬅️ Left Key  : Left (or going back to center from right)")
print("⬆️ Up Key    : Forward (20%)")
print("⬇️ Down     : Backward (13%)")
print("␣ (Space)       : Neutral (15%)")
print("a               : emergency stop")
print("❌ 'q'            : Stop\n")

try:
    listen_keyboard(on_press=on_press)
except KeyboardInterrupt:
    print("❌ Stop (CTRL+C)")
    exit_program()


