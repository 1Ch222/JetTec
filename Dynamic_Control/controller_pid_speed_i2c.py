# controller_pid_speed_i2c.py

import smbus2
import time
from zed_interface import ZEDWrapper
from pid_controller import PIDController

# --- PCA9685 CONFIG ---
I2C_BUS = 7
I2C_ADDR = 0x40
LED0_ON_L = 0x06
MODE1 = 0x00
PRESCALE = 0xFE
FREQ = 100  # Hz

bus = smbus2.SMBus(I2C_BUS)

def pca9685_init(f):
    bus.write_byte_data(I2C_ADDR, MODE1, 0x00)
    time.sleep(0.005)
    prescale_val = int(round(25000000.0 / (4096 * f)) - 1)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x10)
    bus.write_byte_data(I2C_ADDR, PRESCALE, prescale_val)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x80)
    time.sleep(0.005)
    print(f"✅ PCA9685 initialisé à {f} Hz")

def set_pwm(channel, duty_cycle):
    on_time = 0
    off_time = int(duty_cycle * 4096 / 100)
    reg_base = LED0_ON_L + 4 * channel
    bus.write_byte_data(I2C_ADDR, reg_base + 0, on_time & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg_base + 1, on_time >> 8)
    bus.write_byte_data(I2C_ADDR, reg_base + 2, off_time & 0xFF)
    bus.write_byte_data(I2C_ADDR, reg_base + 3, off_time >> 8)

# --- MOTEUR ---
MOTOR_CHANNEL = 0
MOTOR_NEUTRAL = 15.0  # point neutre
MOTOR_LIMITS = (13.0, 20.0)  # backward, forward

def set_motor_duty(duty):
    set_pwm(MOTOR_CHANNEL, duty)

# --- INIT PCA + ZED ---
pca9685_init(FREQ)
zed = ZEDWrapper()

# --- INIT PID ---
pid = PIDController(kp=40.0, ki=2.0, kd=5.0, output_limits=MOTOR_LIMITS)

# --- BOUCLE PID ---
setpoint_speed = 1.0  # m/s (modifiable à la volée plus tard)

try:
    print("🧠 Contrôle PID en cours...")
    while True:
        vitesse = zed.get_velocity()
        now = time.time()

        if vitesse is not None:
            duty = pid.compute(setpoint_speed, vitesse, now)
            set_motor_duty(duty)
            print(f"🎯 Cible: {setpoint_speed:.2f} m/s | 🚗 Mesurée: {vitesse:.2f} m/s | PWM: {duty:.2f}%")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("❌ Arrêt utilisateur")
    set_motor_duty(MOTOR_NEUTRAL)
    bus.write_byte_data(I2C_ADDR, MODE1, 0x10)
    bus.close()
    zed.close()
