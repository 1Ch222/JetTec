# JetTec

This repository serves as a testing ground for the JetTec rover, an autonomous robot powered by the NVIDIA Jetson Orin Nano, a ZED X Mini camera, and a PCA9685 board. For the moment I only implemented a simple controller for both speed and steering via, controlled via SSH.

## 🚗 Dynamic Control

The rover’s dynamic behavior is managed over the I2C bus, directly interfacing with the **PCA9685** board **without relying on the Adafruit library**.

- **Steering** is controlled through a servo motor.
- **Throttle** is managed via an Electronic Speed Controller (ESC) connected to a DC motor.

Both actuators receive **100Hz PWM signals**, enabling smooth and responsive movement.
