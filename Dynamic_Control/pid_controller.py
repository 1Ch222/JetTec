#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

        self.output_limits = output_limits  # (min, max)

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def compute(self, setpoint, measured_value, current_time):
        error = setpoint - measured_value

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time

        # Proportionnel
        P = self.kp * error

        # Intégral
        self.integral += error * dt
        I = self.ki * self.integral

        # Dérivé
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        D = self.kd * derivative

        output = P + I + D

        # Clamp output
        min_output, max_output = self.output_limits
        if min_output is not None:
            output = max(min_output, output)
        if max_output is not None:
            output = min(max_output, output)

        # Sauvegarder pour la prochaine itération
        self.previous_error = error
        self.previous_time = current_time

        return output
