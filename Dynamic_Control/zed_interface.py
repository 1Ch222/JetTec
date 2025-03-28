#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyzed.sl as sl
import math as m
import time

class ZEDWrapper:
    def __init__(self):
        self.zed = sl.Camera()

        # --- Paramètres d'initialisation ---
        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE

        # --- Ouverture de la caméra ---
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"❌ Erreur à l'ouverture de la ZED : {status}")

        # --- Activation du positional tracking ---
        tracking_params = sl.PositionalTrackingParameters()
        tracking_status = self.zed.enable_positional_tracking(tracking_params)
        if tracking_status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"❌ Erreur lors de l'activation du tracking : {tracking_status}")

        # --- Runtime + pose ---
        self.runtime = sl.RuntimeParameters()
        self.pose = sl.Pose()
        self.prev_position = None
        self.prev_time = None

    def get_velocity(self):
        if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
            translation = self.pose.get_translation()
            x, y, z = translation.get()

            now = time.time()

            if self.prev_position is None:
                self.prev_position = (x, y, z)
                self.prev_time = now
                return 0.0  # Premier appel

            dx = x - self.prev_position[0]
            dy = y - self.prev_position[1]
            dz = z - self.prev_position[2]
            dt = now - self.prev_time if self.prev_time else 1e-6

            speed = ((dx**2 + dy**2 + dz**2) ** 0.5) / dt

            # DEBUG : Afficher position et vitesse
            print(f"[ZED] x={x:.3f}, y={y:.3f}, z={z:.3f}, speed={speed:.3f} m/s")

            self.prev_position = (x, y, z)
            self.prev_time = now

            return speed
        return None

    def get_yaw(self):
        if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
            orientation = self.pose.get_orientation()
            qx, qy, qz, qw = orientation.get()
            return self.quaternion_to_yaw(qx, qy, qz, qw)
        return None

    @staticmethod
    def quaternion_to_yaw(qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return m.degrees(m.atan2(siny_cosp, cosy_cosp))

    def close(self):
        self.zed.disable_positional_tracking()
        self.zed.close()
        print("📦 ZED correctement fermée.")
