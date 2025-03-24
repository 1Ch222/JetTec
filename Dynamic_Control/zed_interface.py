#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyzed.sl as sl
import math as m

"initialization of Zed X Mini camera"

class ZEDWrapper:
    def __init__(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.depth_mode = sl.DEPTH_MODE.NONE

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Error ZED : {status}")

        self.runtime = sl.RuntimeParameters()
        self.pose = sl.Pose()

    def get_velocity(self):
        if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
            velocity = sl.Velocity()
            self.zed.get_velocity(velocity, sl.REFERENCE_FRAME.WORLD)
            vx, vy, vz = velocity.get()
            speed = (vx**2 + vy**2 + vz**2) ** 0.5
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
        self.zed.close()
