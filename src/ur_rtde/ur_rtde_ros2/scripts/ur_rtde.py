import os
import time
from typing import List
import rtde_control
import rtde_receive
import numpy as np


class URrtde:
    """
    Solely Python APIs.
    ROS-independent.
    """

    def __init__(
        self,
        ip,
        prefix="",
        home_config=None,
    ):
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)

    # ----------------------------------------------------------------------
    def get_qpos(self):
        return self.rtde_r.getActualQ()

    # ----------------------------------------------------------------------
    def get_qvel(self):
        return self.rtde_r.getActualQd()

    # ----------------------------------------------------------------------
    def ctrl_qvel(self, qvel: np.ndarray, acceleration=2.0):
        assert len(qvel) == 6
        self.rtde_c.speedJ(qvel.reshape(-1).tolist(), acceleration=acceleration, time=0.001)  # time=0 会报错

    # ----------------------------------------------------------------------
    def move_to_qpos(self, qpos: np.ndarray, speed=0.1, acceleration=0.5, asynchronous=False):
        assert len(qpos) == 6
        self.rtde_c.moveJ(qpos.reshape(-1).tolist(), speed, acceleration, asynchronous)

    # ----------------------------------------------------------------------
    def servo_stop(self):
        self.rtde_c.servoStop()

    # ----------------------------------------------------------------------
    # def ctrl_qpos_servo(self, qpos: np.ndarray, servo_rate=100, safe_max_vel=5.0):
    def ctrl_qpos_servo(self, qpos: np.ndarray, servo_rate=10, safe_max_vel=100.0):
        assert len(qpos) == 6

        # limit the target qpos for safety
        curr_qpos = np.asarray(self.get_qpos())
        diff = qpos - curr_qpos
        dist = np.linalg.norm(diff)
        max_dist = np.sqrt(safe_max_vel**2 * 6) / servo_rate
        if dist > max_dist:
            diff = diff / dist * max_dist
            print("Warning: exceed max safe velocity for servoJ!")
        qpos_limited = curr_qpos + diff

        self.rtde_c.servoJ(qpos_limited.reshape(-1).tolist(), 0.5, 0.5, 1.0 / servo_rate, 0.1, 300.0)

        if dist > max_dist:
            return False
        else:
            return True
