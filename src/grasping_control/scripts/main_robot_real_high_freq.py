#!/usr/bin/env python3
import threading
import time
from typing import List, Optional, Union

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration, Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from ur_rtde_msgs.msg import VectorStamped


def time_to_seconds(t) -> float:
    return t.nanoseconds / 1e9


def stamp_to_seconds(input) -> float:
    return input.sec + input.nanosec / 1e9


class RobotRealHighFreq(Node):
    def __init__(self):
        super().__init__("robot_real_high_freq")

        # -------- hyper-parameters begin --------
        self.rviz_viz = self.declare_parameter("rviz_viz", False).get_parameter_value().bool_value

        self.n_arm_joints = 6
        self.n_hand_joints = 16
        self.window_size = 4
        self.hf_commands_window_size = 5
        self.timer_period = 0.01  # seconds
        # -------- hyper-parameters end --------

        if self.rviz_viz:
            self.joint_names = [
                "arm_shoulder_pan_joint",
                "arm_shoulder_lift_joint",
                "arm_elbow_joint",
                "arm_wrist_1_joint",
                "arm_wrist_2_joint",
                "arm_wrist_3_joint",
            ] + [f"rh_joint_{i}" for i in range(16)]  # TODO
            self.robot_joint_state_vis_pub = self.create_publisher(JointState, "visualize/robot_joint_states", 1)

        self.sent_hf_commands_window = []
        self.sent_hf_commands_stamp_window = []
        self.joint_pos_command_high_freq: Optional[np.ndarray] = None
        self.sent_num_command_high_freq: int = 0

        self.lock = threading.Lock()
        # subscriber for low-freq ctrl command
        self.joint_pos_command_sub = self.create_subscription(
            JointState,
            "robot/ctrl/qpos_low_freq",
            self._robot_command_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        # publisher for arm and hand
        self.arm_joint_pos_command_pub = self.create_publisher(VectorStamped, "arm/ctrl/qpos_servo", 10)
        self.hand_joint_pos_command_pub = self.create_publisher(JointState, "/cmd_leap", 10)

        self.timer = self.create_timer(self.timer_period, self._timer_callback, callback_group=ReentrantCallbackGroup())

        time.sleep(0.2)
        self.get_logger().info("Ready ...")

    def _publish_arm_joint_pos_command(self, joint_pos):
        assert len(joint_pos) == self.n_arm_joints
        msg = VectorStamped()
        msg.data = np.asarray(joint_pos).tolist()
        self.arm_joint_pos_command_pub.publish(msg)

    def _publish_hand_joint_pos_command(self, joint_pos):
        assert len(joint_pos) == self.n_hand_joints
        msg = JointState()
        msg.position = joint_pos.tolist()
        self.hand_joint_pos_command_pub.publish(msg)

    def _robot_command_callback(self, msg):
        self.get_logger().debug(f"Received low-freqency joint command: {list(msg.position)}")

        current_time = time_to_seconds(self.get_clock().now())
        target_command = np.asarray(msg.position)
        target_reach_time = stamp_to_seconds(msg.header.stamp)

        if len(self.sent_hf_commands_window) >= 2:
            self.lock.acquire()
            xs = self.sent_hf_commands_stamp_window + [target_reach_time]
            ys = self.sent_hf_commands_window + [target_command]
            self.lock.release()
            spline_func = CubicSpline(xs, ys, bc_type="natural")
            x_new = np.arange(current_time, target_reach_time, self.timer_period)
            y_new = spline_func(x_new)
        else:
            y_new = np.asarray([target_command])

        self.lock.acquire()
        self.joint_pos_command_high_freq = y_new
        self.sent_num_command_high_freq = 0
        self.lock.release()

    def _timer_callback(self):
        self.lock.acquire()

        if (self.joint_pos_command_high_freq is not None) and self.sent_num_command_high_freq < len(
            self.joint_pos_command_high_freq
        ):
            command = self.joint_pos_command_high_freq[self.sent_num_command_high_freq]
            self._publish_arm_joint_pos_command(command[: self.n_arm_joints])
            self._publish_hand_joint_pos_command(command[self.n_arm_joints :])
            self.sent_num_command_high_freq += 1

            # maintain the window
            self.sent_hf_commands_window.append(command)
            self.sent_hf_commands_stamp_window.append(time_to_seconds(self.get_clock().now()))
            if len(self.sent_hf_commands_window) > self.hf_commands_window_size:
                self.sent_hf_commands_window.pop(0)
                self.sent_hf_commands_stamp_window.pop(0)

            if self.rviz_viz:
                msg = JointState()
                msg.name = self.joint_names
                msg.position = list(command)
                self.robot_joint_state_vis_pub.publish(msg)

            self.get_logger().debug(f"Sending high-frequency command: {command}.")

        self.lock.release()


def main(args=None):
    rclpy.init(args=args)
    robot = RobotRealHighFreq()
    robot.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    rclpy.spin(robot)


if __name__ == "__main__":
    main()
