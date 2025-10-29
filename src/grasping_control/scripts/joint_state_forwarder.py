#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ur_rtde_msgs.msg import VectorStamped
from ft_leap.srv import LeapPosition

from math import pi
import numpy as np

class JointStateForwarder(Node):
    def __init__(self):
        super().__init__('joint_state_forwarder')
        self.ur_joint_state = None
        self.hand_joint_state = None

        self.ur_joint_names = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint", "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint"]

        self.joint_state_pub = self.create_publisher(JointState, '/robot_joint_states', 10)

        self.ur_joint_state_sub = self.create_subscription(VectorStamped, '/arm/state/qpos', self.ur_joint_state_callback, 10)
        self.hand_joint_state_sub = self.create_subscription(JointState, '/irmhand/joint_states', self.hand_joint_state_callback, 10)

        self.hand_joint_pos_client = self.create_client(LeapPosition, "/leap_position")
        while not self.hand_joint_pos_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().info("Service not available, waiting...")

        # self.hand_joint_state = JointState()
        # self.hand_joint_state.name = [f"rh_joint_{m}" for m in range(16)]
        # self.hand_timer = self.create_timer(0.04, self._update_hand_joint_pos)

        self.joint_state = JointState()
        self.timer = self.create_timer(0.04, self.publish_joint_state)

    def ur_joint_state_callback(self, msg):
        self.ur_joint_state = msg

    def hand_joint_state_callback(self, msg):
        for i in range(len(msg.position)):
            msg.position[i] -= pi
        self.hand_joint_state = msg

    # def _update_hand_joint_pos(self):
    #     """
    #     The updated joint pos is the received msg - 180 degree.
    #     """
    #     req = LeapPosition.Request()
    #     future = self.hand_joint_pos_client.call_async(req)
    #     # rclpy.spin_until_future_complete(self.node, future)
    #     while future.result() is None:
    #         continue
    #     hand_joint_pos = np.asarray(future.result().position) - pi
    #     self.hand_joint_state.position = hand_joint_pos.tolist()
    #     self.hand_joint_state.velocity = [0.0] * 16
    #     self.hand_joint_state.effort = [0.0] * 16

    def publish_joint_state(self):
        if self.ur_joint_state is not None and self.hand_joint_state is not None:
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state.name = list(self.ur_joint_names) + list(self.hand_joint_state.name)
            self.joint_state.position = list(self.ur_joint_state.data) + list(self.hand_joint_state.position)
            self.joint_state.velocity = [0.0] * 6 + list(self.hand_joint_state.velocity)
            self.joint_state.effort = [0.0] * 6 + list(self.hand_joint_state.effort)
            self.joint_state_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    joint_state_forwarder = JointStateForwarder()
    rclpy.spin(joint_state_forwarder)
    joint_state_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()