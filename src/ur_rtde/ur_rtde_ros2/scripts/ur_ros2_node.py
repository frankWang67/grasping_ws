#!/usr/bin/env python3
import os
import time
from typing import List

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from ur_rtde_msgs.msg import VectorStamped

from ur_rtde import URrtde


class URNode:
    def __init__(self, node: Node, ip=str, name=str, state_pub_rate=30, servo_rate=100):
        self.node = node
        self.ur = URrtde(ip=ip)

        # state joint pos publisher
        self.qpos_pub = self.node.create_publisher(VectorStamped, f"{name}/state/qpos", 1)
        self.timer = self.node.create_timer(
            1.0 / state_pub_rate,
            self.state_pub_timer_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        # control joint vel subscriber
        self.ctrl_qvel_sub = self.node.create_subscription(
            VectorStamped,
            f"{name}/ctrl/qvel",
            self.ctrl_qvel_cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # control joint pos servo subscriber
        self.servo_rate = servo_rate
        self.ctrl_qpos_servo_sub = self.node.create_subscription(
            VectorStamped,
            f"{name}/ctrl/qpos_servo",
            self.ctrl_qpos_servo_cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        time.sleep(0.2)

    def ctrl_qvel_cb(self, msg):
        self.node.get_logger().debug(f"ctrl_qvel_cb(): received message: {msg.data}")
        qvel = np.asarray(msg.data)
        self.ur.ctrl_qvel(qvel)

    def ctrl_qpos_servo_cb(self, msg):
        self.node.get_logger().debug(f"ctrl_qpos_servo_cb(): received message: {msg.data}")
        qpos = np.asarray(msg.data)
        success = self.ur.ctrl_qpos_servo(qpos, servo_rate=self.servo_rate)
        if not success:
            self.node.get_logger().warn("Exceed max safe velocity for servoJ!")

    def state_pub_timer_cb(self):
        msg = VectorStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = list(self.ur.get_qpos())
        self.qpos_pub.publish(msg)
        # self.node.get_logger().debug("Publishing qpos.")


def main():
    from threading import Thread
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=None)
    node = Node("node_name")
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ur_node = URNode(
        node,
        ip="192.168.54.130",
        name="arm",
        state_pub_rate=30,
        servo_rate=100,
    )

    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
