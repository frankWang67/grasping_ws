import rclpy
from rclpy.node import Node
from ur_rtde_msgs.msg import VectorStamped
import numpy as np
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class Test(Node):
    def __init__(self, node: Node):
        self.node = node

        self.curr_qpos = None
        self.init_qpos = None
        self.qpos_sub = self.node.create_subscription(
            VectorStamped,  # Message type
            "arm/state/qpos",  # Topic name
            self.receive_qpos_cb,  # Callback function
            1,  # QoS (Quality of Service) depth
            callback_group=ReentrantCallbackGroup(),
        )

        test_ctrl_qvel = False
        test_ctrl_qpos_servo = True

        if test_ctrl_qvel:
            self.qvel_pub = self.node.create_publisher(VectorStamped, "arm/ctrl/qvel", 10)
            # self.ctrl_qvel_timer = self.node.create_timer(0.01, self.ctrl_qvel_timer_callback)  # Publish every 100ms

        elif test_ctrl_qpos_servo:
            self.servo_rate = 100.0
            self.timestep = 1.0 / self.servo_rate
            self.one_step_time_record = time.time()
            self.qpos_servo_pub = self.node.create_publisher(VectorStamped, "arm/ctrl/qpos_servo", 10)
            # self.ctrl_qpos_servo_timer = self.node.create_timer(
            #     1.0 / self.servo_rate, self.ctrl_qpos_servo_timer_callback
            # )

    def receive_qpos_cb(self, msg):
        qpos = np.asarray(msg.data)
        self.curr_qpos = qpos.copy()

    # def ctrl_qvel_timer_callback(self):
    #     # Create a message to publish control velocities (6 joint velocities)
    #     msg = VectorStamped()
    #     msg.data = np.array([0, 0, 0, 0, 0, 0.05]).tolist()
    #     self.qvel_pub.publish(msg)
    #     self.node.get_logger().info(f"Published ctrl qvel: {msg.data}")

    def ctrl_qpos_servo(self, qpos):
        msg = VectorStamped()
        msg.data = qpos.reshape(-1).tolist()
        self.qpos_servo_pub.publish(msg)
        # self.node.get_logger().info(f"Published ctrl qpos servo: {msg.data}")

    def step(self, refresh=False):
        while (time.time() - self.one_step_time_record) < self.timestep:
            time.sleep(0.0001)
        self.one_step_time_record = time.time()


def main():
    from threading import Thread
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=None)
    node = Node("node_name")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    test = Test(node)

    while test.curr_qpos is None:
        time.sleep(0.01)

    target_qpos = test.curr_qpos.copy()

    while True:
        t1 = time.time()
        target_qpos += 1.0 / test.servo_rate * np.array([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        # t2 = time.time()
        # print(f"ctrl_qpos_servo time: {t2 - t1}")
        test.step()
        test.ctrl_qpos_servo(target_qpos)
        t3 = time.time()
        print(f"time: {t3 - t1}")


if __name__ == "__main__":
    main()
