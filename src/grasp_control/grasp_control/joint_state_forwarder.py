import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from math import pi

class JointStateForwarder(Node):
    def __init__(self):
        super().__init__('joint_state_forwarder')
        self.ur_joint_state = None
        self.hand_joint_state = None

        self.joint_state_pub = self.create_publisher(JointState, '/robot/joint_states', 10)

        self.ur_joint_state_sub = self.create_subscription(JointState, '/joint_states', self.ur_joint_state_callback, 10)
        self.hand_joint_state_sub = self.create_subscription(JointState, '/irmhand/joint_states', self.hand_joint_state_callback, 10)

        self.timer = self.create_timer(0.01, self.publish_joint_state)

    def ur_joint_state_callback(self, msg):
        self.ur_joint_state = msg

    def hand_joint_state_callback(self, msg):
        for i in range(len(msg.position)):
            msg.position[i] -= pi
        self.hand_joint_state = msg

    def publish_joint_state(self):
        if self.ur_joint_state is not None and self.hand_joint_state is not None:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.ur_joint_state.name + self.hand_joint_state.name
            joint_state.position = self.ur_joint_state.position + self.hand_joint_state.position
            joint_state.velocity = self.ur_joint_state.velocity + self.hand_joint_state.velocity
            joint_state.effort = self.ur_joint_state.effort + self.hand_joint_state.effort
            self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    joint_state_forwarder = JointStateForwarder()
    rclpy.spin(joint_state_forwarder)
    joint_state_forwarder.destroy_node()
    rclpy.shutdown()