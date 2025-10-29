import rclpy.node
from sensor_msgs.msg import JointState
from math import pi

class JointStatePublisherNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('joint_state_publisher_test')

        self.hand_joint_names = [
            "rh_joint_1", "rh_joint_0", "rh_joint_2", "rh_joint_3",
            "rh_joint_5", "rh_joint_4", "rh_joint_6", "rh_joint_7",
            "rh_joint_9", "rh_joint_8", "rh_joint_10", "rh_joint_11",
            "rh_joint_12", "rh_joint_13", "rh_joint_14", "rh_joint_15",
        ]

        self.joint_angles = [
            -0.009878633543848991,
            0.892358660697937,
            0.6800774335861206,
            0.0966896116733551,
            -0.009488124400377274,
            0.8485274910926819,
            0.7616491317749023,
            0.16408711671829224,
            -0.05877069756388664,
            0.9360740184783936,
            0.8136745691299438,
            0.15329396724700928,
            1.7735064029693604,
            0.08963482081890106,
            0.31112679839134216,
            -0.357099711894989,
        ]
        for i in range(len(self.joint_angles)):
            self.joint_angles[i] = self.joint_angles[i] + pi

        self.publisher = self.create_publisher(JointState, '/cmd_leap', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = self.create_joint_state_msg()
        self.publisher.publish(msg)

    def create_joint_state_msg(self):
        msg = JointState()
        msg.name = self.hand_joint_names
        msg.position = self.joint_angles
        return msg
    
def main():
    rclpy.init()
    node = JointStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()