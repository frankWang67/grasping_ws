import rclpy
import tf2_ros
import geometry_msgs.msg
from rclpy.executors import ExternalShutdownException

class TableFramePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('table_frame_publisher')

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()

        self.static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transformStamped.header.frame_id = "arm_base_link"
        self.static_transformStamped.child_frame_id = "table"

        transform = geometry_msgs.msg.Transform()
        transform.translation.x = -0.420
        transform.translation.y = -0.764
        transform.translation.z = 0.212
        transform.rotation.x = -0.7071068
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 0.7071068

        self.static_transformStamped.transform = transform
        self.broadcaster.sendTransform(self.static_transformStamped)

def main(args=None):
    rclpy.init(args=args)

    publisher = TableFramePublisher()

    try:
        rclpy.spin(publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        publisher.destroy_node()


if __name__ == '__main__':
    main()
