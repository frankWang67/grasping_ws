import cv2
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer

from groundingdino.util.inference import annotate

from grasping_perception.detection import DetectionNode, TEXT_PROMPTS
from grasping_perception.utils import draw_masks

COLOR_IMAGE_TOPIC = "/rgb/image_raw"
DEPTH_IMAGE_TOPIC = "/depth_to_rgb/image_raw"
CAMERA_INFO_TOPIC = "/rgb/camera_info"

CLASS_NAME_LIST = [
    # "Yellow_Mustard_Bottle",
    "Workpiece_A",
    "Workpiece_B",
    "Workpiece_C"
]
CLASS_NAME_DICT = {
    TEXT_PROMPTS[0]: CLASS_NAME_LIST[0],
    TEXT_PROMPTS[1]: CLASS_NAME_LIST[1],
    TEXT_PROMPTS[2]: CLASS_NAME_LIST[2]
}

class ObjectPointcloudPublisher(DetectionNode):
    def __init__(self, node_name="object_pointcloud_publisher", publish_img=False):
        Node.__init__(self, node_name)

        self.initialize_models()

        self.publish_img = publish_img
        if self.publish_img:
            self.img_pub = self.create_publisher(RosImage, "/annotated_img", 10)

        # 使用嵌套字典存储发布器: {class_name: {instance_id: publisher}}
        self.cloud_publishers = {
            CLASS_NAME_LIST[0]: {}, 
            CLASS_NAME_LIST[1]: {}, 
            CLASS_NAME_LIST[2]: {}
        }
        # # 用于跟踪已发布实例的字典
        # self.active_instances = {
        #     CLASS_NAME_LIST[0]: set(),
        #     CLASS_NAME_LIST[1]: set(),
        #     CLASS_NAME_LIST[2]: set()
        # }

        # Get camera matrix
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10
        )

        self.color_sub = Subscriber(self, RosImage, COLOR_IMAGE_TOPIC)
        self.depth_sub = Subscriber(self, RosImage, DEPTH_IMAGE_TOPIC)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )

        # Register the callback function for synchronized messages
        self.sync.registerCallback(self.listener_callback)

        self.get_logger().info('Synchronized subscriber node started.')

    def camera_info_callback(self, msg):
        self.get_logger().info("Waiting for camera parameters...")
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera parameters ready.")
        # 获取一次后取消订阅
        self.destroy_subscription(self.camera_info_sub)

    def listener_callback(self, color_msg: RosImage, depth_msg: RosImage):
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            cv_color = cv2.rotate(cv_color, cv2.ROTATE_90_CLOCKWISE)
            cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_90_CLOCKWISE)
            h, w = cv_color.shape[0], cv_color.shape[1]
            cv_color = cv_color[h//2:h*3//4, w//4:w*3//4]
            cv_depth = cv_depth[h//2:h*3//4, w//4:w*3//4]
            self.h, self.w = h, w
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        rgb_color = cv2.cvtColor(cv_color, cv2.COLOR_BGR2RGB)
        boxes, logits, phrases = self.detect(rgb_color)
        all_masks = self.segment(rgb_color, boxes)

        self.update_object_clouds(boxes, phrases, all_masks, color_msg.header, cv_depth, cv_color)
        
        if self.publish_img:
            annotated_img = annotate(rgb_color, boxes, logits, phrases)
            annotated_img = draw_masks(annotated_img, all_masks)
            img_msg = self.bridge.cv2_to_imgmsg(annotated_img, header=color_msg.header)
            self.img_pub.publish(img_msg)

    def pixel_to_3d(self, u, v, depth):
        """将像素坐标和深度值转换为3D坐标"""
        # 归一化像素坐标
        x = (u - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
        y = (v - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
        
        # 计算3D坐标
        point_x = depth * x
        point_y = depth * y
        point_z = depth
        
        return [point_x, point_y, point_z]

    def depth_to_pointcloud(self, cv_depth, mask, cv_color):
        """将深度图像转换为点云"""
        points = []
        
        # 获取mask中的非零像素坐标
        y_indices, x_indices = np.where(mask > 0)
        
        for y, x in zip(y_indices, x_indices):
            # 获取深度值（单位：米）
            depth = cv_depth[y, x] / 1000.0  # 假设深度单位为毫米
            
            if depth <= 0 or depth > 10.0:  # 过滤无效深度
                continue
            
            # 将像素坐标转换为3D坐标
            point = self.pixel_to_3d(y+self.h//2, self.w-(x+self.w//4), depth)
            
            # 获取颜色信息（如果提供了RGB图像）
            color = None
            if cv_color is not None:
                b, g, r = cv_color[y, x]
                # 将RGB颜色转换为浮点数表示
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                color = rgb
            
            points.append((point[0], point[1], point[2], color))
        
        return points
    
    def create_pointcloud_msg(self, points, header):
        """发布点云消息"""
        # 创建点云字段
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1)
        ]
        
        # 创建点云消息
        cloud_msg = pc2.create_cloud(header, fields, points)
        
        return cloud_msg
    
    def update_object_clouds(self, boxes, phrases, all_masks, header, cv_depth, cv_color):
        # current_instances = {
        #     CLASS_NAME_LIST[0]: set(),
        #     CLASS_NAME_LIST[1]: set(),
        #     CLASS_NAME_LIST[2]: set()
        # }
        object_id_getter = {
            CLASS_NAME_LIST[0]: 0,
            CLASS_NAME_LIST[1]: 0,
            CLASS_NAME_LIST[2]: 0
        }
        
        for i in range(boxes.shape[0]):
            class_name = CLASS_NAME_DICT[phrases[i]]
            instance_id = object_id_getter[class_name]
            object_id_getter[class_name] += 1
            points_i = self.depth_to_pointcloud(cv_depth, all_masks[i], cv_color)
            pointcloud = self.create_pointcloud_msg(points_i, header)
            
            # # 记录当前实例
            # current_instances[class_name].add(instance_id)
            
            # 如果这是新实例，创建新的发布器
            if instance_id not in self.cloud_publishers[class_name]:
                topic_name = f'/{class_name}_{instance_id}/pointcloud'
                self.cloud_publishers[class_name][instance_id] = self.create_publisher(
                    PointCloud2, topic_name, 10
                )            
            # 发布点云
            self.cloud_publishers[class_name][instance_id].publish(pointcloud)
        
        # # 移除不再存在的实例的发布器
        # for class_name in CLASS_NAME_LIST:
        #     disappeared_instances = self.active_instances[class_name] - current_instances[class_name]
        #     for instance_id in disappeared_instances:
        #         if instance_id in self.publishers[class_name]:
        #             # 注意: ROS 2中没有直接删除发布器的方法，但可以停止使用它
        #             # 可以记录日志表示该实例不再存在
        #             self.get_logger().info(f'实例 {class_name}_{instance_id} 不再存在')
            
        #     # 更新活跃实例
        #     self.active_instances[class_name] = current_instances[class_name]

def main(args=None):
    rclpy.init(args=args)
    
    publisher_node = ObjectPointcloudPublisher(publish_img=True)
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()