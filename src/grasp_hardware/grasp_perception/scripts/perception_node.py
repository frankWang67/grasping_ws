#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import os
import threading

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Transform, TransformStamped
from std_srvs.srv import Trigger
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import json
import ros2_numpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs_py import point_cloud2
import requests

from grasp_perception.sam_utils import show_masks
from grasp_perception.pointcloud_utils import segment_and_visualize_pcd
from grasp_perception.tf_utils import matrix_to_transform


def pointcloud2_to_xyz_rgb(pc_msg: PointCloud2):
    # fields=['x', 'y', 'z', 'rgb'] 指定只提取这4个字段
    pc_np = point_cloud2.read_points_numpy(pc_msg, field_names=('x', 'y', 'z', 'rgb'))

    # 提取 xyz
    xyz = pc_np[:, :3].view(np.float32).reshape(-1, 3)

    # 提取 rgb（float32 编码 -> uint32 -> R/G/B）
    rgb_f = pc_np[:, 3].view(np.float32)
    rgb_u32 = rgb_f.view(np.uint32)
    rgb = np.zeros((len(rgb_u32), 3), dtype=np.uint8)
    rgb[:, 0] = (rgb_u32 >> 16) & 0xFF  # R
    rgb[:, 1] = (rgb_u32 >> 8) & 0xFF   # G
    rgb[:, 2] = rgb_u32 & 0xFF          # B

    return xyz, rgb

class GraspPerception(Node):
    def __init__(self):
        super().__init__('grasp_perception_node')

        self.bridge = CvBridge()
        self.latest_pc = None
        self.latest_image = None
        self.clicked_points = []
        self.click_mode = False
        
        self.img_lock = threading.Lock()
        self.pc_lock = threading.Lock()
        self.click_lock = threading.Lock()

        self.sensor_cbg = MutuallyExclusiveCallbackGroup()
        self.service_cbg = MutuallyExclusiveCallbackGroup()
        self.timer_cbg = MutuallyExclusiveCallbackGroup()
        self.timer_cbg_unaffected = MutuallyExclusiveCallbackGroup()

        self.verbose = self.declare_parameter('verbose', False).get_parameter_value().bool_value

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.static_tf_buffer = []

        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10,
            callback_group=self.sensor_cbg
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pc_callback,
            10,
            callback_group=self.sensor_cbg
        )
        self.object_pointcloud_pub = self.create_publisher(PointCloud2, '/segmented_object_pointcloud', 10)
        self.pc_to_cam_tf_pub = self.create_publisher(TransformStamped, '/tf_pc_to_camera', 10)

        self.srv = self.create_service(
            Trigger,
            '/capture_pointcloud',
            self.handle_capture,
            callback_group=self.service_cbg
        )
        self.get_logger().info('Ready to capture point cloud via service call /capture_pointcloud')

        # 开启图像显示线程
        # self.image_timer = self.create_timer(0.01, self.image_display_loop, callback_group=self.timer_cbg)
        self.segmentation_timer = self.create_timer(1.0, self.object_segmentation_loop, callback_group=self.timer_cbg)
        self.unaffected_timer = self.create_timer(0.01, self.unaffected_timer_callback, callback_group=self.timer_cbg_unaffected)
        self.get_logger().info("Node started. Call /click_uv to begin clicking.")

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.img_lock:
                self.latest_image = image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def pc_callback(self, msg):
        if self.verbose:
            self.get_logger().info(f"Received PointCloud2 message with \
                                {msg.width}x{msg.height} points", throttle_duration_sec=1.0)
        with self.pc_lock:
            self.latest_pc = msg

    def handle_capture(self, request, response):
        print("Service /capture_pointcloud called, entering click mode...")
        # 重置点击列表
        self.clicked_points.clear()
        self.click_mode = True

        # 开一个线程等待点击完成
        threading.Thread(target=self._click_session, daemon=True).start()

        response.success = True
        return response
    
    def _click_session(self):
        while self.click_mode and rclpy.ok():
            # 等待 'c' 被按下，图像窗口中 click_mode 会被设置为 False
            time.sleep(0.1)

        with self.click_lock:
            points = self.clicked_points.copy()

        with self.pc_lock:
            pc_data = ros2_numpy.numpify(self.latest_pc)['xyz']
            pc_color = ros2_numpy.numpify(self.latest_pc)['rgb']
            # pc_data, pc_color = pointcloud2_to_xyz_rgb(self.latest_pc)

        with self.img_lock:
            img = self.latest_image.copy() if self.latest_image is not None else None

        img_bytes = cv2.imencode('.jpg', img)[1].tobytes()

        self.get_logger().info(f"Click session done. {len(points)} points saved.")
        print("Captured points:", points)

        # save
        save_root = './src/grasp_perception/debug'
        cv2.imwrite(os.path.join(save_root, f'image.jpg'), img)
        np.savez(os.path.join(save_root, 'clicked_points.npz'), points=np.array(points))
        np.savez(os.path.join(save_root, 'pointcloud.npz'), pc_data=pc_data)
        np.savez(os.path.join(save_root, 'pointcloud_color.npz'), pc_color=pc_color)

        return

        # 发送请求
        response = requests.post(
            'http://10.21.70.145:8000/sam2',
            files={'image': ('image.jpg', img_bytes, 'image/jpeg')},
            data={'clicks': json.dumps(points)}
        )

        masks = cv2.imdecode(np.frombuffer(response.content, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        masks = np.transpose(masks.astype(np.float32), (2, 0, 1))  # Convert to (C, H, W) format
        masks = masks / 255

        show_masks(img, masks, scores=np.ones(masks.shape[0]), input_labels=np.ones(len(points)), point_coords=np.array(points), borders=True)

    def unaffected_timer_callback(self):
        # public static tf
        if len(self.static_tf_buffer) > 0:
            self.tf_static_broadcaster.sendTransform(self.static_tf_buffer)

    def image_display_loop(self):
        cv2.namedWindow("depth_image")
        cv2.setMouseCallback("depth_image", self.mouse_callback)

        while rclpy.ok():
            with self.img_lock:
                img = self.latest_image.copy() if self.latest_image is not None else None

            if img is not None:
                normed = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
                display = np.uint8(normed)

                # 显示已点击的点
                if self.click_mode:
                    for pt in self.clicked_points:
                        cv2.circle(display, pt, 4, (255), -1)

                cv2.imshow("depth_image", display)

            # 关键：即使不在点击模式，也要调用 waitKey() 维持窗口响应
            key = cv2.waitKey(10) & 0xFF
            if key == ord('c') and self.click_mode:
                self.get_logger().info("Clicking finished.")
                self.click_mode = False
            elif key == ord('q'):
                self.get_logger().info("Quit window.")
                rclpy.shutdown()
                break

        cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.click_mode:
            with self.click_lock:
                self.clicked_points.append((x, y))
                self.get_logger().info(f"Clicked: ({x}, {y})")

    def object_segmentation_loop(self):
        """
            Segment object point cloud based on ROI
        """
        with self.pc_lock:
            if self.latest_pc is None:
                return
            pc = self.latest_pc

        pc_data = ros2_numpy.numpify(pc)['xyz']
        result = segment_and_visualize_pcd(pc_data)

        # public static tf (from camera_link to pc_frame)
        tf_cam_to_pc = TransformStamped(
            header=self.create_header('camera_depth_optical_frame'),
            child_frame_id='pc_frame',
            transform=matrix_to_transform(result['pc_frame'])
        )
        self.static_tf_buffer = [tf_cam_to_pc]
        self.pc_to_cam_tf_pub.publish(tf_cam_to_pc)

        # publish PointCloud2
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=self.create_header('pc_frame'),
            points=result['transformed_obj_pcd'].points
        )
        self.object_pointcloud_pub.publish(cloud_msg)

    def create_header(self, frame_id):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h


def main(args=None):
    rclpy.init(args=args)
    node = GraspPerception()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
