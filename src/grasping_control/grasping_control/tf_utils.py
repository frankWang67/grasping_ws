import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.logging import get_logger

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.spatial.transform import Rotation as R

class TfQuerier:
    def __init__(self, node: Node):
        self._node = node
        self._logger = get_logger('tf_querier')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self._node)
        
        self._logger.info("TF2 querier initialized.")

    def get_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 0.1) -> TransformStamped | None:
        """
        查询从源坐标系(source_frame)到目标坐标系(target_frame)的最新坐标变换。

        这个函数会阻塞一小段时间（由timeout_sec定义），以等待变换在TF树中变为可用。

        Args:
            target_frame (str): 目标坐标系的名称 (例如: "odom")。
            source_frame (str): 源坐标系的名称 (例如: "base_link")。
            timeout_sec (float): 等待变换可用的最长秒数。默认为0.1秒。

        Returns:
            geometry_msgs.msg.TransformStamped or None: 如果成功找到变换，则返回一个包含平移和旋转信息的
                                                        TransformStamped消息。如果发生超时或其他异常，
                                                        则返回None。
        """
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                when,
                timeout=Duration(seconds=timeout_sec)
            )
            self._logger.debug(
                f"Successfully queried transform from '{source_frame}' to '{target_frame}'."
            )
            return trans
        except TransformException as ex:
            self._logger.warn(
                f"Failed to query transform from '{source_frame}' to '{target_frame}': {ex}"
            )
            return None

    def get_homogeneous_matrix(self, pose):
        """
        Convert translation and rotation to a homogeneous matrix.

        :param pose: np.ndarray: [x, y, z, qw, qx, qy, qz]
        :return: 4x4 homogeneous matrix.
        """
        matrix = np.eye(4)
        matrix[:3, :3] = R.from_quat([pose[4], pose[5], pose[6], pose[3]]).as_matrix()
        matrix[:3, 3] = pose[:3]
        return matrix
        
    def get_transform_matrix(self, target_frame, source_frame):
        """
        Get the transformation matrix from source_frame to target_frame.

        :param target_frame: The target frame name.
        :param source_frame: The source frame name.
        :return: 4x4 transformation matrix.
        """
        transform = self.get_transform(target_frame, source_frame)
        matrix = np.eye(4)
        matrix[:3, :3] = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]).as_matrix()
        matrix[:3, 3] = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
        return matrix
    
    def get_pose_from_matrix(self, matrix):
        """
        Convert a homogeneous matrix to a pose.

        :param matrix: 4x4 homogeneous matrix.
        :return: np.ndarray: [x, y, z, qw, qx, qy, qz]
        """
        pose = np.zeros(7)
        pose[:3] = matrix[:3, 3]
        quat = R.from_matrix(matrix[:3, :3]).as_quat()
        pose[3] = quat[3]
        pose[4:] = quat[:3]
        return pose
        
    def transform_pose(self, pose, target_frame, source_frame):
        """
        Transform a position from source_frame to target_frame.

        :param pose: np.ndarray: [x, y, z, qw, qx, qy, qz]
        :param target_frame: The target frame name.
        :param source_frame: The source frame name.
        :return: np.ndarray: [x, y, z, qw, qx, qy, qz]
        """
        pose_mat = self.get_homogeneous_matrix(pose)
        transf_mat = self.get_transform_matrix(target_frame, source_frame)
        res_mat = transf_mat @ pose_mat

        res_pose = np.zeros(7)
        res_pose[:3] = res_mat[:3, 3]
        res_quat = R.from_matrix(res_mat[:3, :3]).as_quat()
        res_pose[3] = res_quat[3]
        res_pose[4:] = res_quat[:3]
        return res_pose