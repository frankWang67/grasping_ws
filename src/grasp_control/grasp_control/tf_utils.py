import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.logging import get_logger

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

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

    def get_transform_at_time(self, target_frame: str, source_frame: str, time: rclpy.time.Time, timeout_sec: float = 0.1) -> TransformStamped | None:
        """
        查询从源坐标系到目标坐标系在特定时间点的坐标变换。

        Args:
            target_frame (str): 目标坐标系的名称。
            source_frame (str): 源坐标系的名称。
            time (rclpy.time.Time): 需要查询变换的特定ROS时间点。
            timeout_sec (float): 等待变换可用的最长秒数。

        Returns:
            geometry_msgs.msg.TransformStamped or None: 如果成功，返回变换信息；否则返回None。
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout=Duration(seconds=timeout_sec)
            )
            self._logger.debug(
                f"Successfully queried transform from '{source_frame}' to '{target_frame}' at time {time}."
            )
            return trans
        except TransformException as ex:
            self._logger.warn(
                f"Failed to query transform from '{source_frame}' to '{target_frame}' at time {time}: {ex}"
            )
            return None