import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from moveit_action_msgs.action import MoveitAction
from leap_hand.srv import LeapPosition
from grasp_control.tf_utils import TfQuerier

box_center = (0.125, 0.305, 0.265)
box_size = (0.51, 0.33, 0.005)  # (length, width, height)
wall_thickness = 0.001

class MyRobotController:
    def __init__(self, node: rclpy.node.Node, world_frame="base_link", planning_eef="wrist_3_link"):
        self.node = node
        self.world_frame = world_frame
        self.planning_eef = planning_eef

        self.logger = self.node.get_logger("my_robot_controller")

        self.tf_querier = TfQuerier(self.node)

        self.moveit_action_client = ActionClient(self.node, MoveitAction, "ur_moveit")
        self.moveit_action_client.wait_for_server()

        self.hand_query_client = self.node.create_client(LeapPosition, '/leap_position')
        self.hand_req = LeapPosition.Request()
        self.hand_cmd_pub = self.node.create_publisher(JointState, '/cmd_ones', 10) 
        self.hand_joint_names = [
            "joint_1", "joint_0", "joint_2", "joint_3",
            "joint_5", "joint_4", "joint_6", "joint_7",
            "joint_9", "joint_8", "joint_10", "joint_11",
            "joint_12", "joint_13", "joint_14", "joint_15",
        ]

    def send_goal(self, position, orientation):
        """发送目标位姿到Action Server"""
        goal_msg = MoveitAction.Goal()
        goal_msg.target_pose.position.x = position[0]
        goal_msg.target_pose.position.y = position[1]
        goal_msg.target_pose.position.z = position[2]
        goal_msg.target_pose.orientation.x = orientation[0]
        goal_msg.target_pose.orientation.y = orientation[1]
        goal_msg.target_pose.orientation.z = orientation[2]
        goal_msg.target_pose.orientation.w = orientation[3]

        self.logger.info('Waiting for action server...')
        self.moveit_action_client.wait_for_server()

        self.logger.info('Sending goal request...')
        self._send_goal_future = self.moveit_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理服务器对目标的响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Goal rejected :(')
            return

        self.logger.info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """处理最终结果"""
        result = future.result().result
        self.logger.info(f'Result: success={result.success}, message="{result.message}"')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """处理来自服务器的反馈信息"""
        feedback = feedback_msg.feedback
        self.logger.info(f'Received feedback: {feedback.status}')

    def get_T_matrix(self, translation, rotation):
        """
        Convert translation and rotation to a transformation matrix.

        :param translation: Translation vector (x, y, z).
        :param rotation: Rotation quaternion (x, y, z, w).
        :return: Transformation matrix.
        """
        matrix = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        return matrix
        
    def get_transform_matrix(self, target_frame, source_frame):
        """
        Get the transformation matrix from source_frame to target_frame.

        :param target_frame: The target frame name.
        :param source_frame: The source frame name.
        :return: Transformation matrix.
        """
        transform = self.tf_querier.get_transform(target_frame, source_frame)
        matrix = self.get_T_matrix(transform.transform.translation, transform.transform.rotation)
        return matrix
        
    def get_matrix_from_pose(self, pose: PoseStamped):
        """
        Convert a PoseStamped object to a transformation matrix.

        :param pose: The PoseStamped object.
        :return: Transformation matrix.
        """
        matrix = self.get_T_matrix(pose.pose.position, pose.pose.orientation)
        return matrix
    
    def get_pose_from_matrix(self, matrix):
        """
        Convert a transformation matrix to a PoseStamped object.

        :param matrix: The transformation matrix.
        :return: PoseStamped object.
        """
        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = self.node.get_clock().now()
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = matrix[2, 3]
        quaternion = R.from_matrix(matrix[:3, :3]).as_quat()
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose
        
    def get_frame_pose(self, frame_name):
        """
        Get the pose of a specific frame.

        :param frame_name: The name of the frame.
        :return: PoseStamped object.
        """
        tf_pose = self.tf_querier.get_transform(self.world_frame, frame_name)

        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = self.node.get_clock().now()
        pose.pose.position.x = tf_pose.transform.translation.x
        pose.pose.position.y = tf_pose.transform.translation.y
        pose.pose.position.z = tf_pose.transform.translation.z
        pose.pose.orientation.x = tf_pose.transform.rotation.x
        pose.pose.orientation.y = tf_pose.transform.rotation.y
        pose.pose.orientation.z = tf_pose.transform.rotation.z
        pose.pose.orientation.w = tf_pose.transform.rotation.w

        return pose
        
    def transform_pose_to_planning_eef(self, pose: PoseStamped, original_frame):
        """
        Transform a PoseStamped object to the end effector frame for moveit planning.

        :param pose: The PoseStamped object to transform.
        :param source_frame: The source frame name.
        :return: Transformed PoseStamped object.
        """
        T_base_original = self.get_matrix_from_pose(pose)
        T_original_eef = self.get_transform_matrix(original_frame, self.planning_eef)

        T_base_eef = T_base_original @ T_original_eef
        transformed_pose = self.get_pose_from_matrix(T_base_eef)
        transformed_pose.header.stamp = self.node.get_clock().now()

        return transformed_pose
        
    def move_arm_to_eef_pose(self, pose: PoseStamped, pose_link):
        """
        Move the robot to the pose of a specified end effector link.

        :param pose: PoseStamped object of the target pose.
        :param pose_link: The specified end effector link, of which the pose is.
        :return: True if the motion was successful, False otherwise.
        """
        eef_pose = self.transform_pose_to_planning_eef(pose, pose_link)
        self.send_goal(eef_pose.pose.position, eef_pose.pose.orientation)

    def get_hand_joint_state(self):
        future = self.hand_query_client.call_async(self.hand_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def set_hand_joint_angle(self, joint_angle: np.ndarray):
        joint_state = JointState()
        joint_state.header.stamp = self.node.get_clock().now()
        joint_state.name = self.hand_joint_names
        joint_state.position = joint_angle.tolist()

        self.hand_cmd_pub.publish(joint_state)
    