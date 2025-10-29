#!/usr/bin/env python3

from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import rtde_receive, rtde_control
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from grasp_perception.ur10_kinematics import UR10Kinematics
from grasp_perception.rtde_client import RTDEClient


def pose_to_euler(pose: Pose):
    q = pose.orientation
    # 四元数转欧拉角（roll, pitch, yaw）
    r = R.from_quat([q.x, q.y, q.z, q.w])
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

class SixDofMarkerNode(Node):
    def __init__(self):
        super().__init__('six_dof_marker')
        self.hand_kin = UR10Kinematics()
        self.ur_client = RTDEClient()
        self.server = InteractiveMarkerServer(self, 'six_dof_marker_server')

        self.ik_timer_cbg = MutuallyExclusiveCallbackGroup()
        self.srv_cbg = MutuallyExclusiveCallbackGroup()

        # Initialize at current position
        # joint_limits = self.hand_kin.joint_limits
        # self.initial_joints = np.random.uniform(joint_limits[0], joint_limits[1], size=(6,)).tolist()
        self.initial_joints = self.ur_client.get_joints()
        self.get_logger().info(f'Initial joints: {self.initial_joints}')
        
        self.target_joints_lock = Lock()
        self.target_joints = self.initial_joints.copy()

        self.pose_pub = self.create_publisher(PoseStamped, 'interactive_pose', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        mogen_target = self.hand_kin.compute_fk(self.target_joints)  # Precompute FK to initialize the kinematics chain

        self.mogen_target_lock = Lock()
        self.mogen_target = mogen_target
        self.make_6dof_marker('pose_marker', mogen_target['position'], mogen_target['orientation'])

        # Service to command UR
        self.cmd_ur_srv = self.create_service(Trigger, 'move_ur10e', self.move_ur_callback, callback_group=self.srv_cbg)

        # Add a timer to continuously compute IK
        # self.create_timer(0.1, self.inverse_kinematics_loop, callback_group=self.ik_timer_cbg)

    def make_6dof_marker(self, name, position, quaternion):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = name
        int_marker.description = '6D Pose Control'
        int_marker.scale = 0.3
        int_marker.pose.position.x = position[0]
        int_marker.pose.position.y = position[1]
        int_marker.pose.position.z = position[2]
        int_marker.pose.orientation.x = quaternion[1]
        int_marker.pose.orientation.y = quaternion[2]
        int_marker.pose.orientation.z = quaternion[3]
        int_marker.pose.orientation.w = quaternion[0]

        # Create a grey box as the marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # 6-DOF Controls
        self.add_6dof_controls(int_marker)

        # Add callback
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def add_6dof_controls(self, int_marker):
        # Rotation and translation around X/Y/Z
        for axis, name in zip(['x', 'y', 'z'], ['X', 'Y', 'Z']):
            control = InteractiveMarkerControl()
            setattr(control.orientation, axis, 1.0)
            control.name = f'rotate_{name}'
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            setattr(control.orientation, axis, 1.0)
            control.name = f'move_{name}'
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

    def process_feedback(self, feedback):
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose
        self.pose_pub.publish(pose_msg)
        # self.get_logger().info(f'Updated pose:\n{pose_msg.pose}')

        goal_position = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z]
        # roll, pitch, yaw = pose_to_euler(feedback.pose)
        # goal_euler_angles = [roll, pitch, yaw]
        goal_quaternion = [feedback.pose.orientation.w, feedback.pose.orientation.x,
                           feedback.pose.orientation.y, feedback.pose.orientation.z]

        with self.mogen_target_lock:
            self.mogen_target['position'] = goal_position
            self.mogen_target['orientation'] = goal_quaternion

    def inverse_kinematics_loop(self):
        """
        This method can be used to continuously compute inverse kinematics
        based on the latest marker pose.
        """
        with self.mogen_target_lock:
            goal_position = self.mogen_target['position']
            goal_orientation = self.mogen_target['orientation']

        self.get_logger().info(f'Mogen target: {self.mogen_target}')
        result = self.hand_kin.compute_ik(goal_position, goal_orientation, initial_guess=self.target_joints)
        
        if result is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hand_kin.joint_names
        msg.position = result[0].tolist()  # Take the first solution
        self.joint_state_pub.publish(msg)
        self.get_logger().info(f'Computed joint angles: {msg.position}')

        # update target joints
        with self.target_joints_lock:
            self.target_joints = result[0].tolist()

    def move_ur_callback(self, request, response):
        with self.target_joints_lock:
            target_joints = self.target_joints

        self.ur_client.move_joints(target_joints)
        response.success = True
        response.message = 'UR10e moved to target joints.'
        self.get_logger().info(f'Moved UR10e to joints: {target_joints}')

        return response


def main():
    rclpy.init()
    node = SixDofMarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
