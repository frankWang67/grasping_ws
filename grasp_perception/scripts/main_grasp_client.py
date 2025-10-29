#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from threading import Lock
import time
import numpy as np
from scipy.spatial.transform import Rotation as SciR
import ros2_numpy
import yaml
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Transform, TransformStamped
import tf2_ros
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

import os
from grasp_perception.pointcloud_utils import get_generated_grasp_from_server
from grasp_perception.ur10_kinematics import UR10Kinematics
from grasp_perception.rtde_client import RTDEClient
import grasp_perception.leap_hand_utils as lhu


# Override some functions in leap_hand_utils to adapt to three fingers

def allegro_to_LEAPhand(joints, zeros = True):
    joints = np.array(joints)
    ret_joints = joints + 3.14159
    if zeros:
        ret_joints[0] = ret_joints[4] = 3.14
    return ret_joints

def LEAPhand_to_sim_ones(joints):
    sim_min = np.array([-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -0.349, -0.47, -1.20, -1.34])
    sim_max = np.array([1.047,    2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  2.094,  2.443, 1.90,  1.88])
    joints = lhu.unscale(joints, sim_min, sim_max)
    return joints

def pos_quat_to_transform(pos, quat):
    # quat: xyzw
    matrix = np.eye(4)
    matrix[:3, 3] = pos
    matrix[:3, :3] = SciR.from_quat(quat).as_matrix()  # Convert to SciPy rotation
    return matrix

class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client_node')

        # tf broadcaster and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        self.leap_js_lock = Lock()
        self.vis_data_lock = Lock()
        self.leap_js_cbg = MutuallyExclusiveCallbackGroup()
        self.pc_tf_cbg = MutuallyExclusiveCallbackGroup()
        self.pcd_cbg = MutuallyExclusiveCallbackGroup()
        self.vis_cbg = MutuallyExclusiveCallbackGroup()
        self.ik_cbg = MutuallyExclusiveCallbackGroup()
        self.ctrl_cbg = MutuallyExclusiveCallbackGroup()

        load_from_file = False

        # hand_hw_config = yaml.safe_load(open(os.path.join(get_package_share_directory('grasp_perception'), 'config', 'lz_gripper.yaml'), 'r'))['kinematics']
        hand_hw_config = yaml.safe_load(open(os.path.join(get_package_share_directory('grasp_perception'), 'config', 'leap.yaml'), 'r'))['kinematics']

        self.joint_names = hand_hw_config['joint_names']
        self.hand_palm_frame = hand_hw_config['root_link']

        self.enable_update_grasp = True
        self.graspgen_joint_names = self.joint_names
        self.hw_joint_names = [f'joint_{i}' for i in range(16)]
        self.hand_curr_joints_ones = np.zeros(16,)
        self.hand_home_joints_ones = LEAPhand_to_sim_ones(np.zeros(16,))

        gengrasp_to_hw_joint_remap = []
        for i in self.hw_joint_names:
            gengrasp_to_hw_joint_remap.append(self.graspgen_joint_names.index(i))
        self.gengrasp_to_hw_joint_remap = np.array(gengrasp_to_hw_joint_remap)

        # UR10 kinematics
        self.ur_kin = UR10Kinematics()
        self.ur_client = RTDEClient()
        ur_initial_joints = self.ur_client.get_joints()
        ur_initial_joints[-1] = 0
        self.ur_initial_joints = ur_initial_joints.copy()
        self.ur_target_joints = ur_initial_joints.copy()

        # Hand and Arm target joints
        self.hand_arm_target_joints = None

        # Transforms
        self.T_cam_to_world = pos_quat_to_transform(
            pos=[1.205, -0.0339003, 0.926984],
            quat=[-0.693306, 0.005935, 0.720590, 0.006432]
        )
        self.T_depth_to_cam = pos_quat_to_transform(
            pos=[0.000000, 0.000000, 0.000000],
            quat=[0.5, -0.5,  0.5, -0.5]
        )
        self.T_pc_to_depth = np.eye(4,)
        self.T_palm_to_pc = np.eye(4,)
        self.T_wrist_to_palm = pos_quat_to_transform(
            pos=[-0.0571, -0.03384, 0],
            quat=[7.07105483e-01, -7.07108080e-01, -9.38187392e-07,  9.38183946e-07]
        )

        # === 文件路径 ===
        if load_from_file:
            debug_path = os.path.join(get_package_share_directory('grasp_perception'), 'debug', 'example_grasp', 'sem_MilkCarton_f5b5a24adc6826ace41b639931f9ca1')
            pc_path = os.path.join(debug_path, 'partial_pc.npy')
            pose_path = os.path.join(debug_path, 'scene_cfg.npy')
            grasp_path_lst = os.listdir(os.path.join(debug_path, 'generated_grasp'))
            grasp_qpos = np.load(
                os.path.join(debug_path, 'generated_grasp', np.random.choice(grasp_path_lst)), allow_pickle=True).item()['grasp_qpos']

            # === 加载数据 ===
            point_cloud = np.load(pc_path)  # (N, 3)
            object_pose = np.load(pose_path, allow_pickle=True).item()['scene']['sem_MilkCarton_f5b5a24adc6826ace41b639931f9ca1']['pose']  # (4, 4)
            # point_cloud[:, :2] -= object_pose[:2]   # re-localize point cloud to origin

            with self.vis_data_lock:
                # self.grasp_qpos = grasp_qpos.astype(np.float64)
                self.grasp_qpos = None
                self.point_cloud = point_cloud.astype(np.float64)
        else:
            with self.vis_data_lock:
                self.grasp_qpos = None
                self.point_cloud = None

        # === 发布器 ===
        # self.pc_publisher = self.create_publisher(PointCloud2, '/segmented_object_pointcloud', 10)
        self.pub_hand_cmd_ones = self.create_publisher(JointState, '/cmd_ones', 10)
        self.pub_hand_cmd_leap = self.create_publisher(JointState, '/cmd_leap', 10)
        self.joint_publisher = self.create_publisher(JointState, '/hand_joint_states', 10)
        self.arm_joint_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.hand_joint_subscriber = self.create_subscription(
            JointState,
            '/leap_position',
            self.leap_js_callback,
            10,
            callback_group=self.leap_js_cbg
        )
        self.pc_tf_subscriber = self.create_subscription(
            TransformStamped,
            '/tf_pc_to_camera',
            self.pc_tf_callback,
            10,
            callback_group=self.pc_tf_cbg
        )
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/segmented_object_pointcloud',
            self.pc_callback,
            10,
            callback_group=self.pcd_cbg
        )

        self.ctrl_srv = self.create_service(Trigger, '/execute_grasp', self.ctrl_callback, callback_group=self.ctrl_cbg)

        # Initialize hand pose
        # self.command_hand_to_joints(q=self.hand_home_joints_ones)
        self.slowly_move_hand_to_joints(q=self.hand_home_joints_ones, duration=1.0)

        timer_period = 1.0  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.vis_cbg)
        # self.ik_timer = self.create_timer(timer_period, self.solve_ur_ik_loop, callback_group=self.ik_cbg)

    def command_hand_to_joints(self, q):
        # move immediately to target
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.hw_joint_names
        msg.position = q.tolist()
        self.pub_hand_cmd_ones.publish(msg)

    def slowly_move_hand_to_joints(self, q, duration=1.0):
        # slowly move to target
        with self.leap_js_lock:
            curr_joint_ones = self.hand_curr_joints_ones.copy()

        t_start = time.time()
        while time.time() - t_start < duration:
            current_time = time.time() - t_start
            fraction = current_time / duration
            current_q = curr_joint_ones + fraction * (q - curr_joint_ones)
            self.command_hand_to_joints(current_q)
            time.sleep(0.05)

    def leap_js_callback(self, msg):
        # TODO: joint order?
        curr_joints = np.array(msg.position)
        curr_joints -= 3.14159
        with self.leap_js_lock:
            self.hand_curr_joints_ones[:] = LEAPhand_to_sim_ones(curr_joints)

    def pc_tf_callback(self, msg):
        transform = msg.transform
        self.T_pc_to_depth = pos_quat_to_transform(
            pos=[transform.translation.x, transform.translation.y, transform.translation.z],
            quat=[transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        )

    def pc_callback(self, msg):
        obj_pcd = ros2_numpy.numpify(msg)['xyz']
        obj_pose = np.concatenate([np.mean(obj_pcd, axis=0), [1, 0, 0, 0]])

        print(f"Object pose: {obj_pose}")

        try:
            gen_grasp = get_generated_grasp_from_server(obj_pcd, obj_pose)
        except:
            self.get_logger().error("Failed to get generated grasp from server")
            return
        grasp_qpos = gen_grasp['grasp_qpos'][0]

        grasp_id = np.random.choice(grasp_qpos.shape[0])
        grasp_qpos = grasp_qpos.astype(np.float64)[grasp_id]
        
        with self.vis_data_lock:
            self.grasp_qpos = grasp_qpos
            self.point_cloud = obj_pcd.astype(np.float64)
            print(f"Received grasp qpos: {self.grasp_qpos}")

    def timer_callback(self):
        if not self.enable_update_grasp:
            return
        
        with self.vis_data_lock:
            if self.grasp_qpos is None or self.point_cloud is None:
                return
            latest_grasp_qpos = self.grasp_qpos
            latest_point_cloud = self.point_cloud

        header = self.create_header('world')

        # object point cloud
        cloud_msg = pc2.create_cloud_xyz32(
            header=header,
            points=latest_point_cloud.tolist()
        )

        # finger joint states
        finger_joint_msg = JointState()
        finger_joint_msg.header = header
        finger_joint_msg.name = self.joint_names
        finger_joint_msg.position = latest_grasp_qpos[7:].tolist()

        # wrist tf
        wrist_tf = TransformStamped()
        wrist_tf.header = header
        # wrist_tf.header.frame_id = 'world'
        wrist_tf.header.frame_id = 'pc_frame'
        wrist_tf.child_frame_id = self.hand_palm_frame
        wrist_tf.transform.translation.x = latest_grasp_qpos[0]
        wrist_tf.transform.translation.y = latest_grasp_qpos[1]
        wrist_tf.transform.translation.z = latest_grasp_qpos[2]
        wrist_tf.transform.rotation.x = latest_grasp_qpos[4]
        wrist_tf.transform.rotation.y = latest_grasp_qpos[5]
        wrist_tf.transform.rotation.z = latest_grasp_qpos[6]
        wrist_tf.transform.rotation.w = latest_grasp_qpos[3]
        
        self.T_palm_to_pc = pos_quat_to_transform(
            pos=[latest_grasp_qpos[0], latest_grasp_qpos[1], latest_grasp_qpos[2]],
            quat=[latest_grasp_qpos[4], latest_grasp_qpos[5], latest_grasp_qpos[6], latest_grasp_qpos[3]]
        )

        # self.pc_publisher.publish(cloud_msg)
        gengrasp_joint_pos = latest_grasp_qpos[7:]
        hw_joint_pos = np.array(gengrasp_joint_pos)[self.gengrasp_to_hw_joint_remap]
        hw_joint_pos_ones = LEAPhand_to_sim_ones(hw_joint_pos)
        # self.command_hand_to_joints(q=lhu.LEAPhand_to_sim_ones(hw_joint_pos))
        # self.slowly_move_hand_to_joints(q=hw_joint_pos_ones, duration=1.0)

        # solve ik here
        # ------------------------------------------------
        # trans = None
        
        # try:
        #     max_retrys = 10
        #     num_retrys = 0
        #     tf_when = rclpy.time.Time()
        #     tf_duration = rclpy.duration.Duration(seconds=0.5)
        #     # while not self.tf_buffer.can_transform('world', 'wrist_mount', tf_when, timeout=tf_duration):
        #     #     num_retrys += 1
        #     #     if num_retrys > max_retrys:
        #     #         self.get_logger().warn('Transform not available, skipping IK solution', throttle_duration_sec=1.0)
        #     #         return
        #     #     time.sleep(0.01)
        #     trans: Transform = self.tf_buffer.lookup_transform(
        #         'world',         # target frame
        #         'wrist_mount',       # source frame
        #         tf_when,
        #         timeout=tf_duration
        #     ).transform
        #     self.get_logger().warn('Transform get', throttle_duration_sec=1.0)
        # except tf2_ros.LookupException:
        #     self.get_logger().warn('Transform not available yet', throttle_duration_sec=1.0)
        # except tf2_ros.ExtrapolationException:
        #     self.get_logger().warn('Transform extrapolation error', throttle_duration_sec=1.0)
        # except tf2_ros.ConnectivityException:
        #     self.get_logger().warn('Transform connectivity error', throttle_duration_sec=1.0)

        # if trans is None:
        #     return

        # goal_position = [trans.translation.x, trans.translation.y, trans.translation.z]
        # goal_quaternion = [trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z]

        T_wrist_to_world = self.T_cam_to_world @ self.T_depth_to_cam @ self.T_pc_to_depth @ self.T_palm_to_pc @ self.T_wrist_to_palm
        goal_position = T_wrist_to_world[:3, 3].tolist()
        goal_quaternion = SciR.from_matrix(T_wrist_to_world[:3, :3]).as_quat()[[3, 0, 1, 2]].tolist()

        result = self.ur_kin.compute_ik(goal_position, goal_quaternion, initial_guess=self.ur_initial_joints)
        
        if result is None:
            return

        ur_js_msg = JointState()
        ur_js_msg.header.stamp = self.get_clock().now().to_msg()
        ur_js_msg.name = self.ur_kin.joint_names
        ur_js_msg.position = result[0].tolist()  # Take the first solution
        self.get_logger().info(f'Computed joint angles: {ur_js_msg.position}', throttle_duration_sec=1.0)

        self.ur_target_joints = result[0].tolist()

        if self.hand_arm_target_joints is None:
            self.hand_arm_target_joints = {
                "hand": self.hand_home_joints_ones.copy(),
                "arm": self.ur_initial_joints.copy()
            }
        else:
            self.hand_arm_target_joints["hand"] = hw_joint_pos_ones.copy()
            self.hand_arm_target_joints["arm"] = self.ur_target_joints.copy()
        
        # publish visualization
        self.joint_publisher.publish(finger_joint_msg)
        self.br.sendTransform(wrist_tf)
        self.arm_joint_publisher.publish(ur_js_msg)        
        # ------------------------------------------------

    def solve_ur_ik_loop(self):
        trans = None
        
        try:
            while not self.tf_buffer.can_transform('world', self.hand_palm_frame, rclpy.time.Time()):
                time.sleep(0.01)
            trans: Transform = self.tf_buffer.lookup_transform(
                'world',         # target frame
                self.hand_palm_frame,       # source frame
                rclpy.time.Time(),    # 最新的TF（也可以指定时间）
            ).transform
            self.get_logger().warn('Transform get', throttle_duration_sec=1.0)
        except tf2_ros.LookupException:
            self.get_logger().warn('Transform not available yet', throttle_duration_sec=1.0)
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn('Transform extrapolation error', throttle_duration_sec=1.0)
        except tf2_ros.ConnectivityException:
            self.get_logger().warn('Transform connectivity error', throttle_duration_sec=1.0)

        if trans is None:
            return

        goal_position = [trans.translation.x, trans.translation.y, trans.translation.z]
        goal_quaternion = [trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z]

        result = self.ur_kin.compute_ik(goal_position, goal_quaternion, initial_guess=self.ur_initial_joints)
        
        if result is None or result[0] is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.ur_kin.joint_names
        msg.position = result[0].tolist()  # Take the first solution
        self.arm_joint_publisher.publish(msg)
        self.get_logger().info(f'Computed joint angles: {msg.position}', throttle_duration_sec=1.0)

        self.ur_target_joints = result[0].tolist()

    def ctrl_callback(self, request, response):
        self.enable_update_grasp = False

        hand_target_joint_ones = self.hand_arm_target_joints["hand"]
        arm_target_joint = self.hand_arm_target_joints["arm"]

        self.slowly_move_hand_to_joints(q=self.hand_home_joints_ones, duration=1.0)

        self.get_logger().info(f'Moving hand to target joints: {hand_target_joint_ones}, arm to target joints: {arm_target_joint}', throttle_duration_sec=1.0)
        self.ur_client.move_joints(arm_target_joint)

        # FIXME: THIS IS HARDCODE! squeeze to grasp the object
        squeeze_joint_ones = hand_target_joint_ones.copy()
        # squeeze_joint_ones[[1, 5, 11]] += 0.3
        self.slowly_move_hand_to_joints(q=squeeze_joint_ones, duration=3.0)

        time.sleep(2)
        self.ur_client.move_joints(self.ur_initial_joints)

        self.enable_update_grasp = True

        response.success = True
        response.message = 'UR10e and hand moved to target joints.'
        self.get_logger().info(f'Moved UR10e and hand to joints: {hand_target_joint_ones}, {arm_target_joint}', throttle_duration_sec=1.0)

        return response

    def create_header(self, frame_id):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h


def main(args=None):
    rclpy.init(args=args)
    node = GraspClient()
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
