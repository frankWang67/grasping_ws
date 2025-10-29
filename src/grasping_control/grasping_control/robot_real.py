#!/usr/bin/env python3
import time
import numpy as np
from builtin_interfaces.msg import Time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ur_rtde_msgs.msg import VectorStamped

# cuRobo
import torch
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

def time_to_seconds(t) -> float:
    return t.nanoseconds / 1e9


def seconds_to_stamp(seconds_float):
    sec = int(seconds_float)  # Integer part for seconds
    nanosec = int((seconds_float - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


class RobotReal:
    def __init__(self, node: Node, virtual_hardware=False, use_high_freq_interp=False, ctrl_freq=10):
        
        if use_high_freq_interp:
            raise NotImplementedError

        # hyper-parameters
        self.n_arm_joints = 6
        self.arm_joint_names = [
            "arm_shoulder_pan_joint",
            "arm_shoulder_lift_joint",
            "arm_elbow_joint",
            "arm_wrist_1_joint",
            "arm_wrist_2_joint",
            "arm_wrist_3_joint",
        ]
        # self.hand_joint_names = [f"rh_joint_{i}" for i in range(16)]  # TODO
        # self.hand_joint_names = [
        #     "rh_joint_1", "rh_joint_0", "rh_joint_2", "rh_joint_3",
        #     "rh_joint_5", "rh_joint_4", "rh_joint_6", "rh_joint_7",
        #     "rh_joint_9", "rh_joint_8", "rh_joint_10", "rh_joint_11",
        #     "rh_joint_12", "rh_joint_13", "rh_joint_14", "rh_joint_15",
        # ]
        self.hand_joint_names = [
            "rh_joint_0", "rh_joint_1", "rh_joint_2", "rh_joint_3",
            "rh_joint_4", "rh_joint_5", "rh_joint_6", "rh_joint_7",
            "rh_joint_8", "rh_joint_9", "rh_joint_10", "rh_joint_11",
            "rh_joint_12", "rh_joint_13", "rh_joint_14", "rh_joint_15",
        ]
        self.joint_names = self.arm_joint_names + self.hand_joint_names
        self.n_hand_joints = 16
        self.n_joints = self.n_arm_joints + self.n_hand_joints
        self.ctrl_freq = ctrl_freq  # Hz
        self.timestep = 1.0 / self.ctrl_freq

        # variable
        self.curr_joint_pos = np.zeros((self.n_joints))
        self.target_joint_pos = self.curr_joint_pos.copy()
        self.one_step_time_record = time.time()

        self.node = node
        self.virtual_hardware = virtual_hardware
        self.use_high_freq_interp = use_high_freq_interp

        # receive states of the robot
        if not virtual_hardware:
            self.arm_qpos_sub = self.node.create_subscription(
                JointState,
                "/robot_joint_states",
                self._joint_pos_callback,
                10,
                callback_group=ReentrantCallbackGroup(),
            )

        # control command publisher
        if self.use_high_freq_interp:
            self.node.get_logger().warn("Please launch the 'main_robot_real_high_freq.py' node.")
            self.robot_joint_pos_command_pub = self.node.create_publisher(JointState, "robot/ctrl/qpos_low_freq", 10)
        else:
            self.arm_joint_pos_command_pub = self.node.create_publisher(VectorStamped, "arm/ctrl/qpos_servo", 10)
            self.hand_joint_pos_command_pub = self.node.create_publisher(JointState, "/cmd_leap_interp", 10)

        self.wait_for_initialization()

        # cuRobo initialization
        self.world_config = {
            "cuboid": {
                "wall_1": {
                    "dims": [5.0, 5.0, 0.2],  # x, y, z
                    "pose": [0.0, 0.0, 0.8, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                },
                # "wall_2": {
                #     "dims": [1.0, 0.2, 0.6],  # x, y, z
                #     "pose": [0.0, -0.8, 0.7, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                # },
                # "wall_3": {
                #     "dims": [0.2, 1.0, -0.3],  # x, y, z
                #     "pose": [0.0, -0.5, 0.4, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                # },
            },
        }
        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            # "ur5_leap.yaml",
            "ur5e.yml",
            self.world_config,
            interpolation_dt=0.05,
        )
        self.motion_gen = MotionGen(self.motion_gen_config)
        self.motion_gen.warmup()

    #############################################################################
    # receiving state
    #############################################################################

    def _joint_pos_callback(self, msg):
        qpos = np.asarray(msg.position)
        self.curr_joint_pos = qpos

    def get_joint_pos(self, names=None, update=True):
        """
        Args:
            update: if False, it will return the current self.curr_joint_pos,
                    but it does not mean the self.curr_joint_pos is the latest received joint pos.
        """
        indices = (
            [self.joint_names.index(name) for name in names] if names is not None else [i for i in range(self.n_joints)]
        )
        return self.curr_joint_pos[indices].copy()

    def get_target_joint_pos(self, names=None):
        indices = (
            [self.joint_names.index(name) for name in names] if names is not None else [i for i in range(self.n_joints)]
        )
        return self.target_joint_pos[indices].copy()

    #############################################################################
    # sending control command
    #############################################################################

    def _publish_robot_joint_pos_command(self, joint_pos):
        """
        Publish command to high-frequency interpolator node.
        """
        assert len(joint_pos) == self.n_joints
        msg = JointState()

        # expect the robot to reach the goal waypoint in one timestep
        expected_time = time_to_seconds(self.node.get_clock().now()) + self.timestep
        msg.header.stamp = seconds_to_stamp(expected_time)

        leap_offset = np.pi if not self.virtual_hardware else 0.0
        msg.position = [q for q in joint_pos[: self.n_arm_joints]] + [
            (q + leap_offset) for q in joint_pos[self.n_arm_joints :]
        ]  # real leap hand: add 180 degree

        self.robot_joint_pos_command_pub.publish(msg)

    def _publish_arm_joint_pos_command(self, arm_joint_pos):
        assert len(arm_joint_pos) == self.n_arm_joints
        msg = VectorStamped()
        msg.data = np.asarray(arm_joint_pos).tolist()
        self.arm_joint_pos_command_pub.publish(msg)

    def _publish_hand_joint_pos_command(self, hand_joint_pos):
        assert len(hand_joint_pos) == self.n_hand_joints
        leap_offset = np.pi if not self.virtual_hardware else 0.0

        msg = JointState()
        msg.name= self.hand_joint_names
        msg.position = [(q + leap_offset) for q in hand_joint_pos]  # real leap hand: add 180 degree
        self.hand_joint_pos_command_pub.publish(msg)

    def ctrl_joint_pos(self, target_joint_pos, names=None):
        indices = (
            [names.index(name) for name in self.joint_names] if names is not None else [i for i in range(self.n_joints)]
        )
        q = np.asarray(target_joint_pos)[indices].tolist()
        if self.use_high_freq_interp:
            self._publish_robot_joint_pos_command(q)

        self.target_joint_pos[:] = q

    def ctrl_arm_joint_pos(self, target_joint_pos, names=None):
        indices = (
            [names.index(name) for name in self.arm_joint_names]
            if names is not None
            else [i for i in range(self.n_arm_joints)]
        )
        q = np.asarray(target_joint_pos)[indices].tolist()
        if not self.use_high_freq_interp:
            self._publish_arm_joint_pos_command(q)

        self.target_joint_pos[: self.n_arm_joints] = q

    def ctrl_hand_joint_pos(self, target_joint_pos, names=None):
        indices = (
            [names.index(name) for name in self.hand_joint_names]
            if names is not None
            else [i for i in range(self.n_hand_joints)]
        )
        q = np.asarray(target_joint_pos)[indices].tolist()
        if not self.use_high_freq_interp:
            self._publish_hand_joint_pos_command(q)

        self.target_joint_pos[self.n_arm_joints :] = q

    #############################################################################
    # Planning and Execution with cuRobo
    #############################################################################
    def plan_traj(self, target_eef_pos, names=None):
        if isinstance(target_eef_pos, np.ndarray):
            target_eef_pos = target_eef_pos.tolist()
        elif isinstance(target_eef_pos, torch.Tensor):
            target_eef_pos = target_eef_pos.tolist()
        elif isinstance(target_eef_pos, list):
            target_eef_pos = target_eef_pos
        else:
            raise TypeError(f"target_eef_pos must be np.ndarray, torch.Tensor or list, but got {type(target_eef_pos)}")
        
        goal_pose = Pose.from_list(target_eef_pos)
        start_state = CuroboJointState.from_position(
            torch.Tensor(self.curr_joint_pos[: self.n_arm_joints]).unsqueeze(0).cuda(),
            # joint_names=[
            #     "arm_shoulder_pan_joint",
            #     "arm_shoulder_lift_joint",
            #     "arm_elbow_joint",
            #     "arm_wrist_1_joint",
            #     "arm_wrist_2_joint",
            #     "arm_wrist_3_joint",
            # ],
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        result = self.motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=5))
        if result.success:
            traj = result.get_interpolated_plan()
        else:
            traj = None
            self.node.get_logger().warning(f"Failed to plan trajectory for {target_eef_pos}")
        return traj
    
    def execute_traj(self, traj):
        joint_pos_traj = traj.position.tolist()
        for q in joint_pos_traj:
            self.ctrl_arm_joint_pos(q)
            self.step()

    #############################################################################
    # others
    #############################################################################

    def wait_for_initialization(self):
        time.sleep(0.2)
        if not self.virtual_hardware:
            # make sure having received joint states
            self.node.get_logger().info("Waiting for joint state initialization.")
            while np.all(self.curr_joint_pos[:] == 0):
                time.sleep(0.1)

            q = self.get_joint_pos()
            if self.use_high_freq_interp:
                # send multiple control command to high-freq-interpolation for padding the window
                for i in range(self.ctrl_freq):
                    self.ctrl_joint_pos(q)
                    self.step()

        self.target_joint_pos = self.curr_joint_pos.copy()
        self.node.get_logger().info("Robot initialization done.")

    def step(self, refresh=False):
        while (time.time() - self.one_step_time_record) < self.timestep:
            time.sleep(0.001)
        self.one_step_time_record = time.time()

def main():
    from threading import Thread

    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=None)
    node = Node("node_name")
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot_real = RobotReal(node, virtual_hardware=False, use_high_freq_interp=True)

    target_joint_pos = robot_real.get_joint_pos()

    while True:
        t1 = time.time()

        robot_real.update_joint_pos()

        robot_real.step()


if __name__ == "__main__":
    main()
