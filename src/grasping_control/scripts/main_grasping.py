#!/home/wshf/.conda/envs/grasping/bin/python
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from threading import Thread

from grasping_perception.grasp_pose_sampler import GraspPoseSampler
from grasping_control.robot_real import RobotReal
from grasping_control.tf_utils import TfQuerier

HAND_JOINT_NAMES = [
    "rh_joint_1", "rh_joint_0", "rh_joint_2", "rh_joint_3",
    "rh_joint_5", "rh_joint_4", "rh_joint_6", "rh_joint_7",
    "rh_joint_9", "rh_joint_8", "rh_joint_10", "rh_joint_11",
    "rh_joint_12", "rh_joint_13", "rh_joint_14", "rh_joint_15",
]

HAND_HOME_QPOS = np.zeros(16)
HAND_HOME_QPOS[13] += 1.61

class MainGrasping:
    def __init__(self, node: Node):
        self.node = node
        self.tf_querier = TfQuerier(self.node)
        self.robot_real = RobotReal(self.node)
        self.grasp_pose_sampler = GraspPoseSampler(self.node)

    def plan_and_execute(self, qpos, stage):
        T_base_palm = self.tf_querier.get_homogeneous_matrix(qpos[:7])
        T_palm_tool0 = self.tf_querier.get_transform_matrix("arm_tool0", "rh_palm_lower")
        T_base_tool0 = T_base_palm @ T_palm_tool0
        pose_tool0 = self.tf_querier.get_pose_from_matrix(T_base_tool0)

        traj = self.robot_real.plan_traj(pose_tool0)
        if traj is None:
            self.node.get_logger().warning(f"Failed to plan trajectory in stage {stage}.")
            return False
        self.robot_real.execute_traj(traj)
        self.robot_real.ctrl_hand_joint_pos(qpos[7:], HAND_JOINT_NAMES)
        return True

    def grasp(self, object_topic):
        grasp_pose_dict = self.grasp_pose_sampler(object_topic)
        for i in range(grasp_pose_dict["pregrasp_qpos"].shape[1]):
            pregrasp_qpos = grasp_pose_dict["pregrasp_qpos"][0, i, :]
            grasp_qpos = grasp_pose_dict["grasp_qpos"][0, i, :]
            squeeze_qpos = grasp_pose_dict["squeeze_qpos"][0, i, :]

            pregrasp_qpos[:7] = self.tf_querier.transform_pose(pregrasp_qpos[:7], "arm_base_link", "table")
            grasp_qpos[:7] = self.tf_querier.transform_pose(grasp_qpos[:7], "arm_base_link", "table")
            squeeze_qpos[:7] = self.tf_querier.transform_pose(squeeze_qpos[:7], "arm_base_link", "table")

            if not self.plan_and_execute(pregrasp_qpos, "pregrasp"):
                self.node.get_logger().warning(f"Planning failed on the {i}-th pregrasp, try the next one.")
                continue
            if not self.plan_and_execute(grasp_qpos, "grasp"):
                self.node.get_logger().warning(f"Planning failed on the {i}-th grasp, try the next one.")
                continue
            if not self.plan_and_execute(squeeze_qpos, "squeeze"):
                self.node.get_logger().warning(f"Planning failed on the {i}-th squeeze, try the next one.")
                continue
            
            final_pose = squeeze_qpos[:7]
            final_pose = self.tf_querier.transform_pose(final_pose, "table", "arm_base_link")
            final_pose[2] += 0.2
            final_pose = self.tf_querier.transform_pose(final_pose, "arm_base_link", "table")

            if not self.plan_and_execute(np.concatenate((final_pose, squeeze_qpos[7:])), "final"):
                self.node.get_logger().warning(f"Planning failed on the {i}-th final, try the next one.")
                continue

            input("Press Enter to quit.")
            quit_pose = final_pose
            quit_pose = self.tf_querier.transform_pose(quit_pose, "table", "arm_base_link")
            quit_pose[0], quit_pose[1] = 0.36, 0.0
            quit_pose = self.tf_querier.transform_pose(quit_pose, "arm_base_link", "table")

            if not self.plan_and_execute(np.concatenate((quit_pose, HAND_HOME_QPOS)), "quit"):
                self.node.get_logger().warning(f"Planning failed on the {i}-th quit, try the next one.")
                continue

            return True
        return False
    
def main():
    rclpy.init(args=None)
    node = Node("main_grasping")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    main_grasping = MainGrasping(node)
    main_grasping.grasp("/Workpiece_A_0/pointcloud")
    main_grasping.grasp("/Workpiece_A_0/pointcloud")
    # main_grasping.grasp("/Yellow_Mustard_Bottle_0/pointcloud")

    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
