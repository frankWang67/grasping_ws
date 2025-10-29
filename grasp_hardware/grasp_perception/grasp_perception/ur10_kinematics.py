import os
import math
from scipy.spatial.transform import Rotation as R
import torch
import pytorch_kinematics as pk

from ament_index_python import get_package_share_directory


class UR10Kinematics(object):
    def __init__(self):
        # load robot description from URDF and specify end effector link
        ur10e_urdf_path = os.path.join(
            get_package_share_directory('grasp_perception'),
            'config',
            'ur10e.urdf'
        )
        ee_frame = "tool0"  # "flange"
        chain = pk.build_serial_chain_from_urdf(open(ur10e_urdf_path).read(), ee_frame)
        # prints out the (nested) tree of links
        print(chain)
        # prints out list of joint names
        print(chain.get_joint_parameter_names())
        joint_names = chain.get_joint_parameter_names()
        lower_limits, higher_limits = chain.get_joint_limits()
        joint_limits = torch.tensor([lower_limits, higher_limits], dtype=torch.float32)
        print("Joint Limits:", joint_limits)
        device = torch.device("cpu")

        self.chain = chain
        self.joint_names = joint_names
        self.joint_limits = joint_limits
        self.device = device

        # goals are specified as Transform3d poses in the **robot frame**
        # so if you have the goals specified in the world frame, you also need the robot frame in the world frame
        pos = torch.tensor([0.0, 0.0, 0.0], device=device)
        rot = torch.tensor([1.0, 0.0, 0.0, 0.0], device=device)
        rob_tf = pk.Transform3d(pos=pos, rot=rot, device=device)

        self.rob_tf = rob_tf

    def compute_fk(self, joint_angles):
        """
        Compute forward kinematics for the UR10 robot given joint angles.
        :param joint_angles: A tensor of joint angles.
        :return: The end-effector pose as a Transform3d object.
        """
        # Convert joint angles to a tensor
        joint_angles_tensor = torch.tensor(joint_angles, device=self.device)

        # Compute forward kinematics
        end_effector_pose = self.chain.forward_kinematics(joint_angles_tensor)
        end_effector_matrix = end_effector_pose.get_matrix()[0]

        fk_result = {
            "position": end_effector_matrix[:3, 3].cpu().numpy().astype(float),
            "orientation": R.from_matrix(end_effector_matrix[:3, :3].cpu().numpy()).as_quat().astype(float)[[3, 0, 1, 2]]
        }

        return fk_result

    def compute_ik(self, goal_position, goal_quaternion, initial_guess):
        retry_configs = 0.1 * torch.randn((10, 6), device=self.device) + torch.tensor(initial_guess, device=self.device)

        # specify goals as Transform3d poses in world frame
        goal_in_world_frame_tf = pk.Transform3d(pos=goal_position, rot=goal_quaternion, device=self.device)
        # convert to robot frame (skip if you have it specified in robot frame already, or if world = robot frame)
        goal_in_rob_frame_tf = self.rob_tf.inverse().compose(goal_in_world_frame_tf)

        # get robot joint limits
        lim = torch.tensor(self.chain.get_joint_limits(), device=self.device)

        # create the IK object
        # see the constructor for more options and their explanations, such as convergence tolerances
        ik = pk.PseudoInverseIK(self.chain, max_iterations=30, num_retries=50,
                                retry_configs=retry_configs,
                                joint_limits=lim.T,
                                early_stopping_any_converged=True,
                                early_stopping_no_improvement="all",
                                debug=False,
                                lr=1.0,
                                regularlization=1e-9,
                                optimizer_method='adam')
        # solve IK
        sol = ik.solve(goal_in_rob_frame_tf)

        if not sol.converged[0].any():
            print("No IK solution found.")
            return None
        else:
            idx = torch.where(sol.converged[0])[0]
            converged_solutions = sol.solutions[0, idx].reshape(-1, 6)
            joint_distance = torch.norm(converged_solutions-torch.tensor(initial_guess), dim=-1)
            best_solution_idx = torch.argmin(joint_distance)
            return converged_solutions[best_solution_idx].reshape(-1, 6)
