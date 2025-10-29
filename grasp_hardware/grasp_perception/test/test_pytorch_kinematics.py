import math
import torch
import pytorch_kinematics as pk

# load robot description from URDF and specify end effector link
chain = pk.build_serial_chain_from_urdf(open("/home/peter/ur10e.urdf").read(), "flange")
# prints out the (nested) tree of links
print(chain)
# prints out list of joint names
print(chain.get_joint_parameter_names())
device = torch.device("cpu")

# goals are specified as Transform3d poses in the **robot frame**
# so if you have the goals specified in the world frame, you also need the robot frame in the world frame
pos = torch.tensor([0.0, 0.0, 0.0], device=device)
rot = torch.tensor([0.0, 0.0, 0.0], device=device)
rob_tf = pk.Transform3d(pos=pos, rot=rot, device=device)

# specify goals as Transform3d poses in world frame
goal_in_world_frame_tf = pk.Transform3d(pos=[0.2, 0.2, 0.2], rot=[0.0, 0.0, 0.0], device=device)
# convert to robot frame (skip if you have it specified in robot frame already, or if world = robot frame)
goal_in_rob_frame_tf = rob_tf.inverse().compose(goal_in_world_frame_tf)

# get robot joint limits
lim = torch.tensor(chain.get_joint_limits(), device=device)

# create the IK object
# see the constructor for more options and their explanations, such as convergence tolerances
breakpoint()
ik = pk.PseudoInverseIK(chain, max_iterations=30, num_retries=10,
                        joint_limits=lim.T,
                        early_stopping_any_converged=True,
                        early_stopping_no_improvement="all",
                        debug=False,
                        lr=0.2)
# solve IK
sol = ik.solve(goal_in_rob_frame_tf)
# num goals x num retries x DOF tensor of joint angles; if not converged, best solution found so far
print(sol.solutions)
# num goals x num retries can check for the convergence of each run
print(sol.converged)
# num goals x num retries can look at errors directly
print(sol.err_pos)
print(sol.err_rot)
