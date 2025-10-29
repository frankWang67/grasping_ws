import time
import hydra
import torch
import numpy as np

import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from dexlearn.network.models import *
from dexlearn.dataset import minkowski_collate_fn
from dexlearn.utils.util import set_seed

from grasping_perception.utils import inverse_pose_transform, sort_by_z_axis_angle

class GraspPoseSampler:
    def __init__(self, node: rclpy.node.Node, config_path="/home/wshf/DexLearn/dexlearn/config", config_name="base"):
        self.node = node

        hydra.main(config_path=config_path, config_name=config_name, version_base=None)(self.load_model)()
        self.qos = QoSProfile(depth=1)

        self.grasp_pose_dict = None
        self.ready = False
        self.rate = self.node.create_rate(10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def load_model(self, config):
        self.config = config
        set_seed(config.seed)
        config.wandb.mode = "disabled"
        self.model = eval(config.algo.model.name)(config.algo.model)

        # load ckpt if exists
        if config.ckpt is not None:
            ckpt = torch.load(config.ckpt, map_location="cuda", weights_only=False)
            self.model.load_state_dict(ckpt["model"])
            ckpt_iter = ckpt["iter"]
            print("loaded ckpt from", config.ckpt)
        else:
            raise ValueError("No model checkpoint found!")
        
    def sample(self, object_pc):
        with torch.no_grad():
            data = {
                "point_clouds": object_pc,
                "coors": object_pc / self.config.algo.model.backbone.voxel_size,
                "feats": object_pc,
                "grasp_type_id": 0
            }
            data = minkowski_collate_fn([data])
            robot_pose, log_prob = self.model.sample(data, self.config.algo.test_grasp_num)

            # select top k predictions with higher log_prob
            topk_indices = torch.topk(log_prob, self.config.algo.test_topk, dim=1).indices
            batch_indices = (
                torch.arange(robot_pose.size(0))
                .unsqueeze(1)
                .expand(-1, self.config.algo.test_topk)
            )
            robot_pose = robot_pose[batch_indices, topk_indices]
            log_prob = log_prob[batch_indices, topk_indices]
            res = {
                "pregrasp_qpos": robot_pose[..., 0, :],
                "grasp_qpos": robot_pose[..., 1, :],
                "squeeze_qpos": robot_pose[..., 2, :],
                # "grasp_error": -log_prob
            }

            return res

    def callback(self, msg):
        transform = self.tf_buffer.lookup_transform(
            target_frame="table",
            source_frame=msg.header.frame_id,
            time=rclpy.time.Time()
        )
        msg = do_transform_cloud(msg, transform)
        
        points_raw = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        if len(points_raw) == 0:
            self.get_logger().warn("Point cloud is empty!")
            return

        # 转成numpy数组 (N, 3)，每行 [x, y, z]
        points = np.stack([points_raw['x'], points_raw['y'], points_raw['z']], axis=-1).astype(np.float32)
        
        xy_offset = np.mean(points[:, :2], axis=0)
        points[:, :2] -= xy_offset

        # 转torch tensor (N, 3)
        cloud_tensor = torch.from_numpy(points[:, :3])  # float32

        # 调用采样函数
        self.grasp_pose_dict = self.sample(cloud_tensor)
        _, indices = sort_by_z_axis_angle(self.grasp_pose_dict["pregrasp_qpos"][0, :, 3:7])
        self.grasp_pose_dict["pregrasp_qpos"] = self.grasp_pose_dict["pregrasp_qpos"][:, indices]
        self.grasp_pose_dict["grasp_qpos"] = self.grasp_pose_dict["grasp_qpos"][:, indices]
        self.grasp_pose_dict["squeeze_qpos"] = self.grasp_pose_dict["squeeze_qpos"][:, indices]

        delta_qpos_1 = self.grasp_pose_dict["grasp_qpos"] - self.grasp_pose_dict["pregrasp_qpos"]
        delta_qpos_1[:, :, 2] *= 0.2
        delta_qpos_1[:, :, -1] = np.pi / 8
        delta_qpos_1[:, :, 7:-1] *= 1.5
        delta_qpos_2 = self.grasp_pose_dict["squeeze_qpos"] - self.grasp_pose_dict["grasp_qpos"]
        delta_qpos_2[:, :, 2] *= 0.2
        delta_qpos_2[:, :, -1] = np.pi / 8
        delta_qpos_2[:, :, 7:-1] *= 1.5
        self.grasp_pose_dict["grasp_qpos"] = self.grasp_pose_dict["pregrasp_qpos"] + delta_qpos_1
        self.grasp_pose_dict["squeeze_qpos"] = self.grasp_pose_dict["grasp_qpos"] + delta_qpos_2
        
        for key in self.grasp_pose_dict.keys():
            self.grasp_pose_dict[key][:, :, :7] = inverse_pose_transform(self.grasp_pose_dict[key][:, :, :7], torch.from_numpy(xy_offset))
            self.grasp_pose_dict[key] = self.grasp_pose_dict[key].cpu().numpy()

        # 收到一条消息就退出
        self.node.destroy_subscription(self.subscription)
        self.ready = True
        
    def __call__(self, object_topic):
        self.subscription = self.node.create_subscription(PointCloud2, object_topic, self.callback, self.qos)

        while not self.ready:
            self.rate.sleep()
        self.ready = False

        return self.grasp_pose_dict