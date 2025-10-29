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

from grasping_perception.utils import inverse_pose_transform

class GraspPoseSamplerNode(rclpy.node.Node):
    def __init__(self, config_path="/home/wshf/DexLearn/dexlearn/config", config_name="base"):
        super().__init__('grasp_pose_sampler')

        hydra.main(config_path=config_path, config_name=config_name, version_base=None)(self.load_model)()
        self.qos = QoSProfile(depth=1)

        self.grasp_pose_dict = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
                "grasp_error": -log_prob
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

        # xy_offset = np.array([0, 0])

        # 转torch tensor (N, 3)
        cloud_tensor = torch.from_numpy(points[:, :3])  # float32

        # 调用采样函数
        self.grasp_pose_dict = self.sample(cloud_tensor)

        # 收到一条消息就退出
        self.destroy_subscription(self.subscription)
        self.visualize_grasp_pose(xy_offset)
        
    def __call__(self, object_topic):
        self.subscription = self.create_subscription(PointCloud2, object_topic, self.callback, self.qos)

    def visualize_grasp_pose(self, xy_offset):
        self.joint_pub = self.create_publisher(JointState, '/leaphand/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.joint_names = ['j12', 'j13', 'j14', 'j15', 'j1', 'j0', 'j2', 'j3', 'j5', 'j4', 'j6', 'j7', 'j9', 'j8', 'j10', 'j11']
        self.joint_names = [
            "joint_1", "joint_0", "joint_2", "joint_3",
            "joint_5", "joint_4", "joint_6", "joint_7",
            "joint_9", "joint_8", "joint_10", "joint_11",
            "joint_12", "joint_13", "joint_14", "joint_15",
        ]
        self.joint_positions = self.grasp_pose_dict["pregrasp_qpos"][0, 0, 7:]
        self.joint_positions = self.joint_positions.numpy()
        
        key = "squeeze_qpos"
        self.grasp_pose_dict[key][:, :, :7] = inverse_pose_transform(self.grasp_pose_dict[key][:, :, :7], torch.from_numpy(xy_offset))
        pos = self.grasp_pose_dict[key][0, 0, :7]
        pos = pos.numpy().tolist()

        self.base_pose = Pose()
        self.base_pose.position.x = pos[0]
        self.base_pose.position.y = pos[1]
        self.base_pose.position.z = pos[2]
        self.base_pose.orientation.w = pos[3]
        self.base_pose.orientation.x = pos[4]
        self.base_pose.orientation.y = pos[5]
        self.base_pose.orientation.z = pos[6]

        self.timer = self.create_timer(1.0/30, self.timer_callback)

    def timer_callback(self):
        # 发布关节状态
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.joint_positions.tolist()
        self.joint_pub.publish(js)

        # 发布base位姿
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'table'
        t.child_frame_id = 'palm_lower'
        t.transform.translation.x = self.base_pose.position.x
        t.transform.translation.y = self.base_pose.position.y
        t.transform.translation.z = self.base_pose.position.z
        t.transform.rotation.x = self.base_pose.orientation.x
        t.transform.rotation.y = self.base_pose.orientation.y
        t.transform.rotation.z = self.base_pose.orientation.z
        t.transform.rotation.w = self.base_pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    grasp_pose_sampler = GraspPoseSamplerNode()
    # grasp_pose_sampler("/Workpiece_A_0/pointcloud")
    grasp_pose_sampler("/Yellow_Mustard_Bottle_0/pointcloud")
    rclpy.spin(node=grasp_pose_sampler)
    rclpy.shutdown()

if __name__ == '__main__':
    main()