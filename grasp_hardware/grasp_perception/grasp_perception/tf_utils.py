import numpy as np
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R


def matrix_to_transform(mat: np.ndarray) -> Transform:
    """
    将 4x4 变换矩阵转换为 ROS2 geometry_msgs.msg.Transform。

    Args:
        mat (np.ndarray): 4x4 SE(3) 变换矩阵

    Returns:
        Transform: ROS2 的 Transform 消息
    """
    assert mat.shape == (4, 4)

    # 平移部分
    trans = mat[:3, 3]

    # 旋转部分转四元数
    rot = R.from_matrix(mat[:3, :3])
    quat = rot.as_quat()  # x, y, z, w

    tf_msg = Transform()
    tf_msg.translation.x = trans[0]
    tf_msg.translation.y = trans[1]
    tf_msg.translation.z = trans[2]
    tf_msg.rotation.x = quat[0]
    tf_msg.rotation.y = quat[1]
    tf_msg.rotation.z = quat[2]
    tf_msg.rotation.w = quat[3]

    return tf_msg
