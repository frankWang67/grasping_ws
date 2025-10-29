import socket
import pickle
import open3d as o3d
from copy import deepcopy
import numpy as np


def normal_to_rotation_matrix(normal):
    # 归一化 normal
    z = normal / np.linalg.norm(normal)

    # 选一个与 z 不共线的向量作为参考轴
    temp = np.array([0, 1, 0]) if abs(z[0]) < 0.99 else np.array([1, 0, 0])

    # x 轴 = temp 和 z 的叉乘
    x = np.cross(temp, z)
    x /= np.linalg.norm(x)

    # y 轴 = z 和 x 的叉乘
    y = np.cross(z, x)

    # 构造旋转矩阵（列向量分别是 x, y, z）
    R_mat = np.stack([x, y, z], axis=1)
    return R_mat


def segment_and_visualize_pcd(pc_np):
    seg_result = {}

    estimate_table_plane(pc_np, seg_result=seg_result)

    # relocalize_object_pointcloud(seg_result)

    # draw_segmentation_result(seg_result)

    return seg_result


def estimate_table_plane(pc_np, distance_threshold=0.01, ransac_n=3, num_iterations=1000, seg_result=None):
    """
    从(N,3)点云中提取桌面平面参数：法向量和一个平面点
    """
    # 转换为Open3D格式
    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(pc_np.astype(np.float64))

    # 预处理
    # pc_o3d = pc_o3d.voxel_down_sample(voxel_size=0.01)  # 根据实际需要调整voxel大小
    min_bound = np.array([-0.2, -0.3, 0.1])
    max_bound = np.array([0.25, 0.15, 0.85])
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    # 裁剪出 ROI 中的点
    pc_o3d = pc_o3d.crop(bbox)

    # 使用RANSAC拟合平面
    plane_model, inliers = pc_o3d.segment_plane(distance_threshold=distance_threshold,
                                                ransac_n=ransac_n,
                                                num_iterations=num_iterations)
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)  # 单位法向量

    point_on_plane = pc_o3d.points[inliers[0]]  # 平面上的一个点

    # 后处理，去除离群点
    table_pcd = pc_o3d.select_by_index(inliers)  # 平面点
    obj_pcd = pc_o3d.select_by_index(inliers, invert=True)  # 非平面点
    obj_pcd = obj_pcd.voxel_down_sample(voxel_size=0.001)  # 根据实际需要调整voxel大小
    obj_pcd = obj_pcd.farthest_point_down_sample(min(4096, len(obj_pcd.points)))  # 保留最多2048个点
    
    # 聚类+分割
    # pcd 已经是你的 PointCloud 对象
    labels = np.array(obj_pcd.cluster_dbscan(eps=0.05, min_points=100, print_progress=True))

    # 忽略噪声点（label 为 -1）
    valid_labels = labels[labels >= 0]

    if valid_labels.size > 0:
        # 找到最大簇的label
        largest_cluster_label = np.bincount(valid_labels).argmax()

        # 提取最大簇
        indices = np.where(labels == largest_cluster_label)[0]
        object_pcd = obj_pcd.select_by_index(indices)
    else:
        object_pcd = o3d.geometry.PointCloud()  # 如果没有有效簇，返回空点云

    seg_result['pcd'] = pc_o3d
    seg_result['obj_pcd'] = deepcopy(object_pcd)

    # project object point cloud onto the table
    table_project_matrix = np.eye(3) - np.outer(normal, normal)
    pc_frame_trans = point_on_plane + table_project_matrix @ (np.mean(np.asarray(object_pcd.points), axis=0) - point_on_plane)
    pc_frame_rot = normal_to_rotation_matrix(-normal)
    pc_frame = np.eye(4)
    pc_frame[:3, :3] = pc_frame_rot
    pc_frame[:3, 3] = pc_frame_trans

    # transform object point cloud
    seg_result['transformed_obj_pcd'] = object_pcd.transform(np.linalg.inv(pc_frame)).scale(0.65, center=np.zeros(3, dtype=np.float64))
    seg_result['pc_frame'] = pc_frame    # 4x4 matrix of the pc frame

    seg_result['plane'] = {
        'normal': -normal,      # revert normal direction
        'point': point_on_plane,
        'inliers': inliers,
    }


def relocalize_object_pointcloud(seg_result):
    obj_pcd = seg_result['obj_pcd']
    plane_normal = seg_result['plane']['normal']

    # rotate point cloud
    R = normal_to_rotation_matrix(plane_normal)
    T = np.eye(4)
    T[:3, :3] = R.T
    obj_pcd.transform(T)

    # translate point cloud
    pcd_min_z = np.min(np.asarray(obj_pcd.points)[:, 2])
    translation = np.array([0, 0, -pcd_min_z])  #
    T = np.eye(4)
    T[:3, 3] = translation
    obj_pcd.transform(T)

    # get object pose
    object_center = np.mean(np.asarray(obj_pcd.points), axis=0)
    seg_result['object_pose'] = np.concatenate([object_center, [1, 0, 0, 0]])

    seg_result['obj_pcd'] = obj_pcd


def draw_segmentation_result(seg_result):
    """
    可视化分割结果
    """
    pcd = seg_result['pcd']

    # 提取平面内点和非平面点
    plane_cloud = pcd.select_by_index(seg_result['plane']['inliers'])
    plane_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # 红色：桌面
    # object_cloud = pcd.select_by_index(seg_result['plane']['inliers'], invert=True)
    # object_cloud.paint_uniform_color([0.0, 1.0, 0.0])  # 绿色：非桌面点
    object_cloud = seg_result['obj_pcd']
    object_cloud.paint_uniform_color([0.0, 1.0, 0.0])

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    coord_frame.transform(seg_result['pc_frame'])  # 将坐标系变换到 pc_frame

    # 可视化平面+点云
    o3d.visualization.draw_geometries([plane_cloud, object_cloud, coord_frame])


def get_generated_grasp_from_server(obj_pcd, obj_pose):
    payload = {'points': obj_pcd, 'array': obj_pose}

    print(f"Sent pointcloud P with shape {obj_pcd.shape}")
    print(f"Sent obj_pose {obj_pose}")

    # 序列化
    data = pickle.dumps(payload)

    # 连接服务器
    HOST = '10.21.70.145'  # 替换为远程IP
    PORT = 50008

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(len(data).to_bytes(8, 'big'))  # 先发送数据长度
        s.sendall(data)  # 再发送数据

        # 接收返回值
        length = int.from_bytes(s.recv(8), 'big')
        recv_data = b''
        while len(recv_data) < length:
            recv_data += s.recv(length - len(recv_data))

        result = pickle.loads(recv_data)

    return result
