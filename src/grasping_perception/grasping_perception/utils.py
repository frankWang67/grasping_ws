import cv2
import torch
import numpy as np

def calculate_iou_cxcywh(boxes1: torch.Tensor, boxes2: torch.Tensor, epsilon: float = 1e-6) -> torch.Tensor:
    """
    计算两组 bounding boxes 之间的 IoU。
    Boxes 的格式为 [center_x, center_y, width, height]。

    Args:
        boxes1 (torch.Tensor): 第一组 bounding boxes，形状为 (N, 4)。
        boxes2 (torch.Tensor): 第二组 bounding boxes，形状为 (M, 4)。
        epsilon (float): 防止除以零的小数。

    Returns:
        torch.Tensor: IoU 矩阵，形状为 (N, M)。
    """
    # ===== 步骤 1: 将 [cx, cy, w, h] 转换为 [x1, y1, x2, y2] =====
    # box1_x1, box1_y1, box1_x2, box1_y2
    b1_x1 = boxes1[..., 0] - boxes1[..., 2] / 2
    b1_y1 = boxes1[..., 1] - boxes1[..., 3] / 2
    b1_x2 = boxes1[..., 0] + boxes1[..., 2] / 2
    b1_y2 = boxes1[..., 1] + boxes1[..., 3] / 2
    
    # box2_x1, box2_y1, box2_x2, box2_y2
    b2_x1 = boxes2[..., 0] - boxes2[..., 2] / 2
    b2_y1 = boxes2[..., 1] - boxes2[..., 3] / 2
    b2_x2 = boxes2[..., 0] + boxes2[..., 2] / 2
    b2_y2 = boxes2[..., 1] + boxes2[..., 3] / 2

    # 为了利用 PyTorch 的广播机制，我们需要调整张量的维度
    # 让 b1 的形状变为 (N, 1, 4) 和 b2 的形状变为 (1, M, 4)
    # 这样它们之间的操作就会为每个 b1 中的 box 和每个 b2 中的 box 计算
    b1_x1 = b1_x1.unsqueeze(1)
    b1_y1 = b1_y1.unsqueeze(1)
    b1_x2 = b1_x2.unsqueeze(1)
    b1_y2 = b1_y2.unsqueeze(1)

    b2_x1 = b2_x1.unsqueeze(0)
    b2_y1 = b2_y1.unsqueeze(0)
    b2_x2 = b2_x2.unsqueeze(0)
    b2_y2 = b2_y2.unsqueeze(0)

    # ===== 步骤 2: 计算交集面积 =====
    inter_x1 = torch.max(b1_x1, b2_x1)
    inter_y1 = torch.max(b1_y1, b2_y1)
    inter_x2 = torch.min(b1_x2, b2_x2)
    inter_y2 = torch.min(b1_y2, b2_y2)

    # 使用 clamp(min=0) 来替代 max(0, ...)，确保宽高不为负
    inter_w = torch.clamp(inter_x2 - inter_x1, min=0)
    inter_h = torch.clamp(inter_y2 - inter_y1, min=0)
    
    intersection_area = inter_w * inter_h

    # ===== 步骤 3: 计算并集面积 =====
    # b1 和 b2 的原始面积
    b1_area = boxes1[..., 2] * boxes1[..., 3]  # (N,)
    b2_area = boxes2[..., 2] * boxes2[..., 3]  # (M,)

    # 调整维度以进行广播
    b1_area = b1_area.unsqueeze(1) # (N, 1)
    b2_area = b2_area.unsqueeze(0) # (1, M)

    union_area = b1_area + b2_area - intersection_area

    # ===== 步骤 4: 计算 IoU =====
    iou = intersection_area / (union_area + epsilon)
    
    return iou
    
def bbox_deduplicate(boxes_1: torch.Tensor, logits_1, phrases_1, boxes_2: torch.Tensor, logits_2, phrases_2, iou_threshold: float):
    iou = calculate_iou_cxcywh(boxes_1, boxes_2)
    dup_matrix = iou > iou_threshold
    dup_mask_1, dup_mask_2 = dup_matrix.any(dim=1), dup_matrix.any(dim=0)
    dedup_mask_1, dedup_mask_2 = ~dup_mask_1, ~dup_mask_2

    dup_pairs = dup_matrix.nonzero()
    dup_logits_1, dup_logits_2 = logits_1[dup_pairs[:, 0]], logits_2[dup_pairs[:, 1]]
    keep_mask_1 = dup_logits_1 > dup_logits_2
    keep_mask_2 = ~keep_mask_1

    dedup_mask_1[dup_pairs[:, 0][keep_mask_1]] = True
    dedup_mask_2[dup_pairs[:, 1][keep_mask_2]] = True

    return boxes_1[dedup_mask_1], logits_1[dedup_mask_1], phrases_1[dedup_mask_1.numpy()], boxes_2[dedup_mask_2], logits_2[dedup_mask_2], phrases_2[dedup_mask_2.numpy()]

def draw_masks(img, masks, alpha=0.5):
    overlay = img.copy()
    for mask in masks:
        color = np.random.random(3) * 255
        color = color.astype(img.dtype)
        overlay[mask] = color
    img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    return img

def _sqrt_positive_part(x: torch.Tensor) -> torch.Tensor:
    """
    Returns torch.sqrt(torch.max(0, x))
    but with a zero subgradient where x is 0.
    """
    ret = torch.zeros_like(x)
    positive_mask = x > 0
    if torch.is_grad_enabled():
        ret[positive_mask] = torch.sqrt(x[positive_mask])
    else:
        ret = torch.where(positive_mask, torch.sqrt(x), ret)
    return ret

def standardize_quaternion(quaternions: torch.Tensor) -> torch.Tensor:
    """
    Convert a unit quaternion to a standard form: one in which the real
    part is non negative.

    Args:
        quaternions: Quaternions with real part first,
            as tensor of shape (..., 4).

    Returns:
        Standardized quaternions as tensor of shape (..., 4).
    """
    return torch.where(quaternions[..., 0:1] < 0, -quaternions, quaternions)

def torch_quaternion_to_matrix(quaternions: torch.Tensor) -> torch.Tensor:
    """Convert rotations given as quaternions to rotation matrices.

    Args:
        quaternions: quaternions with real part first, as tensor of shape (..., 4).

    Returns:
        Rotation matrices as tensor of shape (..., 3, 3).
    """

    quaternions = torch.as_tensor(quaternions)
    r, i, j, k = torch.unbind(quaternions, -1)
    two_s = 2.0 / (quaternions * quaternions).sum(-1)

    o = torch.stack(
        (
            1 - two_s * (j * j + k * k),
            two_s * (i * j - k * r),
            two_s * (i * k + j * r),
            two_s * (i * j + k * r),
            1 - two_s * (i * i + k * k),
            two_s * (j * k - i * r),
            two_s * (i * k - j * r),
            two_s * (j * k + i * r),
            1 - two_s * (i * i + j * j),
        ),
        -1,
    )
    return o.reshape(quaternions.shape[:-1] + (3, 3))

def torch_matrix_to_quaternion(matrix: torch.Tensor) -> torch.Tensor:
    """
    Convert rotations given as rotation matrices to quaternions.

    Args:
        matrix: Rotation matrices as tensor of shape (..., 3, 3).

    Returns:
        quaternions with real part first, as tensor of shape (..., 4).
    """
    if matrix.size(-1) != 3 or matrix.size(-2) != 3:
        raise ValueError(f"Invalid rotation matrix shape {matrix.shape}.")

    batch_dim = matrix.shape[:-2]
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = torch.unbind(
        matrix.reshape(batch_dim + (9,)), dim=-1
    )

    q_abs = _sqrt_positive_part(
        torch.stack(
            [
                1.0 + m00 + m11 + m22,
                1.0 + m00 - m11 - m22,
                1.0 - m00 + m11 - m22,
                1.0 - m00 - m11 + m22,
            ],
            dim=-1,
        )
    )

    # we produce the desired quaternion multiplied by each of r, i, j, k
    quat_by_rijk = torch.stack(
        [
            # pyre-fixme[58]: `**` is not supported for operand types `Tensor` and
            #  `int`.
            torch.stack([q_abs[..., 0] ** 2, m21 - m12, m02 - m20, m10 - m01], dim=-1),
            # pyre-fixme[58]: `**` is not supported for operand types `Tensor` and
            #  `int`.
            torch.stack([m21 - m12, q_abs[..., 1] ** 2, m10 + m01, m02 + m20], dim=-1),
            # pyre-fixme[58]: `**` is not supported for operand types `Tensor` and
            #  `int`.
            torch.stack([m02 - m20, m10 + m01, q_abs[..., 2] ** 2, m12 + m21], dim=-1),
            # pyre-fixme[58]: `**` is not supported for operand types `Tensor` and
            #  `int`.
            torch.stack([m10 - m01, m20 + m02, m21 + m12, q_abs[..., 3] ** 2], dim=-1),
        ],
        dim=-2,
    )

    # We floor here at 0.1 but the exact level is not important; if q_abs is small,
    # the candidate won't be picked.
    flr = torch.tensor(0.1).to(dtype=q_abs.dtype, device=q_abs.device)
    quat_candidates = quat_by_rijk / (2.0 * q_abs[..., None].max(flr))

    # if not for numerical problems, quat_candidates[i] should be same (up to a sign),
    # forall i; we pick the best-conditioned one (with the largest denominator)
    out = quat_candidates[
        torch.nn.functional.one_hot(q_abs.argmax(dim=-1), num_classes=4) > 0.5, :
    ].reshape(batch_dim + (4,))
    return standardize_quaternion(out)

def inverse_pose_transform(robot_pose, xy_offset):
    robot_pose[:, :, :2] += xy_offset
    # robot_pose[:, :, 0] -= 0.05
    robot_pose[:, :, 1] -= 0.05 * robot_pose[:, :, 1] + 0.03
    # robot_pose[:, :, 1] -= 0.04
    robot_pose[:, :, 2] -= 0.015

    # # 提取当前旋转矩阵
    # tmp_rot = torch_quaternion_to_matrix(torch.tensor(robot_pose[:, :, 3:7]))

    # # 把位置加回去
    # robot_pose[:, :, :3] += (tmp_rot @ torch.tensor([0.05, 0, 0]))
    # # robot_pose[:, :, :3] += (tmp_rot @ torch.tensor([0, 0.01, 0]))

    # # 再逆旋转（180°的逆就是再旋转180°）
    # delta_rot = torch_quaternion_to_matrix(torch.tensor([0, 1, 0, 0]).view(1, 1, 4))
    # tmp_rot = tmp_rot @ delta_rot.transpose(-1, -2)
    # robot_pose[:, :, 3:7] = torch_matrix_to_quaternion(tmp_rot)

    return robot_pose

def quats_to_rotmat(quats: torch.Tensor) -> torch.Tensor:
    """
    输入: (N,4) [w, x, y, z]
    输出: (N,3,3) 旋转矩阵
    """
    w, x, y, z = quats[:,0], quats[:,1], quats[:,2], quats[:,3]
    N = quats.shape[0]
    R = torch.zeros((N,3,3), dtype=quats.dtype, device=quats.device)

    R[:,0,0] = 1 - 2*(y**2 + z**2)
    R[:,0,1] = 2*(x*y - z*w)
    R[:,0,2] = 2*(x*z + y*w)

    R[:,1,0] = 2*(x*y + z*w)
    R[:,1,1] = 1 - 2*(x**2 + z**2)
    R[:,1,2] = 2*(y*z - x*w)

    R[:,2,0] = 2*(x*z - y*w)
    R[:,2,1] = 2*(y*z + x*w)
    R[:,2,2] = 1 - 2*(x**2 + y**2)
    return R

def sort_by_z_axis_angle(quats: torch.Tensor):
    R = quats_to_rotmat(quats)           # (N,3,3)
    z_axes = R[:, :, 2]                  # 每个旋转后的z轴 (N,3)
    z_axes = z_axes / z_axes.norm(dim=1, keepdim=True)  # 归一化

    # 与世界z轴 [0,0,1] 点乘
    dot = z_axes[:,2]   # 因为 [0,0,1] 只取z分量
    dot = torch.clamp(dot, -1.0, 1.0)    # 防止数值误差超界
    angles = torch.acos(dot)             # (N,)

    # 排序
    sorted_angles, indices = torch.sort(angles)
    return sorted_angles, indices