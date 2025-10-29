import os
import cv2
import json
from matplotlib import pyplot as plt
import numpy as np
import open3d as o3d
import requests


def show_mask(mask, ax, random_color=False, borders = True):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask = mask.astype(np.uint8)
    mask_image =  mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    if borders:
        import cv2
        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        # Try to smooth contours
        contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
        mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2) 
    ax.imshow(mask_image)

def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   

def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0, 0, 0, 0), lw=2))    

def show_masks(image, masks, scores, point_coords=None, box_coords=None, input_labels=None, borders=True):
    for i, (mask, score) in enumerate(zip(masks, scores)):
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        show_mask(mask, plt.gca(), borders=borders)
        if point_coords is not None:
            assert input_labels is not None
            show_points(point_coords, input_labels, plt.gca())
        if box_coords is not None:
            # boxes
            show_box(box_coords, plt.gca())
        if len(scores) > 1:
            plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()

# load the following image, pointcloud, and clicked points
# save_root = './src/grasp_perception/debug'
# cv2.imwrite(os.path.join(save_root, f'image.jpg'), img)
# np.savez(os.path.join(save_root, 'clicked_points.npz'), points=np.array(points))
# np.savez(os.path.join(save_root, 'pointcloud.npz'), pc_data=pc_data)

# load here
image = cv2.imread(os.path.join('./src/grasp_perception/debug', 'image.jpg'))
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
img_bytes = cv2.imencode('.png', image)[1].tobytes()
points = np.load(os.path.join('./src/grasp_perception/debug', 'clicked_points.npz'))['points'].tolist()
pc_data = np.load(os.path.join('./src/grasp_perception/debug', 'pointcloud.npz'))['pc_data']
pc_color = np.load(os.path.join('./src/grasp_perception/debug', 'pointcloud_color.npz'))['pc_color']

print(f"Send image shape: {image.shape}")

# 发送请求
import time
t_start = time.time()
response = requests.post(
    'http://10.21.70.145:8000/sam2',
    files={'image': ('image.png', img_bytes, 'image/png')},
    data={'clicks': json.dumps(points)}
)
print(f"Segmentation on cloud done in {time.time() - t_start:.2f} seconds")

# 解析服务器返回的掩码
masks = cv2.imdecode(np.frombuffer(response.content, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
masks_resized = cv2.resize(masks, (848, 480), interpolation=cv2.INTER_NEAREST)
masks_resized = masks_resized.astype(np.float32)
masks_resized = masks_resized / 255

breakpoint()
# image = cv2.resize(image, (848, 480), interpolation=cv2.INTER_LINEAR)
# show_masks(image, np.transpose(masks, (2, 0, 1)) / 255, scores=np.ones(masks.shape[0]), input_labels=np.ones(len(points)), point_coords=np.array(points), borders=True)

pc_resized = pc_data.reshape(480, 848, 3)

mask_id = 1
masks_coords = masks_resized[:, :, mask_id].nonzero()
pc_masked = pc_resized[masks_coords[0], masks_coords[1], :]
roi_coords = np.where(np.linalg.norm(pc_resized - pc_resized.mean(axis=(0, 1)), axis=-1) < 1.0)  # Example ROI condition

pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(pc_masked)

# paint masks_coords[pc_masked] as red, other points as black
pc_colors = image / 255.0
# pc_colors = pc_color.reshape(480, 848, 3) / 255.0

pcd.points = o3d.utility.Vector3dVector(pc_resized[roi_coords[0], roi_coords[1], :].reshape(-1, 3))
pcd.colors = o3d.utility.Vector3dVector(pc_colors[roi_coords[0], roi_coords[1], :].reshape(-1, 3))  # Red for masked points

# pcd.points = o3d.utility.Vector3dVector(pc_resized[masks_coords[0], masks_coords[1], :].reshape(-1, 3))
# pcd.colors = o3d.utility.Vector3dVector(pc_colors[masks_coords[0], masks_coords[1], :].reshape(-1, 3))  # Red for masked points

o3d.visualization.draw([{
    "name": "object",
    "geometry": pcd,
    "point_size": 0.2
}])

breakpoint()
