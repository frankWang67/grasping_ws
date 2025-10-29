#!/home/wshf/.conda/envs/grasping/bin python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

import torch
from torchvision import transforms
from torchvision.ops import box_convert
from groundingdino.util.inference import load_model, predict, annotate
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

from grasping_perception.utils import *

COLOR_IMAGE_TOPIC = "/rgb/image_raw"

DINO_MODEL_CONFIG_PATH = "/home/wshf/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
DINO_MODEL_CHECKPOINT_PATH = "/home/wshf/GroundingDINO/weights/groundingdino_swint_ogc.pth"

BOX_THRESHOLD = 0.3
TEXT_THRESHOLD = 0.25
AREA_THRESHOLD = 0.1
IOU_THRESHOLD = 0.9

TEXT_PROMPTS = [
    # "yellow mustard bottle",
    "black workpiece",
    "grey circle workpiece",
    "black cone workpiece"
]
ELASTIC_PROMPT = TEXT_PROMPTS[0]
CLASS_COUNT = len(TEXT_PROMPTS)

SAM2_CHECKPOINT = "/home/wshf/sam2/checkpoints/sam2.1_hiera_large.pt"
SAM2_MODEL_CONFIG = "configs/sam2.1/sam2.1_hiera_l.yaml"

class DetectionNode(Node):
    def __init__(self, node_name="detection_node"):
        super().__init__(node_name)

        self.initialize_models()
        
        self.subscription = self.create_subscription(
            RosImage,
            COLOR_IMAGE_TOPIC,
            self.listener_callback,
            10) # qos_profile_sensor_data or a number like 10
        self.get_logger().info("Node launched.")

    def initialize_models(self):
        # 初始化cv_bridge
        self.bridge = CvBridge()

        # Grounding DINO模型初始化
        self.get_logger().info("Start loading model...")
        start_time = time.time()
        self.model = load_model(DINO_MODEL_CONFIG_PATH, DINO_MODEL_CHECKPOINT_PATH)
        self.get_logger().info(f"Model loading time: {time.time() - start_time}")
        self.transform = transforms.Compose([
            # This one transform handles 3 steps:
            # 1. Converts NumPy/PIL Image to FloatTensor
            # 2. Scales pixel values from [0, 255] to [0.0, 1.0]
            # 3. Changes image layout from HWC to CHW
            transforms.ToTensor(),

            # Models pre-trained on ImageNet often require this normalization step.
            # Check GroundingDINO's documentation to see if it's needed.
            # If so, uncomment the following line.
            # transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

        # SAM2模型初始化
        self.predictor = SAM2ImagePredictor(build_sam2(SAM2_MODEL_CONFIG, SAM2_CHECKPOINT))

    def listener_callback(self, msg: RosImage):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            h, w = cv_image.shape[0], cv_image.shape[1]
            cv_image = cv_image[h//2:h*3//4, w//4:w*3//4]
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        boxes, logits, phrases = self.detect(rgb_image)
        annotated_img = annotate(rgb_image, boxes, logits, phrases)

        all_masks = self.segment(rgb_image, boxes)
        annotated_img = draw_masks(annotated_img, all_masks)

        cv2.imshow("detection", annotated_img)
        cv2.waitKey(10)

    def detect_one_class(self, img_tensor, prompt):
        boxes, logits, phrases = predict(self.model, img_tensor, prompt, BOX_THRESHOLD, TEXT_THRESHOLD)

        box_areas = boxes[:, 2] * boxes[:, 3]
        box_mask = box_areas < AREA_THRESHOLD
        boxes, logits, phrases = boxes[box_mask], logits[box_mask], np.array(phrases)[box_mask.numpy()]

        return boxes, logits, phrases
    
    def detect(self, rgb_image):
        img_tensor = self.transform(rgb_image)
        boxes_list, logits_list, phrases_list = [], [], []
        for i in range(CLASS_COUNT):
            boxes_i, logits_i, phrases_i = self.detect_one_class(img_tensor, TEXT_PROMPTS[i])
            if TEXT_PROMPTS[i] == ELASTIC_PROMPT:
                logits_i += 0.5
            boxes_list.append(boxes_i)
            logits_list.append(logits_i)
            phrases_list.append(phrases_i)
        
        for i in range(CLASS_COUNT):
            for j in range(i+1, CLASS_COUNT):
                boxes_list[i], logits_list[i], phrases_list[i], boxes_list[j], logits_list[j], phrases_list[j] = \
                    bbox_deduplicate(boxes_list[i], logits_list[i], phrases_list[i], boxes_list[j], logits_list[j], phrases_list[j], IOU_THRESHOLD)

        boxes = torch.concat(boxes_list, dim=0)
        logits = torch.concat(logits_list, dim=0)
        phrases = np.concatenate(phrases_list, axis=0)

        return boxes, logits, phrases
    
    def segment(self, rgb_image, boxes):
        h, w, _ = rgb_image.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()
        all_masks = []
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.predictor.set_image(rgb_image)
            for i in range(boxes.shape[0]):
                masks_i, scores_i, logits_i = self.predictor.predict(box=xyxy[i])
                sorted_ind_i = np.argsort(scores_i)[::-1]
                all_masks.append(masks_i[sorted_ind_i[0]].astype(np.bool_))
        all_masks = np.array(all_masks)

        return all_masks

def main(args=None):
    rclpy.init(args=args)
    
    detection_node = DetectionNode()
    
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()