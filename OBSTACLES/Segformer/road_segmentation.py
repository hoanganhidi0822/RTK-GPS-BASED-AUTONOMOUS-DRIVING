import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt
from transformers import SegformerFeatureExtractor, SegformerForSemanticSegmentation
from OBSTACLES.Segformer.utils import predict, draw_segmentation_map, image_overlay
from OBSTACLES.Segformer.config import VIS_LABEL_MAP as LABEL_COLORS_LIST
import time
# --- Thiết lập thông số ---
DEVICE = 'cuda'  # hoặc 'cpu'
MODEL_PATH = 'OBSTACLES/Segformer/model_iou_v1'
IMAGE_SIZE = (640, 480)
ROAD_CLASS = 1
ROWS_TO_CHECK = [160, 180, 200, 230, 300]
WEIGHTS = np.array([0.25, 0.5, 0.17, 0.15, 0.05], dtype=np.float32)
X_REF = 320  # Vị trí trung tâm ảnh tham chiếu

# --- Load model 1 lần ---
extractor = SegformerFeatureExtractor()
model = SegformerForSemanticSegmentation.from_pretrained(MODEL_PATH).to(DEVICE).eval()

# --- Biến toàn cục cho PID ---
def Find_center_points_from_labels(labels: np.ndarray, rows, road_class=1):
    result = []
    for h in rows:
        row = labels[h] == road_class
        indices = np.flatnonzero(row)
        if len(indices) == 0:
            center_x = 320
        else:
            center_x = (indices[0] + indices[-1]) // 2
        result.append((center_x, h))
    return result

# PID control parameters
pre_t = time.time()
error_arr = np.zeros(5)
brake = 0
def PID(error, p, i, d):
    global pre_t, error_arr 
    # Shift and store error history
    error_arr[1:] = error_arr[:-1]
    error_arr[0] = error 
    # Calculate Proportional term
    P = error * p
    # Calculate delta time
    delta_t = time.time() - pre_t
    pre_t = time.time() 
    # Calculate Integral term
    I = np.sum(error_arr) * delta_t * i
    # Calculate Derivative term (if error_arr[1] exists)
    if delta_t > 0:
        D = (error - error_arr[1]) / delta_t * d
    else:
        D = 0
    # Compute the total PID output
    angle = P + I + D
    # Apply output limit
    if abs(angle) > 30:
        angle = np.sign(angle) * 30
    
    return float(angle)


# fig, ax = plt.subplots(figsize=(10, 6))
# plt.ion()
def get_steering_angle(image: np.ndarray, p=0.2, i=0.0001,d = 0.0, debug: bool = False) -> int:
    """
    Dự đoán segmentation, tính trung điểm road, tính error và trả về góc lái.
    
    :param image: ảnh RGB (numpy array) có kích thước bất kỳ
    :param p: hệ số tỉ lệ PID
    :param i: hệ số tích phân PID
    :return: góc lái (-30 đến +30 độ)
    """
    image_resized = cv2.resize(image, IMAGE_SIZE)
    labels = predict(model, extractor, image_resized, DEVICE).cpu().numpy()

    center_points = Find_center_points_from_labels(labels, ROWS_TO_CHECK, road_class=ROAD_CLASS)
    x_coords = np.fromiter((pt[0] for pt in center_points), dtype=np.int32)
    error = np.dot(WEIGHTS, x_coords) - X_REF

    angle = PID(error, p, i, d)

    if debug:
        seg_map = draw_segmentation_map(torch.tensor(labels), LABEL_COLORS_LIST)
        overlay = image_overlay(image_resized, seg_map)

        # Vẽ các điểm center
        for pt in center_points:
            cv2.circle(overlay, pt, 5, (0, 0, 255), -1)
    
    return round(angle), overlay