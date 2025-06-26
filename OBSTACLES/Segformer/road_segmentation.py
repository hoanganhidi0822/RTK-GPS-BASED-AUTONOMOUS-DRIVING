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
MODEL_PATH = 'OBSTACLES/Segformer/model_iou_v2'
IMAGE_SIZE = (640, 480)
ROAD_CLASS = 1
ROWS_TO_CHECK = [170, 190, 220, 270, 320]
WEIGHTS = np.array([0.0, 0.0, 0.8, 0.0, 0.0], dtype=np.float32)
X_REF =  325 # Vị trí trung tâm ảnh tham chiếu

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

pre_t = time.time()
error_arr = np.zeros(5)
integral = 0
INTEGRAL_LIMIT = 5  # Ngưỡng chống bão hòa

def PID(error, p, i, d):
    global pre_t, error_arr, integral

    # Shift lỗi
    error_arr[1:] = error_arr[:-1]
    error_arr[0] = error

    # Tính thời gian
    cur_t = time.time()
    delta_t = cur_t - pre_t
    pre_t = cur_t

    # PID components
    P = error * p

    if delta_t > 0:
        integral += error * delta_t
        # Giới hạn tích phân
        integral = np.clip(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        I = integral * i
        D = ((error - error_arr[1]) / delta_t) * d
    else:
        I = 0
        D = 0

    angle = P + I + D

    # Chống bão hòa: reset tích phân nếu vượt ngưỡng
    if abs(angle) > 20:
        integral = 0  # hoặc: integral *= 0.8

    # Giới hạn đầu ra
    angle = np.clip(angle, -20, 20)

    return float(angle)


def detect_car_position_from_mask(labels: np.ndarray, car_class=2, row_start=120) -> str:
    """
    Phát hiện bên trái hay phải có nhiều xe hơn từ row_start trở xuống.

    :param labels: segmentation labels (np.ndarray)
    :param car_class: nhãn class xe (mặc định 2)
    :param row_start: dòng bắt đầu kiểm tra
    :return: "left", "right", hoặc "center"
    """
    h, w = labels.shape
    region = labels[row_start:, :]  # chỉ kiểm tra từ row_start trở xuống

    left_area = region[:, :w//2]
    right_area = region[:, w//2:]

    left_car_count = np.sum(left_area == car_class)
    right_car_count = np.sum(right_area == car_class)

    if right_car_count > left_car_count:
        return "right"
    elif left_car_count > right_car_count:
        return "left"
    else:
        return "center"

def find_midpoint_segment(mask, height):
    # Lấy 1 hàng từ ảnh nhị phân
    line = mask[height, :]
    arr = np.argwhere(line == 1)

    # Nếu không có điểm hợp lệ, thử dòng dự phòng cách đó 50 pixel
    if len(arr) == 0 and height + 50 < mask.shape[0]:
        line = mask[height + 50, :]
        arr = np.argwhere(line == 1)

    # Nếu vẫn không có gì -> trả giá trị mặc định
    if len(arr) == 0:
        return 0, mask.shape[1] // 2, (0,), (0,)

    # Trung điểm giữa điểm đầu và cuối
    center = int(arr.mean())
    error = mask.shape[1] // 2 - center
    return error, center, arr[0], arr[-1]

def find_right_lane(mask, height, lane_width=200):
    line = mask[height, :]
    arr = np.argwhere(line == 1)

    if len(arr) == 0 and height + 50 < mask.shape[0]:
        line = mask[height + 50, :]
        arr = np.argwhere(line == 1)

    if len(arr) == 0:
        return 0, mask.shape[1] // 2, (0,), (0,)

    max_lane = arr[-1][0]  # điểm ngoài cùng bên phải
    center = max_lane - lane_width
    error = mask.shape[1] // 2 - center
    return error, center, arr[0], arr[-1]


def find_left_lane(mask, height, lane_width=200):
    line = mask[height, :]
    arr = np.argwhere(line == 1)

    if len(arr) == 0 and height + 50 < mask.shape[0]:
        line = mask[height + 50, :]
        arr = np.argwhere(line == 1)

    if len(arr) == 0:
        return 0, mask.shape[1] // 2, (0,), (0,)

    min_lane = arr[0][0]  # điểm ngoài cùng bên trái
    center = min_lane + lane_width
    error = mask.shape[1] // 2 - center
    return error, center, arr[0], arr[-1]

def is_intersection(mask: np.ndarray, row: int = 240, threshold: int = 450, road_class: int = 1) -> bool:
    """
    Kiểm tra xem có phải đang ở ngã tư không, dựa trên độ rộng vùng road tại 1 hàng cụ thể.

    :param mask: ảnh segmentation (H x W), mỗi pixel là class index
    :param row: hàng để kiểm tra (mặc định 240)
    :param threshold: ngưỡng chiều rộng vùng road để coi là ngã tư
    :param road_class: chỉ số class của road trong mask
    :return: True nếu đang ở ngã tư, False nếu không
    """
    if row >= mask.shape[0]:
        row = mask.shape[0] - 1

    line = mask[row, :]  # Lấy hàng row
    road_pixels = np.argwhere(line == road_class)

    if len(road_pixels) == 0:
        return False

    road_width = road_pixels[-1][0] - road_pixels[0][0]
    return road_width >= threshold

def get_steering_angle(image: np.ndarray, p=0.05, i=0.0000, d=0.01, debug: bool = False) -> int:
    """
    Dự đoán segmentation, tính trung điểm road, tính error và trả về góc lái.

    :param image: ảnh RGB (numpy array)
    :param p: hệ số tỉ lệ PID
    :param i: hệ số tích phân PID
    :param d: hệ số đạo hàm PID
    :param debug: nếu True thì trả về hình overlay để hiển thị
    :return: góc lái (-30 đến +30 độ), ảnh overlay nếu debug
    """
    image_resized = cv2.resize(image, IMAGE_SIZE)
    labels = predict(model, extractor, image_resized, DEVICE).cpu().numpy()

    car_side = detect_car_position_from_mask(labels, car_class=2, row_start=190)
    intersection = is_intersection(labels, row=240, threshold=600, road_class=1)
    print(f"[LOG] car side: {car_side},Intersection:  {intersection}")

    center_points = []
    left_points = []
    right_points = []

    
    row = 200  # dùng một hàng duy nhất
    if car_side == 'right':
        _, center, left, right = find_left_lane(labels, row)
    elif car_side == 'left':
        _, center, left, right = find_right_lane(labels, row)
    else:
        _, center, left, right = find_midpoint_segment(labels, row)

    # Tính toán các điểm x sau khi dịch offset
    x_center = center + 40
    x_left = np.clip(left[0] + 105, 0, labels.shape[1] - 1)
    x_right = np.clip(right[0] - 40, 0, labels.shape[1] - 1)

    # Lựa chọn x theo vị trí xe
    if car_side == 'right':
        x_coord = x_left
    elif car_side == 'left':
        x_coord = x_right
    else:
        x_coord = x_center

    error = x_coord - X_REF
    angle = PID(error, p = 0.1, i = 0.0001, d = 0.013)

    overlay = None
    if debug:
        seg_map = draw_segmentation_map(torch.tensor(labels), LABEL_COLORS_LIST)
        overlay = image_overlay(image_resized, seg_map)

        # Vẽ các điểm trên ảnh overlay
        cv2.circle(overlay, (x_center, row), 5, (0, 0, 255), -1)    # Trung tâm (đỏ)
        cv2.circle(overlay, (x_left, row), 5, (255, 0, 0), -1)      # Lề trái (xanh dương)
        cv2.circle(overlay, (x_right, row), 5, (0, 255, 255), -1)   # Lề phải (vàng)

    

    return round(angle), overlay, intersection 
