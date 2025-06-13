import numpy as np
import cv2
import torch
import matplotlib.pyplot as plt
from transformers import SegformerFeatureExtractor, SegformerForSemanticSegmentation
from utils import predict, draw_segmentation_map, image_overlay
from config import VIS_LABEL_MAP as LABEL_COLORS_LIST

# --- Cấu hình ---
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
MODEL_PATH = "model_iou_v2"
IMAGE_PATH = "image_0384.jpg"  # <-- đường dẫn ảnh cần xử lý
IMAGE_SIZE = (640, 480)
ROAD_CLASS = 1
ROWS_TO_CHECK = [130,170, 190, 220, 270]


# --- Hàm hỗ trợ ---
def find_road_edges_and_center(mask, row, class_id=ROAD_CLASS):
    """Tìm biên trái, phải và trung điểm vùng road tại dòng row"""
    if row >= mask.shape[0]:
        row = mask.shape[0] - 1
    line = mask[row]
    road_pixels = np.where(line == class_id)[0]
    if len(road_pixels) == 0:
        return None, None, None  # không có đường tại dòng này
    x_left = int(road_pixels[0])
    x_right = int(road_pixels[-1])
    x_center = (x_left + x_right) // 2
    return x_left, x_right, x_center

# --- Load model ---
extractor = SegformerFeatureExtractor()
model = SegformerForSemanticSegmentation.from_pretrained(MODEL_PATH).to(DEVICE).eval()

# --- Load ảnh gốc ---
image_bgr = cv2.imread(IMAGE_PATH)
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
image_resized = cv2.resize(image_rgb, IMAGE_SIZE)

# --- Dự đoán segmentation ---
with torch.no_grad():
    labels = predict(model, extractor, image_resized, DEVICE).cpu().numpy()

# --- Tạo ảnh segmentation map và overlay ---
seg_map = draw_segmentation_map(torch.tensor(labels), LABEL_COLORS_LIST)
overlay = image_overlay(image_resized, seg_map)

# --- Lưu ảnh segmentation cơ bản ---
cv2.imwrite("seg_map.png", cv2.cvtColor(seg_map, cv2.COLOR_RGB2BGR))
cv2.imwrite("overlay.png", cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))

# --- Vẽ các điểm trên ảnh overlay ---
for row in ROWS_TO_CHECK:
    x_left, x_right, x_center = find_road_edges_and_center(labels, row)
    if None in (x_left, x_right, x_center):
        continue
    cv2.circle(overlay, (x_left, row), 6, (255, 0, 0), -1)      # trái (xanh dương)
    cv2.circle(overlay, (x_right, row), 6, (0, 255, 255), -1)    # phải (vàng)
    cv2.circle(overlay, (x_center, row), 6, (255,222, 33), -1)   # giữa (tím/hồng)

# --- Lưu ảnh có điểm ---
cv2.imwrite("overlay_with_edges_and_centers.png", cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))

# --- Hiển thị (tùy chọn) ---
plt.figure(figsize=(10, 6))
plt.imshow(overlay)
plt.title("")
plt.axis("off")
plt.show()