import argparse
import os
import time
import cv2
import torch
import matplotlib.pyplot as plt

from transformers import SegformerFeatureExtractor, SegformerForSemanticSegmentation
from Segformer.config import VIS_LABEL_MAP as LABEL_COLORS_LIST
from Segformer.utils import (
    draw_segmentation_map,
    image_overlay,
    predict
)

# --- Tham số dòng lệnh ---
parser = argparse.ArgumentParser()
parser.add_argument('--device', default='cuda:0', help='Device: cuda or cpu')
parser.add_argument('--imgsz', type=int, nargs='+', default=[640, 480], help='Image size (width height)')
parser.add_argument('--model_path', default='Segformer/model_iou', type=str, required=0, help='Path to fine-tuned SegFormer model directory')
args = parser.parse_args()

# --- Thiết lập mô hình ---
device = args.device
img_width, img_height = args.imgsz
extractor = SegformerFeatureExtractor()
model = SegformerForSemanticSegmentation.from_pretrained(args.model_path).to(device).eval()

# --- Mở camera ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")

plt.ion()  # bật chế độ interactive mode cho matplotlib
fig, ax = plt.subplots(figsize=(10, 6))

frame_count = 0
total_fps = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (img_width, img_height))
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Dự đoán phân đoạn
    start_time = time.time()
    labels = predict(model, extractor, rgb_frame, device)
    end_time = time.time()
    fps = 1 / (end_time - start_time)
    total_fps += fps
    frame_count += 1

    # Vẽ segmentation map
    seg_map = draw_segmentation_map(labels.cpu(), LABEL_COLORS_LIST)
    overlaid = image_overlay(rgb_frame, seg_map)
    overlaid = cv2.cvtColor(overlaid, cv2.COLOR_BGR2RGB)

    # Hiển thị bằng matplotlib
    ax.clear()
    ax.imshow(overlaid)
    ax.set_title(f"FPS: {fps:.2f}")
    ax.axis('off')
    plt.pause(0.001)

    if plt.get_fignums() == []:  # Nếu cửa sổ bị đóng
        break

# --- Kết thúc ---
cap.release()
plt.close()

# In FPS trung bình
avg_fps = total_fps / frame_count
print(f"Average FPS: {avg_fps:.2f}")
