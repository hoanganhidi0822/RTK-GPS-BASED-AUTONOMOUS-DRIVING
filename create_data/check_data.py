import os
import cv2
import numpy as np

image_dir = '/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/create_data/augmented/images'
label_dir = '/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/create_data/augmented/labels'
output_dir = '/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/create_data/augmented/overlays'
os.makedirs(output_dir, exist_ok=True)

alpha = 0.5  # độ trong suốt của lớp mask

for filename in os.listdir(image_dir):
    if not filename.endswith('.jpg'):
        continue

    # Tạo đường dẫn tương ứng
    image_path = os.path.join(image_dir, filename)
    label_path = os.path.join(label_dir, filename.replace('.jpg', '.png'))
    output_path = os.path.join(output_dir, filename)

    # Đọc ảnh
    img = cv2.imread(image_path)
    mask = cv2.imread(label_path)

    if img is None or mask is None:
        print(f"❌ Bỏ qua: {filename} (thiếu ảnh hoặc mask)")
        continue

    # Resize mask nếu cần (chỉ phòng trường hợp lệch kích thước)
    if img.shape[:2] != mask.shape[:2]:
        mask = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)

    # Tạo ảnh overlay
    overlay = cv2.addWeighted(img, 1 - alpha, mask, alpha, 0)

    # Lưu kết quả
    cv2.imwrite(output_path, overlay)
    print(f"✅ Overlay saved: {output_path}")
