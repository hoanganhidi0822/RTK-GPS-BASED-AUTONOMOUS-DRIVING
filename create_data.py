import os
import json
import numpy as np
import cv2
from PIL import Image
import base64
import io
import shutil

# Danh sách lớp và màu tương ứng
ALL_CLASSES = ['background', 'road', 'car', 'person']
LABEL_COLORS_LIST = [
    (0, 0, 0),           # background - black
    (180, 120, 31),      # road      - RGB (31,120,180) -> BGR (180,120,31)
    (154, 61, 106),      # car       - RGB (106,61,154) -> BGR (154,61,106)
    (28, 26, 227),       # person    - RGB (227,26,28)  -> BGR (28,26,227)
]
COLOR_DICT = {label: color for label, color in zip(ALL_CLASSES, LABEL_COLORS_LIST)}

# Đường dẫn thư mục
input_dir = '/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/undistorted_images'  # chứa file .json và ảnh gốc
output_img_dir = 'Data/images'
output_mask_dir = 'Data/labels'

# Tạo thư mục đầu ra nếu chưa có
os.makedirs(output_img_dir, exist_ok=True)
os.makedirs(output_mask_dir, exist_ok=True)

# Duyệt qua tất cả các file JSON
for filename in os.listdir(input_dir):
    if not filename.endswith('.json'):
        continue

    json_path = os.path.join(input_dir, filename)
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Lấy tên ảnh gốc
    image_name = data['imagePath']
    image_path = os.path.join(input_dir, image_name)

    # Đọc ảnh gốc
    if os.path.exists(image_path):
        img = cv2.imread(image_path)
    else:
        # Nếu ảnh không tồn tại thì giải mã từ base64
        imageData = data['imageData']
        img_bytes = base64.b64decode(imageData)
        img = np.array(Image.open(io.BytesIO(img_bytes)))

    h, w = img.shape[:2]
    mask = np.zeros((h, w, 3), dtype=np.uint8)  # Ảnh màu 3 kênh

    # Vẽ từng shape với màu theo nhãn
    for shape in data['shapes']:
        label = shape['label']
        if label not in COLOR_DICT:
            print(f"⚠️ Label không xác định: {label} trong {filename}")
            continue
        color = COLOR_DICT[label]
        points = np.array(shape['points'], dtype=np.int32)
        cv2.fillPoly(mask, [points], color)

    # Lưu ảnh gốc vào Data/images
    dst_img_path = os.path.join(output_img_dir, os.path.splitext(filename)[0] + '.jpg')
    cv2.imwrite(dst_img_path, img)

    # Lưu mask màu vào Data/labels
    dst_mask_path = os.path.join(output_mask_dir, os.path.splitext(filename)[0] + '.png')
    cv2.imwrite(dst_mask_path, mask)

    print(f"✅ Processed: {filename}")
