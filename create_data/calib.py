import cv2
import numpy as np
import glob
import os

# Đọc thông số camera
camera_matrix = np.loadtxt('/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/OBSTACLES/camera_param/camera_matrix.txt', dtype=np.float32)
dist_coeffs = np.loadtxt('/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/OBSTACLES/camera_param/distortion_coefficients.txt', dtype=np.float32)

# Đường dẫn thư mục ảnh gốc và ảnh đã hiệu chỉnh
input_folder = 'captured_images/'
output_folder = 'undistorted/'

# Tạo thư mục lưu ảnh nếu chưa tồn tại
os.makedirs(output_folder, exist_ok=True)

# Danh sách đường dẫn ảnh
image_paths = glob.glob(os.path.join(input_folder, '*.*'))  # đọc tất cả định dạng

for img_path in image_paths:
    img = cv2.imread(img_path)
    if img is None:
        print(f"Lỗi đọc ảnh: {img_path}")
        continue

    # Lấy kích thước ảnh
    h, w = img.shape[:2]
    image_size = (w, h)

    # Tính toán bản đồ hiệu chỉnh cho mỗi ảnh
    map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, image_size, cv2.CV_16SC2)

    # Hiệu chỉnh ảnh
    undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Tạo tên file mới và lưu ảnh
    filename = os.path.basename(img_path)
    output_path = os.path.join(output_folder, filename)
    cv2.imwrite(output_path, undistorted)
    print(f"Đã lưu: {output_path}")
