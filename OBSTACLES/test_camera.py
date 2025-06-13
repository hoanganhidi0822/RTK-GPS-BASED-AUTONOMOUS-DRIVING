
import cv2
import matplotlib.pyplot as plt
import numpy as np
cap = cv2.VideoCapture(0)

plt.ion()  # Bật chế độ interactive cho matplotlib
fig, ax = plt.subplots(figsize=(10, 6))

camera_matrix = np.loadtxt('OBSTACLES/camera_param/camera_matrix.txt',dtype=np.float32)
dist_coeffs = np.loadtxt('OBSTACLES/camera_param/distortion_coefficients.txt',dtype=np.float32)
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (640, 480), cv2.CV_16SC2)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    raw_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
    
    

    # Hiển thị bằng matplotlib
    ax.clear()
    ax.imshow(rgb)
    
    ax.axis("off")
    plt.pause(0.01)

    if plt.get_fignums() == []:  # Nếu người dùng đóng cửa sổ
        break

cap.release()
plt.close()
