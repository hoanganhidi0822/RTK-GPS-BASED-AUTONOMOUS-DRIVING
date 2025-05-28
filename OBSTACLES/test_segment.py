from Segformer.road_segmentation import get_steering_angle
import cv2
import matplotlib.pyplot as plt

cap = cv2.VideoCapture("/mnt/NewVolume/Documents/Researches/2024_Project/Depth Map-Based Obstacle Position Detection/DATA_XE/video2.mp4")

plt.ion()  # Bật chế độ interactive cho matplotlib
fig, ax = plt.subplots(figsize=(10, 6))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    angle, seg_map, is_intersection = get_steering_angle(rgb, debug=1)
    print(f"Góc lái: {angle}°, Intersection: {is_intersection}")

    # Hiển thị bằng matplotlib
    ax.clear()
    ax.imshow(seg_map)
    ax.set_title(f"Góc lái: {angle}°")
    ax.axis("off")
    plt.pause(0.1)

    if plt.get_fignums() == []:  # Nếu người dùng đóng cửa sổ
        break

cap.release()
plt.close()
