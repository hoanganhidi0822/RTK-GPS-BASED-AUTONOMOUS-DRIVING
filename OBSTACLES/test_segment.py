from road_segmentation import get_steering_angle  # giả sử bạn lưu ở steering_module.py
import cv2

cap = cv2.VideoCapture(2)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    angle = get_steering_angle(rgb,debug=1)
    print(f"Góc lái: {angle}°")
