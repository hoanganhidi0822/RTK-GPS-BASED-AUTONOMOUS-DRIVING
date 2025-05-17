import cv2
import numpy as np
import matplotlib.pyplot as plt

from face import FaceRecognition

# Initialize FaceRecognition with models
face_rec = FaceRecognition(
    face_detection_model='/home/hoang-anh/Downloads/faceTracking/model/face_detection_yunet_2023mar.onnx',
    face_recognition_model='/home/hoang-anh/Downloads/faceTracking/model/face_recognition_sface_2021dec.onnx'
)

# Mở camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Không thể mở camera")
    exit()

# Kích hoạt chế độ interactive để cập nhật frame liên tục
plt.ion()
fig, ax = plt.subplots()

im = ax.imshow(np.zeros((320, 320, 3), dtype=np.uint8))  # tạo ảnh trống ban đầu
plt.axis("off")  # Ẩn trục

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không thể đọc frame từ camera")
        break

    frame = cv2.resize(frame, (320, 320))

    # Detect and recognize face
    coords, name = face_rec.detect_and_recognize(frame)

    if coords is not None:
        x, y, w, h = coords[0], coords[1], coords[2], coords[3]
        center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2

        cv2.circle(frame, (x + w // 2, y + h // 2), 2, (0, 0, 255), thickness=2)
        cv2.putText(frame, name, (x, y - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)

    # Chuyển frame từ BGR sang RGB để hiển thị bằng matplotlib
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Cập nhật ảnh trong figure
    im.set_data(frame_rgb)
    fig.canvas.flush_events()
    plt.pause(0.001)  # nhỏ hơn 1ms để làm mượt

    # Để thoát vòng lặp bằng cách đóng cửa sổ matplotlib
    if not plt.fignum_exists(fig.number):
        break

# Dọn dẹp
cap.release()
plt.ioff()
plt.close()
