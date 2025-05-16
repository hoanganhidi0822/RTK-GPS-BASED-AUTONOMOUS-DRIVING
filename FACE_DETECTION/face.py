# face_detection_only.py

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time
class FaceDetection:
    def __init__(self, face_detection_model, score_threshold=0.9, nms_threshold=0.3, top_k=5000):
        self.detector = cv.FaceDetectorYN.create(
            face_detection_model,
            "",
            (320, 320),
            score_threshold,
            nms_threshold,
            top_k
        )

    def detect_faces(self, frame):
        # time.sleep(0.01)
        faces = self.detector.detect(frame)
        if faces[1] is not None:
            boxes = [face[:-1].astype(np.int32) for face in faces[1]]
            return boxes
        return []


# if __name__ == "__main__":
#     model_path = "/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/FACE_DETECTION/model/face_detection_yunet_2023mar.onnx"
#     detector = FaceDetection(model_path)

#     cap = cv.VideoCapture(0)
#     if not cap.isOpened():
#         print("Không mở được webcam")
#         exit()

#     plt.ion()  # Bật chế độ tương tác cho matplotlib

#     fig, ax = plt.subplots()

#     img_disp = None

#     consecutive_face_frames = 0  # Đếm số frame có mặt liên tiếp

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         h, w = frame.shape[:2]
#         detector.detector.setInputSize((w, h))

#         boxes = detector.detect_faces(frame)

#         if len(boxes) > 0:
#             consecutive_face_frames += 1
#         else:
#             consecutive_face_frames = 0

#         for box in boxes:
#             x, y, bw, bh = box[0], box[1], box[2], box[3]
#             cv.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

#         frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

#         if img_disp is None:
#             img_disp = ax.imshow(frame_rgb)
#         else:
#             img_disp.set_data(frame_rgb)

#         ax.set_title("Face Detection")
#         ax.axis("off")
#         plt.pause(0.01)

#         if consecutive_face_frames >= 500:
#             print("Phát hiện mặt liên tục trong 10 frame!")
#             cap.release()
#             plt.close()
#             exit(True)
