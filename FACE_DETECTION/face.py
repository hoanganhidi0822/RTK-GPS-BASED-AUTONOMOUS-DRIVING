# face_recognition.py

import cv2 as cv
import numpy as np
import joblib

class FaceRecognition:
    def __init__(self, face_detection_model, face_recognition_model, score_threshold=0.9, nms_threshold=0.3, top_k=5000):
        self.detector = cv.FaceDetectorYN.create(
            face_detection_model,
            "",
            (320, 320),
            score_threshold,
            nms_threshold,
            top_k
        )
        self.recognizer = cv.FaceRecognizerSF.create(
            face_recognition_model, ""
        )
        self.svc = joblib.load('/home/hoang-anh/Downloads/faceTracking/model/svc.pkl')  # Load your SVM classifier
        self.mydict = ['Hoang Anh', 'Quoc Kha']

    def detect_and_recognize(self, frame):
        h, w = frame.shape[:2]
        self.detector.setInputSize((w, h))  # <<< DÒNG CẦN THÊM VÀO

        faces = self.detector.detect(frame)
        results = []

        if faces[1] is not None:
            for face in faces[1]:
                coords = face[:-1].astype(np.int32)
                face_align = self.recognizer.alignCrop(frame, face)
                face_feature = self.recognizer.feature(face_align).reshape(1, -1)
                test_predict = self.svc.predict(face_feature)
                recognized_name = self.mydict[test_predict[0]]
                results.append((coords, recognized_name))

        return results
