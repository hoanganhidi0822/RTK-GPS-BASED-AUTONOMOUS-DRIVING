import cv2 as cv
import numpy as np
import joblib
from sklearn.metrics.pairwise import cosine_similarity

class FaceRecognition:
    def __init__(self,
                 face_detection_model,
                 face_recognition_model,
                 svc_path,
                 mydict,
                 score_threshold=0.9,
                 nms_threshold=0.3,
                 top_k=5000):
        
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
        self.model = joblib.load(svc_path)  # Load SVM classifier
        
        # Load known features and labels for cosine similarity check
        self.svc = self.model['svc']
        self.known_features = self.model['features']  # shape (N, 128)
        self.known_labels = self.model['labels']      # shape (N,)
        
        self.mydict = mydict

    def detect_and_recognize(self, frame, cosine_threshold=0.5):
        h, w = frame.shape[:2]
        self.detector.setInputSize((w, h))
        faces = self.detector.detect(frame)
        results = []

        if faces[1] is not None:
            for face in faces[1]:
                try:
                    coords = face[:-1].astype(np.int32)
                    face_align = self.recognizer.alignCrop(frame, face)
                    face_feature = self.recognizer.feature(face_align).reshape(1, -1)

                    # SVM predict
                    test_predict = self.svc.predict(face_feature)
                    label = test_predict[0]

                    # Tính cosine similarity với known features cùng label
                    candidates = self.known_features[self.known_labels == label]
                    if len(candidates) > 0:
                        similarities = cosine_similarity(face_feature, candidates)
                        max_sim = np.max(similarities)
                    else:
                        max_sim = 0.0

                    if max_sim > cosine_threshold and 0 <= label < len(self.mydict):
                        recognized_name = self.mydict[label]
                    else:
                        recognized_name = " "

                    results.append((coords, recognized_name))
                except Exception as e:
                    print(f"[ERROR] Face recognition failed: {e}")
        return results
