import cv2 as cv
import numpy as np
import joblib
import pickle


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a_norm = a / (np.linalg.norm(a, axis=1, keepdims=True) + 1e-9)
    b_norm = b / (np.linalg.norm(b, axis=1, keepdims=True) + 1e-9)
    return a_norm @ b_norm.T


def _load_model_no_sklearn(path: str):
    class _Dummy:
        pass

    class _Unpickler(pickle.Unpickler):
        def find_class(self, module, name):
            if module.startswith("sklearn"):
                return _Dummy
            return super().find_class(module, name)

    with open(path, "rb") as f:
        return _Unpickler(f).load()

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
        try:
            self.model = joblib.load(svc_path)  # Load SVM classifier
        except ModuleNotFoundError as exc:
            if "sklearn" in str(exc):
                print(f"[WARN] sklearn not available, using fallback loader: {exc}")
                self.model = _load_model_no_sklearn(svc_path)
            else:
                raise
        
        # Load known features and labels for cosine similarity check
        self.svc = self.model.get('svc')
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

                    if self.svc is not None and hasattr(self.svc, "predict"):
                        # SVM predict
                        test_predict = self.svc.predict(face_feature)
                        label = test_predict[0]

                        # Tính cosine similarity với known features cùng label
                        candidates = self.known_features[self.known_labels == label]
                        if len(candidates) > 0:
                            similarities = cosine_similarity(face_feature, candidates)
                            max_sim = float(np.max(similarities))
                        else:
                            max_sim = 0.0
                    else:
                        # Fallback: nearest neighbor by cosine similarity
                        sims = cosine_similarity(face_feature, self.known_features).reshape(-1)
                        if sims.size > 0:
                            best_idx = int(np.argmax(sims))
                            max_sim = float(sims[best_idx])
                            label = int(self.known_labels[best_idx])
                        else:
                            max_sim = 0.0
                            label = -1

                    if max_sim > cosine_threshold and 0 <= label < len(self.mydict):
                        recognized_name = self.mydict[label]
                    else:
                        recognized_name = " "

                    results.append((coords, recognized_name))
                except Exception as e:
                    print(f"[ERROR] Face recognition failed: {e}")
        return results
