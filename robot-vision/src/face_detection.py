import cv2
import dlib
from imutils import face_utils
import face_recognition
import pickle
import numpy as np


def resize(img, scale_percent):
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    return resized


class FaceDetector:
    def __init__(self, dlib_predictor_model_path, label_encoder_path,
                 recognition_model_path, recognition_threshold=0.0, detection_method='cnn'):
        self.face_detection_method = detection_method
        self.recognition_threshold = recognition_threshold
        self.facial_landmarks_predictor = dlib.shape_predictor(
            dlib_predictor_model_path)
        self.face_recognizer = pickle.loads(
            open(recognition_model_path, "rb").read())
        self.label_encoder = pickle.loads(
            open(label_encoder_path, "rb").read())

    def detect_faces(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(
            rgb, model=self.face_detection_method)
        return boxes, ["Tara"]

    def get_landmarks(self, frame, box):
        (top, right, bottom, left) = box
        landmarks = self.facial_landmarks_predictor(
            frame, dlib.rectangle(left=left, top=top, right=right, bottom=bottom))
        landmarks = face_utils.shape_to_np(landmarks)
        return landmarks

    def recognize_faces(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # detect the (x, y)-coordinates of the bounding boxes
        # corresponding to each face in the input frame, then compute
        # the facial embeddings for each face
        boxes = face_recognition.face_locations(rgb,
                                                model=self.face_detection_method)

        names = []
        unknown_cnt = 0
        if (len(boxes) > 0):
            encodings = face_recognition.face_encodings(rgb, boxes)

            # loop over the facial embeddings
            for encoding in encodings:
                # attempt to match each face in the input image to our known encodings
                # perform classification to recognize the face

                # ValueError: Expected 2D array, got 1D array instead:
                # Reshape your data either using array.reshape(-1, 1) if your data has a single feature
                # or array.reshape(1, -1) if it contains a single sample.
                encoding = encoding.reshape(1, -1)

                preds = self.face_recognizer.predict_proba(encoding)[0]
                j = np.argmax(preds)
                proba = preds[j]
                name = self.label_encoder.classes_[j]

                if (proba < self.recognition_threshold):
                    name = "unknown_" + str(unknown_cnt)
                    unknown_cnt = unknown_cnt + 1

                # update the list of names
                names.append(name)
        return boxes, names
