import cv2
import dlib
from imutils import face_utils
import face_recognition
import pickle


class FaceDetector:
    def __init__(self, dlib_predictor_model_path, detection_method='cnn'):
        self.face_detection_method = detection_method
        # self.facial_encodings = facial_encodings_path
        # self.data = pickle.loads(open(facial_encodings_path, "rb").read())
        self.facial_landmarks_predictor = dlib.shape_predictor(dlib_predictor_model_path)

    def detect_faces(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(
            rgb, model=self.face_detection_method)
        return boxes

    def get_landmarks(self, frame, box):
        (top, right, bottom, left) = box
        landmarks = self.facial_landmarks_predictor(
            frame, dlib.rectangle(left=left, top=top, right=right, bottom=bottom))
        landmarks = face_utils.shape_to_np(landmarks)
        return landmarks

    # def get_faces(self, frame):
    #     # convert the input frame from BGR to RGB
    #     rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    #     # detect the (x, y)-coordinates of the bounding boxes
    #     # corresponding to each face in the input frame, then compute
    #     # the facial embeddings for each face
    #     boxes = face_recognition.face_locations(rgb,
    #         model=self.face_detection_method)
    #     encodings = face_recognition.face_encodings(rgb, boxes)
    #     names = []

    #     # nearest neighbor algorithm
    #     # loop over the facial embeddings
    #     for encoding in encodings:
    #         # attempt to match each face in the input image to our known encodings
    #         matches = face_recognition.compare_faces(self.data["encodings"],
    #             encoding)
    #         name = "Unknown"

    #         # check to see if we have found a match
    #         if True in matches:
    #             # find the indexes of all matched faces then initialize a
    #             # dictionary to count the total number of times each face
    #             # was matched
    #             matchedIdxs = [i for (i, b) in enumerate(matches) if b]
    #             counts = {}

    #             for i in matchedIdxs:
    #                 name = self.data["names"][i]
    #                 counts[name] = counts.get(name, 0) + 1

    #             name = max(counts, key=counts.get)

    #         # update the list of names
    #         names.append(name)
    #     return boxes, names
