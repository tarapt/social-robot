import cv2
import dlib
from imutils import face_utils
import face_recognition
import pickle

class FaceDetector:
    
    def __init__(self, facial_encodings_path, dlib_predictor_model_path, detection_method = 'cnn'):
        self.face_detection_method = detection_method
        self.facial_encodings = facial_encodings_path
        self.data = pickle.loads(open(facial_encodings_path, "rb").read())
        self.facial_landmarks_predictor = dlib.shape_predictor(dlib_predictor_model_path)

    def get_faces(self, frame):
        # convert the input frame from BGR to RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # detect the (x, y)-coordinates of the bounding boxes
        # corresponding to each face in the input frame, then compute
        # the facial embeddings for each face
        boxes = face_recognition.face_locations(rgb,
            model=self.face_detection_method)
        encodings = face_recognition.face_encodings(rgb, boxes)
        names = []

        # loop over the facial embeddings
        for encoding in encodings:
            # attempt to match each face in the input image to our known encodings
            matches = face_recognition.compare_faces(self.data["encodings"],
                encoding)
            name = "Unknown"

            # check to see if we have found a match
            if True in matches:
                # find the indexes of all matched faces then initialize a
                # dictionary to count the total number of times each face
                # was matched
                matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                counts = {}

                for i in matchedIdxs:
                    name = self.data["names"][i]
                    counts[name] = counts.get(name, 0) + 1

                name = max(counts, key=counts.get)
            
            # update the list of names
            names.append(name)
        return boxes, names

    def get_landmarks(self, frame, box):
        (top, right, bottom, left) = box
        landmarks = self.facial_landmarks_predictor(frame, dlib.rectangle(left=left, top=top, right=right, bottom=bottom))
        landmarks = face_utils.shape_to_np(landmarks)
        return landmarks

    def draw_face_details(self, frame, box, name, landmarks, worldCoordinates):
        (top, right, bottom, left) = box

        # loop over the (x, y)-coordinates for the facial landmarks
        # and draw them on the image
        for (x, y) in landmarks:
            cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)

        # draw the predicted face name on the image
        cv2.rectangle(frame, (left, top), (right, bottom),
            (0, 255, 0), 2)
        y = top - 15 if top - 15 > 15 else top + 15

        X, Y, Z = worldCoordinates
        float_formatter = lambda x: "%.1f" % x
        depthString = '(' + str(float_formatter(X)) + 'm,' + str(float_formatter(Y)) + 'm,' + str(float_formatter(Z)) + 'm)'
        cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
            0.75, (0, 255, 0), 2)
        cv2.putText(frame, depthString, (left, y - 25), cv2.FONT_HERSHEY_SIMPLEX,
            0.75, (0, 255, 0), 2)
        
        return frame