import cv2
import numpy as np
import requests
from face_detection import FaceDetector
import matlab.engine
from base_camera import BaseCamera

PERSON_LOCATION_SERVICE_URL = "http://localhost:5100/detections"
LEFT_IMAGE_PATH = "static_data/left.png"
RIGHT_IMAGE_PATH = "static_data/right.png"
RECTIFIED_LEFT_IMAGE_PATH = "static_data/rectified_left.png"
RECTIFIED_RIGHT_IMAGE_PATH = "static_data/rectified_right.png"
ANNOTATED_LEFT_IMAGE_PATH = "static_data/annotated_left.png"
ANNOTATED_RIGHT_IMAGE_PATH = "static_data/annotated_right.png"
DLIB_PREDICTOR_MODEL_PATH = "../../trained_models/shape_predictor_5_face_landmarks.dat"

# max possible for the logitech cameras
MAX_CAMERA_WIDTH = 1280
MAX_CAMERA_HEIGHT = 960


def set_resolution(cam_1, cam_2, w, h):
    cam_1.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cam_1.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cam_2.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cam_2.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    print("CAMERA_WIDTH = " +
          str(int(self.stereoCamera.left.get(cv2.CAP_PROP_FRAME_WIDTH))))
    print("CAMERA_HEIGHT = " +
          str(int(self.stereoCamera.left.get(cv2.CAP_PROP_FRAME_HEIGHT))))


def draw_landmarks(frame, landmarks):
    for (x, y) in landmarks:
        cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
    return frame


def get_formatted_distance(point3d):
    def float_formatter(x):
        return "%.1f" % x
    depthString = '(' + str(float_formatter(point3d[0])) + 'm,' + str(
        float_formatter(point3d[1])) + 'm,' + str(float_formatter(point3d[2])) + 'm)'
    return depthString


def draw_face_bounding_box_with_details(
        frame, box, landmarks, point3d, name="unknown"):
    # draw the bounding box around the face
    (top, right, bottom, left) = box
    cv2.rectangle(frame, (left, top), (right, bottom),
                  (0, 255, 0), 2)
    y = top - 15 if top - 15 > 15 else top + 15
    # write the predicted face name on the image
    cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (0, 255, 0), 2)

    depthString = get_formatted_distance(point3d)
    # write the world coordinates of the face
    cv2.putText(frame, depthString, (left, y - 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (0, 255, 0), 2)

    draw_landmarks(frame, landmarks)
    return frame


def merge(left_image, right_image):
    return np.concatenate((left_image, right_image), axis=1)


class Camera(BaseCamera):
    video_source_1 = 1
    video_source_2 = 2
    faceDetector = FaceDetector(DLIB_PREDICTOR_MODEL_PATH)
    matlab_engine = matlab.engine.start_matlab()

    @staticmethod
    def set_video_sources(source_1, source_2):
        Camera.video_source_1 = source_1
        Camera.video_source_2 = source_2

    @staticmethod
    def frames():
        camera_1 = cv2.VideoCapture(Camera.video_source_1)
        camera_2 = cv2.VideoCapture(Camera.video_source_2)
        if not camera_1.isOpened() and camera_2.isOpened():
            raise RuntimeError('Could not start the cameras.')

        while True:
            # read current frame
            _, img_1 = camera_1.read()
            _, img_2 = camera_2.read()

            # Stereo Rectification
            cv2.imwrite(LEFT_IMAGE_PATH, img_1)
            cv2.imwrite(RIGHT_IMAGE_PATH, img_2)
            Camera.matlab_engine.stereo_rectify(nargout=0)
            img_1 = cv2.imread(RECTIFIED_LEFT_IMAGE_PATH)
            img_2 = cv2.imread(RECTIFIED_RIGHT_IMAGE_PATH)

            # Get Facial Landmarks
            leftFaces = Camera.faceDetector.detect_faces(img_1)
            rightFaces = Camera.faceDetector.detect_faces(img_2)

            # in case of multiple faces, use face recognition to establish
            # correspondence between the faces
            if len(leftFaces) > 0 and len(rightFaces) > 0:
                leftFace = leftFaces[0]
                rightFace = rightFaces[0]
                leftLandmarks = Camera.faceDetector.get_landmarks(
                    img_1, leftFace)
                rightLandmarks = Camera.faceDetector.get_landmarks(
                    img_2, rightFace)

                leftLandmarksArray = matlab.uint16(leftLandmarks.tolist())
                rightLandmarksArray = matlab.uint16(rightLandmarks.tolist())
                point3d = Camera.matlab_engine.triangulate_points(leftLandmarksArray, rightLandmarksArray, nargout=1)

                print(point3d)
                data = {"x": point3d[0][0], "z": point3d[0][2], 'th': 0}
                requests.post(PERSON_LOCATION_SERVICE_URL, data=data)

                img_1 = draw_face_bounding_box_with_details(
                    img_1, leftFace, leftLandmarks, point3d[0])
                img_2 = draw_face_bounding_box_with_details(
                    img_2, rightFace, rightLandmarks, point3d[0])

            img = merge(img_1, img_2)

            # encode as a jpeg image and return it
            yield cv2.imencode('.jpg', img)[1].tobytes()
