import argparse
import time
import cv2
import numpy as np
from head_pose_estimation import HeadPoseEstimator
from depth_estimation import DepthEstimator
from face_detection import FaceDetector
import face_detection
import os
import requests

CAMERA_WIDTH = 1280  # MAX_CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960  # MAX_CAMERA_HEIGHT = 960

CONFIDENCE_THRESHOLD = 0

# Directories to store frames for each camera
LEFT_PATH = "experiments/{:04d}/{:06d}_left.jpg"
RIGHT_PATH = "experiments/{:04d}/{:06d}_right.jpg"

dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"

POSE_DETECTION_PATH = "static_data/current.jpg"
POSE_DETECTION_URL = "http://localhost:5000/get_pose"

class Person:
    def __init__(self, faceLandmarks, body, leftHand=None, rightHand=None, name="unknown"):
        self.name = name
        self.faceLandmarks = faceLandmarks
        self.body = body
        self.leftHand = leftHand
        self.rightHand = rightHand


class StereoPerson:
    def __init__(self, leftPerson, rightPerson, name="unknown"):
        self.name = name
        self.left = leftPerson
        self.right = rightPerson


class StereoFrame:
    def __init__(self, leftFrame, rightFrame):
        self.left = leftFrame
        self.right = rightFrame


class StereoCamera:
    def __init__(self, leftCameraId, rightCameraId):
        self.left = cv2.VideoCapture(leftCameraId)
        self.right = cv2.VideoCapture(rightCameraId)

    def isOpened(self):
        return ((self.left is not None and self.left.isOpened()) and
                (self.right is not None and self.right.isOpened()))

    def retrieve(self):
        _, leftFrame = self.left.retrieve()
        _, rightFrame = self.right.retrieve()
        return StereoFrame(leftFrame, rightFrame)

    def hasFrames(self):
        return self.left.grab() and self.right.grab()

    def release(self):
        self.left.release()
        self.right.release()


class CameraError(Exception):
    pass


class Vision:
    def __init__(self, increaseResolution, id1=-1, id2=-1):
        self.setup_cameras(id1, id2)
        if increaseResolution == True:
            self.increase_resolution()

    def draw_landmarks(self, frame, landmarks):
        # loop over the (x, y)-coordinates for the facial landmarks
        # and draw them on the image
        for (x, y) in landmarks:
            cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
        return frame

    def setup_cameras(self, id1=-1, id2=-1, max_trials=10):
        if id1 == -1 or id2 == -1:
            cnt = 1
            stereoCamera = StereoCamera(0, 1)
            while(cnt < max_trials and not stereoCamera.isOpened()):
                print("Attempt #%d to open cameras..." % cnt)
                stereoCamera = StereoCamera(0, 1)
                if not stereoCamera.isOpened():
                    stereoCamera = StereoCamera(1, 2)
                if not stereoCamera.isOpened():
                    stereoCamera = StereoCamera(0, 2)
                cnt += 1
            self.stereoCamera = stereoCamera
        else:
            self.stereoCamera = StereoCamera(id1, id2)

        if not self.stereoCamera.isOpened():
            raise CameraError

    def increase_resolution(self):
        self.stereoCamera.left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.stereoCamera.left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.stereoCamera.right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.stereoCamera.right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        print "CAMERA_WIDTH = " + str(int(self.stereoCamera.left.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print "CAMERA_HEIGHT = " + str(int(self.stereoCamera.left.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def rectify_uncalibrated(self, stereoFrame):
        left = stereoFrame.left
        right = stereoFrame.right
        faceDetector = FaceDetector(facial_encodings_path, dlib_predictor_model_path)
        left_boxes = faceDetector.detect_faces(left)
        right_boxes = faceDetector.detect_faces(right)

        if(len(left_boxes) == 0):
            return stereoFrame

        left_points = faceDetector.get_landmarks(left, left_boxes[0])
        right_points = faceDetector.get_landmarks(right, right_boxes[0])

        # print(left_boxes, right_boxes)
        # print(left_points, right_points)

        #Computation of the fundamental matrix
        F, mask= cv2.findFundamentalMat(left_points, right_points)

        height, width = left.shape[:2]
        retval, H1, H2 = cv2.stereoRectifyUncalibrated(left_points, right_points, F, (width, height))

        left_new = cv2.warpPerspective(left, H1, (height, width))
        right_new = cv2.warpPerspective(right, H2, (height, width))
        return StereoFrame(left_new, right_new)

def get_experiment_count():
    f = open("experiments/experiment_count.txt", "r")
    return int(f.read())


def increment_experiment_count(experiment_count):
    f = open("experiments/experiment_count.txt", "w")
    f.write(str(experiment_count + 1))

if __name__ == '__main__':
    detection_method = 'cnn'
    increaseResolution = False

    experiment_count = get_experiment_count()
    format_string = "{:04d}"
    if not os.path.exists('experiments/' + format_string.format(experiment_count)):
        os.mkdir('experiments/' + format_string.format(experiment_count))

    try:
        robot_vision = Vision(increaseResolution, 0, 1)
    except CameraError:
        print("[ERROR] Cameras couldn't start. Exiting...")
    else:
        totalFrames = 0
        # time.sleep(2.0)

        print("[INFO] Capturing frames...")

        while True:
            if not robot_vision.stereoCamera.hasFrames():
                continue
            stereoFrame = robot_vision.stereoCamera.retrieve()
            print("[INFO] Captured frame %d..." % totalFrames)

            # Start timer
            # timer = cv2.getTickCount()

            # rectifiedFrame = robot_vision.rectify_uncalibrated(stereoFrame)

            # Calculate Frames per second (FPS)
            # fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

            # Display FPS on frame
            # cv2.putText(stereoFrame.left, "FPS : " + str(int(fps)),
            #             (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
            # cv2.putText(stereoFrame.right, "FPS : " + str(int(fps)),
            #             (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            cv2.imshow("Left Camera", stereoFrame.left)
            cv2.imshow("Right Camera", stereoFrame.right)

            # Save the frames
            cv2.imwrite(LEFT_PATH.format(experiment_count, totalFrames), stereoFrame.left)
            cv2.imwrite(RIGHT_PATH.format(experiment_count, totalFrames), stereoFrame.right)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            totalFrames += 1

        # do a bit of cleanup
        robot_vision.stereoCamera.release()
        cv2.destroyAllWindows()
        increment_experiment_count(experiment_count)
