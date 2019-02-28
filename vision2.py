import argparse
import time
import cv2
import numpy as np
import os
from PIL import Image
import matlab.engine
from face_detection import FaceDetector

CAMERA_WIDTH = 1280  # MAX_CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960  # MAX_CAMERA_HEIGHT = 960

# Directories to store frames for each camera
LEFT_PATH = "experiments/{:04d}/{:06d}_left.jpg"
RIGHT_PATH = "experiments/{:04d}/{:06d}_right.jpg"

LEFT_IMAGE_PATH = "static_data/left.png"
RIGHT_IMAGE_PATH = "static_data/right.png"
RECTIFIED_LEFT_IMAGE_PATH = "static_data/rectified_left.png"
RECTIFIED_RIGHT_IMAGE_PATH = "static_data/rectified_right.png"
ANNOTATED_LEFT_IMAGE_PATH = "static_data/annotated_left.png"
ANNOTATED_RIGHT_IMAGE_PATH = "static_data/annotated_right.png"
DLIB_PREDICTOR_MODEL_PATH = "./trained_models/shape_predictor_5_face_landmarks.dat"

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
        self.stereoCamera = StereoCamera(id1, id2)
        if increaseResolution == True:
            self.increase_resolution()

    def increase_resolution(self):
        self.stereoCamera.left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.stereoCamera.left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.stereoCamera.right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.stereoCamera.right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        print "CAMERA_WIDTH = " + str(int(self.stereoCamera.left.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print "CAMERA_HEIGHT = " + str(int(self.stereoCamera.left.get(cv2.CAP_PROP_FRAME_HEIGHT)))

def get_experiment_count():
    f = open("experiments/experiment_count.txt", "r")
    return int(f.read())

def increment_experiment_count(experiment_count):
    f = open("experiments/experiment_count.txt", "w")
    f.write(str(experiment_count + 1))

def draw_landmarks(frame, landmarks):
    for (x, y) in landmarks:
        cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
    return frame

def get_formatted_distance(point3d):
    float_formatter = lambda x: "%.1f" % x
    depthString = '(' + str(float_formatter(point3d[0])) + 'm,' + str(float_formatter(point3d[1])) + 'm,' + str(float_formatter(point3d[2])) + 'm)'
    return depthString

def draw_face_bounding_box_with_details(frame, box, landmarks, point3d, name="unknown"):
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

if __name__ == '__main__':
    increaseResolution = False
    matlab_engine = matlab.engine.start_matlab()
    faceDetector = FaceDetector(DLIB_PREDICTOR_MODEL_PATH)

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
            timer = cv2.getTickCount()

            # Stereo Rectification 
            cv2.imwrite(LEFT_IMAGE_PATH, stereoFrame.left)
            cv2.imwrite(RIGHT_IMAGE_PATH, stereoFrame.right)
            matlab_engine.stereo_rectify(nargout=0)
            stereoFrame = StereoFrame(cv2.imread(RECTIFIED_LEFT_IMAGE_PATH), cv2.imread(RECTIFIED_RIGHT_IMAGE_PATH))

            # Get Facial Landmarks
            leftFaces = faceDetector.detect_faces(stereoFrame.left)
            rightFaces = faceDetector.detect_faces(stereoFrame.right)

            # in case of multiple faces, use face recognition to establish correspondence between the faces

            if len(leftFaces) > 0 and len(rightFaces) > 0:
                leftFace = leftFaces[0] 
                rightFace = rightFaces[0]
                leftLandmarks = faceDetector.get_landmarks(stereoFrame.left, leftFace)
                rightLandmarks = faceDetector.get_landmarks(stereoFrame.right, rightFace)

                leftLandmarksArray = matlab.uint16(leftLandmarks.tolist())
                rightLandmarksArray = matlab.uint16(rightLandmarks.tolist())
                point3d = matlab_engine.triangulate_points(leftLandmarksArray, rightLandmarksArray, nargout=1)

                print(point3d)

                draw_face_bounding_box_with_details(stereoFrame.left, leftFace, leftLandmarks, point3d[0])
                draw_face_bounding_box_with_details(stereoFrame.right, rightFace, rightLandmarks, point3d[0])

            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

            # Display FPS on frame
            cv2.putText(stereoFrame.left, "FPS : " + str(int(fps)),
                        (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
            cv2.putText(stereoFrame.right, "FPS : " + str(int(fps)),
                        (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

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

##############################################################################################
# def cv_to_pil(img):
#     print(np.shape(img))
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     print(type(img))
#     print(np.shape(img))
#     im_pil = Image.fromarray(img)
#     return im_pil

# def pil_to_matlab(image, matlab_engine):
#     print(type(image))
#     print(type(image.getdata()))
#     print(type(list(image.getdata())))
#     print(image.size)
#     image_mat = matlab_engine.uint8(np.asarray(image.getdata()).tolist())
#     image_mat.reshape((image.size[0], image.size[1], 3))
#     print(image_mat.size)
#     return image_mat

# def cv_to_matlab(img):
#     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#     img = matlab.uint8(img)
#     print(img.size)
#     img.reshape(np.shape(img))
#     print(img.size)
#     return img


# def stereo_rectify(stereoFrame, matlab_engine):
#     matlab_engine.workspace["I1"] = cv_to_matlab(stereoFrame.left)
#     matlab_engine.workspace["I2"] = cv_to_matlab(stereoFrame.right)
#     # Undistort the images.
#     I1 = matlab_engine.eval("undistortImage(I1, stereoParams.CameraParameters1);")
#     I2 = matlab_engine.eval("undistortImage(I2, stereoParams.CameraParameters2);")
#     # Stereo rectify the images.
#     I1, I2 = matlab_engine.eval("rectifyStereoImages(I1, I2, stereoParams);")
#     return I1, I2