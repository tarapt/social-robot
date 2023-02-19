import cv2
import os
import numpy as np
import pyzed.sl as sl
import requests
import json
import math
import time
import argparse
import signal
import sys
from face_detection import FaceDetector

DLIB_PREDICTOR_MODEL_PATH = "../trained_models/shape_predictor_5_face_landmarks.dat"
LABEL_ENCODER_PATH = "../trained_models/faces_label_encoder.pickle"
FACE_RECOGNITION_MODEL_PATH = "../trained_models/faces_svm_model.pickle"
global zed
zec = None


def draw_landmarks(frame, landmarks):
    for (x, y) in landmarks:
        cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
    return frame


def get_formatted_distance(point3d, distance):
    def float_formatter(x):
        return "%.1f" % x
    depthString = str(float_formatter(distance)) + ' (' + str(float_formatter(point3d[0])) + ',' + str(
        float_formatter(point3d[1])) + ',' + str(float_formatter(point3d[2])) + ')'
    return depthString


def draw_face_bounding_box_with_details(
        frame, box, point3d=None, distance=None, landmarks=None, name=None):
    # draw the bounding box around the face
    (top, right, bottom, left) = box
    cv2.rectangle(frame, (left, top), (right, bottom),
                  (0, 255, 0), 2)
    y = top - 15 if top - 15 > 15 else top + 15

    info = ""
    if name is not None:
        info += name + ": "

    if distance is not None:
        depthString = get_formatted_distance(point3d, distance)
        info += depthString

    cv2.putText(frame, info, (left, y - 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (0, 255, 0), 2)
    return frame


def merge(left_image, right_image):
    return np.concatenate((left_image, right_image), axis=1)


def resize(img, scale_percent):
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    return resized


def load_image_into_numpy_array(image):
    # the image_ocv has four channels, remove the last channel
    ar = image.get_data()
    ar = ar[:, :, 0:3]
    (im_height, im_width, channels) = image.get_data().shape
    return np.array(ar).reshape((im_height, im_width, 3)).astype(np.uint8)


def signal_handler(sig, frame):
    if zed is not None:
        zed.close()
        sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-i", "--client-ip", help="host ip of the person location service")
    ap.add_argument("-s", "--svo", help="name of the svo file to load")
    args = vars(ap.parse_args())

    # setting the class variables allows them to persist across all class objects created in future
    if args.get('svo'):
        Camera.svoFile = args['svo']
    if args.get('client_ip'):
        Camera.PERSON_LOCATION_SERVICE_HOST = args['client_ip']

    # Create a camera object
    global zed
    zed = sl.Camera()
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_QUALITY
    init_params.coordinate_units = sl.UNIT.UNIT_METER
    init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
    init_params.sdk_verbose = False
    init_params.depth_stabilization = True
    init_params.save("initParameters")
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Couldn't open the Zed Camera.")

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()

    # Use STANDARD sensing mode
    runtime_parameters.sensing_mode = sl.SENSING_MODE.SENSING_MODE_FILL

    zed.set_camera_settings(
        sl.CAMERA_SETTINGS.CAMERA_SETTINGS_BRIGHTNESS, 6)
    zed.set_camera_settings(
        sl.CAMERA_SETTINGS.CAMERA_SETTINGS_CONTRAST, 6)

    # Create a face detector instance
    faceDetector = FaceDetector(DLIB_PREDICTOR_MODEL_PATH, LABEL_ENCODER_PATH,
                                FACE_RECOGNITION_MODEL_PATH, detection_method='cnn')

    image_zed = sl.Mat()
    depth_image_zed = sl.Mat()
    point_cloud = sl.Mat()

    def float_formatter(x):
        return "%.1f" % x

    zedCaptureAverageTime = 0
    faceRecognitionAverageTime = 0
    frameCnt = 0
    while True:
        timer = cv2.getTickCount()
        zedCaptureStart = time.time()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zedCaptureEnd = time.time()
            zedCaptureElapsedTime = zedCaptureEnd - zedCaptureStart
            frameCnt += 1
            zedCaptureAverageTime = (
                zedCaptureAverageTime * (frameCnt - 1) + zedCaptureElapsedTime) / frameCnt

            # Retrieve left image
            zed.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(
                depth_image_zed, sl.MEASURE.MEASURE_DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(
                point_cloud, sl.MEASURE.MEASURE_XYZRGBA)

            image_ocv = load_image_into_numpy_array(image_zed)

            faceRecognitionStartTime = time.time()
            faceBBoxes, names = faceDetector.recognize_faces(image_ocv)
            faceRecognitionElapsedTime = time.time() - faceRecognitionStartTime
            faceRecognitionAverageTime = (
                faceRecognitionAverageTime * (frameCnt - 1) + faceRecognitionElapsedTime) / frameCnt

            detectedLocations = []
            detectedNames = []
            for faceBBox, name in zip(faceBBoxes, names):
                (top, right, bottom, left) = faceBBox

                x = round((left + right) / 2)
                y = round((top + bottom) / 2)
                err, point3d = point_cloud.get_value(x, y)

                # measure the distance using Euclidean distance
                distance = math.sqrt(point3d[0] * point3d[0] +
                                     point3d[1] * point3d[1] +
                                     point3d[2] * point3d[2])

                if not np.isnan(distance) and not np.isinf(distance):
                    detectedLocations.append(point3d.tolist())
                    detectedNames.append(name)
                    image_ocv = draw_face_bounding_box_with_details(
                        image_ocv, faceBBox, point3d, distance, name=name)
                else:
                    image_ocv = draw_face_bounding_box_with_details(
                        image_ocv, faceBBox, name=name)

            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            cv2.putText(image_ocv, "FPS : " + str(int(fps)),
                        (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
            cv2.putText(image_ocv, "FRAME : " + str(frameCnt),
                        (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

            print("fps=%d frameCnt=%d zed_cur=%.4f, zed_avg= %.4f face_cur= %.4f face_avg= %.4f" % (
                fps, frameCnt, zedCaptureElapsedTime, zedCaptureAverageTime, faceRecognitionElapsedTime, faceRecognitionAverageTime))

            cv2.imshow("Zed", image_ocv)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    zed.close()
    cv2.destroyAllWindows()
