import cv2
import numpy as np
import pyzed.sl as sl
import requests
import math
import os
import json
import logging
import collections
import time
import io
from base_camera import BaseCamera

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO)

FACE_RECOGNITION_SERVICE_URL = "http://localhost:5300/faces"
ROBOT_LOCATION_SERVICE_URL = "http://localhost:5200/position"


def draw_landmarks(frame, landmarks):
    for (x, y) in landmarks:
        cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
    return frame


def get_formatted_distance(point3d, distance):
    def format(x):
        return "%.1f" % (x / 1000)
    depthString = str(format(distance)) + ' (' + str(format(point3d[0])) + ',' + str(
        format(point3d[1])) + ',' + str(format(point3d[2])) + ')'
    return depthString


def draw_face_bounding_box_with_details(
        frame, box, point3d=None, distance=None, depthPoint=None, name=None):
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
        print(box)
        print(depthPoint)
        cv2.circle(frame, depthPoint, 10, (0, 0, 255), -1)
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


def getRobotPose():
    try:
        response = requests.get(ROBOT_LOCATION_SERVICE_URL)
        # Handle error if deserialization fails (because of no text or bad format)
        try:
            data = response.json()
            # check that .json() did NOT return an empty dict
            if data:
                try:
                    return data['robotPose']
                except (IndexError, KeyError, TypeError):
                    # data does not have the inner structure you expect
                    logging.error("data does not have the expected structure")
            else:
                logging.warning("response.json() returned an empty dict")

        except ValueError:
            # no JSON returned
            logging.error("no JSON received...")
    except requests.exceptions.ConnectionError:
        logging.error("Can't connect to " + ROBOT_LOCATION_SERVICE_URL)
    return None


def getFaceInformation(image_ocv):
    try:
        imageFile = io.BytesIO()
        np.save(imageFile, image_ocv)
        response = requests.post(
            FACE_RECOGNITION_SERVICE_URL, data=imageFile.getvalue())
        # Handle error if deserialization fails (because of no text or bad format)
        try:
            data = response.json()
            # check that .json() did NOT return an empty dict
            if data:
                try:
                    faceBBoxes = data['faceBBoxes']
                    names = data['names']
                    return faceBBoxes, names
                except (IndexError, KeyError, TypeError):
                    # data does not have the inner structure you expect
                    logging.error("data does not have the expected structure")
            else:
                logging.warning("response.json() returned an empty dict")

        except ValueError:
            # no JSON returned
            logging.error("no JSON received...")
    except requests.exceptions.ConnectionError:
        logging.error("Can't connect to " + FACE_RECOGNITION_SERVICE_URL)
    return None


# BFS is too slow, a fast method would be to sample some equally spaced 10 * 10 points from faceBBox, and check whether they are within face radius
def bfs(source, pointCloud, isWithinFace):
    dx = [-1, 0, 1, 0]
    dy = [0, 1, 0, -1]
    visited = set()
    visited.add(source)
    queue = collections.deque([source])
    while queue:
        cell = queue.popleft()
        err, point3d = pointCloud.get_value(cell[0], cell[1])
        if (err == sl.ERROR_CODE.SUCCESS):
            distance = math.sqrt(point3d[0] * point3d[0] +
                                 point3d[1] * point3d[1] +
                                 point3d[2] * point3d[2])
            if (not np.isnan(distance) and not np.isinf(distance)):
                return cell, point3d, distance
        for i in range(4):
            x = cell[0] + dx[i]
            y = cell[1] + dy[i]
            neighbor = (x, y)
            if isWithinFace(x, y) and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return None


def getFaceDepth(faceBBox, pointCloud):
    (top, right, bottom, left) = faceBBox
    centerX = (left + right) // 2
    centerY = (top + bottom) // 2
    radius = min((right - left) // 2, (top - bottom) // 2)

    def isWithinFace(x, y):
        if x >= 0 and y >= 0:
            dist = (x - centerX) * (x - centerX) + \
                (y - centerY) * (y - centerY)
            return dist < (radius * radius)
        return False

    if isWithinFace(centerX, centerY):
        return bfs((centerX, centerY), pointCloud, isWithinFace)
    return None


class Camera(BaseCamera):
    svoFile = None
    zedInitParamsFileName = None
    PERSON_LOCATION_SERVICE_URL = "http://localhost:5100/detections"
    zed = None

    @staticmethod
    def close():
        if Camera.zed is not None:
            print("Closing Zed Camera")
            Camera.zed.close()

    @staticmethod
    def frames():
        # Create a camera object
        Camera.zed = sl.Camera()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_QUALITY
        init_params.coordinate_units = sl.UNIT.UNIT_MILLIMETER
        init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
        init_params.sdk_verbose = False
        init_params.depth_stabilization = True
        init_params.save("initParameters")
        # Open the camera
        err = Camera.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Couldn't open the Zed Camera.")

        # Create and set RuntimeParameters after opening the camera
        runtime_parameters = sl.RuntimeParameters()

        # Use STANDARD sensing mode
        runtime_parameters.sensing_mode = sl.SENSING_MODE.SENSING_MODE_FILL

        Camera.zed.set_camera_settings(
            sl.CAMERA_SETTINGS.CAMERA_SETTINGS_BRIGHTNESS, 6)
        Camera.zed.set_camera_settings(
            sl.CAMERA_SETTINGS.CAMERA_SETTINGS_CONTRAST, 6)

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
            if Camera.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                currentRobotPose = getRobotPose()
                zedCaptureEnd = time.time()
                zedCaptureElapsedTime = zedCaptureEnd - zedCaptureStart
                frameCnt += 1
                zedCaptureAverageTime = (
                    zedCaptureAverageTime * (frameCnt - 1) + zedCaptureElapsedTime) / frameCnt

                # Retrieve left image
                Camera.zed.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT)
                # Retrieve depth map. Depth is aligned on the left image
                Camera.zed.retrieve_measure(
                    depth_image_zed, sl.MEASURE.MEASURE_DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                Camera.zed.retrieve_measure(
                    point_cloud, sl.MEASURE.MEASURE_XYZRGBA)

                image_ocv = load_image_into_numpy_array(image_zed)

                faceRecognitionStartTime = time.time()
                faceBBoxes = []
                names = []
                result = getFaceInformation(image_ocv)
                if result is not None:
                    faceBBoxes, names = result
                faceRecognitionElapsedTime = time.time() - faceRecognitionStartTime
                faceRecognitionAverageTime = (
                    faceRecognitionAverageTime * (frameCnt - 1) + faceRecognitionElapsedTime) / frameCnt

                detectedLocations = []
                detectedNames = []
                for faceBBox, name in zip(faceBBoxes, names):
                    ret = getFaceDepth(faceBBox, point_cloud)
                    if ret is None:
                        image_ocv = draw_face_bounding_box_with_details(
                            image_ocv, faceBBox, name=name)
                    else:
                        depthPoint, point3d, distance = ret
                        detectedLocations.append(point3d[0:3].tolist())
                        detectedNames.append(name)
                        image_ocv = draw_face_bounding_box_with_details(
                            image_ocv, faceBBox, point3d, distance, depthPoint, name=name)

                if len(detectedLocations) > 0:
                    data = {'locations': detectedLocations,
                            'robotPose': currentRobotPose,
                            'names': detectedNames}
                    try:
                        requests.post(
                            Camera.PERSON_LOCATION_SERVICE_URL, json=data)
                    except requests.exceptions.ConnectionError:
                        logging.error("Can't connect to " +
                                      Camera.PERSON_LOCATION_SERVICE_URL)

                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
                cv2.putText(image_ocv, "FPS : " + str(int(fps)),
                            (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
                cv2.putText(image_ocv, "FRAME : " + str(frameCnt),
                            (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

                logging.debug("fps=%d frameCnt=%d zed_cur=%.4f, zed_avg=%.4f face_cur= %.4f face_avg= %.4f" % (
                    fps, frameCnt, zedCaptureElapsedTime, zedCaptureAverageTime, faceRecognitionElapsedTime, faceRecognitionAverageTime))

                # # encode as a jpeg image and return it
                # image_ocv = resize(image_ocv, 50)
                yield cv2.imencode('.jpg', image_ocv)[1].tobytes()
