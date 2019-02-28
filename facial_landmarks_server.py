# Do the videocapture and pose detection api call in another thread, since the api call may take time
# save the image, and the positions for at most 100 images, use a queue with fixed size
# Refer: https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/ 

import os
import cv2
import numpy as np
from flask import Flask, jsonify
from face_detection import FaceDetector

dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"
LEFT_IMAGE_PATH = "static_data/left.jpg"
RIGHT_IMAGE_PATH = "static_data/right.jpg"

static_file_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'static_data')

app = Flask(__name__)
faceDetector = FaceDetector(facial_encodings_path, dlib_predictor_model_path)

@app.route('/')
def status():
    return 'Facial Landmarks Detection Server Running...'

@app.route('/get_landmarks')
def get_landmarks():
    if os.path.isfile(LEFT_IMAGE_PATH) and os.path.isfile(RIGHT_IMAGE_PATH):
        landmarks = jsonify(calculate_pose(cv2.imread(LEFT_IMAGE_PATH), cv2.imread(RIGHT_IMAGE_PATH)))
        return landmarks
    return jsonify('')

def calculate_pose(left, right):
    left_boxes = faceDetector.detect_faces(left)
    right_boxes = faceDetector.detect_faces(right)

    # TODO use face recognition to establish correspondence between the left and right boxes
    
    left_landmarks = []
    for box in left_boxes:
        left_landmarks.append(faceDetector.get_landmarks(left, box))

    right_landmarks = []
    for box in right_boxes:
        right_landmarks.append(faceDetector.get_landmarks(right, box))
    
    x = np.array(left_landmarks)
    x.reshape([x.shape[0] * x.shape[1], 2])
    y = np.array(right_landmarks)
    y.reshape([y.shape[0] * y.shape[1], 2])

    return {'left': x.tolist(), 'right': y.tolist()}