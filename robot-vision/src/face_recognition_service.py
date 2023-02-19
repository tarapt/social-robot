from flask import Flask, jsonify, request
import numpy as np
import os.path
import logging
import io
from face_detection import FaceDetector

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO)

DLIB_PREDICTOR_MODEL_PATH = "../trained_models/shape_predictor_5_face_landmarks.dat"
LABEL_ENCODER_PATH = "../trained_models/faces_label_encoder.pickle"
FACE_RECOGNITION_MODEL_PATH = "../trained_models/faces_svm_model.pickle"
IMAGE_FILE_NAME = "../data/current_frame.npy"

faceDetector = FaceDetector(DLIB_PREDICTOR_MODEL_PATH, LABEL_ENCODER_PATH,
                            FACE_RECOGNITION_MODEL_PATH, detection_method='cnn')

app = Flask(__name__)


@app.route('/')
def status():
    return 'Face Recognition Service Running...'


@app.route('/faces', methods=['POST'])
def recognitions():
    data = request.data
    img = np.load(io.BytesIO(data))
    if img is not None:
        faceBBoxes, names = faceDetector.recognize_faces(img)
        data = {'faceBBoxes': faceBBoxes, 'names': names}
        return jsonify(data)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5300)
