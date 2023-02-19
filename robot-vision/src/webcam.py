import cv2
import numpy as np
from face_detection import FaceDetector
import time

cam = cv2.VideoCapture(0)

DLIB_PREDICTOR_MODEL_PATH = "../trained_models/shape_predictor_5_face_landmarks.dat"
LABEL_ENCODER_PATH = "../trained_models/faces_label_encoder.pickle"
FACE_RECOGNITION_MODEL_PATH = "../trained_models/faces_svm_model.pickle"


def draw_face_bounding_box_with_details(frame, box, name=None):
    # draw the bounding box around the face
    (top, right, bottom, left) = box
    cv2.rectangle(frame, (left, top), (right, bottom),
                  (0, 255, 0), 2)
    y = top - 15 if top - 15 > 15 else top + 15

    if name is not None:
        # write the predicted face name on the image
        cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 255, 0), 2)
    return frame


if not cam.isOpened():
    raise RuntimeError('Could not start the cameras.')

faceDetector = FaceDetector(
    DLIB_PREDICTOR_MODEL_PATH, LABEL_ENCODER_PATH, FACE_RECOGNITION_MODEL_PATH, detection_method='cnn')

frameCnt = 0
faceRecognitionAverageTime = 0
while True:
    timer = cv2.getTickCount()
    # read current frame
    _, img = cam.read()
    frameCnt += 1

    faceRecognitionStartTime = time.time()
    boxes, names = faceDetector.recognize_faces(img)
    faceRecognitionElapsedTime = time.time() - faceRecognitionStartTime
    faceRecognitionAverageTime = (
        faceRecognitionAverageTime * (frameCnt - 1) + faceRecognitionElapsedTime) / frameCnt

    for box, name in zip(boxes, names):
        draw_face_bounding_box_with_details(img, box, name=name)

    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(img, "FPS : " + str(int(fps)),
                (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
    cv2.putText(img, "FRAME : " + str(frameCnt),
                (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

    print("fps=%d frameCnt=%d face_cur= %.4f face_avg= %.4f" % (
        fps, frameCnt, faceRecognitionElapsedTime, faceRecognitionAverageTime))

    cv2.imshow("Webcam", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
