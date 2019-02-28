from face_detection import FaceDetector
import cv2
import numpy as np

left = cv2.imread('left.jpg')
right = cv2.imread('right.jpg')

cv2.imshow('left', left)
cv2.imshow('right', right)

dlib_predictor_model_path = "../trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "../trained_models/facial_encodings.pickle"

faceDetector = FaceDetector(facial_encodings_path, dlib_predictor_model_path)
left_boxes = faceDetector.detect_faces(left)
right_boxes = faceDetector.detect_faces(right)

left_points = faceDetector.get_landmarks(left, left_boxes[0])
right_points = faceDetector.get_landmarks(right, right_boxes[0])

print(np.shape(left_points))
print(np.shape(right_points))

#Computation of the fundamental matrix
F, mask= cv2.findFundamentalMat(left_points, right_points)

left_points = left_points[:,:][mask.ravel()==1]
right_points = right_points[:,:][mask.ravel()==1]

print(np.shape(left_points))
print(np.shape(right_points))

left_points = np.int32(left_points)
right_points = np.int32(right_points)

left_points = left_points.reshape((left_points.shape[0] * 2, 1))
right_points = right_points.reshape((right_points.shape[0] * 2, 1))

print(np.shape(left_points))
print(np.shape(right_points))

height, width = left.shape[:2]
retval, H1, H2 = cv2.stereoRectifyUncalibrated(left_points, right_points, F, (width, height))

left_new = cv2.warpPerspective(left, H1, (height, width))
right_new = cv2.warpPerspective(right, H2, (height, width))

cv2.imshow('left_new', left_new)
cv2.imshow('right_new', right_new)

cv2.waitKey(0)