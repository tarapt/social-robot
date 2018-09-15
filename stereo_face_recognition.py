from imutils.video import VideoStream
import face_recognition
import argparse
import imutils
import pickle
import time
import cv2
import dlib
import numpy as np
from scipy import stats
from imutils import face_utils

float_formatter = lambda x: "%.1f" % x
np.set_printoptions(formatter={'float_kind':float_formatter})

# MIN_DISPARITY = 16
# NUM_DISPARITY = 112-MIN_DISPARITY
# def getStereoSGBMObject():
#     window_size = 3
#     stereo = cv2.StereoSGBM_create(minDisparity = MIN_DISPARITY,
#         numDisparities = NUM_DISPARITY,
#         blockSize = 16,
#         P1 = 8*3*window_size**2,
#         P2 = 32*3*window_size**2,
#         disp12MaxDiff = 1,
#         uniquenessRatio = 10,
#         speckleWindowSize = 100,
#         speckleRange = 32
#     )
#     return stereo

# stereoMatcher = getStereoSGBMObject()

# def get3DScene(leftFrame, rightFrame):
# 	grayLeft = cv2.cvtColor(leftFrame, cv2.COLOR_BGR2GRAY)
# 	grayRight = cv2.cvtColor(rightFrame, cv2.COLOR_BGR2GRAY)
# 	disparity = stereoMatcher.compute(grayLeft, grayRight).astype(np.float32) / 16.0
# 	cv2.imshow('disparity', (disparity-MIN_DISPARITY)/NUM_DISPARITY)

# 	h, w = leftFrame.shape[:2]
# 	f = 760 # pixels
# 	baseline = 0.15 # meters
# 	Q = np.float32([[1, 0, 0,  -0.5*w],
# 					[0, 1, 0,  -0.5*h],
# 					[0, 0, 0,     f], 
# 					[0, 0, -1.0/baseline,      0]])
# 	# Q = np.float32([[ 1.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.01534975e+02],
#  	# 				[ 0.00000000e+00,  1.00000000e+00,  0.00000000e+00, -2.48044674e+02],
#  	# 				[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.28357378e+02],
#  	# 				[ 0.00000000e+00,  0.00000000e+00,  1.50143709e-01, -0.00000000e+00]])
# 	points = cv2.reprojectImageTo3D(disparity, Q)
# 	return points

# def get_coordinates(points, top, right, bottom, left):
# 	x = int((right + left) / 2)
# 	y = int((top + bottom) / 2)
# 	return points[y][x]

predictor_model = "shape_predictor_68_face_landmarks.dat"
predictor = dlib.shape_predictor(predictor_model)

def get_faces(frame):
	# convert the input frame from BGR to RGB then resize it to have
	# a width of 750px (to speedup processing)
	rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

	# detect the (x, y)-coordinates of the bounding boxes
	# corresponding to each face in the input frame, then compute
	# the facial embeddings for each face
	boxes = face_recognition.face_locations(rgb,
		model=args["detection_method"])
	encodings = face_recognition.face_encodings(rgb, boxes)
	names = []

	# loop over the facial embeddings
	for encoding in encodings:
		# attempt to match each face in the input image to our known encodings
		matches = face_recognition.compare_faces(data["encodings"],
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
				name = data["names"][i]
				counts[name] = counts.get(name, 0) + 1

			name = max(counts, key=counts.get)
		
		# update the list of names
		names.append(name)
	return boxes, names

def get_world_coordinates(leftLandmarks, rightLandmarks, leftBox):
	# left camera's optical centre coordinates in pixels
	cx = 324  
	cy = 234

	# lense focal length in pixels
	fx = 764
	fy = 760 			

	# distance in metres between the two cameras
	baseline = 0.17 	

	depths = []
	for (p1, p2) in zip(leftLandmarks, rightLandmarks):
		(x1, _) = p1
		(x2, _) = p2
		disparity = abs(x1 - x2)
		if disparity > 0:
			depth = (fx * baseline) / disparity
			depths.append(depth)

	(top, right, bottom, left) = leftBox
	u = (right + left) // 2
	v = (top + bottom) // 2
	Z = stats.tmean(depths)
	X = Z / fx * (u - cx)
	Y = Y = Z / fy * (v - cy)
	return (X, Y, Z)

def get_landmarks(frame, box):
	(top, right, bottom, left) = box
	landmarks = predictor(frame, dlib.rectangle(left=left, top=top, right=right, bottom=bottom))
	landmarks = face_utils.shape_to_np(landmarks)
	return landmarks

def draw_face_details(frame, box, name, landmarks, worldCoordinates):
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
	depthString = '(' + str(float_formatter(X)) + 'm,' + str(float_formatter(Y)) + 'm,' + str(float_formatter(Z)) + 'm)'
	cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
		0.75, (0, 255, 0), 2)
	cv2.putText(frame, depthString, (left, y - 25), cv2.FONT_HERSHEY_SIMPLEX,
		0.75, (0, 255, 0), 2)
	
	return frame

def draw_faces(leftFrame, rightFrame):
	leftBoxes, leftNames = get_faces(leftFrame)
	rightBoxes, rightNames = get_faces(rightFrame)

	# loop over each face
	for (leftBox, leftName, rightBox, rightName) in zip(leftBoxes, leftNames, rightBoxes, rightNames):
		leftLandmarks = get_landmarks(leftFrame, leftBox)
		rightLandmarks = get_landmarks(rightFrame, rightBox)

		worldCoordinates = get_world_coordinates(leftLandmarks, rightLandmarks, leftBox)		

		leftFrame = draw_face_details(leftFrame, leftBox, leftName, leftLandmarks, worldCoordinates)
		rightFrame = draw_face_details(rightFrame, rightBox, rightName, rightLandmarks, worldCoordinates)
	return leftFrame, rightFrame

ap = argparse.ArgumentParser()
ap.add_argument("-e", "--encodings", required=True,
	help="path to serialized db of facial encodings")
ap.add_argument("-d", "--detection-method", type=str, default="cnn",
	help="face detection model to use: either `hog` or `cnn`")
args = vars(ap.parse_args())

data = pickle.loads(open(args["encodings"], "rb").read())

left = cv2.VideoCapture(1)	
right = cv2.VideoCapture(2)

while True:
	if not (left.grab() and right.grab()):
		print("No more frames")
		break

	_, leftFrame = left.retrieve()
	_, rightFrame = right.retrieve()
	
	leftFrame, rightFrame = draw_faces(leftFrame, rightFrame)

	cv2.imshow("Left Camera", leftFrame)
	cv2.imshow("Right Camera", rightFrame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# do a bit of cleanup
left.release()
right.release()
cv2.destroyAllWindows()