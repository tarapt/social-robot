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

OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create
}

predictor_model = "shape_predictor_68_face_landmarks.dat"
predictor = dlib.shape_predictor(predictor_model)

def get_faces(frame):
	# convert the input frame from BGR to RGB
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

def draw_faces(leftFrame, rightFrame, leftBoxes, rightBoxes, names):
	# loop over each face
	for name in names:
		if(leftBoxes.get(name) != None and rightBoxes.get(name) != None):
			leftLandmarks = get_landmarks(leftFrame, leftBoxes[name])
			rightLandmarks = get_landmarks(rightFrame, rightBoxes[name])

			worldCoordinates = get_world_coordinates(leftLandmarks, rightLandmarks, leftBoxes[name])		

			# update the frames with the new information of the person 
			leftFrame = draw_face_details(leftFrame, leftBoxes[name], name, leftLandmarks, worldCoordinates)
			rightFrame = draw_face_details(rightFrame, rightBoxes[name], name, rightLandmarks, worldCoordinates)
	return leftFrame, rightFrame

def changeBoxFormat1(box):
	(top, right, bottom, left) = box
	return (left, top, right - left, bottom - top)

def changeBoxFormat2(box):
	(x, y, w, h) = [int(v) for v in box]
	return (y, x + w, y + h, x)

ap = argparse.ArgumentParser()
ap.add_argument("-e", "--encodings", required=True,
	help="path to serialized db of facial encodings")
ap.add_argument("-d", "--detection-method", type=str, default="cnn",
	help="face detection model to use: either `hog` or `cnn`")
ap.add_argument("-t", "--tracker", type=str, default="csrt",
	help="OpenCV object tracker type")
ap.add_argument("-s", "--skip-frames", type=int, default=20,
	help="# of skip frames between detections")
args = vars(ap.parse_args())

data = pickle.loads(open(args["encodings"], "rb").read())

# Should these be declared inside the loop locally
# In that case, if face recognition fails like in the case of occlusion,
# the tracking information won't be present to aid.
leftBoundingBoxes = {}
rightBoundingBoxes = {}

names = set()
leftTrackers = {}
rightTrackers = {}
left = cv2.VideoCapture(1)	
right = cv2.VideoCapture(2)
totalFrames = 0

while True:
	if not (left.grab() and right.grab()):
		print("No more frames")
		break

	_, leftFrame = left.retrieve()
	_, rightFrame = right.retrieve()
	
 	# Start timer
	timer = cv2.getTickCount()

	if totalFrames % args["skip_frames"] == 0:
		# names.clear()
		# leftTrackers.clear()
		# rightTrackers.clear()

		leftBoxes, leftNames = get_faces(leftFrame)
		rightBoxes, rightNames = get_faces(rightFrame)
		names.update(leftNames)
		names.update(rightNames)
		for (box, name) in zip(leftBoxes, leftNames):
			leftBoundingBoxes[name] = box
			if(leftTrackers.get(name) != None):
				del leftTrackers[name]
			tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
			tracker.init(leftFrame, changeBoxFormat1(box))
			leftTrackers[name] = tracker		
		for (box, name) in zip(rightBoxes, rightNames):
			rightBoundingBoxes[name] = box
			if(rightTrackers.get(name) != None):
				del rightTrackers[name]
			tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
			tracker.init(rightFrame, changeBoxFormat1(box))
			rightTrackers[name] = tracker
		
		# don't do this:
		# delete the trackers of those that are not present in the frame currently
		# for name in names:
		# 	if(name not in leftNames):
		# 		if(leftTrackers.get(name) != None):
		# 			del leftTrackers[name]
		# 		if(leftBoundingBoxes.get(name) != None):
		# 			del leftBoundingBoxes[name]
		# 	if(name not in rightNames):
		# 		if(rightTrackers.get(name) != None):
		# 			del rightTrackers[name]
		# 		if(rightBoundingBoxes.get(name) != None):
		# 			del rightBoundingBoxes[name]
	else:
		for name in names:
			if(leftTrackers.get(name) != None):
				(success, box) = leftTrackers[name].update(leftFrame)
				if(not success):
					print(name + " couldn't be tracked in the left Frame. Use face recognition.")
				else:
					leftBoundingBoxes[name] = changeBoxFormat2(box)

			if(rightTrackers.get(name) != None):
				(success, box) = rightTrackers[name].update(rightFrame)
				if(not success):
					print(name + " couldn't be tracked in the right Frame. Use face recognition.")
				else:
					rightBoundingBoxes[name] = changeBoxFormat2(box)
	
	leftFrame, rightFrame = draw_faces(leftFrame, rightFrame, leftBoundingBoxes, rightBoundingBoxes, names)

	# Calculate Frames per second (FPS)
	fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
	# Display FPS on frame
	cv2.putText(leftFrame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
	cv2.putText(rightFrame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)


	cv2.imshow("Left Camera", leftFrame)
	cv2.imshow("Right Camera", rightFrame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	totalFrames += 1

# do a bit of cleanup
left.release()
right.release()
cv2.destroyAllWindows()