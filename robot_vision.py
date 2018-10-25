from imutils.video import VideoStream
import argparse
import time
import cv2
import numpy as np
from head_pose_estimation import HeadPoseEstimator
from face_detection import FaceDetector
from depth_estimation import DepthEstimator

float_formatter = lambda x: "%.1f" % x
np.set_printoptions(formatter={'float_kind':float_formatter})
dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"

OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create
}

def draw_faces(leftFrame, rightFrame, leftBoxes, rightBoxes, names, faceDetector):
	# loop over each face
	for name in names:
		if(leftBoxes.get(name) != None and rightBoxes.get(name) != None):
			leftLandmarks = faceDetector.get_landmarks(leftFrame, leftBoxes[name])
			rightLandmarks = faceDetector.get_landmarks(rightFrame, rightBoxes[name])

			depthEstimator = DepthEstimator()
			worldCoordinates = depthEstimator.get_world_coordinates(leftLandmarks, rightLandmarks, leftBoxes[name])		

			# update the frames with the new information of the person 
			leftFrame = faceDetector.draw_face_details(leftFrame, leftBoxes[name], name, leftLandmarks, worldCoordinates)
			rightFrame = faceDetector.draw_face_details(rightFrame, rightBoxes[name], name, rightLandmarks, worldCoordinates)
	return leftFrame, rightFrame

def changeBoxFormat1(box):
	(top, right, bottom, left) = box
	return (left, top, right - left, bottom - top)

def changeBoxFormat2(box):
	(x, y, w, h) = [int(v) for v in box]
	return (y, x + w, y + h, x)

ap = argparse.ArgumentParser()
ap.add_argument("-d", "--detection-method", type=str, default="cnn",
	help="face detection model to use: either `hog` or `cnn`")
ap.add_argument("-t", "--tracker", type=str, default="csrt",
	help="OpenCV object tracker type")
ap.add_argument("-s", "--skip-frames", type=int, default=20,
	help="# of skip frames between detections")
args = vars(ap.parse_args())

# Should these be declared inside the loop locally
# In that case, if face recognition fails like in the case of occlusion,
# the tracking information won't be present to aid.
leftBoundingBoxes = {}
rightBoundingBoxes = {}

names = set()
leftTrackers = {}
rightTrackers = {}
left = cv2.VideoCapture(0)	
right = cv2.VideoCapture(1)
totalFrames = 0

while True:
	if not (left.grab() and right.grab()):
		print("No more frames")
		break

	_, leftFrame = left.retrieve()
	_, rightFrame = right.retrieve()
	
	faceDetector = FaceDetector(facial_encodings_path, dlib_predictor_model_path, args['detection_method'])

 	# Start timer
	timer = cv2.getTickCount()

	if totalFrames % args["skip_frames"] == 0:
		# names.clear()
		# leftTrackers.clear()
		# rightTrackers.clear()

		leftBoxes, leftNames = faceDetector.get_faces(leftFrame)
		rightBoxes, rightNames = faceDetector.get_faces(rightFrame)
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
	
	leftFrame, rightFrame = draw_faces(leftFrame, rightFrame, leftBoundingBoxes, rightBoundingBoxes, names, faceDetector)

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