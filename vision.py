# This module task is to have the 3D location of a person in a map.
# If at some moment of time the person is not in the frame the map should have
# the last known location in it. The face recognition should work with as much accuracy as 
# possible. Use a neural network or SVM if possible. Have a very high threshold for face
# recognition matching. This is because inaccurate recognition is undesirable. Morever use person 
# detection and recognition if possible instead of just considering the facial features.

# TODO
# Run the calibration setup every time the cameras position is disturbed to update the 
# camera extrinisics, since the cameras may not lie on the same line and face the same plane.
# Presently the disparity calculation isn't taking into account the extrinsic parameters. 
# Feed the disparity array to an opencv function inorder to take care of distortion and other things.
# If that's not possible, need to implement the distortion model.

# Head Pose Estimation
# --------------------
# Presently, the angles calculate fluctuate rapidly, since a slight head movement
# would lead to large change in angle, this would be a problem because the
# robot would not be able to stay at a stable position. To fix this reduce the size of the angle's
# domain from 360 to 8 (octants) or 4 (quadrants), which ever works well practically.
# If the detected person is facing the opposite direction, issue a voice command like 'excuse me'
# to ask the person to turn towards the robot. Do this if maneuvering to the opposite side of the person 
# isn't possible. First choice should be to take a path to face the person while maintaining the 
# safe distance around the person, i.e, try to reach the diameterically opposite end with the 
# person as the centre and safe distance as the radius.
 
# from imutils.video import VideoStream
import argparse
import time
import cv2
import numpy as np
from head_pose_estimation import HeadPoseEstimator
from depth_estimation import DepthEstimator
import face_detection
import os

dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"
CAMERA_WIDTH = 1280 # MAX_CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960 # MAX_CAMERA_WIDTH = 960

# Directories to store frames for each camera
LEFT_PATH = "experiments/{:04d}/{:06d}_left.jpg"
RIGHT_PATH = "experiments/{:04d}/{:06d}_right.jpg"

# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
# OPENCV_OBJECT_TRACKERS = {
# 	"csrt": cv2.TrackerCSRT_create,
# 	"kcf": cv2.TrackerKCF_create,
# 	"boosting": cv2.TrackerBoosting_create,
# 	"mil": cv2.TrackerMIL_create,
# 	"tld": cv2.TrackerTLD_create,
# 	"medianflow": cv2.TrackerMedianFlow_create,
# 	"mosse": cv2.TrackerMOSSE_create
# }

# initialize OpenCV's special multi-object tracker
# consider using an array of trackers instead, to know which of the tracker failed
# trackers = cv2.MultiTracker_create()

class Face:
	def __init__(self, name, leftBBox, rightBBox):
		self.name = name
		self.leftBBox = leftBBox
		self.rightBBox = rightBBox

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
	def __init__(self, id1=-1, id2=-1):
		self.setup_cameras(id1, id2)
		self.increase_resolution()

	def draw_faces(self, stereoFrame, detectedFaces, faceDetector):
		detections = []
		# loop over each face
		for face in detectedFaces:
			leftLandmarks = faceDetector.get_landmarks(stereoFrame.left, face.leftBBox)
			rightLandmarks = faceDetector.get_landmarks(stereoFrame.right, face.rightBBox)

			pose_estimator = HeadPoseEstimator()
			cube_image_coords_L, euler_angle_L = pose_estimator.get_head_pose(leftLandmarks)
			cube_image_coords_R, euler_angle_R = pose_estimator.get_head_pose(rightLandmarks)
			stereoFrame.left = pose_estimator.draw_head_pose(cube_image_coords_L, euler_angle_L, stereoFrame.left)
			stereoFrame.right = pose_estimator.draw_head_pose(cube_image_coords_R, euler_angle_R, stereoFrame.right)

			depthEstimator = DepthEstimator()
			worldCoordinates = depthEstimator.get_world_coordinates(leftLandmarks, rightLandmarks, face.leftBBox)
			
			detections.append({'name': face.name, 'position' : list(worldCoordinates), 'theta': euler_angle_L.tolist()})
			
			# update the frames with the new information of the person 
			stereoFrame.left = faceDetector.draw_face_details(stereoFrame.left, face.leftBBox, face.name, leftLandmarks, worldCoordinates)
			stereoFrame.right = faceDetector.draw_face_details(stereoFrame.right, face.rightBBox, face.name, rightLandmarks, worldCoordinates)
		return stereoFrame, detections

	def setup_cameras(self, id1=-1, id2=-1, max_trials=10):
		if id1 == -1 or id2 == -1:
			cnt = 1
			stereoCamera = StereoCamera(0, 1)
			while(cnt < max_trials and not stereoCamera.isOpened()):
				print("Attempt #%d to open cameras..." % cnt)
				stereoCamera = StereoCamera(0, 1)
				if not stereoCamera.isOpened():
					stereoCamera = StereoCamera(1, 2)
				if not stereoCamera.isOpened():
					stereoCamera = StereoCamera(0, 2)
				cnt += 1
			self.stereoCamera = stereoCamera
		else:
			self.stereoCamera = StereoCamera(id1, id2)
		
		if not self.stereoCamera.isOpened():
			raise CameraError

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

if __name__ == '__main__':
	skipFrames = 0
	detection_method = 'cnn'

	experiment_count = get_experiment_count()

	format_string = "{:04d}"
	if not os.path.exists('experiments/' + format_string.format(experiment_count)):
		os.mkdir('experiments/' + format_string.format(experiment_count))		

	try:
		robot_vision = Vision(1, 2)
	except CameraError:
		print("[ERROR] Cameras couldn't start. Exiting...")
	else:
		totalFrames = 0
		# time.sleep(2.0)

		print("[INFO] Capturing frames...")

		lastKnownLocations = {}

		while True:
			if not robot_vision.stereoCamera.hasFrames():
				continue
			stereoFrame = robot_vision.stereoCamera.retrieve()
			print("[INFO] Captured frame %d..." % totalFrames)

			# Start timer
			timer = cv2.getTickCount()

			if totalFrames % (skipFrames + 1) == 0:
				faceDetector = face_detection.FaceDetector(
					facial_encodings_path, dlib_predictor_model_path, detection_method)
				leftBoxes, names = faceDetector.get_faces(stereoFrame.left)
				rightBoxes, _ = faceDetector.get_faces(stereoFrame.right)

				detected_faces = []
				for leftBox, rightBox, name in zip(leftBoxes, rightBoxes, names):
					detected_faces.append(Face(name, leftBox, rightBox))

				stereoFrame, detections = robot_vision.draw_faces(
					stereoFrame, detected_faces, faceDetector)
				
				# TODO: Beware of wrong detections, increase the confidence threshold
				# for person in detections:
				# 	lastKnownLocations[person['name']] = person

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
			# cv2.imwrite(LEFT_PATH.format(experiment_count, totalFrames), stereoFrame.left)
			# cv2.imwrite(RIGHT_PATH.format(experiment_count, totalFrames), stereoFrame.right)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			totalFrames += 1

		# do a bit of cleanup
		robot_vision.stereoCamera.release()
		cv2.destroyAllWindows()
		increment_experiment_count(experiment_count)