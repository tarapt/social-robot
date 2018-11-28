# This module task is to have the 3D location of a person in a map.
# If at some moment of time the person is not in the frame the map should have
# the last known location in it. The face recognition should work with as much accuracy as 
# possible. Use a neural network or SVM if possible. Have a very high threshold for face
# recognition matching. This is because inaccurate recognition is undesirable. Morever use person 
# detection and recognition if possible instead of just considering the facial features.

# Run the calibration setup every time the cameras position is disturbed to update the 
# camera extrinisics, since the cameras may not lie on the same line and face the same plane.
# Presently the disparity calculation isn't taking into account the extrinsic parameters. 
# Feed the disparity array to an opencv function inorder to take care of distortin and other things.
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
 
from imutils.video import VideoStream
import argparse
import time
import cv2
import numpy as np
from head_pose_estimation import HeadPoseEstimator
from face_detection import FaceDetector
from depth_estimation import DepthEstimator
from threading import Lock, Thread

dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"

# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create
}

# initialize OpenCV's special multi-object tracker
# consider using an array of trackers instead, to know which of the tracker failed
trackers = cv2.MultiTracker_create()

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
	def __init__(self):
		self.setup_cameras()
	
	detectedPersonsLock = Lock()
	detectedPersons = []

	def acquireDetectedPersonsLock(self):
		self.detectedPersonsLock.acquire()

	def releaseDetectedPersonsLock(self):
		self.detectedPersonsLock.release()

	def getDetectedPersons(self):
		return self.detectedPersons

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

	def setup_cameras(self, max_trials=10):
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
		if not stereoCamera.isOpened():
			raise CameraError

	def _capture_and_process_frames(self, skipFrames=0, detection_method='cnn'):
		totalFrames = 0
		# time.sleep(2.0)

		print("[INFO] Capturing frames...")
		while True:
			if not self.stereoCamera.hasFrames():
				continue
			stereoFrame = self.stereoCamera.retrieve()
			print("[INFO] Captured frame %d..." % totalFrames)
			# Start timer
			timer = cv2.getTickCount()
			
			if totalFrames % (skipFrames + 1) == 0:
				faceDetector = FaceDetector(facial_encodings_path, dlib_predictor_model_path, detection_method)
				leftBoxes, names = faceDetector.get_faces(stereoFrame.left)
				rightBoxes, _ = faceDetector.get_faces(stereoFrame.right)

				detected_faces = []
				for leftBox, rightBox, name in zip(leftBoxes, rightBoxes, names):
					detected_faces.append(Face(name, leftBox, rightBox))
				
				stereoFrame, detections = self.draw_faces(stereoFrame, detected_faces, faceDetector)

				self.detectedPersonsLock.acquire()
				global detectedPersons
				detectedPersons = detections
				self.detectedPersonsLock.release()

			# Calculate Frames per second (FPS)
			fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

			# Display FPS on frame
			cv2.putText(stereoFrame.left, "FPS : " + str(int(fps)), (400,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
			cv2.putText(stereoFrame.right, "FPS : " + str(int(fps)), (400,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

			cv2.imshow("Left Camera", stereoFrame.left)
			cv2.imshow("Right Camera", stereoFrame.right)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			totalFrames += 1

		# do a bit of cleanup
		self.stereoCamera.release()
		cv2.destroyAllWindows()

	def capture_and_process_frames(self, skipFrames=0, detection_method='cnn'):
		Thread(target=self._capture_and_process_frames).start()

if __name__ == '__main__':
	vision = Vision()
	vision.capture_and_process_frames()