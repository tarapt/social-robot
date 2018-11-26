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
		return self.left.isOpened() and self.right.isOpened()

	def retrieve(self):
		_, leftFrame = self.left.retrieve()
		_, rightFrame = self.right.retrieve()
		return StereoFrame(leftFrame, rightFrame)

	def hasFrames(self):
		return self.left.grab() and self.right.grab()

	def release(self):
		self.left.release()
		self.right.release()

class Vision:
	detectedPersonsLock = Lock()
	detectedPersons = []

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

	def setup_cameras(self):
		cnt = 10
		stereoCamera = StereoCamera(0, 1)
		while(cnt > 0 and not stereoCamera.isOpened()):
			stereoCamera = StereoCamera(0, 1)
			if not stereoCamera.isOpened():
				stereoCamera = StereoCamera(1, 2)
			if not stereoCamera.isOpened():
				stereoCamera = StereoCamera(0, 2)
			cnt -= 1

		if not stereoCamera.isOpened():
			return None
		return stereoCamera

	def _capture_and_process_frames(self, skipFrames=0, detection_method='cnn'):
		stereoCamera = self.setup_cameras()
		if stereoCamera == None:
			print("Couldn't open the cameras.")
			return

		totalFrames = 0
		time.sleep(2.0)
		
		while True:
			if not stereoCamera.hasFrames():
				continue
			stereoFrame = stereoCamera.retrieve()

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
		stereoCamera.release()
		cv2.destroyAllWindows()

	def capture_and_process_frames(self, skipFrames=0, detection_method='cnn'):
		Thread(target=self._capture_and_process_frames).start()

if __name__ == '__main__':
	vision = Vision()
	vision.capture_and_process_frames()