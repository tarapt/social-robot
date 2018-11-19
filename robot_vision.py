from imutils.video import VideoStream
import argparse
import time
import cv2
import numpy as np
from head_pose_estimation import HeadPoseEstimator
from face_detection import FaceDetector
from depth_estimation import DepthEstimator
from flask import Flask, jsonify, abort
from threading import Lock, Thread

dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"

detectedPersonsLock = Lock()
detectedPersons = []

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

	def retrieve(self):
		_, leftFrame = self.left.retrieve()
		_, rightFrame = self.right.retrieve()
		return StereoFrame(leftFrame, rightFrame)

	def hasFrames(self):
		return self.left.grab() and self.right.grab()

	def release(self):
		self.left.release()
		self.right.release()

def draw_faces(stereoFrame, detectedFaces, faceDetector):
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

def capture_and_process_frames(args):
	stereoCamera = StereoCamera(0, 1)
	totalFrames = 0
	skipFrames = args['skip_frames']
	time.sleep(5.0)
	while True:
		if not stereoCamera.hasFrames():
			continue

		stereoFrame = stereoCamera.retrieve()

		# Start timer
		timer = cv2.getTickCount()
		
		if totalFrames % (skipFrames + 1) == 0:
			faceDetector = FaceDetector(facial_encodings_path, dlib_predictor_model_path, args['detection_method'])
			leftBoxes, names = faceDetector.get_faces(stereoFrame.left)
			rightBoxes, _ = faceDetector.get_faces(stereoFrame.right)

			detected_faces = []
			for leftBox, rightBox, name in zip(leftBoxes, rightBoxes, names):
				detected_faces.append(Face(name, leftBox, rightBox))
			
			stereoFrame, detections = draw_faces(stereoFrame, detected_faces, faceDetector)

			detectedPersonsLock.acquire()
			global detectedPersons
			detectedPersons = detections
			detectedPersonsLock.release()

		# Calculate Frames per second (FPS)
		fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

		# Display FPS on frame
		cv2.putText(stereoFrame.left, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
		cv2.putText(stereoFrame.right, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

		cv2.imshow("Left Camera", stereoFrame.left)
		cv2.imshow("Right Camera", stereoFrame.right)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		totalFrames += 1

	# do a bit of cleanup
	stereoCamera.release()
	cv2.destroyAllWindows()

app = Flask(__name__)

@app.route('/detections/', methods=['GET'])
def getDetections():
	detectedPersonsLock.acquire()
	ret = jsonify(detectedPersons)
	detectedPersonsLock.release()
	return ret

if __name__ == '__main__':
	ap = argparse.ArgumentParser()
	ap.add_argument("-d", "--detection-method", type=str, default="cnn",
		help="face detection model to use: either `hog` or `cnn`")
	ap.add_argument("-s", "--skip-frames", type=int, default="0",
		help="number of frames to skip before running face detection ")
	args = vars(ap.parse_args())
	Thread(target=capture_and_process_frames, args = (args, )).start()
	app.run(debug=False)