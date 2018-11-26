from imutils.video import VideoStream
import argparse
import time
import cv2
import numpy as np
from head_pose_estimation import HeadPoseEstimator
from face_detection import FaceDetector
from depth_estimation import DepthEstimator
from threading import Lock, Thread
from AriaPy import *
from ArNetworkingPy import *
from BaseArnlPy import *
from SonArnlPy import *
import sys

dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
facial_encodings_path = "./trained_models/facial_encodings.pickle"

detectedPersonsLock = Lock()
detectedPersons = []

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

def capture_and_process_frames(skipFrames=0, detection_method='cnn'):
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
			
			stereoFrame, detections = draw_faces(stereoFrame, detected_faces, faceDetector)

			detectedPersonsLock.acquire()
			global detectedPersons
			detectedPersons = detections
			detectedPersonsLock.release()

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

if __name__ == '__main__':
	Thread(target=capture_and_process_frames).start()
	
	Aria.init()
	Arnl.init()

	# Create robot and device objects:
	robot = ArRobot()
	gyro = ArAnalogGyro(robot)
	sonar = ArSonarDevice()
	robot.addRangeDevice(sonar)

	# make the server for remote clients (e.g. MobileEyes)
	server = ArServerBase()

	# Create a "simple connector" object and connect to either the simulator
	# or the robot. Unlike the C++ API which takes int and char* pointers, 
	# the Python constructor just takes argv as a list.
	print "Connecting..."
	con = ArSimpleConnector(sys.argv)
	if (not Aria.parseArgs()):
		Aria.logOptions()
		sys.exit(1)
	if (not con.connectRobot(robot)):
		print "Could not connect to robot, exiting"
		sys.exit(1)

	map = ArMap()

	# Make the SONARNL localization task (using the sonar device, of course)
	locTask = ArSonarLocalizationTask(robot, sonar, map)

	# Make the path-planning task (using the Sonar's "current" (most recent)
	# data for obstacle detection.
	pathTask = ArPathPlanningTask(robot, sonar, map)

	# print "Opening server on port 7272..."
	# if (not server.open(7272)):
	# 	print "Could not open server on port 7272, exiting"
	# 	sys.exit(1)
	# print "Server is open on port 7272."


	# Create services for remote clients (e.g. MobileEyes):

	# Provides localization info:
	# serverInfoLocalization = ArServerInfoLocalization(server, robot, locTask)

	# Allows client to manually trigger relocalization and a given point:
	# serverLocHandler = ArServerHandlerLocalization(server, robot, locTask)

	# Provides the map:
	# serverMap = ArServerHandlerMap(server, map)

	# Provides the planned path:
	# serverInfoPath = ArServerInfoPath(server, robot, pathTask)

	# Information about the robot:
	# serverInfoRobot = ArServerInfoRobot(server, robot)

	# Info from range sensors:
	# serverInfoSensor = ArServerInfoSensor(server, robot)

	# Service to add new graphics on the map:
	# drawings = ArServerInfoDrawings(server)
	# drawings.addRobotsRangeDevices(robot)
	# serverInfoPath.addSearchRectangleDrawing(drawings)

	# Modes for driving the robot:
	# modeGoto = ArServerModeGoto(server, robot, pathTask, map,
	# 							locTask.getHomePose())
	# modeStop = ArServerModeStop(server, robot, 0)
	# modeRatioDrive = ArServerModeRatioDrive(server, robot)
	# modeWander = ArServerModeWander(server, robot)
	# modeStop.addAsDefaultMode()
	# modeStop.activate()

	# Simple text commands ("custom commands") in MobileEyes):
	# commands = ArServerHandlerCommands(server)
	# uCCommands = ArServerSimpleComUC(commands, robot)
	# loggingCommands = ArServerSimpleComMovementLogging(commands, robot)
	# gyroCommands = ArServerSimpleComGyro(commands, robot, gyro)
	# configCommands = ArServerSimpleComLogRobotConfig(commands, robot)
	# serverInfoPath.addControlCommands(commands)

	# Service that allows client to read and change ArConfig parameters (used 
	# throughout aria and arnl). Arnl.getTypicalDefaultParamFileName() returns
	# "params/default-arnl.p".
	configFileName = Arnl.getTypicalParamFileName()
	print "Will use config file: ", configFileName
	# serverConfig = ArServerHandlerConfig(server, Aria.getConfig(), Arnl.getTypicalDefaultParamFileName(), Aria.getDirectory())

	# Load the configuration
	print "Loading config file..."
	if (not Aria.getConfig().parseFile(configFileName)):
		print "Warning: Error loading configuration file \"%s\", exiting." % configFileName

	# Run the robot and server threads in the background:
	print "Running..."
	robot.runAsync(1)

	# robot.lock()
	# poseStorage = ArPoseStorage(robot)
	# if (poseStorage.restorePose("robotPose")):
	# 	serverLocHandler.setSimPose(robot.getPose())
	# robot.unlock()
	locTask.localizeRobotAtHomeBlocking()
	# server.runAsync()
	robot.enableMotors()

	name = 'tara_prasad'
	while(True):
		detectedPersonsLock.acquire()
		print detectedPersons
		pos = [0, 0, 0]
		th = 0
		found = False
		for person in detectedPersons:
			if person['name'] == name:
				pos = person['position']
				th = person['theta'][1]
				found = True
				break
		detectedPersonsLock.release()
		
		robot.lock()
		currPose = robot.getPose()
		robot.unlock()
		
		gotoPose = ArPose(currPose.getX() + pos[0] * 1023, currPose.getY() + pos[2] * 1023, currPose.getTh() + th)
		pathTask.pathPlanToPose(gotoPose, True)
		if found:
			time.sleep(30)
		else:
			time.sleep(1)

	robot.waitForRunExit()
