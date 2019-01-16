# Note: The below issues were solved by running the camera loop from the main thread and all other
# threads UI, aria_robot as child threads.
# If the camera output is not needed to be shown on the laptop, then no highgui calls need to be made, in that case,
# the camera loop can be put in a thread with a global variable to determine when to exit.
#
# Remember: The main thread should wait for all child threads to exit first.
#
# [Solved] Abruptly exiting the program causes the cameras to not work afterwards. Produces the
# error VIDEOIO ERROR: V4L: can't open camera by index 1. Pressing q on the opencv window would
# work, but if the windows didn't start due to some reason, it could be a problem.
# So, implement the vision module as a stoppable thread https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread,
# and have the main module listen for the stop commands. write the debugging messages into a
# separate log file instead of the console, to not get in the way of the commands given to the console.
#
# imshow and waitkey issues
# -------------------------
# If the vision module is inside the same file then they are working, but once moved to a different file they aren't.
#
# Some research suggests that issuing these calls from main thread should solve these problems. But how?
# Google: create opencv window in a thread
#
# http://answers.opencv.org/question/88461/stdthread-imshow-no-windows-created/
# naive threading - fastest way to hell.
# probably none of the highgui calls should go into your thread, keep them in the main thread, you're messing with the os here
#
# https://github.com/opencv/opencv/issues/8407 You should interact with UI from the "main" thread only.
#
# Refer these:
# -----------
# http://algomuse.com/c-c/developing-a-multithreaded-real-time-video-processing-application-in-opencv
# https://nrsyed.com/2018/07/05/multithreading-with-opencv-python-to-improve-video-processing-performance/
# https://www.pyimagesearch.com/2016/05/30/displaying-a-video-feed-with-opencv-and-tkinter/
# https://www.pyimagesearch.com/2016/01/04/unifying-picamera-and-cv2-videocapture-into-a-single-class-with-opencv/

from AriaPy import *
from ArNetworkingPy import *
from BaseArnlPy import *
from SonArnlPy import *
from control_unit import ControlUnit
import cv2
import sys
from robot import RobotController
import vision
import face_detection
import math

def get_safe_position(x, y, th):
	#if(math.fabs(th) > 90):
		# TODO check if the formulaes change
	safe_distance = 1
	r = safe_distance
	del_x = r * math.sin(math.radians(th))
	del_y = r - r * math.cos(math.radians(th))
	return x - del_x, y - del_y, th

def update_goal(detections, name, robotController):
	# return True if goal reached
	robot = robotController.robot
	print(detections)
	pos = [0, 0, 0]
	th = 0
	found = False
	for person in detections:
		if person['name'] == name:
			pos = person['position']
			th = person['theta'][1]
			found = True
			print("Person found")
			break

	robot.lock()
	currPose = robot.getPose()
	robot.unlock()

	goal_x, goal_y, heading = get_safe_position(pos[0], pos[2], th)
	scale = 1023 # 1 meter = 1023 steps for the robot
	gotoPose = ArPose(currPose.getX() + goal_x * scale, currPose.getY() + goal_y * scale, currPose.getTh() + th)
	#robotController.pathTask.pathPlanToPose(gotoPose, True)
	# if found:
	#     time.sleep(30)
	# else:
	#     time.sleep(1)
	return False

if __name__ == '__main__':
	dlib_predictor_model_path = "./trained_models/shape_predictor_68_face_landmarks.dat"
	facial_encodings_path = "./trained_models/new_encodings.pickle"
	skipFrames = 0
	detection_method = 'cnn'
	try:
		robotController = RobotController(sys.argv)
		robot = robotController.robot
		robot_vision = vision.Vision(0, 1)
	except vision.CameraError:
		print("[ERROR] Cameras couldn't start. Exiting...")
	else:
		totalFrames = 0
		# time.sleep(2.0)

		print("[INFO] Capturing frames...")

		goal_reached = False
		# lastFrame = None
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
					detected_faces.append(vision.Face(name, leftBox, rightBox))

				stereoFrame, detections = robot_vision.draw_faces(
					stereoFrame, detected_faces, faceDetector)
				
				if not goal_reached:
					goal_reached = update_goal(detections, 'tara_prasad', robotController)

			# Calculate Frames per second (FPS)
			fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

			# Display FPS on frame
			cv2.putText(stereoFrame.left, "FPS : " + str(int(fps)),
						(400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
			cv2.putText(stereoFrame.right, "FPS : " + str(int(fps)),
						(400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

			cv2.imshow("Left Camera", stereoFrame.left)
			cv2.imshow("Right Camera", stereoFrame.right)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			totalFrames += 1

		# do a bit of cleanup
		robot_vision.stereoCamera.release()
		cv2.destroyAllWindows()

		# name = 'tara_prasad'
		# while(True):
		# 	robotVision.acquireDetectedPersonsLock()
		# 	print robotVision.getDetectedPersons()
		# 	pos = [0, 0, 0]
		# 	th = 0
		# 	found = False
		# 	for person in robotVision.getDetectedPersons():
		# 		if person['name'] == name:
		# 			pos = person['position']
		# 			th = person['theta'][1]
		# 			found = True
		# 			break
		# 	robotVision.releaseDetectedPersonsLock()

		# 	robot.lock()
		# 	currPose = robot.getPose()
		# 	robot.unlock()

		# 1 meter = 1023 steps for the robot

		# 	gotoPose = ArPose(currPose.getX() + pos[0] * 1023, currPose.getY() + pos[2] * 1023, currPose.getTh() + th)
		# 	pathTask.pathPlanToPose(gotoPose, True)
		# 	if found:
		# 		time.sleep(30)
		# 	else:
		# 		time.sleep(1)

		# elegantly stop the robot
		robot.disableMotors()
		robot.disableSonar()
		robot.stopRunning()
		robot.disconnect()

		# Suspend calling thread until the ArRobot run loop has exited
		robot.waitForRunExit()
	# finally:
		# elegantly stop the aria server
		# server.close()
