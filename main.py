import time
import robot_vision
from AriaPy import *
from ArNetworkingPy import *
from BaseArnlPy import *
from SonArnlPy import *
import sys

# make the server for remote clients (e.g. MobileEyes)
def setup_server(robot, locTask, map, pathTask, gyro):
	server = ArServerBase()
	
	print "[INFO] Opening server on port 7272..."
	if (not server.open(7272)):
		print "[ERROR] Could not open server on port 7272, exiting"
		sys.exit(1)
	print "[INFO] Server is open on port 7272."

	# Create services for remote clients (e.g. MobileEyes):

	# Provides localization info:
	serverInfoLocalization = ArServerInfoLocalization(server, robot, locTask)

	# Allows client to manually trigger relocalization and a given point:
	serverLocHandler = ArServerHandlerLocalization(server, robot, locTask)

	# Provides the map:
	serverMap = ArServerHandlerMap(server, map)

	# Provides the planned path:
	serverInfoPath = ArServerInfoPath(server, robot, pathTask)

	# Information about the robot:
	serverInfoRobot = ArServerInfoRobot(server, robot)

	# Info from range sensors:
	serverInfoSensor = ArServerInfoSensor(server, robot)

	# Service to add new graphics on the map:
	drawings = ArServerInfoDrawings(server)
	drawings.addRobotsRangeDevices(robot)
	serverInfoPath.addSearchRectangleDrawing(drawings)

	# Modes for driving the robot:
	modeGoto = ArServerModeGoto(server, robot, pathTask, map,
								locTask.getHomePose())
	modeStop = ArServerModeStop(server, robot, 0)
	modeRatioDrive = ArServerModeRatioDrive(server, robot)
	modeWander = ArServerModeWander(server, robot)
	modeStop.addAsDefaultMode()
	modeStop.activate()
	
	# Simple text commands ("custom commands") in MobileEyes):
	commands = ArServerHandlerCommands(server)
	uCCommands = ArServerSimpleComUC(commands, robot)
	loggingCommands = ArServerSimpleComMovementLogging(commands, robot)
	gyroCommands = ArServerSimpleComGyro(commands, robot, gyro)
	configCommands = ArServerSimpleComLogRobotConfig(commands, robot)
	serverInfoPath.addControlCommands(commands)

	# Service that allows client to read and change ArConfig parameters (used 
	# throughout aria and arnl). Arnl.getTypicalDefaultParamFileName() returns
	# "params/default-arnl.p".
	serverConfig = ArServerHandlerConfig(server, Aria.getConfig(), Arnl.getTypicalDefaultParamFileName(), Aria.getDirectory())
	
	print "[INFO] Starting server thread..."
	server.runAsync()
	return server

def connect_to_robot(robot):
	# Create a "simple connector" object and connect to either the simulator
	# or the robot. Unlike the C++ API which takes int and char* pointers, 
	# the Python constructor just takes argv as a list.
	print "[INFO] Connecting..."
	con = ArSimpleConnector(sys.argv)
	if (not Aria.parseArgs()):
		Aria.logOptions()
		sys.exit(1)
	if (not con.connectRobot(robot)):
		print "[ERROR] Could not connect to robot, exiting"
		sys.exit(1)

def initialize_robot():
	Aria.init()
	Arnl.init()

	# Create robot and device objects:
	robot = ArRobot()
	gyro = ArAnalogGyro(robot)
	sonar = ArSonarDevice()
	robot.addRangeDevice(sonar)

	connect_to_robot(robot)

	map = ArMap()

	# Make the SONARNL localization task (using the sonar device, of course)
	locTask = ArSonarLocalizationTask(robot, sonar, map)

	# Make the path-planning task (using the Sonar's "current" (most recent)
	# data for obstacle detection.
	pathTask = ArPathPlanningTask(robot, sonar, map)

	# Load the configuration. Arnl.getTypicalDefaultParamFileName() returns "params/default-arnl.p".
	configFileName = Arnl.getTypicalParamFileName()
	print "[INFO] Will use config file: ", configFileName
	print "[INFO] Loading config file..."
	if (not Aria.getConfig().parseFile(configFileName)):
		print "[Error] Error loading configuration file \"%s\", exiting." % configFileName
		sys.exit(1)

	# Run the robot thread in the background:
	print "[INFO] Robot thread running..."
	robot.runAsync(True)
	
	print "[INFO] Motors enabled..."
	robot.enableMotors()

	locTask.localizeRobotAtHomeBlocking()
	return robot, gyro, sonar, map, locTask, pathTask

if __name__ == '__main__':
	try:
		robot, gyro, sonar, map, locTask, pathTask = initialize_robot()	
		# server = setup_server(robot, locTask, map, pathTask, gyro)
		
		vision = robot_vision.Vision()
	except robot_vision.CameraError:
		print("[ERROR] Cameras couldn't start. Exiting...")
	else:
		print "[INFO] Starting vision thread..."
		vision.capture_and_process_frames()
		pass
		# name = 'tara_prasad'
		# while(True):
		# 	vision.acquireDetectedPersonsLock()
		# 	print vision.getDetectedPersons()
		# 	pos = [0, 0, 0]
		# 	th = 0
		# 	found = False
		# 	for person in vision.getDetectedPersons():
		# 		if person['name'] == name:
		# 			pos = person['position']
		# 			th = person['theta'][1]
		# 			found = True
		# 			break
		# 	vision.releaseDetectedPersonsLock()
			
		# 	robot.lock()
		# 	currPose = robot.getPose()
		# 	robot.unlock()
			
		# 	gotoPose = ArPose(currPose.getX() + pos[0] * 1023, currPose.getY() + pos[2] * 1023, currPose.getTh() + th)
		# 	pathTask.pathPlanToPose(gotoPose, True)
		# 	if found:
		# 		time.sleep(30)
		# 	else:
		# 		time.sleep(1)

		# Suspend calling thread until the ArRobot run loop has exited
		robot.waitForRunExit()
	finally:
		# elegantly close the robot and server threads
		robot.disableMotors()
		robot.disableSonar()
		robot.stopRunning()
		robot.disconnect()
		# server.close()
