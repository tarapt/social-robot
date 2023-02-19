from AriaPy import *
from ArNetworkingPy import *
from BaseArnlPy import *
from SonArnlPy import *
import sys
# from cursed import CursedApp, CursedWindow, CursedMenu
import math
import time
import logging

SERVER_PORT = 7272
start_at_home = True

ROBOT_UPDATES_SERVICE_URL = "http://localhost:5000/robot_position"

# TODO: Decrease the robots maximum linear and angular velocity

# This is a robot server written in Python, using SONARNL for
# path planning and localization.
# After it connects to the robot or simulator, connect with MobileEyes.

logging.basicConfig(
    filename='robot_manager.log',
    format='%(levelname)s: %(message)s',
    filemode='w',
    level=logging.DEBUG)

# Global library initialization, just like the C++ API:
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
logging.info("Connecting...")
con = ArSimpleConnector(sys.argv)
if (not Aria.parseArgs()):
    Aria.logOptions()
    sys.exit(1)
if (not con.connectRobot(robot)):
    logging.critical("Could not connect to robot, exiting")
    sys.exit(1)

map = ArMap()

# Make the SONARNL localization task (using the sonar device, of course)
locTask = ArSonarLocalizationTask(robot, sonar, map)

# Make the path-planning task (using the Sonar's "current" (most recent)
# data for obstacle detection.
pathTask = ArPathPlanningTask(robot, sonar, map)

logging.info("Opening server on port %s...", SERVER_PORT)
if (not server.open(SERVER_PORT)):
    logging.critical(
        "Could not open server on port %s, exiting...",
        SERVER_PORT)
    sys.exit(1)
logging.info("Server is open on port %s.", SERVER_PORT)


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
configFileName = Arnl.getTypicalParamFileName()
logging.info("Will use config file: %s.", configFileName)
serverConfig = ArServerHandlerConfig(server, Aria.getConfig(
), Arnl.getTypicalDefaultParamFileName(), Aria.getDirectory())

# Load the configuration
logging.debug("Loading config file...")
if (not Aria.getConfig().parseFile(configFileName)):
    logging.critical(
        "Error loading configuration file \"%s\", exiting...",
        configFileName)

# Run the robot and server threads in the background:
logging.info("Robot thread is running...")
robot.runAsync(1)

robot.lock()

# poseStorage = ArPoseStorage(robot)
# if start_at_home:
#     if(map.findMapObject("Home") != None):
#         robot.moveTo(map.findMapObject("Home").getPose())
#         logging.info("Robot's position set to Home position.")
#     else:
#         logging.error("Couldn't reset to the home position. Home goal not found.")
# else:
#     if (poseStorage.restorePose("robotPose")):
#         serverLocHandler.setSimPose(robot.getPose())

robot.unlock()
locTask.localizeRobotAtHomeBlocking()

server.runAsync()
robot.enableMotors()
#-------------------------------------------------------------------#

robot.waitForRunExit()
# stopRunning internally calls, waitForRunExit() and disconnect(), disableMotors()
# and disableSonar() do not seem to be needed
robot.stopRunning()
logging.info("Robot has stopped running, quitting...")
Aria.exit()
