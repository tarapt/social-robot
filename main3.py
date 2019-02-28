from AriaPy import *
from ArNetworkingPy import *
from BaseArnlPy import *
from SonArnlPy import *
import sys

import cv2
import vision
import face_detection
import math
import time

SERVER_PORT = 7275

# TODO: Decrease the robots maximum linear and angular velocity

# This is a robot server written in Python, using SONARNL for
# path planning and localization.
# After it connects to the robot or simulator, connect with MobileEyes.

def get_safe_position(x, y, th):
    # if(math.fabs(th) > 90):
        # TODO: check if the formulaes change
    safe_distance = 1
    r = safe_distance
    del_x = r * math.sin(math.radians(th))
    del_y = r * math.cos(math.radians(th))
    return x - del_x, y - del_y, th

def get_position_in_world_coordinates(robot_pose, position_in_robot_coordinates):
    x1, y1, theta1 = robot_pose.getX(), robot_pose.getY(), robot_pose.getTh()
    x2, y2 = position_in_robot_coordinates[0], position_in_robot_coordinates[1]

    r = math.hypot(x2, y2)
    theta1 = math.radians(theta1)
    theta2 = math.atan2(x2, y2)

    x = r * math.cos(theta1 + theta2)
    y = r * math.sin(theta1 + theta2)
    return x1 + x, y1 + y

def goto_position(robot, x, z, th, pathTask):
    robot.lock()
    currPose = robot.getPose()
    robot.unlock()

    # these coordinates are in robot's POV
    goal_x, goal_y, delta_heading = get_safe_position(x, z, th)

    print("Safe position: " + str(goal_x) + " " + str(goal_y))

    scale = 1023  # 1 meter = 1023 steps for the robot
    goal_x = goal_x * scale
    goal_y = goal_y * scale

    goal_X, goal_Y = get_position_in_world_coordinates(
        currPose, [goal_x, goal_y])

    goal_heading = currPose.getTh() + delta_heading
    print("[update_goal]: " + "Changing goal to ",
          goal_X, goal_Y, goal_heading)
    pathTask.pathPlanToPose(ArPose(goal_X, goal_Y, goal_heading), True)

def update_goal(detections, name, robot, pathTask):
    # return True if goal reached
    print("[update_goal]: Persons detected in camera - ", detections)

    found = False
    for person in detections:
        if person['name'] == name:
            pos = person['position']
            th = person['theta'][1]
            found = True
            print("[update_goal]: " + name + " found")
            break

    if not found:
        return

    robot.lock()
    currPose = robot.getPose()
    robot.unlock()

    # these coordinates are in robot's POV
    goal_x, goal_y, delta_heading = get_safe_position(pos[0], pos[2], th)

    scale = 1023  # 1 meter = 1023 steps for the robot
    goal_x = goal_x * scale
    goal_y = goal_y * scale

    goal_X, goal_Y = get_position_in_world_coordinates(
        currPose, [goal_x, goal_y])

    goal_heading = currPose.getTh() + delta_heading
    print("[update_goal]: " + "Changing goal to ",
          goal_X, goal_Y, goal_heading)
    pathTask.pathPlanToPose(ArPose(goal_X, goal_Y, goal_heading), True)
    return False

def get_position(robot):
    robot.lock()
    p = robot.getPose()
    robot.unlock()
    print("Current position is: ({}, {}, {})".format(p.getX(), p.getY(), p.getTh()))
    return p.getX(), p.getY(), p.getTh()

def setup_arnl(start_at_home=False):
    pass

if __name__ == '__main__':
    start_at_home = False

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

    print "Opening server on port " + str(SERVER_PORT) + "..."
    if (not server.open(SERVER_PORT)):
        print "Could not open server on port " + str(SERVER_PORT) + ", exiting"
        sys.exit(1)
    print "Server is open on port " + str(SERVER_PORT) + "."


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
    print "Will use config file: ", configFileName
    serverConfig = ArServerHandlerConfig(server, Aria.getConfig(
    ), Arnl.getTypicalDefaultParamFileName(), Aria.getDirectory())

    # Load the configuration
    print "Loading config file..."
    if (not Aria.getConfig().parseFile(configFileName)):
        print "Warning: Error loading configuration file \"%s\", exiting." % configFileName

    # Run the robot and server threads in the background:
    print "Running..."
    robot.runAsync(1)

    robot.lock()

    poseStorage = ArPoseStorage(robot)
    if start_at_home:
        if(map.findMapObject("Home") != None):
            robot.moveTo(map.findMapObject("Home").getPose())
            print("Robot move to Home. Home Position = ")
            get_position(robot)
        else:
            print("Couldn't reset to the home position.\n")
    else:
        if (poseStorage.restorePose("robotPose")):
            serverLocHandler.setSimPose(robot.getPose())

    robot.unlock()
    locTask.localizeRobotAtHomeBlocking()

    server.runAsync()
    robot.enableMotors()
    #--------------------------------------------#
    # robot, pathTask = setup_arnl(start_at_home=True)
    get_position(robot)
    time.sleep(10.0)

    goto_position(robot, 0, 2, 45, pathTask)
    
    # sleep the main thread for sufficient time to allow the tasks assigned to be completed
    time.sleep(30.0)
    get_position(robot)

    # TODO check for Goal Reached or failed before stop running, instead of sleeping
    # elegantly stop the robot, follow this exact order
    # stopRunning internally calls, waitForRunExit() and disconnect(), disableMotors() and disableSonar() do not seem to be needed
    robot.stopRunning()
    print("Robot stopped running...")
    Aria.exit()