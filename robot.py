import time
# import sys
from AriaPy import *
from ArNetworkingPy import *
from BaseArnlPy import *
from SonArnlPy import *


class RobotError(Exception):
    pass


class RobotController:
    def __init__(self, arg_list):
        Aria.init()
        Arnl.init()

        # Create robot and device objects:
        self.robot = ArRobot()
        self.gyro = ArAnalogGyro(self.robot)
        self.sonar = ArSonarDevice()
        self.robot.addRangeDevice(self.sonar)

        self.connect_to_robot(self.robot, arg_list)

        self.map = ArMap()

        # Make the SONARNL localization task (using the sonar device, of course)
        self.locTask = ArSonarLocalizationTask(
            self.robot, self.sonar, self.map)

        # Make the path-planning task (using the Sonar's "current" (most recent)
        # data for obstacle detection.
        self.pathTask = ArPathPlanningTask(self.robot, self.sonar, self.map)

        # Load the configuration. Arnl.getTypicalDefaultParamFileName() returns "params/default-arnl.p".
        configFileName = Arnl.getTypicalParamFileName()
        print "[INFO] Will use config file: ", configFileName

        # Load the configuration
        print "[INFO] Loading config file..."
        if (not Aria.getConfig().parseFile(configFileName)):
            print "[Error] Error loading configuration file \"%s\", exiting." % configFileName
            raise RobotError

        # self.setup_server()

        # Run the robot thread in the background:
        print "[INFO] Robot thread running..."
        self.robot.runAsync(True)

        # TODO: use the poseStorage to remember the last known location of the robot
        # self.robot.lock()
        # poseStorage = ArPoseStorage(robot)
        # if (poseStorage.restorePose("robotPose")):
        #     serverLocHandler.setSimPose(self.robot.getPose())
        # robot.unlock()
        self.locTask.localizeRobotAtHomeBlocking()

        # # Run the server thread in the background:
        # print "[INFO] Server thread running..."
        # self.server.runAsync()

        print "[INFO] Motors enabled..."
        self.robot.enableMotors()

    def pathPlanToPose(self, goal_X, goal_Y, goal_heading):
        self.pathTask.pathPlanToPose(
            ArPose(goal_X, goal_Y, goal_heading), True)

    def stopServer(self):
        self.server.close()

    # uses: robot, locTask, map, pathTask, gyro
    # make the server for remote clients (e.g. MobileEyes)
    def setup_server(self):
        self.server = ArServerBase()

        print "[INFO] Opening server on port 7272..."
        if (not self.server.open(7272)):
            print "[ERROR] Could not open server on port 7272, exiting"
            raise RobotError
        print "[INFO] Server is open on port 7272."

        # Create services for remote clients (e.g. MobileEyes):

        # Provides localization info:
        serverInfoLocalization = ArServerInfoLocalization(
            self.server, self.robot, self.locTask)

        # Allows client to manually trigger relocalization and a given point:
        serverLocHandler = ArServerHandlerLocalization(
            self.server, self.robot, self.locTask)

        # Provides the map:
        serverMap = ArServerHandlerMap(self.server, self.map)

        # Provides the planned path:
        serverInfoPath = ArServerInfoPath(
            self.server, self.robot, self.pathTask)

        # Information about the robot:
        serverInfoRobot = ArServerInfoRobot(self.server, self.robot)

        # Info from range sensors:
        serverInfoSensor = ArServerInfoSensor(self.server, self.robot)

        # Service to add new graphics on the map:
        drawings = ArServerInfoDrawings(self.server)
        drawings.addRobotsRangeDevices(self.robot)
        serverInfoPath.addSearchRectangleDrawing(drawings)

        # Modes for driving the robot:
        modeGoto = ArServerModeGoto(self.server, self.robot, self.pathTask, self.map,
                                    self.locTask.getHomePose())
        modeStop = ArServerModeStop(self.server, self.robot, 0)
        modeRatioDrive = ArServerModeRatioDrive(self.server, self.robot)
        modeWander = ArServerModeWander(self.server, self.robot)
        modeStop.addAsDefaultMode()
        modeStop.activate()

        # Simple text commands ("custom commands") in MobileEyes):
        commands = ArServerHandlerCommands(self.server)
        uCCommands = ArServerSimpleComUC(commands, self.robot)
        loggingCommands = ArServerSimpleComMovementLogging(
            commands, self.robot)
        gyroCommands = ArServerSimpleComGyro(commands, self.robot, self.gyro)
        configCommands = ArServerSimpleComLogRobotConfig(commands, self.robot)
        serverInfoPath.addControlCommands(commands)

        # Service that allows client to read and change ArConfig parameters (used
        # throughout aria and arnl). Arnl.getTypicalDefaultParamFileName() returns
        # "params/default-arnl.p".
        serverConfig = ArServerHandlerConfig(self.server, Aria.getConfig(
        ), Arnl.getTypicalDefaultParamFileName(), Aria.getDirectory())

    def connect_to_robot(self, robot, arg_list):
        # Create a "simple connector" object and connect to either the simulator
        # or the robot. Unlike the C++ API which takes int and char* pointers,
        # the Python constructor just takes argv as a list.
        print "[INFO] Connecting..."
        con = ArSimpleConnector(arg_list)
        if (not Aria.parseArgs()):
            Aria.logOptions()
            raise RobotError
        if (not con.connectRobot(robot)):
            print "[ERROR] Could not connect to robot, exiting"
            raise RobotError

    def waitForRunExit(self):
        self.robot.waitForRunExit()

# def get_position(robot):
#     robot.lock()
#     p = robot.getPose()
#     robot.unlock()
#     print(p.getX(), p.getY(), p.getTh())
#     return p.getX(), p.getY(), p.getTh()


# if __name__ == '__main__':
#     robotController = RobotController(sys.argv)
#     robot = robotController.robot
#     pathTask = robotController.pathTask

#     x, y, th = get_position(robot)

#     # pathTask.startPathPlanToLocalPose(True)

#     pathTask.pathPlanToPose(ArPose(x + 1000, y, 90), True)

#     state = pathTask.getState()
#     while(state != ArPathPlanningInterface.REACHED_GOAL):
#         x, y, th = get_position(robot)
#         state = pathTask.getState()
#         time.sleep(1)

#     pathTask.pathPlanToPose(ArPose(x, y + 500, 0), True)
#     while(state != ArPathPlanningInterface.REACHED_GOAL):
#         state = pathTask.getState()
#         x, y, th = get_position(robot)
#         state = pathTask.getState()
#         time.sleep(1)

    # robot.waitForRunExit()
