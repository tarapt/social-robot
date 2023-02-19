from AriaPy import *
from ArNetworkingPy import *
import time
import logging

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO)


# if possible add a disconnection callback


def modeCallback(mode):
    print('Mode changed to {}'.format(mode))


def statusCallback(status):
    print('Status changed to '.format(status))


class ArnlClient:
    SERVER_HOST = 'localhost'
    SERVER_PORT = 7272

    GOING_TO_GOAL_STRING = "Going to point"
    GOAL_REACHED_STRING = "Arrived at point"
    CANNOT_FIND_PATH_STRING = "Failed to get to point (Cannot find path)"
    GOAL_FAILED = "Failed to get to point (Failed going to goal)"

    def __init__(self):
        Aria.init()
        self.client = ArClientBase()
        # You can change the hostname or ip address below to connect to a remove server:
        if not self.client.blockingConnect(ArnlClient.SERVER_HOST, ArnlClient.SERVER_PORT, True, 'robot-client-gui'):
            logging.error(
                "Could not connect to server at localhost port 7272, exiting")
            Aria.exit(1)
        self.client.runAsync()
        self.updates = ArClientHandlerRobotUpdate(self.client)
        self.updates.requestUpdates()
        self.getInfoCallsCount = 0
        self.getInfoAverageTime = 0

    # does not seem to be working, moreover, calling this function terminates the client, and other command also don't work
    def stopRobot(self):
        self.client.stopRunning()

    def sendPathPlanRequest(self, goalPose):
        goal_X, goal_Y, goal_heading = int(
            goalPose[0]), int(goalPose[1]), int(goalPose[2])
        logging.info(
            "[goto_position]: Path lanning to goal [%d, %d, %d].", goal_X, goal_Y, goal_heading)
        posePacket = ArNetPacket()
        posePacket.byte4ToBuf(goal_X)
        posePacket.byte4ToBuf(goal_Y)
        posePacket.byte4ToBuf(goal_heading)
        self.client.requestOnce("gotoPose", posePacket)

    def getPosition(self):
        self.updates.lock()
        pose = self.updates.getPose()
        self.updates.unlock()
        return [pose.getX(), pose.getY(), pose.getTh()]

    def getRobotInfo(self):
        getInfoStartTime = time.time()

        self.updates.lock()
        pose = self.updates.getPose()
        mode = self.updates.getMode()
        status = self.updates.getStatus()
        self.updates.unlock()

        getInfoEndTime = time.time()
        getInfoElapsedTime = getInfoEndTime - getInfoStartTime
        self.getInfoCallsCount += 1
        self.getInfoAverageTime = (self.getInfoAverageTime * (
            self.getInfoCallsCount - 1) + getInfoElapsedTime) / self.getInfoCallsCount
        logging.debug("frameCnt=%d getPose_cur=%fs, getPose_avg=%fs" % (
            self.getInfoCallsCount, getInfoElapsedTime, self.getInfoAverageTime))
        return mode, status, pose

    def close(self):
        print("Aria: Exiting...")
        Aria.exit(0)
