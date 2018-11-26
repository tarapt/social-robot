from AriaPy import *
from ArNetworkingPy import *
from ArnlPy import *
from BaseArnlPy import *
from SonArnlPy import *
import sys

Aria.init()
Arnl.init()

parser = ArArgumentParser(sys.argv)
parser.loadDefaultArguments()

robot = ArRobot()

sonarDev = ArSonarDevice()
robot.addRangeDevice(sonarDev)

arMap = ArMap()
arMap.setIgnoreEmptyFileName(True)

pathTask = ArPathPlanningTask(robot, sonarDev, arMap)
