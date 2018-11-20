from AriaPy import *
from ArNetworkingPy import *
import sys

Aria.init()
client = ArClientBase() 
parser = ArArgumentParser(sys.argv)
clientConnector = ArClientSimpleConnector(parser)
parser.loadDefaultArguments()

if not clientConnector.parseArgs() or not parser.checkHelpAndWarnUnparsed():
    clientConnector.logOptions()
    sys.exit()

print("Connecting...\n")
if (not clientConnector.connectClient(client)):
    if (client.wasRejected()):
        print("Server rejected connection, exiting")
    else:
        print("Could not connect to server, exiting")
    sys.exit()


robotUpdates = ArClientHandlerRobotUpdate(client)
robotUpdates.requestUpdates()  # won't work without this
print("Connected to server.")
client.runAsync()

once = True
while(client.getRunningWithLock()):
    if once:
        once = False
        x_offset = 1000
        y_offset = 500
        currPose = robotUpdates.getPose()
        posePacket = ArNetPacket()
        posePacket.byte4ToBuf(int(currPose.getX()) + x_offset)
        posePacket.byte4ToBuf(int(currPose.getY()) + y_offset)
        client.requestOnce("gotoPose", posePacket)

print("Server disconnected.")
Aria.shutdown()