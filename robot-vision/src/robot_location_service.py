from flask import Flask, jsonify
import numpy as np
import logging
import argparse
import signal
import time
from AriaPy import *
from ArNetworkingPy import *

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO)

global arnlClient
app = Flask(__name__)


class ArnlClient:
    SERVER_HOST = 'localhost'
    SERVER_PORT = 7272

    def __init__(self):
        Aria.init()
        self.client = ArClientBase()
        # You can change the hostname or ip address below to connect to a remove server:
        if not self.client.blockingConnect(ArnlClient.SERVER_HOST, ArnlClient.SERVER_PORT, True, 'robot-vision'):
            logging.error(
                "Could not connect to server at localhost port 7272, exiting")
            Aria.exit(1)

        self.client.runAsync()
        self.updates = ArClientHandlerRobotUpdate(self.client)
        self.updates.requestUpdates()
        self.getPoseCallsCount = 0
        self.getPoseAverageTime = 0

    def getPose(self):
        getPoseStartTime = time.time()

        self.updates.lock()
        pose = self.updates.getPose()
        self.updates.unlock()

        getPoseEndTime = time.time()
        getPoseElapsedTime = getPoseEndTime - getPoseStartTime

        self.getPoseCallsCount += 1
        self.getPoseAverageTime = (self.getPoseAverageTime * (
            self.getPoseCallsCount - 1) + getPoseElapsedTime) / self.getPoseCallsCount
        logging.debug("frameCnt=%d getPose_cur=%fs, getPose_avg=%fs" % (
            self.getPoseCallsCount, getPoseElapsedTime, self.getPoseAverageTime))
        return [pose.getX(), pose.getY(), pose.getTh()]

    def close(self):
        logging.info("Aria: Exiting...")
        Aria.exit(0)


def signal_handler(sig, frame):
    arnlClient.close()


@app.route('/')
def status():
    return 'Robot Location Service Running...'


@app.route('/position', methods=['GET'])
def detections():
    data = {'robotPose': arnlClient.getPose()}
    logging.info(data)
    return jsonify(data)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--arnl_ip",
                    help="host ip of the arnl server")
    args = vars(ap.parse_args())

    # setting the class variables allows them to persist across all class objects created in future
    if args.get('arnl_ip'):
        ArnlClient.SERVER_HOST = args['arnl_ip']

    global arnlClient
    arnlClient = ArnlClient()

    app.run(host='0.0.0.0', port=5200)
