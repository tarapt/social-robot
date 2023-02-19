from flask import Flask, jsonify, request
from threading import Lock
import numpy as np
import logging

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO)

app = Flask(__name__)

detectedPersonsLock = Lock()
detectedPersons = {}


def parse_detections(data):
    global detectedPersons
    frameId = data['frameId']
    robotPose = data['robotPose']
    locations = data['locations']
    names = data['names']
    for name, location in zip(names, locations):
        detectedPersons[name] = (robotPose, location, frameId)


@app.route('/')
def status():
    return 'Person Location Service Running...'


@app.route('/detections', methods=['GET', 'POST'])
def detections():
    global detectedPersons
    ret = jsonify('')
    detectedPersonsLock.acquire()
    if request.method == 'GET':
        data = {'locations': list(detectedPersons.values()), 'names': list(
            detectedPersons.keys())}
        ret = jsonify(data)
    else:
        parse_detections(request.json)
        ret = "Success"
    detectedPersonsLock.release()
    return ret


app.run(host='0.0.0.0', port=5100)
