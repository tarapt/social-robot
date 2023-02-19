from flask import Flask, jsonify, request
from threading import Lock

app = Flask(__name__)

detectedPersonsLock = Lock()
detectedPersons = {}


@app.route('/')
def status():
    return 'Person Location Service Running...'


@app.route('/detections', methods=['GET', 'POST'])
def detections():
    global detectedPersons
    ret = jsonify('')
    detectedPersonsLock.acquire()
    if request.method == 'GET':
        ret = jsonify(detectedPersons)
    else:
        detectedPersons = request.form
    detectedPersonsLock.release()
    return ret


app.run(host='127.0.0.1', port=5100)
