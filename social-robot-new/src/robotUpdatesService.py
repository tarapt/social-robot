from flask import Flask, jsonify, request
from threading import Lock

app = Flask(__name__)

positionLock = Lock()
position = None

@app.route('/')
def status():
    return 'Robot Updates Service Running...'

@app.route('/robot_position', methods=['GET', 'POST'])
def robot_position():
    global position
    ret = jsonify('')
    positionLock.acquire()
    if request.method == 'GET':
        ret = position
    else:
        position = request.form
    positionLock.release()
    return ret

app.run(host='127.0.0.1', port= 5000)