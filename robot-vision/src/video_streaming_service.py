#!/usr/bin/env python
from importlib import import_module
import os
import sys
import argparse
import signal
from flask import Flask, render_template, Response
from camera_zed import Camera

app = Flask(__name__)


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def signal_handler(sig, frame):
    Camera.close()
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-i", "--client-ip", help="host ip of the person location service")
    ap.add_argument("-s", "--svo", help="name of the svo file to load")
    args = vars(ap.parse_args())

    # setting the class variables allows them to persist across all class objects created in future
    if args.get('svo'):
        Camera.svoFile = args['svo']
    if args.get('client_ip'):
        PERSON_LOCATION_SERVICE_HOST = args['client_ip']
        Camera.PERSON_LOCATION_SERVICE_URL = "http://" + PERSON_LOCATION_SERVICE_HOST + ":5100/detections"

    signal.signal(signal.SIGINT, signal_handler)
    app.run(host='0.0.0.0', threaded=True)
