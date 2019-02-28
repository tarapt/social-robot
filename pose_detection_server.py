# Do the videocapture and pose detection api call in another thread, since the api call may take time
# save the image, and the positions for at most 100 images, use a queue with fixed size
# Refer: https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/ 

import os
import cv2
from openpose import pyopenpose as op
from flask import Flask, jsonify

POSE_DETECTION_PATH = "static_data/current.jpg"

static_file_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'static_data')
app = Flask(__name__)

@app.route('/')
def status():
    return 'Pose Detection Server Running...'

# @app.route('/get_pose/', defaults={'path': ''})
# @app.route('/get_pose/<string:image_name>', methods=['GET'])
@app.route('/get_pose')
def get_pose():
    # pathToImage = os.path.join(static_file_dir, image_name)
    pathToImage = POSE_DETECTION_PATH
    if os.path.isfile(pathToImage):
        pose = jsonify(calculate_pose(cv2.imread(pathToImage)))
        return pose
    return jsonify('')

# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "/home/tara/Project/openpose/models/"
params["face"] = True
params["hand"] = True

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

def calculate_pose(imageToProcess):
    # Process Image
    datum = op.Datum()
    datum.cvInputData = imageToProcess
    opWrapper.emplaceAndPop([datum])

    # Display Image
    # print("Body keypoints: \n" + str(datum.poseKeypoints))
    # print("Face keypoints: \n" + str(datum.faceKeypoints))
    # print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
    # print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
    # cv2.imshow("Pose Detection Server", datum.cvOutputData)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    cv2.imwrite(POSE_DETECTION_PATH, datum.cvOutputData)
    return {'body': datum.poseKeypoints.tolist(), 'face': datum.faceKeypoints.tolist(), 'left_hand': datum.handKeypoints[0].tolist(), 'right_hand': datum.handKeypoints[1].tolist()}