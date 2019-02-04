# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform
import argparse
import time
from openpose import pyopenpose as op

# Flags
parser = argparse.ArgumentParser()
parser.add_argument("--image_path", default="/home/tara/Project/openpose/examples/media/COCO_val2014_000000000241.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
args = parser.parse_known_args()

# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "/home/tara/Project/openpose/models/"
params["face"] = True
params["face_detector"] = 1
params["body_disable"] = True

# Add others in path?
for i in range(0, len(args[1])):
    curr_item = args[1][i]
    if i != len(args[1])-1: next_item = args[1][i+1]
    else: next_item = "1"
    if "--" in curr_item and "--" in next_item:
        key = curr_item.replace('-','')
        if key not in params:  params[key] = "1"
    elif "--" in curr_item and "--" not in next_item:
        key = curr_item.replace('-','')
        if key not in params: params[key] = next_item

# Construct it from system arguments
# op.init_argv(args[1])
# oppython = op.OpenposePython()

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

# Read image and face rectangle locations
imageToProcess = cv2.imread(args[0].image_path)
faceRectangles = [
    op.Rectangle(330.119385, 277.532715, 48.717274, 48.717274),
    op.Rectangle(24.036991, 267.918793, 65.175171, 65.175171),
    op.Rectangle(151.803436, 32.477852, 108.295761, 108.295761),
]

# Create new datum
datum = op.Datum()
datum.cvInputData = imageToProcess
datum.faceRectangles = faceRectangles

# Process and display image
opWrapper.emplaceAndPop([datum])
# print("Face keypoints: \n" + str(datum.faceKeypoints))
cv2.imshow("OpenPose 1.4.0 - Tutorial Python API", datum.cvOutputData)
cv2.waitKey(0)
