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
parser.add_argument("--image_dir", default="/home/tara/Project/openpose/examples/media/", help="Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).")
parser.add_argument("--no_display", default=False, help="Enable to disable the visual display.")
args = parser.parse_known_args()

# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "/home/tara/Project/openpose/models/"

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

# Read frames on directory
imagePaths = op.get_images_on_directory(args[0].image_dir);

# Read number of GPUs in your system
numberGPUs = op.get_gpu_number()
start = time.time()

# Process and display images
for imageBaseId in range(0, len(imagePaths), numberGPUs):

    # Create datums
    datums = []

    # Read and push images into OpenPose wrapper
    for gpuId in range(0, numberGPUs):

        imageId = imageBaseId+gpuId
        if imageId < len(imagePaths):

            imagePath = imagePaths[imageBaseId+gpuId]
            datum = op.Datum()
            imageToProcess = cv2.imread(imagePath)
            datum.cvInputData = imageToProcess
            datums.append(datum)
            opWrapper.waitAndEmplace([datums[-1]])

    # Retrieve processed results from OpenPose wrapper
    for gpuId in range(0, numberGPUs):

        imageId = imageBaseId+gpuId
        if imageId < len(imagePaths):

            datum = datums[gpuId]
            opWrapper.waitAndPop([datum])

            print("Body keypoints: \n" + str(datum.poseKeypoints))

            if not args[0].no_display:
                cv2.imshow("OpenPose 1.4.0 - Tutorial Python API", datum.cvOutputData)
                key = cv2.waitKey(15)
                if key == 27: break

end = time.time()
print("OpenPose demo successfully finished. Total time: " + str(end - start) + " seconds")
