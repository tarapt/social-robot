import cv2
import os
import argparse
from os import listdir
from os.path import isfile, join

ap = argparse.ArgumentParser()
ap.add_argument("-n", "--name", required=True, help="name of the person")
args = vars(ap.parse_args())

person = args['name']

cap = cv2.VideoCapture(0)

path = "../face_dataset/" + person
if not os.path.exists(path):
    os.makedirs(path)

IMAGE_PATH = path + "/{:06d}.jpg"

frameId = 0
existing_photos = [f for f in listdir(path) if isfile(join(path, f))]
if len(existing_photos) > 0:
    existing_photos.sort(reverse=True)
    frameId = int(existing_photos[0][:6]) + 1

while(True):
    if not (cap.grab()):
        print("No more frames")
        break

    _, frame = cap.retrieve()

    cv2.imshow('Camera', frame)

    key = cv2.waitKey(100) & 0xFF
    if key == ord('s'):
        cv2.imwrite(IMAGE_PATH.format(frameId), frame)
        frameId += 1
        print("Captured frame no. {}".format(frameId))
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()