import cv2

# # The distortion in the left and right edges prevents a good calibration, so
# # discard the edges
# CROP_WIDTH = 960
# def cropHorizontal(image):
#     return image[:,
#             int((CAMERA_WIDTH-CROP_WIDTH)/2):
#             int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

left = cv2.VideoCapture(1)
right = cv2.VideoCapture(2)

# increase the resolution
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960
# left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
# left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
# right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
# right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Different directories for each camera
LEFT_PATH = "capture/left/{:06d}.jpg"
RIGHT_PATH = "capture/right/{:06d}.jpg"

# Filenames are just an increasing number
frameId = 0

# Capture loop from earlier...
while(True):
    if not (left.grab() and right.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    # leftFrame = cropHorizontal(leftFrame)
    _, rightFrame = right.retrieve()
    # rightFrame = cropHorizontal(rightFrame)

    cv2.imshow('left', leftFrame)
    cv2.imshow('right', rightFrame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Save the frames
    # cv2.imwrite(LEFT_PATH.format(frameId), leftFrame)
    # cv2.imwrite(RIGHT_PATH.format(frameId), rightFrame)
    frameId += 1

left.release()
right.release()
cv2.destroyAllWindows()