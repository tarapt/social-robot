import sys
import numpy as np
import cv2

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

REMAP_INTERPOLATION = cv2.INTER_LINEAR

if len(sys.argv) != 2:
    print("Syntax: {0} CALIBRATION_FILE".format(sys.argv[0]))
    sys.exit(1)

calibration = np.load(sys.argv[1], allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"]) 
rightMapX = calibration["rightMapX"]
rightMapY = calibration["rightMapY"]
rightROI = tuple(calibration["rightROI"])
dispartityToDepthMap = calibration["dispartityToDepthMap"]

# leftROI = (10, 10, imageSize[0] - 10, imageSize[1] - 10)
# rightROI = (10, 10, imageSize[0] - 10, imageSize[1] - 10)
# print("Left ROI", leftROI)
# print("Right ROI", rightROI)
print("dispartityToDepthMap = \n", dispartityToDepthMap)

left = cv2.VideoCapture(1)
right = cv2.VideoCapture(2)

# increase the resolution
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# # Increase the resolution
# CAMERA_WIDTH = 1280
# CAMERA_HEIGHT = 720
# left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
# left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
# right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
# right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# # The distortion in the left and right edges prevents a good calibration, so
# # discard the edges
# CROP_WIDTH = 960
# def cropHorizontal(image):
#     return image[:,
#             int((CAMERA_WIDTH-CROP_WIDTH)/2):
#             int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

MIN_DISPARITY = 16
NUM_DISPARITY = 112-MIN_DISPARITY
def getStereoBMObject():
    stereoMatcher = cv2.StereoBM_create()
    stereoMatcher.setMinDisparity(MIN_DISPARITY)
    stereoMatcher.setNumDisparities(NUM_DISPARITY)
    stereoMatcher.setBlockSize(16)
    stereoMatcher.setROI1(leftROI)
    stereoMatcher.setROI2(rightROI)
    stereoMatcher.setSpeckleRange(32)
    stereoMatcher.setSpeckleWindowSize(100)
    return stereoMatcher

def getStereoSGBMObject():
    window_size = 3
    stereo = cv2.StereoSGBM_create(minDisparity = MIN_DISPARITY,
        numDisparities = NUM_DISPARITY,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )
    return stereo

stereoMatcher = getStereoSGBMObject()

# Grab both frames first, then retrieve to minimize latency between cameras
count = 0
while(True):
    if not left.grab() or not right.grab():
        print("No more frames")
        break
    count += 1

    _, leftFrame = left.retrieve()
    # leftFrame = cropHorizontal(leftFrame)
    leftHeight, leftWidth = leftFrame.shape[:2]
    _, rightFrame = right.retrieve()
    # rightFrame = cropHorizontal(rightFrame)
    rightHeight, rightWidth = rightFrame.shape[:2]

    if (leftWidth, leftHeight) != imageSize:
        print("Left camera has different size than the calibration data")
        break

    if (rightWidth, rightHeight) != imageSize:
        print("Right camera has different size than the calibration data")
        break

    fixedLeft = cv2.remap(leftFrame, leftMapX, leftMapY, REMAP_INTERPOLATION)
    fixedRight = cv2.remap(rightFrame, rightMapX, rightMapY, REMAP_INTERPOLATION)
    
    # # with no calibration
    # fixedLeft = leftFrame
    # fixedRight = rightFrame

    # # Print ROI
    # cv2.rectangle(fixedLeft, leftROI[:2], leftROI[2:], (0,0,255), 2)
    # cv2.rectangle(fixedRight, rightROI[:2], rightROI[2:], (0,0,255), 2)
    cv2.imshow('undistorted left', fixedLeft)
    cv2.imshow('undistorted right', fixedRight)
    
    # cv2.imshow('original left', fixedLeft)
    # cv2.imshow('original right', fixedRight)

    grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
    grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)
    
    disparity = stereoMatcher.compute(grayLeft, grayRight).astype(np.float32) / 16.0
    cv2.imshow('disparity', (disparity-MIN_DISPARITY)/NUM_DISPARITY)

    # h, w = fixedLeft.shape[:2]
    # f = 760 # pixels
    # baseline = 0.10 # meters
    # Q = np.float32([[1, 0, 0,  -0.5*w],
    #                 [0, 1, 0,  -0.5*h],
    #                 [0, 0, 0,     f], 
    #                 [0, 0, -1.0/baseline,      0]])
    # points = cv2.reprojectImageTo3D(disparity, dispartityToDepthMap)
    # colors = fixedLeft
    # mask = disparity > disparity.min()
    # out_points = points[mask]
    # out_colors = colors[mask]
    # write_ply('3D_images/out{:06d}.ply'.format(count), out_points, out_colors)

    # depth = stereoMatcher.compute(grayLeft, grayRight)
    # norm_coeff = 255 / depth.max()
    # cv2.imshow("depth", depth * norm_coeff / 255)
    # DEPTH_VISUALIZATION_SCALE = 1024
    # cv2.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

left.release()
right.release()
cv2.destroyAllWindows()