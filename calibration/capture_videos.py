import cv2

left = cv2.VideoCapture(0)
right = cv2.VideoCapture(1)

# Different directories for each camera
LEFT_PATH = "capture/left/{:06d}.png"
RIGHT_PATH = "capture/right/{:06d}.png"

# Filenames are just an increasing number
frameId = 0

while(True):
    if not (left.grab() and right.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    _, rightFrame = right.retrieve()

    cv2.imshow('left', leftFrame)
    cv2.imshow('right', rightFrame)

    k = cv2.waitKey(100) # 10 FPS

    if k == ord('q'):
        break
    
    if frameId % 50 == 0 and frameId != 0: # every 5 seconds
        cv2.imwrite(LEFT_PATH.format(frameId//50), leftFrame)
        cv2.imwrite(RIGHT_PATH.format(frameId//50), rightFrame)
        print("Captured frame - {}".format(frameId//50))        

    frameId += 1

left.release()
right.release()
cv2.destroyAllWindows()