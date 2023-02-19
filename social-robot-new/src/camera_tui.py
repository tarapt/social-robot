import cv2


def main():
    print("Running...")
    cam = cv2.VideoCapture(0)
    print("Opening webcam...")
    while (not cam.isOpened()):
        continue
    print("Webcam is open...")

    print_help()
    key = ''
    while key != 113:
        if cam.grab():
            _, mat = cam.retrieve()
            cv2.imshow("Webcam", mat)
            key = cv2.waitKey(5)
            settings(key, mat)
        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()
    cam.release()
    print("\nFINISH")


# def print_camera_information(cam):
    # print("Resolution: ")
    # print("Camera FPS: {0}.".format(cam.get_camera_fps()))


def print_help():
    print("Help for camera setting controls")
    print("  Detect faces in the frames:         f")
    print("  Quit:                               q\n")


def settings(key, mat):
    if key == 115:  # for 's' key
        pass
    elif key == 43:  # for '+' key
        pass
    elif key == 102:  # for 'f' key
        print("Detecting faces...")


if __name__ == "__main__":
    main()
