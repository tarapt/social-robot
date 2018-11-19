import cv2
import dlib
import numpy as np
from chessboard_detector import ChessBoardDetector
from imutils import face_utils
import argparse

def read_nums_from_file(filename):
    with open(filename) as f:
        array = [[float(x) for x in line.split()] for line in f]
        return array

class ReferenceFrameCalibrator:
    def __init__(self, depth, corners):
        self.depth = depth
        self.corner_model_coords = self.get_corner_model_coords()
        self.rotation_vec, self.translation_vec = self.calibrate(corners)
  
    # camera matrix
    K = read_nums_from_file('./calibration_data/left_camera_matrix.txt')

    # TODO distortion coefficients
    D = [0.0, 0.0, 0.0, 0.0, 0.0]

    cam_matrix = np.array(K).reshape(3, 3).astype(np.float32)
    dist_coeffs = np.array(D).reshape(5, 1).astype(np.float32)

    CHESSBOARD_SIZE = (10, 7)
    CHESSBOARD_CELL_WIDTH = 2.25 # in cm
    FIRST_CORNER_COORDS = [5.8, 5.6] # (x = horizontal offset, y = vertical offset)

    # model coordinates for the chessboard corners
    def get_corner_model_coords(self):
        x = [0, 0, 0]
        columns = self.CHESSBOARD_SIZE[0]
        rows = self.CHESSBOARD_SIZE[1]
        pts = [[x for j in range(columns)] for i in range(rows)]
        for i in range(rows):
            for j in range(columns):
                x = self.FIRST_CORNER_COORDS[0] + j * self.CHESSBOARD_CELL_WIDTH
                y = self.FIRST_CORNER_COORDS[1] + i * self.CHESSBOARD_CELL_WIDTH
                pts[i][j] = [x, y, self.depth]
        return np.array(pts).reshape(rows * columns, 3).astype(np.float32)
    
    def calibrate(self, corners_image_coords):
        _, rotation_vec, translation_vec = cv2.solvePnP(self.corner_model_coords, corners_image_coords, self.cam_matrix, self.dist_coeffs)
        return rotation_vec, translation_vec

    def change_reference_frame(self, points):
        points, _ = cv2.projectPoints(points, self.rotation_vec, self.translation_vec, self.cam_matrix,
                                    self.dist_coeffs)
        return points

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--depth", type=float, help="distance of the chessboard plane from the origin of the new frame in cm")
    args = vars(ap.parse_args())

    left = cv2.VideoCapture(0)	
    # right = cv2.VideoCapture(1)
    detector = ChessBoardDetector()
    depth = args['depth']
    while True:
        if not (left.grab()):
            print("No more frames")
            break

        _, frame = left.retrieve()
        # _, rightFrame = right.retrieve()
        
        grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hasCorners, corners = detector.find_chessboard_corners(grayImage)
        detector.draw_chessboard_corners(grayImage, hasCorners, corners)

        if hasCorners:
            m, n, o = np.shape(corners)
            corners = corners.reshape(m * n, o).astype(np.float32)
            np.savez_compressed('corners.npz', depth = depth, corners = corners)
            referenceFrameCalibrator = ReferenceFrameCalibrator(depth, corners)
            np.savetxt('corners_detected.txt', corners)
            np.savetxt('corners_model_coords.txt', referenceFrameCalibrator.corner_model_coords)
            # np.savetxt('corners_after_transform.txt', referenceFrameCalibrator.change_reference_frame(referenceFrameCalibrator.corner_model_coords))
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    left.release()
    # right.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()