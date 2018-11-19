import cv2
import dlib
import numpy as np
import math
from imutils import face_utils

face_landmark_path = './trained_models/shape_predictor_68_face_landmarks.dat'
def read_nums_from_file(filename):
    with open(filename) as f:
        array = [[float(x) for x in line.split()] for line in f]
        return array

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])

class HeadPoseEstimator:
    # camera matrix
    K = read_nums_from_file('./calibration_data/left_camera_matrix.txt')

    # TODO distortion coefficients
    D = [0.0, 0.0, 0.0, 0.0, 0.0]

    cam_matrix = np.array(K).reshape(3, 3).astype(np.float32)
    dist_coeffs = np.array(D).reshape(5, 1).astype(np.float32)

    # model coordinates for the selected facial landmarks
    object_pts = np.float32([[6.825897, 6.760612, 4.402142],
                            [1.330353, 7.122144, 6.903745],
                            [-1.330353, 7.122144, 6.903745],
                            [-6.825897, 6.760612, 4.402142],
                            [5.311432, 5.485328, 3.987654],
                            [1.789930, 5.393625, 4.413414],
                            [-1.789930, 5.393625, 4.413414],
                            [-5.311432, 5.485328, 3.987654],
                            [2.005628, 1.409845, 6.165652],
                            [-2.005628, 1.409845, 6.165652],
                            [2.774015, -2.080775, 5.048531],
                            [-2.774015, -2.080775, 5.048531],
                            [0.000000, -3.116408, 6.097667],
                            [0.000000, -7.415691, 4.070434]])

    # model coordinates for a line starting from the face
    line_model_coords = np.float32([[20.0, 20.0, 20.0],
                            [0.0, 0.0, 0.0]])

    # model coordinates for the cube around the face
    cube_model_coords = np.float32([[10.0, 10.0, 10.0],
                            [10.0, 10.0, -10.0],
                            [10.0, -10.0, -10.0],
                            [10.0, -10.0, 10.0],
                            [-10.0, 10.0, 10.0],
                            [-10.0, 10.0, -10.0],
                            [-10.0, -10.0, -10.0],
                            [-10.0, -10.0, 10.0]])

    # a line consists of a pair of points, which are obtained from the cube_model/cube_image_coords arrays, by their indices
    # line [0, 1] is the line connecting [10.0, 10.0, 10.0] and [10.0, 10.0, -10.0]
    line_pairs = [[0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]]

    def get_head_pose(self, shape):
        # image coordinates for the selected facial landmarks
        image_pts = np.float32([shape[17], shape[21], shape[22], shape[26], shape[36],
                                shape[39], shape[42], shape[45], shape[31], shape[35],
                                shape[48], shape[54], shape[57], shape[8]])

        _, rotation_vec, translation_vec = cv2.solvePnP(self.object_pts, image_pts, self.cam_matrix, self.dist_coeffs)

        cube_image_coords, _ = cv2.projectPoints(self.cube_model_coords, rotation_vec, translation_vec, self.cam_matrix,
                                            self.dist_coeffs)
        cube_image_coords = tuple(map(tuple, cube_image_coords.reshape(8, 2)))

        # line_image_coords, _ = cv2.projectPoints(self.line_model_coords, rotation_vec, translation_vec, self.cam_matrix,
        #                                     self.dist_coeffs)
        # line_image_coords = tuple(map(tuple, line_image_coords.reshape(2, 2)))

        # calculate euler angle
        rotation_mat, _ = cv2.Rodrigues(rotation_vec)
        # pose_mat = cv2.hconcat((rotation_mat, translation_vec))
        # _, _, _, _, _, _, euler_angle = cv2.decomposeProjectionMatrix(pose_mat)

        return cube_image_coords, rotationMatrixToEulerAngles(rotation_mat)

    def draw_head_pose(self, cube_image_coords, euler_angle, frame):
        for start, end in self.line_pairs:
            cv2.line(frame, cube_image_coords[start], cube_image_coords[end], (0, 0, 255))
        # cv2.line(frame, line_image_coords[0], line_image_coords[0], (0, 0, 0))

        cv2.putText(frame, "X: " + "{:7.2f}".format(euler_angle[0]) + " degrees", (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 0), thickness=2)
        cv2.putText(frame, "Y: " + "{:7.2f}".format(euler_angle[1]) + " degrees", (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 0), thickness=2)
        cv2.putText(frame, "Z: " + "{:7.2f}".format(euler_angle[2]) + " degrees", (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 0), thickness=2)
        return frame

def main():
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(face_landmark_path)

    left = cv2.VideoCapture(0)	
    # right = cv2.VideoCapture(1)
    while True:
        if not (left.grab()):
            print("No more frames")
            break

        _, frame = left.retrieve()
        # _, rightFrame = right.retrieve()
        
        face_rects = detector(frame, 0)

        if len(face_rects) > 0:
            shape = predictor(frame, face_rects[0])
            shape = face_utils.shape_to_np(shape)
            pose_estimator = HeadPoseEstimator()
            cube_image_coords, euler_angle = pose_estimator.get_head_pose(shape)
            frame = pose_estimator.draw_head_pose(cube_image_coords, euler_angle, frame)

        cv2.imshow("demo", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    left.release()
    # right.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()