import cv2

class ChessBoardDetector:
    CHESSBOARD_SIZE = (10, 7)
    TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30,
            0.001)

    def find_chessboard_corners(self, gray_image):
        hasCorners, corners = cv2.findChessboardCorners(gray_image,
                self.CHESSBOARD_SIZE, cv2.CALIB_CB_FAST_CHECK)
        return hasCorners, corners

    def draw_chessboard_corners(self, gray_image, hasCorners, corners):
        if hasCorners:
            cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1),
                    self.TERMINATION_CRITERIA)

        cv2.drawChessboardCorners(gray_image, self.CHESSBOARD_SIZE, corners, hasCorners)
        cv2.imshow('chessboard', gray_image)

        # Needed to draw the window
        cv2.waitKey(1)
