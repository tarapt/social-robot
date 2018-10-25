from scipy import stats

class DepthEstimator:
    def get_world_coordinates(self, leftLandmarks, rightLandmarks, leftBox):
        # left camera's optical centre coordinates in pixels
        cx = 324  
        cy = 234

        # lense focal length in pixels
        fx = 764
        fy = 760 			

        # distance in metres between the two cameras
        baseline = 0.17 	

        depths = []
        for (p1, p2) in zip(leftLandmarks, rightLandmarks):
            (x1, _) = p1
            (x2, _) = p2
            disparity = abs(x1 - x2)
            if disparity > 0:
                depth = (fx * baseline) / disparity
                depths.append(depth)

        (top, right, bottom, left) = leftBox
        u = (right + left) // 2
        v = (top + bottom) // 2
        Z = stats.tmean(depths)
        X = Z / fx * (u - cx)
        Y = Y = Z / fy * (v - cy)
        return (X, Y, Z)