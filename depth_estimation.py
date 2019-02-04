import numpy as np

class DepthEstimator:
    def __init__(self, confidenceThreshold):
        self.confidenceThreshold = confidenceThreshold
    
    def get_world_coordinates(self, leftPoints, rightPoints):
        # left camera's optical centre coordinates in pixels
        cx = 685
        cy = 410

        # lense focal length in pixels
        fx = 1458
        fy = 1462 			

        # distance in metres between the two cameras
        baseline = 0.15 	

        positions = []
        for (p1, p2) in zip(leftPoints, rightPoints):
            (x1, y1, c1) = p1
            (x2, y2, c2) = p2
            if c1 > confidenceThreshold and c2 > confidenceThreshold:
                disparity = abs(x1 - x2)
                if disparity > 0:
                    z = (fx * baseline) / disparity
                    x1 = z / fx * (x1 - cx)
                    y1 = z / fy * (y1 - cy)

                    x2 = z / fx * (x2 - cx)
                    y2 = z / fy * (y2 - cy)

                    x = (x1 + x2)/2.0
                    y = (y1 + y2)/2.0
                    positions.append([x, y, z])
        positions = np.array(positions)
        return positions