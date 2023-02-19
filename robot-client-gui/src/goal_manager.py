import numpy as np
from PIL import Image
import re
import math
import collections
import os.path
import logging
from scipy.spatial import distance

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.DEBUG)

# 0.5 meters, distance the robot should maintain with person
SOCIALLY_ACCEPTABLE_DISTANCE = 500
PIONEERLX_RADIUS = 350  # in mm
EFFECTIVE_SOCIAL_RADIUS = SOCIALLY_ACCEPTABLE_DISTANCE + PIONEERLX_RADIUS
# FRONT_PADDING_AT_SLOW_SPEED = 100 and SIDE_CLEARANCE_AT_SLOW_SPEED = 100 in arnl.p
PIONEERLX_PADDING = 100  # in mm

# Note that sampling angle should be less than asin(PIONEERLX_PADDING/(PIONEERLX_RADIUS + PIONEERLX_PADDING))
# For padding = 100 mm, angle should be < 12.84 degrees
SAMPLING_ANGLE_FOR_COLLISION_CHECKING = 5  # in degrees
SAMPLING_ANGLE_FOR_SAFE_SPOT_SEARCH = 10  # in degrees

GRID_FILE_NAME = '../data/robita_lab_map.npy'
GRID_IMAGE_FILE_NAME = '../data/forbidden_areas.jpg'
GRID_IMAGE_COMPRESSION_RATIO = 0.1
FORBIDDEN_AREAS_MAP_FILE_NAME = '../data/Robita_lab_forbidden.map'

Z_OFFSET_TO_BE_ADDED = 150  # in mm, position of the camera w.r.t the robot


class GridManager:
    def __init__(self):
        self.readForbiddenAreas()
        BOUNDARY = self.findBoundary()
        self.X_RANGE = BOUNDARY[2] - BOUNDARY[0] + 1
        self.Y_RANGE = BOUNDARY[3] - BOUNDARY[1] + 1
        self.NEW_ORIGIN = (BOUNDARY[0], BOUNDARY[1])
        logging.info("New origin is {}.".format(self.NEW_ORIGIN))

        if os.path.isfile(GRID_FILE_NAME):
            logging.info("File found. Loading from disk...")
            self.GRID = np.load(GRID_FILE_NAME)
        else:
            logging.info("No grid file found. Generating...")
            self.GRID = np.ones(
                (self.X_RANGE, self.Y_RANGE), dtype=bool)
            self.markForbiddenAreas()
            self.saveGrid()
            self.saveGridImage()

    def saveGrid(self):
        np.save(GRID_FILE_NAME, self.GRID)
        logging.info("Grid file saved to disk.")

    def saveDataToImage(self, data, filename):
        def booleanToImage(data):
            size = data.shape[::-1]
            databytes = np.packbits(data, axis=1)
            return Image.frombytes(mode='1', size=size, data=databytes)

        im = booleanToImage(data)
        im.thumbnail([int(GRID_IMAGE_COMPRESSION_RATIO * s)
                      for s in im.size], Image.ANTIALIAS)
        im.save(filename, "JPEG")

    def saveGridImage(self):
        def booleanToImage(data):
            size = data.shape[::-1]
            databytes = np.packbits(data, axis=1)
            return Image.frombytes(mode='1', size=size, data=databytes)

        im = booleanToImage(self.GRID)
        im.thumbnail([int(GRID_IMAGE_COMPRESSION_RATIO * s)
                      for s in im.size], Image.ANTIALIAS)
        im.save(GRID_IMAGE_FILE_NAME, "JPEG")
        logging.info("Grid image file saved to disk.")

    def readForbiddenAreas(self):
        if not os.path.isfile(FORBIDDEN_AREAS_MAP_FILE_NAME):
            logging.error(FORBIDDEN_AREAS_MAP_FILE_NAME + " does not exist.")
            raise RuntimeError(
                FORBIDDEN_AREAS_MAP_FILE_NAME + " does not exist.")

        self.FORBIDDEN_AREAS = []
        # Make sure the forbidden areas have 0 degrees orientation
        # In such a case, the first point is the bottom-left and second is the top-right point of the area
        with open(FORBIDDEN_AREAS_MAP_FILE_NAME) as f:
            # lines = f.read().splitlines()
            lines = f.read()  # includes new lines also
            FORBIDDEN_AREA_REGEX = 'Cairn: ForbiddenArea.*'
            forbiddenAreasLines = re.findall(FORBIDDEN_AREA_REGEX, lines)
            for line in forbiddenAreasLines:
                x = line[42:]
                integerRegex = '([-]?\d+)'
                whitespace = '\s'
                coordinatesRegex = (whitespace + integerRegex) * 4
                forbiddenArea = re.findall(coordinatesRegex, x)
                forbiddenArea = list(map(int, forbiddenArea[0]))
                self.FORBIDDEN_AREAS.append(forbiddenArea)

    def findBoundary(self):
        minX = 1000000
        minY = 1000000
        maxX = -1000000
        maxY = -1000000
        for area in self.FORBIDDEN_AREAS:
            minX = min(minX, area[0])
            minY = min(minY, area[1])
            maxX = max(maxX, area[2])
            maxY = max(maxY, area[3])
        return [minX, minY, maxX, maxY]

    def fillWithObstacles(self, source):
        dr = [-1, 0, 1, 0]
        dc = [0, 1, 0, -1]
        self.GRID[source[0]][source[1]] = 0
        queue = collections.deque([source])
        while queue:
            cell = queue.popleft()
            for i in range(4):
                row = cell[0] + dr[i]
                col = cell[1] + dc[i]
                if self.isWithinBounds(row, col) and self.GRID[row][col] == 1:
                    self.GRID[row][col] = 0
                    queue.append((row, col))

    def fillBoundaryWithObstacles(self):
        for i in range(self.GRID.shape[1]):
            if (self.GRID[0][i] == 1):
                self.fillWithObstacles((0, i))
            if (self.GRID[self.GRID.shape[0] - 1][i] == 1):
                self.fillWithObstacles((self.GRID.shape[0] - 1, i))
        for i in range(self.GRID.shape[0]):
            if (self.GRID[i][0] == 1):
                self.fillWithObstacles((i, 0))
            if (self.GRID[i][self.GRID.shape[1] - 1] == 1):
                self.fillWithObstacles((i, self.GRID.shape[1] - 1))

    def isWithinBounds(self, x, y):
        return (x >= 0 and y >= 0 and x < self.GRID.shape[0] and y < self.GRID.shape[1])

    def worldCoordinateToGridCoordinates(self, pointInWorldCoordinates):
        return (pointInWorldCoordinates[0] - self.NEW_ORIGIN[0], pointInWorldCoordinates[1] - self.NEW_ORIGIN[1])

    def checkGridValue(self, pointInWorldCoordinates):
        x, y = self.worldCoordinateToGridCoordinates(pointInWorldCoordinates)
        if self.isWithinBounds(x, y):
            return self.GRID[x][y]
        return False

    def markForbiddenAreas(self):
        # GRID value - True: Free space, False: Obstacle
        for area in self.FORBIDDEN_AREAS:
            bottomLeft = self.worldCoordinateToGridCoordinates(
                (area[0], area[1]))
            topRight = self.worldCoordinateToGridCoordinates(
                (area[2], area[3]))
            # its guranteed that these coordinates are withing the max range
            self.GRID[bottomLeft[0]:topRight[0] + 1,
                      bottomLeft[1]:topRight[1] + 1] = False
        self.fillBoundaryWithObstacles()

    def getNeighbourhoodPoints(self, pointInWorldCoordinates, halfWidth):
        x, y = self.worldCoordinateToGridCoordinates(pointInWorldCoordinates)
        if self.isWithinBounds(x, y) and self.isWithinBounds(x - halfWidth, y - halfWidth) and self.isWithinBounds(x + halfWidth, y + halfWidth):
            return self.GRID[x - halfWidth: x + halfWidth, y - halfWidth: y + halfWidth]
        return None

    def saveNeighbourhoodImage(self, point, halfWidth, filename):
        data = self.getNeighbourhoodPoints(point, halfWidth)
        if data is not None:
            self.saveDataToImage(data, filename)
            logging.info("Neighbourhood preview image around {} saved to disk.".format(point))
            return True
        return False

    def isRobotSafeToPlace(self, point):
        # The following approach (checking for intersection with some sampled points from the circumference) is safe
        # assuming all the obstacles have right angled corners. If the assumption doesn't hold, check the whole area
        # of the robot for intersection, not just the sampled points on the circumference.
        ROBOT_RADIUS = PIONEERLX_RADIUS + PIONEERLX_PADDING
        for theta in range(0, 360, SAMPLING_ANGLE_FOR_COLLISION_CHECKING):
            theta = math.radians(theta)
            x = int(round(point[0] + ROBOT_RADIUS * math.cos(theta)))
            y = int(round(point[1] + ROBOT_RADIUS * math.sin(theta)))
            logging.debug("isRobotSafeToPlace(): theta = {} - x={}, y={}, safe = {}".format(
                int(math.degrees(theta)), x, y, self.checkGridValue((x, y))))
            if not self.checkGridValue((x, y)):
                return False
        return True


class GoalManager:
    # All the distance units are expected to be in mm
    def __init__(self):
        self.gridManager = GridManager()

    def getPositionInWorldCoordinates(self,
                                      currentRobotPose, personPositionInRobotCoordinates):
        x1, y1, theta1 = currentRobotPose[0], currentRobotPose[1], currentRobotPose[2]
        x2, z2 = personPositionInRobotCoordinates[0], (personPositionInRobotCoordinates[2] + Z_OFFSET_TO_BE_ADDED)

        r = math.hypot(x2, z2)
        theta1 = math.radians(theta1)
        theta2 = math.atan2(z2, x2)

        x = x1 + r * math.sin(theta1 + theta2)
        y = y1 - r * math.cos(theta1 + theta2)
        return int(x), int(y)

    def getHeadingInDegrees(self, personPositionInWorldCoordinates, tentativeRobotPosition):
        delX = personPositionInWorldCoordinates[0] - tentativeRobotPosition[0]
        delY = personPositionInWorldCoordinates[1] - tentativeRobotPosition[1]
        return math.degrees(math.atan2(delY, delY))

    def findSafeSpots(self, personPosition):
        safeSpots = []
        trialCnt = 0
        while len(safeSpots) == 0 and trialCnt < 3:
            for theta in range(0, 360, SAMPLING_ANGLE_FOR_SAFE_SPOT_SEARCH):
                theta = math.radians(theta)
                x = int(
                    round(personPosition[0] + trialCnt * SOCIALLY_ACCEPTABLE_DISTANCE + EFFECTIVE_SOCIAL_RADIUS * math.cos(theta)))
                y = int(
                    round(personPosition[1] + trialCnt * SOCIALLY_ACCEPTABLE_DISTANCE + EFFECTIVE_SOCIAL_RADIUS * math.sin(theta)))
                logging.debug("findSafeSpots(): theta = {} - x={}, y={}, safe = {}".format(
                    int(math.degrees(theta)), x, y, self.gridManager.isRobotSafeToPlace((x, y))))
                if self.gridManager.isRobotSafeToPlace((x, y)):
                    safeSpots.append((x, y))
            trialCnt += 1
        return safeSpots

    def findNearestSafeSpot(self, personPositionInWorldCoordinates, safeSpots=None):
        if safeSpots is None:
            safeSpots = self.findSafeSpots(personPositionInWorldCoordinates)
        if len(safeSpots) == 0:
            return None
        distances = distance.cdist(
            [personPositionInWorldCoordinates], safeSpots, 'euclidean')
        return safeSpots[np.argmin(distances)]