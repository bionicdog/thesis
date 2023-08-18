# sources:
# https://nilesh0109.medium.com/camera-image-perspective-transformation-to-different-plane-using-opencv-5e389dd56527
# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
# perspectiveTransform: https://docs.opencv.org/4.x/d2/de8/group__core__array.html#gad327659ac03e5fd6894b90025e6900a7

import cv2
# import glob

import numpy as np
# import matplotlib.pyplot as plt

class Homography:
    def __init__(self, distance_grid):
        # grid
        self.square = [30, 30] # square size in grid (in mm)
        self.grid = (8, 5) # grid size (in squares)
        self.distance_grid = distance_grid

        # criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # destination from calibration (calculated from grid and square)
        self.dstCorners = np.zeros(((self.grid[0]*self.grid[1]), 1, 2))
        for i in range(len(self.dstCorners)):
            self.dstCorners[i][0][0] = (np.floor(i/self.grid[0]) - ((self.grid[1]-1)/2)) * self.square[0] # right to left
            self.dstCorners[i][0][1] = (i % self.grid[0]) * self.square[1] # + distance_grid # top to bottom
    
    def calculateMask(self, image):
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, srcCorners = cv2.findChessboardCorners(grayImage, self.grid, None)

        # if chessboard found
        if ret == True:
            srcCorners2 = cv2.cornerSubPix(grayImage, srcCorners, (5, 5), (-1, -1), self.criteria)
            self.homographyMask, _ = cv2.findHomography(srcCorners2, self.dstCorners)

            # for test
            '''
            dstCornersCalculated = cv2.perspectiveTransform(srcCorners, self.homographyMask)
            for i in range(len(dstCornersCalculated)):
                plt.figure(0)
                plt.plot(dstCornersCalculated[i][0][0], dstCornersCalculated[i][0][1], "-o")
                plt.text(dstCornersCalculated[i][0][0], dstCornersCalculated[i][0][1], i, color="red", fontsize=12)
                averageDifference += np.sqrt(np.square(dstCornersCalculated[i][0][0] - self.dstCorners[i][0][0]) + \
                                        np.square(dstCornersCalculated[i][0][1] - self.dstCorners[i][0][1]))
                averageXDifference += np.abs(dstCornersCalculated[i][0][0] - self.dstCorners[i][0][0])
                averageYDifference += np.abs(dstCornersCalculated[i][0][1] - self.dstCorners[i][0][1])
            averageDifference /= len(dstCornersCalculated)
            averageXDifference /= len(dstCornersCalculated)
            averageYDifference /= len(dstCornersCalculated)
            print("average difference to desired outcome")
            print("global:", averageDifference, "mm")
            print("x-coör:", averageXDifference, "mm")
            print("y-coör:", averageYDifference, "mm")
            plt.show()
            '''
            return self.homographyMask

        return None
    
    def perspectiveTransform(self, pixel_coordinates, mask=None):
        if mask == None:
            mask = self.homographyMask
        world_coordinates = cv2.perspectiveTransform(pixel_coordinates, mask)
        for index in range(len(world_coordinates)):
            world_coordinates[index][0] = [world_coordinates[index][0][0]*3/4,
                                           world_coordinates[index][0][1]*3/4 + self.distance_grid]
        return world_coordinates