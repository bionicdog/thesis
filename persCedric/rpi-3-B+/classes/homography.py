# sources:
# https://nilesh0109.medium.com/camera-image-perspective-transformation-to-different-plane-using-opencv-5e389dd56527
# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
# perspectiveTransform: https://docs.opencv.org/4.x/d2/de8/group__core__array.html#gad327659ac03e5fd6894b90025e6900a7

import cv2
import glob

import numpy as np
import matplotlib.pyplot as plt

# grid size used
grid = [25, 25] # in mm


#termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((9*6, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2)

#arrays to store object points and image points from all the images
objpoints = [] #3d points in real world space
imgpoints = [] #2d points in image plane


class Homography:
    def __init__(self):
        # grid
        self.square = [30, 30] # square size in grid (in mm)
        self.grid = (9, 6) # grid size (in squares)

        # criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # destination from calibration (calculated from grid and square)
        self.dstCorners = np.zeros(((self.square[0]*self.square[1]), 1, 2))
        for i in range(len(self.dstCorners)):
            self.dstCorners[i][0][0] = (5 - np.floor(i/self.grid[0])) * self.square[0] # (5 * grid[0]) - ((i % 6) * grid[0]) # right to left
            self.dstCorners[i][0][1] = (i % self.grid[0]) * self.square[1]# (np.floor(i/9)) * grid[1] # top to bottom
    
    def calculateMask(self, image):
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, srcCorners = cv2.findChessboardCorners(grayImage, self.grid, None)

        # if chessboard found
        if ret == True:
            srcCorners2 = cv2.cornerSubPix(grayImage, srcCorners, (11, 11), (-1, -1), self.criteria)
            homographyMask, _ = cv2.findHomography(srcCorners2, self.dstCorners)

            # for test
            dstCornersCalculated = cv2.perspectiveTransform(srcCorners, homographyMask)
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
            return homographyMask

        return None




'''
def test(imageName):
    img = cv2.imread(imageName)
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #find the chess board corners
    ret, srcCorners = cv2.findChessboardCorners(imgGray, (9, 6), None)
    dstCorners = np.zeros((54, 1, 2))
    for i in range(len(dstCorners)):
        dstCorners[i][0][0] = (5 - np.floor(i/9)) * grid[0] # (5 * grid[0]) - ((i % 6) * grid[0]) # right to left
        dstCorners[i][0][1] = (i % 9) * grid[1]# (np.floor(i/9)) * grid[1] # top to bottom
    #print(dstCorners)

    #if found, add object points, image points (after refining them)
    if ret == True:
        srcCorners2 = cv2.cornerSubPix(imgGray, srcCorners, (11, 11), (-1, -1), criteria)

        # homography
        print(type(srcCorners), np.shape(srcCorners)) # np.ndarray (54, 1, 2)
        print(type(srcCorners2), np.shape(srcCorners2)) # np.ndarry (54, 1, 2)
        # print(srcCorners)
        homographyMask, _ = cv2.findHomography(srcCorners2, dstCorners)
        print(homographyMask)

        dstCornersCalculated = cv2.perspectiveTransform(srcCorners, homographyMask)
        averageDifference = 0
        averageXDifference = 0
        averageYDifference = 0
        for i in range(len(dstCornersCalculated)):
            plt.figure(0)
            plt.plot(dstCornersCalculated[i][0][0], dstCornersCalculated[i][0][1], "-o")
            plt.text(dstCornersCalculated[i][0][0], dstCornersCalculated[i][0][1], i, color="red", fontsize=12)
            averageDifference += np.sqrt(np.square(dstCornersCalculated[i][0][0] - dstCorners[i][0][0]) + \
                                      np.square(dstCornersCalculated[i][0][1] - dstCorners[i][0][1]))
            averageXDifference += np.abs(dstCornersCalculated[i][0][0] - dstCorners[i][0][0])
            averageYDifference += np.abs(dstCornersCalculated[i][0][1] - dstCorners[i][0][1])
        averageDifference /= len(dstCornersCalculated)
        averageXDifference /= len(dstCornersCalculated)
        averageYDifference /= len(dstCornersCalculated)
        print("average difference to desired outcome")
        print("global:", averageDifference, "mm")
        print("x-coör:", averageXDifference, "mm")
        print("y-coör:", averageYDifference, "mm")
        plt.show()
    else:
        return False
    return

imageNames = glob.glob('calibration/testImages/*.jpg')
# for name in imageNames:
#     test(name)

name = imageNames[0]
test(name)
'''