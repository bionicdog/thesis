import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob

from brownconrad import calc_newPixelLoc, inputCalc_newPixelLoc

#termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((9*6, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2)

#arrays to store object points and image points from all the images
objpoints = [] #3d points in real world space
imgpoints = [] #2d points in image plane

def test(imageName):
    img = cv2.imread(imageName)
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #find the chess board corners
    ret, corners = cv2.findChessboardCorners(imgGray, (9, 6), None)

    #if found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(imgGray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        #calibrateCamera returns
        #camera matrix, distortion coefficients, rotation and translation vectors
        ret2, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, imgGray.shape[::-1], None, None)
        #'''
        print("camera matrix")
        print(mtx)
        print("distortion coefficients")
        print(dist)
        print("rotation vectors")
        print(rvecs)
        print("translation vectors")
        print(tvecs)
        #'''

        # test brown-conrad
        inputParams = inputCalc_newPixelLoc(mtx, dist)
        print(corners2[0][0])
        print(calc_newPixelLoc(inputParams, corners2[0][0]))

        corners3 = []
        for cornerTemp in corners2:
            corner = cornerTemp[0]
            corners3.append([calc_newPixelLoc(inputParams, corner)])

        #print(type(corners2[0][0][0]))
        corners3 = np.array(corners3, np.float32)

        #change coordinates to make grid
        #known that points will be on a straight line
        print("DATA")
        testarray = [[corners3[0][0], corners3[9][0]], [corners3[18][0], corners3[27][0]], [corners3[4][0], corners3[13][0]], [corners3[22][0], corners3[31][0]], [corners3[8][0], corners3[17][0]], [corners3[26][0], corners3[35][0]]]
        for [a, b] in testarray:
            print(a, b, np.sqrt(np.square(a[0]-b[0])+np.square(a[1]-b[1])))

        #print(type(corners3[0][0][0]))
        #draw and display corners
        cv2.drawChessboardCorners(img, (9, 6), corners3, ret)
        cv2.imshow('test', img)
        cv2.waitKey(0)

        '''
        dst = cv2.undistort(img, mtx, dist, None, mtx)
        cv2.drawChessboardCorners(dst, (9, 6), corners3, ret)
        cv2.imshow('test2', img)
        cv2.waitKey(0)
        '''
    else:
        return False
    return

imageNames = glob.glob('persCedric/calibration/testImages/*.jpg')
for name in imageNames:
    test(name)

