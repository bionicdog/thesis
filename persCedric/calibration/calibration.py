#source: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html


import numpy as np
import cv2
import glob

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

        #draw and display corners
        cv2.drawChessboardCorners(img, (9, 6), corners2, ret)
        cv2.imshow('test', img)
        cv2.waitKey(0)
    else:
        return False
    return

imageNames = glob.glob('persCedric/calibration/testImages/*.jpg')
for name in imageNames:
    test(name)

