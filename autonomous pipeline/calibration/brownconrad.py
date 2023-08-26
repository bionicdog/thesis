import numpy as np
# import cv2

def inputCalc_newPixelLoc(camMtx, distCoef):
    # calculate inputs for calc_newPixelLoc
    # params
    # camera matrix
    # distortion coefficient (k1 k2 p1 p2 k3 ...)
    
    # optical center
    cx = camMtx[0][2]
    cy = camMtx[1][2]

    # distortion coefficients
    # if distCoef is None
    k1 = 0
    k2 = 0
    k3 = 0
    p1 = 0
    p2 = 0
    if (len(distCoef) == 4):
        k1 = distCoef[0]
        k2 = distCoef[1]
        p1 = distCoef[2]
        p2 = distCoef[3]
    elif (len(distCoef) > 4):
        k1 = distCoef[0]
        k2 = distCoef[1]
        k3 = distCoef[4]
        p1 = distCoef[2]
        p2 = distCoef[3]
    
    return cx, cy, k1, k2, k3, p1, p2

def calc_newPixelLoc(inputParams, dist_point):
    # params
    # inputParams of outputsize inputCalc_newPixelLoc
    # x_distorted (coordinate picture)
    # y_distorted (coordinate picture)

    # taking params out of inputParams
    cx, cy, k1, k2, k3, p1, p2 = inputParams
    x_dist = dist_point[0]
    y_dist = dist_point[1]
    r = np.sqrt(np.square(x_dist - cx) + np.square(y_dist - cy))

    # calculation (Brown-Conrady)
    x_undist = x_dist + (x_dist - cx) * (k1 * np.square(r) + k2 * np.power(r, 4) + k3 * np.power(r, 6)) \
        + p1 * (np.square(r) + 2 * np.square(x_dist - cx)) + 2 * p2 * (x_dist - cx) * (y_dist - cy)
    
    y_undist = y_dist + (y_dist - cx) * (k1 * np.square(r) + k2 * np.power(r, 4) + k3 * np.power(r, 6)) \
        + 2 * p1 * (x_dist - cx) * (y_dist - cy) + p2 * (np.square(r) + 2 * np.square(y_dist - cy)) 
    
    return [x_undist, y_undist]