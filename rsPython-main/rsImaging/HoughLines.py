# -*- coding: utf-8 -*-
"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
@author Jan Lemeire
"""
import sys
import math
import cv2
import numpy as np

# the parameters
rho = 1
theta = np.pi / 180
hough_threshold = 150
minLineLength = 50
maxLineGap = 10

def analyzeHoughLines(img, file_name, canny_kernel_size = 3, canny_low_threshold = 30, canny_ratio = 2):
    global rho, theta, hough_threshold, minLineLength, maxLineGap

    print('** Test Hough Lines **')
    
    height, width = img.shape[:2] 
    img_size =  height *  width
    
    def analyzeLine(edgesImg, _rho, _theta):
        x_min = 0
        x_max = width
        x_mid = width/2
        # with equaton
        
        
    
    def applyHough():
        global rho, theta, hough_threshold, minLineLength, maxLineGap
        try:
            dst = cv2.Canny(img, canny_low_threshold, canny_low_threshold *canny_ratio, None, canny_kernel_size)

            print('Rho = '+str(rho)+' theta = '+str(theta)+' threshold = '+str(hough_threshold))
        
            # Copy edges to the images that will display the results in BGR
            cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
            cdstP = np.copy(cdst)
                    
            lines = cv2.HoughLines(dst, rho, theta, hough_threshold, None, 0, 0) # doc: https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
            if lines is not None:
                print('Hough lines found: '+str(len(lines)))
                for i in range(0, len(lines)):
                    _rho = lines[i][0][0] # meaning: see https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
                    _theta = lines[i][0][1]
                    a = math.cos(_theta)
                    b = math.sin(_theta)
                    x0 = a * _rho
                    y0 = b * _rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv2.line(cdst, pt1, pt2, (0,255,255), 2, cv2.LINE_AA)
            else:
                print('No Hough lines found')
            cv2.imshow(WINDOW_HOUGH_LINES, cdst)
            
            if True: # HoughLinesP
                minLineLength = 50
                maxLineGap = 10
                linesP = cv2.HoughLinesP(dst, rho, theta, hough_threshold, None, minLineLength, maxLineGap)
                
                if linesP is not None:
                    print('Hough linesP found: '+str(len(linesP)))
                    for i in range(0, len(linesP)):
                        pts = linesP[i][0]
                        cv2.line(cdstP, (pts[0], pts[1]), (pts[2], pts[3]), (0,255,255), 3, cv2.LINE_AA)
                else:
                    print('No Hough linesP found')
                cv2.imshow(WINDOW_HOUGH_LINESP, cdstP)
                
        except Exception as e:
            print('Error with Hough: '+ str(e))
    
    def setRho(val):
        global rho
        rho = int(val)
        cv2.setTrackbarPos(TRACK_BAR_RHO, WINDOW_IMAGE, rho)
        applyHough()

    def setTheta(val):
        global theta
        degrees = int(val)
        theta = np.pi / degrees
        cv2.setTrackbarPos(TRACK_BAR_THETA, WINDOW_IMAGE, degrees)
        applyHough()
        
    def setThreshold(val):
        global hough_threshold
        hough_threshold = int(val)
        cv2.setTrackbarPos(TRACK_BAR_THRESHOLD, WINDOW_IMAGE, hough_threshold)
        applyHough()

    def setMinLineLength(val):
        global minLineLength
        minLineLength = int(val)
        cv2.setTrackbarPos(TRACK_BAR_MIN_LINE_LENGTH, WINDOW_IMAGE, minLineLength)
        applyHough()
        
    def setMaxLineGap(val):
        global maxLineGap
        maxLineGap = int(val)
        cv2.setTrackbarPos(TRACK_BAR_MAX_LINE_GAP, WINDOW_IMAGE, maxLineGap)
        applyHough()
    
    WINDOW_IMAGE = 'Image'
    WINDOW_HOUGH_LINES = "Hough Lines - Detected Lines (in red)" 
    WINDOW_HOUGH_LINESP = "Probabilistic Hough Lines - Detected Lines (in red)"
    TRACK_BAR_RHO = 'Rho'
    TRACK_BAR_THETA = 'Theta degrees'  # theta
    TRACK_BAR_THRESHOLD = 'Threshold'    # hough_threshold
    TRACK_BAR_MIN_LINE_LENGTH = 'minLineLength'
    TRACK_BAR_MAX_LINE_GAP = 'maxLineGap'
        
    cv2.namedWindow(WINDOW_IMAGE, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_HOUGH_LINES, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_HOUGH_LINESP, cv2.WINDOW_NORMAL)
        
    cv2.createTrackbar(TRACK_BAR_RHO, WINDOW_IMAGE , rho, 10, setRho)
    cv2.createTrackbar(TRACK_BAR_THETA, WINDOW_IMAGE , 180, 360, setTheta)
    cv2.createTrackbar(TRACK_BAR_THRESHOLD, WINDOW_IMAGE , hough_threshold, 256, setThreshold)
    cv2.createTrackbar(TRACK_BAR_MIN_LINE_LENGTH, WINDOW_IMAGE , minLineLength, 256, setMinLineLength)
    cv2.createTrackbar(TRACK_BAR_MAX_LINE_GAP, WINDOW_IMAGE , maxLineGap, 50, setMaxLineGap)

    cv2.imshow(WINDOW_IMAGE, img)
    applyHough()
    
    print('Press a key to finish')
    while cv2.waitKey() <= 0:
        x = 1 
    cv2.destroyAllWindows()

    
if __name__ == "__main__":
    img_file = 'D:/references/vision/lego brick images/lego-bricks-1x1-3d.jpg'
    img=cv2.imread(img_file)
    analyzeHoughLines(img, img_file)