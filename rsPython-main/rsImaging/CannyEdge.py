# -*- coding: utf-8 -*-
"""
Created on 5 July 2021

@author: Jan Lemeire
"""

import cv2
from Contours import showContours
from ImageUtils import Color, char2color 

# the parameters
blur_radius = 3

kernel_size = 5
low_threshold = 30
ratio = 3

def analyzeCannyEdge(img, file_name):
    global blur_radius, ratio, kernel_size, low_threshold

    print('** Test Canny Edge detector **')
  #  print('Define ranges for the HSV-values.')
  #  print('Click on a point to see the pixel color.')
  #  print('Blur image by setting the radius of the blur filter.')
  #  print('Press c to find contours (minimal size set by trackbar)')
   # print('Press x to print the chosen ranges on the HSV')
   # print('Press character of predefined color to set thresholds for that color')
    
    print('Choose component: r,g, b, h, s, v, a (gray)')
    print('Press q to quit')

    height, width = img.shape[:2] 
    img_size =  height *  width

    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    
    h, s, v = img_HSV[:, :, 0], img_HSV[:, :, 1], img_HSV[:, :, 2]
    img_comp = v

    windows_edges = 'Edges'
    windows_component = 'ImageComponent'
 
    def applyComponent():
        global blur_radius, ratio, kernel_size, low_threshold
        img_blur = cv2.blur(img_comp, (blur_radius,blur_radius)) if blur_radius > 1 else img_comp
        
        #print('low_threshold = '+str(low_threshold))
        detected_edges = cv2.Canny(img_blur, low_threshold, low_threshold*ratio, None, kernel_size)
        mask = detected_edges != 0
        img_edges = img * (mask[:,:,None].astype(img.dtype))

        cv2.imshow(windows_component, img_comp)
        cv2.imshow(windows_edges, detected_edges) # img_edges)

    def setLowThreshold(val):
        global low_threshold 
        low_threshold = int(val)
        cv2.setTrackbarPos(track_low_threshold, windows_edges, low_threshold)
        applyComponent()
        
    def setRatio(val):
        global ratio 
        ratio = int(val)
        cv2.setTrackbarPos(track_ratio, windows_edges, ratio)
        applyComponent()
        
    def setKernelSize(val):
        global kernel_size 
        kernel_size = int(val)
        cv2.setTrackbarPos(track_kernel_size, windows_edges, kernel_size)
        applyComponent()

    def setBlurRadius(val):
        global blur_radius 
        blur_radius = max(int(val), 3)
        cv2.setTrackbarPos(track_blur_radius, windows_edges, blur_radius)
        applyComponent()
        
    cv2.namedWindow(windows_component, cv2.WINDOW_NORMAL)
    cv2.namedWindow(windows_edges)

    track_low_threshold = 'Low Threshold'
    cv2.createTrackbar(track_low_threshold, windows_edges , low_threshold, 150, setLowThreshold)
    track_ratio = 'Threshold Ratio'
    cv2.createTrackbar(track_ratio, windows_edges , ratio, 10, setRatio)
    track_kernel_size = 'Kernel Size'
    cv2.createTrackbar(track_kernel_size, windows_edges , kernel_size, 7, setKernelSize)
    track_blur_radius = 'Blur radius'
    cv2.createTrackbar(track_blur_radius, windows_edges , blur_radius, 20, setBlurRadius)

    
    applyComponent()
    
    print('Press a key to finish')
    while cv2.waitKey() <= 0:
        x = 1 
    #cv2.destroyAllWindows()
    cv2.destroyWindow(windows_edges)
    cv2.destroyWindow(windows_component)
    

    
################ MAIN ################
if __name__== "__main__":
    import ImageUtils
    
#    img_file = 'D:/references/vision/lego brick images/Neal/without lamp/1585148260.82.jpg'
   # img_file = 'D:/references/vision/lego brick images/details/IMG_6240a.jpg'
   # img_file = 'D:/references/vision/lego brick images/lego-bricks-1x1-3d.jpg'
#    img_file = 'D:/references/vision/lego brick images/details/IMG_6247-rose-ROI.jpg'
    img_file = 'D:/references/vision/lego brick images/lego-bricks-1x1-3d.jpg'
    img=cv2.imread(img_file)
    analyzeCannyEdge(img, img_file)
