'''Sources'''
#https://gist.github.com/razimgit/d9c91edfd1be6420f58a74e1837bde18

'''imports'''
import numpy as np
import cv2 #pip install opencv-python

'''personal imports'''
import image

'''constants'''
#blue treshold in HSV
tresh_blue_lower = np.array([100,35,140])
tresh_blue_upper = np.array([130,255,255])
#yellow treshold in HSV
tresh_yellow_lower = np.array([15,35,140])
tresh_yellow_upper = np.array([60,255,255])
#orange treshold in HSV
tresh_orange_lower = np.array([0,35,140])
tresh_orange_upper = np.array([15,255,255])

'''code'''
def blueFilter(img): #https://www.geeksforgeeks.org/filter-color-with-opencv/
    hsv_img = image.Rgb2HsvImage(img)

    #overlay mask
    mask = cv2.inRange(hsv_img, tresh_blue_lower, tresh_blue_upper)

    #remove non-blue regions
    result_img = cv2.bitwise_and(img, img, mask=mask)
    return result_img

def yellowFilter(img):
    hsv_img = image.Rgb2HsvImage(img)

    #overlay mask
    mask = cv2.inRange(hsv_img, tresh_yellow_lower, tresh_yellow_upper)

    #remove non-yellow regions
    result_img = cv2.bitwise_and(img, img, mask=mask)
    return result_img

def orangeFilter(img):
    hsv_img = image.Rgb2HsvImage(img)

    #overlay mask
    mask = cv2.inRange(hsv_img, tresh_orange_lower, tresh_orange_upper)

    #remove non-orange regions
    result_img = cv2.bitwise_and(img, img, mask=mask)
    return result_img

def morphOpen(img): #erosion followed by dilation to delete noise
    kernel = np.ones((5, 5))
    return cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

def getContours(img):
    img_edges = cv2.Canny(img, 100, 200)

    #image.showImage(img_edges)

    #RETR_EXTERNAL: retrieves only extreme outer contours
    #CHAIN_APPROX_SIMPLE: compresses horizontal, vertical, and diagonal segments and leaves only their end points
    contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Simplify contours by reducing number of points
    approx_contours = []
    for contour in contours:
        #approxPolyDP(contour, epsilon, closed) with
        # epsilon being approximation accuracy (maximum distance between original and new curve)
        # closed being true meaning the approximated curve is closed
        approx_contour = cv2.approxPolyDP(contour, 3, closed=True)
        #only convex contours are kept (hulls around the wanted color)
        approx_conv_hull_contour = cv2.convexHull(approx_contour)
        #only look at contours that contain of 3 to 10 points
        if 3 <= len(approx_conv_hull_contour) <= 10:
            approx_contours.append(approx_conv_hull_contour)
    return approx_contours

def contourIsCone(contour):
    #cones or elements pointing up
    # check if points on upper half are within the boundaries of the lower halfs points
    points_upper_half = []
    points_lower_half = []

    #minimal up-right bounding rectangle
    x, y, w, h = cv2.boundingRect(contour)

    #to small to be a cone
    if h < 12:
        return False, [x, y, w, h]

    #vertical center
    vert_center = y + (2*h/3)
    
    #divide points in upper and lower
    for point in contour:
        if (point[0][1] < vert_center):
            points_upper_half.append(point)
        else:
            points_lower_half.append(point)
    
    #set boundaries of lower half
    left_bound = points_lower_half[0][0][0]
    right_bound = points_lower_half[0][0][0]
    for point in points_lower_half:
        if (point[0][0] < left_bound):
            left_bound = point[0][0]
        elif (point[0][0] > right_bound):
            right_bound = point[0][0]
    
    #check if all points of upper half are within boundaries lower half
    for point in points_upper_half:
        if (point[0][0] < left_bound) or (point[0][0] > right_bound):
            return False, [x, y, w, h]
    
    #return True and bounding box if cone/pointing up
    return True, [x, y, w, h]