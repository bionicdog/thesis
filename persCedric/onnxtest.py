'''imports'''
import sys
import os
import cv2
import time

'''personal imports'''
from yolo.yolo_onnx import Yolo
from image_processing.image import readImage, showImage

path = "C:/Users/cedri/Documents/GitHub/VUBRacing/thesis/persCedric/testimages/20230716-1-small.jpg"
# path = "C:/Users/cedri/Documents/GitHub/VUBRacing/thesis/persCedric/testimages/20230716-2-small.jpg"
# path = "C:/Users/cedri/Documents/GitHub/VUBRacing/thesis/persCedric/testimages/20230716-3-small.jpg"

'''HOMOGRAPHY'''
import numpy as np
import matplotlib.pyplot as plt

# grid size used
grid = [25, 25] # in mm

colors = ["gold", "blue", "orange", "red"]

#termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((9*6, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2)

#arrays to store object points and image points from all the images
objpoints = [] #3d points in real world space
imgpoints = [] #2d points in image plane

def calcHomographyMask(img):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #find the chess board corners
    ret, srcCorners = cv2.findChessboardCorners(imgGray, (9, 6), None)
    dstCorners = np.zeros((54, 1, 2))
    for i in range(len(dstCorners)):
        dstCorners[i][0][0] = ((5 - np.floor(i/9)) - 2.5) * grid[0] # (5 * grid[0]) - ((i % 6) * grid[0]) # right to left "- 2.5" is to be positioned in the middle
        dstCorners[i][0][1] = -1 * (i % 9) * grid[1]# (np.floor(i/9)) * grid[1] # bottom to top
    #print(dstCorners)

    #if found, add object points, image points (after refining them)
    if ret == True:
        srcCorners2 = cv2.cornerSubPix(imgGray, srcCorners, (11, 11), (-1, -1), criteria)

        # homography
        # print(type(srcCorners), np.shape(srcCorners)) # np.ndarray (54, 1, 2)
        # print(type(srcCorners2), np.shape(srcCorners2)) # np.ndarry (54, 1, 2)
        # print(srcCorners)
        homographyMask, _ = cv2.findHomography(srcCorners2, dstCorners)
        # print(homographyMask)
        return homographyMask
    else:
        return False

def calcHomographyPerspectiveTransform(srcCorners, homographyMask):
    return cv2.perspectiveTransform(srcCorners, homographyMask)

def showGroundplan(coordinates, class_ids):
    '''
    for i in range(len(class_ids)):
        cv2.circle(combined_img, (int(coordinates[i][0][0]), int(coordinates[i][0][1])), 2, model.get_color(class_ids[i]), 1)
        '''
    plt.figure(0)
    for i in range(len(class_ids)):
        
        color = colors[class_ids[i]]
        plt.plot(coordinates[i][0][0], coordinates[i][0][1], "^", color=color)
    plt.xlabel("x (mm)")
    plt.ylabel("y (mm)")
    plt.show()
    return

'''YOLO'''
img = readImage(path)
model = Yolo("C:/Users/cedri/Documents/GitHub/VUBRacing/thesis/persCedric/yolo/YOLOv8n_FSOCO.onnx")

'''main'''
test_img = img.copy()

# homography mask calculation
mask = calcHomographyMask(test_img)

start_time = time.perf_counter()

# find cones
boxes, scores, class_ids = model.feed_forward(test_img)

print(f"Yolo time: {(time.perf_counter() - start_time)*1000:.2f} ms")

combined_img = model.draw_detections(test_img)


centerpoints = model.xyxyBoxes_to_centerpoints(boxes)

print(np.shape(centerpoints))

world_coordinates = calcHomographyPerspectiveTransform(np.array(centerpoints), mask)

showGroundplan(world_coordinates, class_ids)

cv2.imshow("Detected Objects", combined_img)
cv2.waitKey(0)