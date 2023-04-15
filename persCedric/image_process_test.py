'''imports'''
import sys
import os
import cv2
import time

'''personal imports'''
sys.path.append(os.path.abspath("image_processing"))
from image import readImage, showImage
from edge_detection import blueFilter, yellowFilter, orangeFilter, morphOpen, getContours, contourIsCone

path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03009.png"
#path = "F:/fsoco_bounding_boxes_train/ampera/img/amz_03132.png"
#path = "F:/fsoco_bounding_boxes_train/prom/img/prom_00121.jpg"


img = readImage(path)
#image.showImage(img)
#showImage(morphOpen(blueFilter(img)))
#showImage(morphOpen(yellowFilter(img)))
#showImage(morphOpen(orangeFilter(img)))

#test_img = cv2.bitwise_or(morphOpen(blueFilter(img)), morphOpen(yellowFilter(img)))
#test_img = cv2.bitwise_or(test_img, morphOpen(orangeFilter(img)))
#showImage(test_img)

#test_img = img.copy()
#start_time = time.time()
#blueContours = getContours(morphOpen(blueFilter(img)))
#yellowContours = getContours(morphOpen(yellowFilter(img)))
#orangeContours = getContours(morphOpen(orangeFilter(img)))
#print("Duration: ", time.time()-start_time, " s")
#cv2.drawContours(test_img, blueContours, -1, (255, 0, 0), 2)
#cv2.drawContours(test_img, yellowContours, -1, (0, 255, 0), 2)
#cv2.drawContours(test_img, orangeContours, -1, (0, 255, 0), 2)
#showImage(test_img)



test_img = img.copy()
start_time = time.time()

blueContours = getContours(morphOpen(blueFilter(img)))
blueCones = []
for contour in blueContours:
    isCone, boundary_box = contourIsCone(contour)
    if (isCone):
        blueCones.append(boundary_box)

yellowContours = getContours(morphOpen(yellowFilter(img)))
yellowCones = []
for contour in yellowContours:
    isCone, boundary_box = contourIsCone(contour)
    if (isCone):
        yellowCones.append(boundary_box)

orangeContours = getContours(morphOpen(orangeFilter(img)))
orangeCones = []
for contour in orangeContours:
    isCone, boundary_box = contourIsCone(contour)
    if (isCone):
        orangeCones.append(boundary_box)

print("Duration: ", time.time()-start_time, " s")

for [x, y, w, h] in blueCones:
    cv2.rectangle(test_img, (x, y), (x+w, y+h), (255, 0, 0), thickness=3)
for [x, y, w, h] in yellowCones:
    cv2.rectangle(test_img, (x, y), (x+w, y+h), (0, 255, 0), thickness=3)
for [x, y, w, h] in orangeCones:
    cv2.rectangle(test_img, (x, y), (x+w, y+h), (0, 0, 255), thickness=3)

#find closest blue and yellow cone
#find middle point between these 2
closestBlueCone = [0, 0, 0, 0]
closestYellowCone = [0, 0, 0, 0]
tempDist = 0
for [x, y, w, h] in blueCones:
    if y+(h/2) > tempDist:
        closestBlueCone = [x, y, w, h]
        tempDist = y+(h/2)
tempDist = 0
for [x, y, w, h] in yellowCones:
    if y+(h/2) > tempDist:
        closestYellowCone = [x, y, w, h]
        tempDist = y+(h/2)

#draw line from closest blue cone to closest yellow cone
leftPoint = (round(closestBlueCone[0]+closestBlueCone[2]), round(closestBlueCone[1]+(closestBlueCone[3]/2)))
rightPoint = (round(closestYellowCone[0]), round(closestYellowCone[1]+(closestYellowCone[3]/2)))
cv2.line(test_img, leftPoint, rightPoint, (0, 200, 200), thickness=3)

#draw circle on middle point on that line
middlePoint = (round((rightPoint[0]+leftPoint[0])/2), round((rightPoint[1]+leftPoint[1])/2))
cv2.circle(test_img, middlePoint, 10, (0, 0, 255), thickness=3)

showImage(test_img)