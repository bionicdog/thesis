'''imports'''
import sys
import os
import cv2
import time

'''personal imports'''
from yolo.yolo import Yolo
from yolo.point import Point
from image_processing.image import readImage, showImage

path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03009.png"
#path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03132.png"
#path = "D:/fsoco_bounding_boxes_train/prom/img/prom_00121.jpg"


img = readImage(path)
model = Yolo()


test_img = img.copy()
start_time = time.time()

(yellowCones, blueCones) = model.feed_forward(test_img)

print("Duration: ", time.time()-start_time, " s")
print(blueCones)

for middlePoint in blueCones:
    tmp = middlePoint.getArray()
    cv2.circle(test_img, (round(tmp[0]), round(tmp[1])), 10, (0, 0, 255), thickness=3)

showImage(test_img)