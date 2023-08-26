'''imports'''
import sys
import os
import cv2
import time

'''personal imports'''
from yolo.yolo import Yolo
from yolo.point import Point
from image_processing.image import readImage, showImage

#path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03009.png"
#path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03132.png"
path = "D:/fsoco_bounding_boxes_train/prom/img/prom_00121.jpg"


img = readImage(path)
model = Yolo()


test_img = img.copy()
start_time = time.perf_counter()

(yellowCones, blueCones) = model.feed_forward(test_img)

print(f"Duration: {(time.perf_counter() - start_time)*1000:.2f} ms")
print(blueCones)
'''
for middlePoint in blueCones:
    tmp = middlePoint.getArray()
    cv2.circle(test_img, (round(tmp[0]), round(tmp[1])), 10, (0, 0, 255), thickness=3)
'''

writeFile = "C:/Users/cedri/Desktop/images thesis/YOLO pt/Image_3.png"
cv2.imwrite(writeFile, test_img)
showImage(test_img)