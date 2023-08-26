'''imports'''
import sys
import os
import cv2
import time

'''personal imports'''
from yolo.yolo_onnx import Yolo
from image_processing.image import readImage, showImage

#path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03009.png"
#path = "D:/fsoco_bounding_boxes_train/ampera/img/amz_03132.png"
path = "D:/fsoco_bounding_boxes_train/prom/img/prom_00121.jpg"


img = readImage(path)
model = Yolo("C:/Users/cedri/Documents/GitHub/VUBRacing/thesis/persCedric/yolo/YOLOv8n_FSOCO.onnx")


test_img = img.copy()
start_time = time.perf_counter()

# find cones
boxes, scores, class_ids = model.feed_forward(test_img)

print(f"Duration: {(time.perf_counter() - start_time)*1000:.2f} ms")

combined_img = model.draw_detections(test_img)

'''
for middlePoint in blueCones:
    tmp = middlePoint.getArray()
    cv2.circle(test_img, (round(tmp[0]), round(tmp[1])), 10, (0, 0, 255), thickness=3)
'''

writeFile = "C:/Users/cedri/Desktop/images thesis/YOLO onnx/Image_3.png"
cv2.imwrite(writeFile, combined_img)
showImage(combined_img)