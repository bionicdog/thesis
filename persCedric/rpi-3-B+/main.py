import cv2
import time

from classes.camera import Camera
from classes.yolo import Yolo
from classes.homography import Homography

'''initialisation'''
camera = Camera()
print("Camera initialised!")

model = Yolo()
print("Yolo model initialised!")

homography = Homography()
print("Homography initialised!")



'''running loop'''
if __name__ == '__main__':
    
    homography.calculateMask(camera.get_frame())
    print("Homography mask calculated!")

    while True:
        # duration processing 1 frame
        t0 = time.time()

        frame = camera.get_frame()
        # cv2.imshow("image", frame)
        # cv2.waitKey(0)

        # running Yolo on the frame
        (yellowCones, blueCones) = model.feed_forward(frame)

        # calculating homography