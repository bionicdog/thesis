#source: https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/

import cv2

from picamera import PiCamera
from picamera.array import PiRGBArray

class Camera:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera)

    def get_frame(self):
        self.camera.capture(self.rawCapture, "bgr")
        image = self.rawCapture.array
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        # frame = cv2.imdecode(image, cv2.IMREAD_COLOR)
        frame = cv2.resize(image, (448, 448))
        return frame