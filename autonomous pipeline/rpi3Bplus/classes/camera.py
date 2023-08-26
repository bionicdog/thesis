import cv2
'''
from picamera import PiCamera
from picamera.array import PiRGBArray
'''

class Camera:
    def __init__(self, fps):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 32)
        
        '''
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera)
        '''
    
    def get_frame_homo(self, frame_width, frame_height):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        ret, image = self.cap.read()

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        frame = cv2.rotate(image, cv2.ROTATE_180)
        return frame

    def get_frame(self):
        ret, image = self.cap.read()
        rotated_image = cv2.rotate(image, cv2.ROTATE_180)
        '''
        self.rawCapture.truncate(0)
        self.camera.capture(self.rawCapture, "bgr")
        image = self.rawCapture.array
        '''
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        # frame = cv2.imdecode(image, cv2.IMREAD_COLOR)
        frame = cv2.resize(rotated_image, (448, 448))
        return frame