# import cv2
# import matplotlib.pyplot as plt
import numpy as np
import time
from math import degrees, atan

from classes.camera import Camera
# from classes.yolo_onnx import Yolo
# from classes.homography import Homography
# from classes.delaunay import Delaunay
from classes.robot import Robot, Bicycle_model

from classes.socket import Client

'''functions'''
def slope(line):
    return (line[1][1]-line[0][1])/(line[1][0]-line[0][0])

def calc_angle(line0, line1):
    slope0 = slope(line0)
    slope1 = slope(line1)
    temp = np.abs((slope1-slope0)/(1+(slope1*slope0)))
    if line1[1][0] < 0:
        return np.round(-1*(atan(temp)*180)/3.1415, 0)
    return np.round((atan(temp)*180)/3.1415, 0)

'''initialisation'''
fps = 10
max_speed = 15

camera = Camera(fps)
print("Camera initialised!")

robot = Robot('/dev/ttyUSB0')
print("Control initialised!")

bicycle_model = Bicycle_model(max_speed=max_speed)
print("Bicycle model initialised")

print("Making connection ...")
client_socket = Client("192.168.1.21")
print("Connection established")

'''running loop'''
if __name__ == '__main__':
    # sending first frame for homography mask to be calculated
    frame = camera.get_frame()
    # send_frame_recv_path instead of send_frame to prevent deadlock
    client_socket.send_frame_recv_path(frame)

    t_start = time.perf_counter()
    
    while True:
        # duration processing 1 frame
        t0 = time.perf_counter()

        frame = camera.get_frame()
        t1 = time.perf_counter()

        # sending frame and getting path
        path = client_socket.send_frame_recv_path(frame)
        t2 = time.perf_counter()

        print(np.shape(path))

        # control
        speed = 0
        brake = 0
        if len(path) >= 2:
            '''
            speed = max_speed
            angle = calc_angle([[0, 0], [0.001, 1]], path[:2])
            '''
            speed, angle = bicycle_model.calc_steering_angle(path[2])
            print(angle)
        else:
            print(f"time ran: {(time.perf_counter()-t_start)*1000:.2f} ms")
        robot.set_speed_and_steeringangle(speed, int(angle))