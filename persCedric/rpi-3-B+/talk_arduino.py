from classes.robot import Robot
import time

robot = Robot(device='/dev/ttyUSB1')
speed = 15 # in percent: -100 to 100 (backwards to forwards, stop = 0)
steering = 0 # in degrees: -15 to 15 (left to right)

while True:
    robot.set_speed_and_steeringangle(speed, steering)
    speed, steering = robot.get_output
    time.sleep(5)