import serial # pyserial
from math import degrees, atan2, pow

class Robot():
    def __init__(self, device): #device='/dev/ttyUSB0'
        self.speed = 0
        self.steering_angle = 0
        self.output = ""
        self.ser = serial.Serial(device, 115200, timeout=1)
        self.ser.reset_input_buffer()
    
    def set_speed_and_steeringangle(self, speed, steering_angle):
        # speed -100:100
        self.speed = speed
        self.steering_angle = steering_angle
        string = str(speed) + ";" + str(steering_angle) + "\n"
        self.ser.write(bytes(string.encode('utf-8')))
        self.output = self.ser.readline().decode('utf-8').rstrip()
        return
    
    def get_output(self):
        speed, steering = self.output.split(" ", 1)
        return speed, steering

class Bicycle_model():
    def __init__(self, max_speed, max_angle=15, wheelbase=145):
        self.rear = [0, (-1*wheelbase)]
        self.front = [0, 0]
        self.max_speed = max_speed
        self.max_angle = max_angle

    def calc_steering_angle(self, waypoint):
        if waypoint[0] == 0:
            return 0, 0 # brake
        x = (waypoint[0]/2) + ((pow((self.rear[1] - waypoint[1]), 2))/(2*waypoint[0]))
        if x >= 0:
            angle = degrees(atan2(145, x))
        else:
            angle = -1*degrees(atan2(145, abs(x)))
        
        if angle > self.max_angle:
            return self.max_speed, self.max_angle
        elif angle < (-1*self.max_angle):
            return self.max_speed, (-1*self.max_angle)
        else:
            return self.max_speed, angle

'''
model = Bicycle_model(max_speed=15)

print(model.calc_steering_angle([100, 50]))
print(model.calc_steering_angle([-100, 50]))
'''