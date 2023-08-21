import serial # import pyserial

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