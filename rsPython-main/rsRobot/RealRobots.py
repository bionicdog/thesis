# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 16:22:33 2022

@author: Marco Van Cleemput
"""
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from Robots import Robot

from State import StateDef, States
from Environment import Environment, Environment1D
from Environment2D import Environment2DSquare
from math import pi, cos, sin
import Motors
import serial
import Sensors
import Sensors2D
import numpy as np

#library van de compas module
import lib.IMU as IMU


class RealRobot(Robot):
    
    def __init__(self, name:str, connection, states: States, motors:list, sensors:list, environment: Environment):
        super().__init__(name, motors, sensors, environment)
        self.conn = connection
        self.states = states
        self.stateDef = states.stateDef
        self.laststring = ""
        self.t = 0
        self.turn = False
        
        #IMU.detectIMU()
        #if(IMU.BerryIMUversion == 99):
        #    print(" No BerryIMU found... exiting ")

        #IMU.initIMU()
    
    def setMotors(self, motorInput:dict):
        #self.t = self.t + 1
        # TODO: send motor input to robot via connection
        
        self.motorInput = motorInput
        if self.motorInput['mF'] > 999:
            self.turn = True
            self.motorInput['mF'] = self.motorInput['mF'] - 1000
            self.motorInput['mL'] = self.motorInput['mL'] - 1000
            self.motorInput['mR'] = self.motorInput['mR'] - 1000
            self.motorInput['mB'] = self.motorInput['mB'] - 1000
        else:
            self.turn = False
        
        #motorwaarden ook loggen
        
        string = genString(motorInput)
        if(string != self.laststring):
            self.laststring = string
            self.conn.write(string.encode())
            self.conn.flush()
            print("write: " + string)
            super().setMotors(motorInput, self.t)
            
            
    def readSensors(self):
        # TODO: receive sensor values from robot via connection 
        # blocking recieve

        try:
            self.conn.reset_input_buffer()
            data = self.conn.readline().decode('UTF-8')
            print("readSer")
            #print(self.serdata)
        except ValueError or UnicodeDecodeError as e:
            print(e)

        dataArr = decode(data)
        if isinstance(dataArr, np.ndarray):
            self.t = self.t + 1
            self.states.addFrame()
            for sensor in self.sensors:
                if sensor.name == "sEF":
                    sensor.setObservation(dataArr[0,0],self.t)
                if sensor.name == "sEL":
                    sensor.setObservation(dataArr[0,1],self.t)
               #     self.states.setValue('v', self.t, dataArr[0,1])
                if sensor.name == "sEB":
                    sensor.setObservation(dataArr[0,2],self.t)
                if sensor.name == "sER":
                    sensor.setObservation(-dataArr[0,3],self.t)
                    
                if sensor.name == "sDF":
                    sensor.setObservation(dataArr[1,3],self.t)
               #     self.states.setValue('x', self.t, dataArr[1,0])
                if sensor.name == "sDL":
                    sensor.setObservation(dataArr[1,1],self.t)
                if sensor.name == "sDB":
                    sensor.setObservation(dataArr[1,2],self.t)
                if sensor.name == "sDR":
                    sensor.setObservation(dataArr[1,0],self.t)
                    
                if sensor.name == "tchF":
                    sensor.setObservation(dataArr[2,3],self.t)
                if sensor.name == "tchL":
                    sensor.setObservation(dataArr[2,0],self.t)
                if sensor.name == "tchB":
                    sensor.setObservation(dataArr[2,1],self.t)
                if sensor.name == "tchR":
                    sensor.setObservation(dataArr[2,2],self.t)
                    
                ax = dataArr[3,0]  
                ay = dataArr[3,1]
                if sensor.name == "sAX":
                    ax = 0
                    if self.t > 2:
                        valArr = self.getMotorOrSensor("sEB").values
                        x2 = valArr[self.t-1]
                        x1 = valArr[self.t-2]
                        ax = (abs(x2) - abs(x1)) / 0.15
                    sensor.setObservation(round(ax),self.t)
                    #sensor.setObservation(-(0.707*ax + 0.707*ay),self.t)
               #     self.states.setValue('a', self.t, -(0.707*ax + 0.707*ay))
                if sensor.name == "sAY":
                    
                    sensor.setObservation(0.707*ax - 0.707*ay,self.t)
                
                if sensor.name == "sGZ":
                    sensor.setObservation(dataArr[3,2],self.t)
                
                #if sensor.name == "sAZ":
                #    sensor.setObservation(dataArr[3,2],self.t)
                
               # for sensor in self.sensors:
               #     self.states.setValues( {sensor.statevar : sensor.value}, self.t)
                
                
        sensorsdict = { sensor.name : sensor.value for sensor in self.sensors }
            
        if isinstance(dataArr, np.ndarray):
                sensorsdict["millis"] = dataArr[3,3]
            
        return sensorsdict
    
    def readMotors(self):
        return { motor.name : (motor.value if self.turn is False else 0) for motor in self.motors }

#### #### #### ####  functions #### #### #### ####

def genString(dic):
    string = ""
    for key, val in dic.items():
        string += str(val)
        string += ","
    string = string[:-1]
    string += "\n"
    return string

def decode(data):
    data = data[:-2]
    sens = np.array(data.split(';'))
    outarr = []
    if(sens.size == 5):
        mil = np.array(sens[0].split(','))
        millis = mil.astype(float)[0]

        enc = np.array(sens[1].split(','))
        encoderArray = enc.astype(float)

        dist = np.array(sens[2].split(','))
        distArray = dist.astype(float)

        touch = np.array(sens[3].split(','))
        touchArray = touch.astype(float)

        motor = np.array(sens[4].split(','))
        motorArray = motor.astype(float)

        #ACCx = IMU.readACCx()
        #ACCy = IMU.readACCy()
        #GYRz = IMU.readGYRz()
        #ACCz = IMU.readACCz()

        #IMUArr = np.array([ACCx, ACCy, GYRz, millis])
        IMUArr = [0,0,0,millis]
        #IMUArray = IMUArr.astype(float)
        #accelArray = [ACCx, ACCy, ACCz]

        outarr = np.vstack((encoderArray, distArray, touchArray, IMUArr))
        """
        data += ';'
        for a in IMUArray:
            data += str(a)
            data += ','
            
        data = data[:-1]
        #print("Data string:")
        #print(data)
        data += '\n'
        #f.write(data)
        """
        # print("time = " + str(millis))
        # print("encoder = " + str(encoderArray))
        # print("dist = " + str(distArray))
        # print("touch = " + str(touchArray))
        # print("acc = " + str(accelArray))
        return outarr
    else:
        return -1
    
#### #### #### ####  Create Robots #### #### #### ####    
    
def createRealRobot1D (connection, walls:tuple = (-100, 100)):
    
    environment1D = Environment1D(walls)
    variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
    relationDict ={'v': 'a', 'x': 'v'}    
    def forwardMath1D(states, t, dts):
        states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
        states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
    stateDef = StateDef(variables, relationDict, forwardMath1D)
    
    #FMotor = Motors.OurMotor("mF", ["a", "v"], param1Value = 1, param2Value = 1)
    #LMotor = Motors.OurMotor("mL", ["a", "v"], param1Value = 1, param2Value = 1)
    BMotor = Motors.OurMotor("mB", ["a", "v"], param1Value = 1, param2Value = 1)
    #RMotor = Motors.OurMotor("mR", ["a", "v"], param1Value = 1, param2Value = 1)
    
    #encoderF = Sensors.LineairSensorWithConstant("sEF", "v", 'lvF')
    #encoderL = Sensors.LineairSensorWithConstant("sEL", "v", 'lvL')
    #encoderB = Sensors.LineairSensorWithConstant("sEB", "v", 'lvB', 60.46)
    encoderB = Sensors.LineairSensor("sEB", "v", 'lvB', 60.46)
    #encoderR = Sensors.LineairSensorWithConstant("sER", "v", 'lvR')
    
    #distanceSensorF = Sensors.DistanceSensorWithConstant("sDF", "x", 'lwF', environment1D.walls[1], 'ldF', 67)
    #distanceSensorL = Sensors.DistanceSensorWithConstant("sDL", "x", 'lwL', environment1D.walls[1], 'ldL', 59.29)
    distanceSensorL = Sensors.DistanceSensor("sDL", "x", 'lwL', environment1D.walls[1], 'ldL', 59.29)
    #distanceSensorB = Sensors.DistanceSensorWithConstant("sDB", "x", 'lwB', environment1D.walls[0], 'ldB', 67)
    #distanceSensorR = Sensors.DistanceSensorWithConstant("sDR", "x", 'lwR', environment1D.walls[0], 'ldR', 59.21)
    distanceSensorR = Sensors.DistanceSensor("sDR", "x", 'lwR', environment1D.walls[0], 'ldR', 59.21)
    
    #touchSensorF = Sensors.EventSensor("tchF", "x", 'lwF', environment1D.walls[1])
    touchSensorL = Sensors.EventSensor("tchL", "x", 'lwL', environment1D.walls[1])
    #touchSensorB = Sensors.EventSensor("tchB", "x", 'lwB', environment1D.walls[0])
    touchSensorR = Sensors.EventSensor("tchR", "x", 'lwR', environment1D.walls[0])
    
    #accelerometerX = Sensors.LineairSensorWithConstant("sAX", "a", 'laX', 60.46)
    #accelerometerY = Sensors.LineairSensorWithConstant("sAY", "a", 'laY')
    
    states = States(stateDef, {'a':0, 'v':0, 'x':0})
    return RealRobot("Omnibot1D",connection, states, [BMotor], [distanceSensorL, touchSensorL, distanceSensorR, touchSensorR, encoderB], environment1D)

def createRealRobot2Dsquare (connection, wallDist:int = 100):
    #create environment for estimation
    env2D = Environment2DSquare(wallDist=wallDist)
    
    # state variables in the absolute coordinate system
    variables = ['ax', 'ay', 'vx', 'vy', 'x', 'y']
    # angular acceleration/rotation/position
    variables.extend(['ao', 'vo', 'o'])
    # relative coordinate system
    variables.extend(['axR', 'ayR', 'vxR', 'vyR'])
    # bumpvar
    variables.extend(['b'])
    
    relationDict = {'vxR':'axR', 'vyR':'ayR', 'vo':'ao', 'ax':'axR', 'ay':'ayR','vx':'ax', 'vy':'ay', 'x': 'vx', 'y': 'vy', 'o': 'vo'}   
    
    def forwardMath2D(states, t, dts):
        # orientation robot
        #states.setValue('vo', t, (states['ao'][t] * dts[t] + states['vo'][t - 1]))
        states.setValue('o', t, ((states['vo'][t-1] + states['vo'][t]) / 2 * dts[t] + states['o'][t - 1])%(2*pi))
        # if bump axR depends on ax
        if states['b'][t] == True:
            states.setValue('axR', t, states['ax'][t]*cos(states['o'][t]) + states['ay'][t]*sin(states['o'][t]))
            states.setValue('ayR', t, -states['ax'][t]*sin(states['o'][t]) + states['ay'][t]*cos(states['o'][t]))
        # relative coordinate system (aR are independent variables)
        states.setValue('vxR', t, states['axR'][t] * dts[t] + states['vxR'][t - 1]) 
        states.setValue('vyR', t, states['ayR'][t] * dts[t] + states['vyR'][t - 1]) 
        # absolute coordinate system (a depends on aR)
        states.setValue('ax', t, states['axR'][t]*cos(states['o'][t]) - states['ayR'][t]*sin(states['o'][t]))
        states.setValue('ay', t, states['axR'][t]*sin(states['o'][t]) + states['ayR'][t]*cos(states['o'][t]))
        states.setValue('vx', t, states['vxR'][t]*cos(states['o'][t]) - states['vyR'][t]*sin(states['o'][t])) 
        states.setValue('vy', t, states['axR'][t]*sin(states['o'][t]) + states['ayR'][t]*cos(states['o'][t])) 
        states.setValue('x', t, (states['vx'][t-1] + states['vx'][t]) / 2 * dts[t] + states['x'][t - 1])
        states.setValue('y', t, (states['vy'][t-1] + states['vy'][t]) / 2 * dts[t] + states['y'][t - 1])
        
    stateDef = StateDef(variables, relationDict, forwardMath2D)
    
    #rotation not implemented yet
    FMotor = Motors.OurMotor("mF", ["ayR", "vyR"], param1Value = 1, param2Value = 1)
    LMotor = Motors.OurMotor("mL", ["axR", "vxR"], param1Value = 1, param2Value = 1)
    BMotor = Motors.OurMotor("mB", ["ayR", "vyR"], param1Value = 1, param2Value = 1)
    RMotor = Motors.OurMotor("mR", ["axR", "vxR"], param1Value = 1, param2Value = 1)
    
    encoderF = Sensors.LineairSensor("sEF", "vyR", 'lvF')
    encoderL = Sensors.LineairSensor("sEL", "vxR", 'lvL')
    encoderB = Sensors.LineairSensor("sEB", "vyR", 'lvB')
    encoderR = Sensors.LineairSensor("sER", "vxR", 'lvR')
    
    distanceSensorF = Sensors2D.UnidirectionalDistanceSensor("sDF", env2D, paramName='lxF', maxVal = 2000, orientationOffset=0)
    distanceSensorL = Sensors2D.UnidirectionalDistanceSensor("sDL", env2D, paramName='lxL', maxVal = 2000, orientationOffset=3*(pi/2))
    distanceSensorB = Sensors2D.UnidirectionalDistanceSensor("sDB", env2D, paramName='lxB', maxVal = 2000, orientationOffset=pi)
    distanceSensorR = Sensors2D.UnidirectionalDistanceSensor("sDR", env2D, paramName='lxR', maxVal = 2000, orientationOffset=pi/2)
    
    touchSensorF = Sensors.EventSensor("tchF", "x", 'lwF', wallDist-1)
    touchSensorL = Sensors.EventSensor("tchL", "y", 'lwL', wallDist-1)
    touchSensorB = Sensors.EventSensor("tchB", "x", 'lwB', wallDist-1)
    touchSensorR = Sensors.EventSensor("tchR", "y", 'lwR', wallDist-1)
    
    accelerometerX = Sensors.LineairSensor("sAX", "axR", 'laX')
    accelerometerY = Sensors.LineairSensor("sAY", "ayR", 'laY')
    #accelerometerZ = Sensors.LineairSensor("sAZ", "a", 'laZ')
    
    gyroscopez = Sensors.LineairSensor("sGZ", "vo", 'lo')
    
    initialValues = {var:0 for var in variables}
    initialValues['b'] = False
    
    states = States(stateDef, initialValues)
    return RealRobot("Omnibot2D",connection, states, [FMotor, LMotor, BMotor, RMotor], [accelerometerX, accelerometerY, gyroscopez, distanceSensorF, distanceSensorL, distanceSensorB, distanceSensorR, touchSensorF, touchSensorL, touchSensorB, touchSensorR, encoderF, encoderL, encoderB, encoderR], env2D)


### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###
if __name__ == "__main__":
    
    ser = serial.Serial("/dev/ttyACM0", 115200)
    
    #Robot = createRealRobot2Dsquare(ser)
    Robot = createRealRobot1D(ser, walls = (-100, 25))
    
    i = 0
    while i < 10:
        i = i+1
        sensors = Robot.readSensors()
        sensors.update(Robot.readMotors())
        print(sensors)    
        
        