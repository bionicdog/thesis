# -*- coding: utf-8 -*-
"""
Created on Thu Jul 14 09:01:58 2022

@author: Nick
"""

import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from rsLibrary.Monitor import gMonitorT, createMonitorT
from Robot2D import createSimRobot2DSquareOnlyVelocity
from EnvironmentWithObstacles import Environment2DWithObstacles
from Robots import SimRobot
import Sensors
import Motors

def addGripperToSimRobot(robot:SimRobot, noiseLevel:int = 0):
    robot.name+=" with gripper" 
    
    from scipy.stats import norm
    noiseDistribution = norm(scale = noiseLevel) if noiseLevel > 0 else None
    #variables for gripper:
    #   p = stretch of the spring, g = angle of gripper motor, 
    #   prox = discrete proximity of object
    gripperVariables = {'p':0, 'g':0, 'prox':0}
    robot.addVariables(gripperVariables)
    #motors and sensors for gripper
    potentiometerGripper = Sensors.LineairSensor("sP", 'p', 'lp', 10, noiseDistr = noiseDistribution)
    servoGripper = Motors.LineairMotor("moG", statevars=['g'], paramName='lmG', paramValue=10)
    proximitySensor = Sensors.EventSensor("sProx", "prox", "lProx", paramValue=2)
    robot.motors.extend([servoGripper])
    robot.sensors.extend([potentiometerGripper, proximitySensor])
    return robot

def createSimRobotWithGripper2DOnlyVelocity(env:Environment2DWithObstacles, noiseLevel:int = 0):
    #no differential drive like Sabil's
    #also no elbow for the gripper
    robot = createSimRobot2DSquareOnlyVelocity(noiseLevel = noiseLevel)
    robot.environment = env
    return addGripperToSimRobot(robot, noiseLevel)
    
    
if __name__== "__main__":
    rob = createSimRobotWithGripper2DOnlyVelocity(None) #just to test robot creation
    print(rob.motors)
    print(rob.motorsAndSensors)
    print(rob.name)
