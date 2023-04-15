# -*- coding: utf-8 -*-
"""
Created on Wed May  4 10:44:23 2022

@author: woute
"""

import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..')


from Environment2D import Environment2DSquare
from EnvironmentWithObstacles import Environment2DWithObstacles, createSquareEnvironmentWithRandomObstacles
from rsLibrary.Monitor import gMonitorT, createMonitorT
import Robot2D
import RobotWithGripper
import Sensors2D
import Robots
import matplotlib.pyplot as plt
from math import pi, cos, sin


class Visualizer():
    def __init__(self, robot):
        self.robot = robot
        
    def __plotEnvironment(self, environment):
        def plotWall(wall, color='b'):
            x = [wall.pointA[0], wall.pointB[0]]
            y = [wall.pointA[1], wall.pointB[1]]
            plt.plot(x, y, color)
        def plotObject(obj, color='mo'):
            plt.plot(obj.x, obj.y, color)
            
        try:
            for wall in environment.walls:
                plotWall(wall)
            for obj in environment.objects:
                plotObject(obj)
        except AttributeError as e:
            print(e)
        
    
    def __plotUnidirectionalDistanceSensor(self, sensor, t):
        x = self.robot.states[sensor.statevars[0]][t]
        y = self.robot.states[sensor.statevars[1]][t]
        o = self.robot.states[sensor.statevars[2]][t]+sensor.orientationOffset
        dist = sensor.environment.distance2wall(x, y, o)
        x2 = x+dist*cos(o)
        y2 = y+dist*sin(o)
        plt.plot([x, x2], [y, y2], ':')
    
    def __plotRobot(self, t):
        self.__plotEnvironment(self.robot.environment)
        x = self.robot.states['x'][t]
        y = self.robot.states['y'][t]
        plt.plot(x, y, 'ro')
        # vx = self.robot.states['vx'][t]
        # vy = self.robot.states['vy'][t]
        vx = 0
        vy = 0
        # print(vx, vy)
        for sensor in self.robot.sensors:
            if isinstance(sensor, Sensors2D.UnidirectionalDistanceSensor):
                self.__plotUnidirectionalDistanceSensor(sensor, t)
        if vx!=0 or vy!=0:
            plt.arrow(x, y, vx, vy, head_width=0.8, length_includes_head=True)
    
    def plotRobot(self, t):
        self.__plotRobot(t)
    
    def showRobot(self):
        self.plotRobot(-1)
        plt.show()

    
    
    def plotDistanceVsAngle(self, t):
        NR_POINTS = 1000
        angles = [a*1/NR_POINTS for a in range(0, int(2*pi*NR_POINTS))] 
        x = self.robot.states['x'][t]
        y = self.robot.states['y'][t]
        distances = [self.robot.environment.distance2wall(x, y, angle) for angle in angles]
        plt.plot(angles, distances)
        plt.xlabel('angle (rad)')
        plt.ylabel('distance')

    
    
    

if __name__ == "__main__":
    FLAG_ACCELERATION = False
    FLAG_GRIPPER = True
    
    if FLAG_ACCELERATION:
        rob = Robot2D.createSimRobot2DSquare(wallDist = 15, noiseLevel = 0)
        motorInputs = [[5, 0, 0], [5, 0, 0], [0, 0, pi/2], [0, 0, 0], [0, -4, 0], [0,-6,0], [1,2,0], [3, 3, 0]]  
        
    else:
        if not FLAG_GRIPPER:
            #motorInputs = [[50, 0, 0], [30, 0, 0], [20, 0, pi/2*10], [20, 40, 0], [20, 20, 0], [0, 40, 0], [20, 40, 0], [-500, 0, pi/4*10]]  
            motorInputs = [[650, 0, 0], [100, 0, 0], [-60, 0, 0], [200, 0, 0], [-200, 0, 0], [0, 0, 0], [-900, -0, 0], [-800, 0, 0], [0, 0, 0]]
            rob = Robot2D.createSimRobot2DSquareOnlyVelocity(wallDist = 80, noiseLevel = 0)
        else:
            env = createSquareEnvironmentWithRandomObstacles(wallDist=80, nrObjects=2)
            rob = RobotWithGripper.createSimRobotWithGripper2DOnlyVelocity(env)
            motorInputs = [[650, 0, 0, 0], [100, 0, 0, 0], [-60, 0, 0, 0], [200, 0, 0, 0], [-200, 0, 0, 0], [0, 0, 0, 0], [-900, -0, 0, 0], [-800, 0, 0, 0], [0, 0, 0, 0]]
        
        
    vis = Visualizer(rob)
    motorNames = [motor.name for motor in rob.motors]
    print(motorNames)
    #running
    gMonitorT().printTitle()
    gMonitorT().printLastFrame()
    for i in range(0, len(motorInputs)):
        t = i + 1
        motor_input = {}
        for name, value in zip(motorNames, motorInputs[i]):
            motor_input[name] = value
        gMonitorT().setValues( motor_input, t )
        rob.setMotors(motor_input)
        
        sensorValues = rob.readSensors()
        gMonitorT().setValues( sensorValues , t)
        gMonitorT().printLastFrame()
        plt.subplot(121)
        vis.plotRobot(t)
        plt.subplot(122)
        vis.plotDistanceVsAngle(t-1)
        plt.show()
        
        #BUG: LAST FRAME IN WALL
        
    