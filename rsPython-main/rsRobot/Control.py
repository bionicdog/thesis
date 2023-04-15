#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Control.py
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - December 2021
"""
from rsLibrary.Monitor import gMonitorT
import time
#### #### #### ####  RobotControl  #### #### #### #### 
class RobotControl:
    """
    steers the robot.
    Is implemented as an iterator.

    """
    def hasNext(self):
        """
        returns false when the experiment has ended

        """
        raise NotImplementedError

    def nextCommand(self):
        """
        returns the next command for the robot

        """
        raise NotImplementedError
    
#### #### #### ####  Control Implementations  #### #### #### #### 

class ControlByArray(RobotControl):
    
    def __init__(self, commands: list):
        self.commands = commands;
        self.i = 0
        
    def hasNext(self):
        return self.i < len(self.commands)

    def nextCommand(self):
        cmd = self.commands[self.i]
        self.i += 1
        return cmd

# a control to give the robot a certain speed, used to stop the robot with speed = 0
class ControlBySpeed(RobotControl):
    
    def __init__(self, speed):
        self.speed = speed;
        self.steps = 0
    def hasNext(self):
        return True

    def nextCommand(self):
        return self.speed
    
    def timeFunc(self):
        self.steps += 1

class Control1DTurn180(RobotControl):
    
    def __init__(self, speed, maxSteps):
        self.speed = speed;
        self.step = 0
        self.steps = 0
        self.maxSteps = maxSteps
        self.startTime = 0
    
    def hasNext(self):
        return (self.steps < self.maxSteps)

    def nextCommand(self, time):
        self.steps += 1
        output = 0
        if self.step == 0:
            self.startTime = time
            self.step += 1
            
        if self.step == 1:
            output = self.speed + 1000
            
            if abs(self.startTime - time) > 5:
                self.step += 1
                
        return output
    

class KeyboardControl(RobotControl):
    def __init__(self, _motors, _keyboard):
        self.motors = _motors;
        self.keyboard = _keyboard
        
    def hasNext(self):
        raise NotImplementedError

    def nextCommand(self):
        raise NotImplementedError    
    
    # this is code from Nikolai
    def motorInput(self):
        ret = [0, 0]
        if self.keyboard.direction[0] == 1:
            ret[0] = 1
        if self.keyboard.direction[1] == 1:
            ret[1] = -1
        if self.keyboard.direction[2] == 1:
            ret[0] = -1
        if self.keyboard.direction[3] == 1:
            ret[1] = 1
        return ret


from Environment import Environment1D
class Control1DForwardUntilWall(RobotControl):
    
    def __init__(self, environment1D: Environment1D, nbrSteps: int = 20, motorInput:int = 5):
        self.environment1D = environment1D
        self.nbrSteps = nbrSteps
        self.motorInput = motorInput
        self.steps = 0
        
        
    def hasNext(self):
        return self.steps < self.nbrSteps

    def nextCommand(self):
        self.steps += 1
        if self.environment1D.touchedWall:
            self.motorInput = -self.motorInput
        return self.motorInput    
    
class Control1DForwardUntilX(RobotControl):
    
    def __init__(self, nbrSteps: int = 20, motorInput:int = 5, x:int=52):
        self.nbrSteps = nbrSteps
        self.motorInput = motorInput
        self.steps = 0
        self.x = x
        
        
    def hasNext(self):
        return self.steps < self.nbrSteps

    def nextCommand(self):
        self.steps += 1
        print(gMonitorT()['ex'][len(gMonitorT()['ex'])-2])
        print(self.x)
        if gMonitorT()['ex'][len(gMonitorT()['ex'])-2] > self.x:
            self.motorInput = 0
        return self.motorInput  
        
class Control1DForwardAndBackForCalibration(RobotControl):
    
    def __init__(self, nbrSteps: int = 2000, speed:int = 5):
        self.nbrSteps = nbrSteps
        self.speed = speed
        self.motorInput = speed
        self.steps = 0
        self.step = 0
        self.T = 0
        
    def hasNext(self):
        return self.steps < self.nbrSteps

    def nextCommand(self):
        self.steps += 1
        print('control: ' + str(self.T) + ' diff: ' + str(self.T - time.time()))
        if self.step == 0:
            self.T = time.time()
            self.step += 1
            
        if self.step == 1:
            if abs(self.T - time.time()) > 6:
                self.motorInput = 0
                self.step += 1
                self.T = time.time()
                
        if self.step == 2:
            if abs(self.T - time.time()) > 2:
                self.motorInput = -self.speed
                self.step += 1
                
        if self.step == 3:
            if abs(self.T - time.time()) > 6:
                self.motorInput = 0
                self.steps = 2000
                
        return self.motorInput
    
class Control1DUntilWallWithAcc(RobotControl):
    
    def __init__(self, environment1D: Environment1D, nbrSteps: int = 20, speed:int = 5):
        self.environment1D = environment1D
        self.nbrSteps = nbrSteps
        self.motorInput = speed
        self.steps = 0
        self.a = 1
        
        
    def hasNext(self):
        return self.steps < self.nbrSteps

    def nextCommand(self, time):
        
        v = gMonitorT()['ev'][len(gMonitorT()['ev'])-1]
        
        self.steps += 1
        
        if self.environment1D.touchedWall:
            self.motorInput = -self.motorInput
            self.a = 1
        
        #if standing still reset acceleration 
        if abs(v) < 0.1:
            self.a = 1
            
        nextCommand = self.motorInput * (self.a/10)
        
        if self.a < 11:
            self.a += 0.5
            
        return nextCommand    

from scipy.stats import uniform
class Control1DForwardUntilWallRandom(RobotControl):
    ''' same as previous one, but varying speed randomly from uniform distribution '''
    def __init__(self, environment1D: Environment1D, nbrSteps: int = 20, motorInputMin:int = 5, motorInputMax:int = 15):
        self.environment1D = environment1D
        self.nbrSteps = nbrSteps
        self.motorInputMin = motorInputMin
        self.motorInputMax = motorInputMax
        self.uniformDistr = uniform(motorInputMin, motorInputMax)
        self.steps = 0
        self.direction = 1 # 1 or -1
        
        
    def hasNext(self):
        return self.steps < self.nbrSteps

    def nextCommand(self):
        self.steps += 1
        if self.environment1D.touchedWall:
            self.direction = -self.direction

        return self.uniformDistr.rvs(1)[0] * self.direction  
    
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Control.py ***')
    
    if False:
        ctrl = ControlByArray([10, 15, 5, 10, -20, -20]  )
        while ctrl.hasNext():
            cmd = ctrl.nextCommand()
            print('Next command: '+str(cmd))
    
    if True:
        from Robots import createSimRobot1D
        from rsLibrary.Monitor import gMonitorT, createMonitorT
        import Sensors
        
        walls = (-10, 32)
        simRobot = createSimRobot1D(walls = walls)
        simRobot.sensors.append(Sensors.EventSensor('tch2', 'x', 'lw', walls[0]))
        simRobot.sensors.append(Sensors.DistanceSensor('dist', 'x', 'lw', walls[1]))
        simRobot.sensors.append(Sensors.DistanceSensor('dist2', 'x', 'lw', walls[0]))
        simRobot.printConfiguration()
        
         # global monitor
        monitorT = createMonitorT('cmd', 'dt', 'T', "mX", simRobot.stateDef.variables, 'tch', 'tch2', 'dist', 'dist2')
        monitorT.printTitle()
      #  ctrl = Control1DForwardUntilWall(simRobot.environment, nbrSteps = 30, motorInput = 5)
        ctrl = Control1DForwardUntilWallRandom(simRobot.environment, nbrSteps = 30, motorInputMin = 5, motorInputMax = 15)
        
        
        while ctrl.hasNext():
            cmd = ctrl.nextCommand()
          #  print('Next command: '+str(cmd))
            simRobot.setMotors({"mX" : cmd})
            monitorT.setValue('cmd', cmd)
            sensorValues = simRobot.readSensors()
            monitorT.setValues( sensorValues )
            monitorT.printLastFrame()
            
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### ### #### #### #### #### #### #### ###