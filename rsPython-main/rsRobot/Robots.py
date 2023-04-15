#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot.py: defines a robot
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from State import StateDef, States
from Environment import Environment, Environment1D

import Motors
import Sensors
from Control import RobotControl
# from Constraints import RobotConstraintGenerator

class Robot:
    
    def __init__(self, name:str, motors:list, sensors:list, environment:Environment=None):
        self.name = name
        self.motors = motors
        self.sensors = sensors
        self.motorsAndSensors = []
        self.motorsAndSensors.extend(self.motors)
        self.motorsAndSensors.extend(self.sensors)
        self.environment = environment
    
    def robot2motor(robotCommand):
        """  turns robot command into motor input  """
        raise NotImplementedError
    
    def setMotors(self, motorInputs:dict, t):
        for motor in self.motors:
            if motor.name in motorInputs:
                motor.setMotorInput( motorInputs[motor.name], t )
            else:
                motor.setMotorInput( 0, t )
        
    def readSensors(self):
        return {0, 0}

    def getMotorOrSensor(self, name: str):
        for m in self.motors:
            if m.name == name:
                return m
        for s in self.sensors:
            if s.name == name:
                return s
        return None

    def motorsAndSensors(self) -> list:
        return self.motors + self.sensors
    
    def printConfiguration(self):
        print(' == Robot '+self.name+ ' configuration ==')
        for motor in self.motors:
            print('Motor '+motor.name+': ', end ='')
            motor.printMotorFunc()
        for sensor in self.sensors:
            print('Sensor '+sensor.name+': ', end='')
            sensor.printObserFunc()
    def motorAndSensorData2List(self) -> list:
        data_list = []
        for motor_sensor in self.motorsAndSensors:
            for idx, v in enumerate(motor_sensor.values):
                if idx >= len(data_list):
                    data_list.append({})
                data_list[idx][motor_sensor.name] = v                
        return data_list
            
class SimRobot(Robot):
    
    def __init__(self, name:str, states: States, motors:list, sensors:list, environment: Environment, maxDeltas:dict = {'o':6}):
        super().__init__(name, motors, sensors, environment)
        self.states = states
        self.stateDef = states.stateDef
        self.t = 0
        # self.constraintGen = RobotConstraintGenerator(self, maxDeltas)
        
    def setMotors(self, motorInputs:dict):
        '''
        Function assumes that each motor independently sets some of the state variables
        Override this method when assumption is not met
        '''
        
        self.t = self.t + 1
        super().setMotors(motorInputs, self.t)  # to store all input values
        
        self.states.addFrame()
        for motor in self.motors:
            if motor.name in motorInputs:
                state_change = motor.motorFunc(motorInputs[motor.name], self.states, self.t)
            else:
                state_change = motor.motorFunc(0, self.states, self.t)
            self.states.setValues( state_change, self.t)
        self.states.calcDepVars(self.t)
        
        #  After setting the state variables by the motors, check the environment whether the action is possible.
        self.environment.checkState(self.states, True)
        
        for sensor in self.sensors:
            sensor.generateObservation(self.states)
        
    def readSensors(self):
        return { sensor.name : sensor.value for sensor in self.sensors }

    def printConfiguration(self):
        super().printConfiguration()
        if self.environment != None:
            print('Environment: ', end ='')
            self.environment.print()
    
    def calculateCovariances(self, printit:bool = False):
        for motorSensor in self.motorsAndSensors:
            motorSensor.analyzeResiduals()
            if printit:
                print('Sensor '+motorSensor.name+": mean = {:.2f}, stddev = {:.2f}, cov = {:.2f}.".format(motorSensor.residualsMean, motorSensor.residualsStddev, motorSensor.residualsCov) )
         
    def addVariables(self, vardict:dict):
        #add a list of independent variables to the states and statedef
        self.stateDef.addIndependentVars(vardict.keys())
        for k in vardict.keys():
            self.states.addVariable(k, vardict[k])
    
    # def getQState(self, motorBabbleValue=0.1):
    #     return QualitativeState(StateSpaceSample(self, motorBabbleValue))
        
class LogRobot(RobotControl, Robot):

    def __init__(self, name, data:list):
        super(RobotControl, self).__init__(name, [], [])
        self.data = data
        self.nbrFrames = len(data)
        self.t=0
        self.currentFrame = self.data[0] if self.nbrFrames > 0 else None
        
    def hasNext(self) -> bool:
        if self.t < self.nbrFrames:
            self.currentFrame = self.data[self.t]
            self.t += 1
            return True
        else:
            return False

    def nextCommand(self):
        return self.currentFrame
    
    def setMotors(self, motorInputs:dict):
        super().setMotors(motorInputs, self.t)
        
    def readSensors(self) -> dict:
        return self.currentFrame

    def motorAndSensorNames(self) -> list:
        return [k for k in self.currentFrame] if self.nbrFrames > 0 else []

#### #### #### ####  Robot Instantiations #### #### #### #### 

def createSimRobot1D (walls:tuple = (-100, 100), noiseLevel:int = 0):

    environment1D = Environment1D(walls)
    variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
    relationDict ={'v': 'a', 'x': 'v'}    
    def forwardMath1D(states, t, dts):
        states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
        states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
    stateDef = StateDef(variables, relationDict, forwardMath1D)
    
    ourMotor = Motors.OurMotor("mX", ["a", "v"], param1Value = 1, param2Value = 1)
    
    from scipy.stats import norm
    noiseDistribution = norm(scale = noiseLevel) if noiseLevel > 0 else None
    
    accelerometer = Sensors.LineairSensor("sA", "a", 'la', 10, noiseDistr = noiseDistribution)
    odometer = Sensors.LineairSensor("sV", "v", 'lv', 10, noiseDistr = noiseDistribution)
   # distanceSensor = Sensors.LineairSensor("sX", "x", 'lx', 10, noiseDistr = noiseDistribution)
    distanceSensor =  Sensors.DistanceSensor("sX", "x", paramWallValue = walls[1], noiseDistr = noiseDistribution) 
    touchSensor = Sensors.EventSensor("tch", "x", 'lw', environment1D.walls[1])
    
    states = States(stateDef, {'a':0, 'v':0, 'x':0})
    return SimRobot("robot sim 1D", states, [ourMotor], [accelerometer, odometer, distanceSensor, touchSensor], environment1D)


def createSimRobot1DOnlyVelocity (walls:tuple = (-100, 100), noiseLevel:int = 0):
    environment1D = Environment1D( walls, accvar = None)
    # simple model with motor immediately determining velocity
    # velocity is constant in the period
    variables = ['v', 'x']
    relationDict ={'x': 'v'}    
    def forwardMath1D(states, t, dts):
        states.setValue('x', t, states['v'][t] * dts[t] + states['x'][t - 1])
    stateDef = StateDef(variables, relationDict, forwardMath1D)
    
    ourMotor = Motors.LineairMotor("moV", statevars=['v'], paramValue=10) # VelocityMotor("moV")
    
    from scipy.stats import norm
    noiseDistribution = norm(scale = noiseLevel) if noiseLevel > 0 else None

    distanceSensor = Sensors.LineairSensor("sX", "x", 'lx', 10, noiseDistr = noiseDistribution)
    touchSensor = Sensors.EventSensor("tch", "x", 'lw', environment1D.walls[1])  
     
    sensors = [distanceSensor, touchSensor]
    
    # global monitor
    createMonitorT("moV", stateDef.variables, [s.name for s in sensors])
    
    states = States(stateDef, {'v':0, 'x':0})
    return SimRobot("robot sim 1D only pos", states, [ourMotor], [distanceSensor, touchSensor], environment1D)

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Robots.py ***')
    from Control import ControlByArray
    
    if True:
        
        simRobot = createSimRobot1D(walls = (-100, 5), noiseLevel = 0)
        simRobot.printConfiguration()
        
         # global monitor
        createMonitorT('dt', 'mX', simRobot.stateDef.variables, [s.name for s in simRobot.sensors]).printTitle()
    
        
        motor = simRobot.motors[0].name
        motor_input = [1, 1, 2, 2, -2, -2]  
        
        # running   
        ctrl = ControlByArray( motor_input )
        t = 1
        while ctrl.hasNext():
            cmd = ctrl.nextCommand()
            key_value = {motor : cmd}
            gMonitorT().setValues( key_value, t )
            simRobot.setMotors( key_value)
            sensorValues = simRobot.readSensors()
            gMonitorT().setValues( sensorValues, t )
            gMonitorT().printLastFrame()
            t += 1
        
        
        data = simRobot.motorAndSensorData2List()
        
        if False:
            import pickle
            file = open('dataOfSimRobot1D.pkl', 'wb')
            pickle.dump(data, file)
            print('Motor and sensor data written to pickle file '+file.name)
            file.close()
        
    if False:
        simRobot = createSimRobot1DOnlyVelocity(walls = (-100, 100), noiseLevel = 5)
        simRobot.printConfiguration()
        
        motor = simRobot.motors[0].name
        motor_input = [10, 10, 20, 20, -20, -20]  
        
        # running   
        gMonitorT().printTitle()
        gMonitorT().printLastFrame()
        for i in range(0, len(motor_input)):
            t = i + 1
            key_value = {motor : motor_input[i]}
            gMonitorT().setValues( key_value, t )
            simRobot.setMotors(key_value)
            sensorValues = simRobot.readSensors()
            gMonitorT().setValues( sensorValues , t)
            gMonitorT().printLastFrame()
            
    ## replay log
    if False:
        import pickle
        file = open('data2.1.pkl', 'rb')
        data = pickle.load(file)
        file.close()
        print(' * * *  Replaying robot data of '+file.name+' * * *')
        
        logRobot = LogRobot(file.name, data)
       
        print('Variables in file: '+str(logRobot.motorAndSensorNames()))
        # global monitor
       # createMonitorT(logRobot.motorAndSensorNames())  # all data items
        createMonitorT('mF', 'mB', 'sDL', 'sEF', 'sEB', 'sAY') # use this to select the sensors you want to see


        # running   
        gMonitorT().setColumnWidth(8)
        gMonitorT().printTitle()
        gMonitorT().printLastFrame()
        
        while logRobot.hasNext():
            robot_commands = logRobot.nextCommand()
            
            logRobot.setMotors(robot_commands)
            gMonitorT().setValues(robot_commands)
            
            sensorValues = logRobot.readSensors()
            gMonitorT().setValues( sensorValues )
            
            gMonitorT().printLastFrame()

        