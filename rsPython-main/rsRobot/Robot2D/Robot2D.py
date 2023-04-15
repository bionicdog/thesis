# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 14:25:12 2022

@author: woute
"""
import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from rsLibrary.Monitor import gMonitorT, createMonitorT
from State import States, StateDef
from Environment2D import Environment2DSquare
import Motors
import Sensors
import Sensors2D
from Robots import SimRobot
from scipy.stats import rv_continuous as distribution
from math import pi, cos, sin

              
def createSimRobot2DSquareOnlyVelocity (wallDist:int = 100, noiseLevel: int = 0):
    """
    2D-world (x and y) with walls at x=wallDist and y=wallDist, no acceleration 
    variables. Motors determine velocity immediately.
    The variables correspond to the relative coordinate system of the robot.
    Robot can move in x and y direction simultaneously, similar to an omniwheel
    robot. A third motor is added that acts as a steering wheel, to simulate
    rotation.
    """
         
    env2D = Environment2DSquare(wallDist = wallDist, accvars = None)
    variables = ['vx', 'vy', 'x', 'y']
    # orientation 'o' corresponds to the angle between the coordinate systems 
    # of the robot (relative) and the environment (absolute)
    variables.extend(['vo', 'o'])
    # speed variables in the relative coordinate system
    variables.extend(['vxR', 'vyR'])
    
    relationDict ={'vx':'vxR', 'vy':'vyR', 'x': 'vx', 'y': 'vy', 'o': 'vo'}
    #No acceleration, vxR/vyR/vo -> independent variables
    def forwardMath2D(states, t, dts):
        # relation between two coordinate systems = orientation
        states.setValue('o', t, (states['vo'][t] * dts[t] + states['o'][t - 1])%(2*pi))
        # relative CS
        states.setValue('vx', t, states['vxR'][t]*cos(states['o'][t]) - states['vyR'][t]*sin(states['o'][t]))
        states.setValue('vy', t, states['vxR'][t]*sin(states['o'][t]) + states['vyR'][t]*cos(states['o'][t]))
        # absolute CS
        states.setValue('x', t, states['vx'][t] * dts[t] + states['x'][t - 1])
        states.setValue('y', t, states['vy'][t] * dts[t] + states['y'][t - 1])
        
         
    stateDef = StateDef(variables, relationDict, forwardMath2D)
    
    #stateDef.indepVariables.remove('vo') # we do not want these state variables to be changed by the estimation
  #  stateDef.indepVariables.remove('b') 

    
    # two motors for velocity: one in both directions x and y
    ourMotorX = Motors.LineairMotor("moVx", statevars=['vxR'], paramName = 'lmX', paramValue=10)
    ourMotorY = Motors.LineairMotor("moVy", statevars=['vyR'], paramName = 'lmy', paramValue=10)
    # one motor for angular speed
    ourMotorO = Motors.LineairMotor("moVo", statevars=['vo'], paramName = 'lmO', paramValue=10)
    
    motors = [ourMotorX, ourMotorY, ourMotorO]
    
    from scipy.stats import norm
    noiseDistribution = norm(scale = noiseLevel) if noiseLevel > 0 else None
    
    encoderX = Sensors.LineairSensor("sX", "x", 'lx', 10, noiseDistr = noiseDistribution)
    encoderY = Sensors.LineairSensor("sY", "y", 'ly', 10, noiseDistr = noiseDistribution)
    distanceSensorX = Sensors2D.UnidirectionalDistanceSensor("dstX", env2D, paramName='ldX', maxVal = 2000, orientationOffset=0)
    distanceSensorY = Sensors2D.UnidirectionalDistanceSensor("dstY", env2D, paramName='ldY', maxVal = 2000, orientationOffset=pi/2)
    orientationSensor = Sensors.LineairSensor("sO", "o", 'lo', 10, noiseDistr = noiseDistribution)
    #TODO: improve EventSensor to detect the crossing of Walls(=lines) instead
    # of threshold values (=points). This works in a square environment only
    touchSensorX = Sensors.EventSensor("tchX", "x", 'lwx', wallDist-1)
    touchSensorY = Sensors.EventSensor("tchY", "y", 'lwy', wallDist-1)
    
    sensors = [encoderX, encoderY, distanceSensorX, distanceSensorY, orientationSensor, touchSensorX, touchSensorY]
    
    # global monitor
    createMonitorT(stateDef.variables)
    
    states = States(stateDef, {'vx':0, 'vy':0, 'vo':0, 'x':0, 'y':0, 'o':0, 'vxR':0, 'vyR':0, 'b':False})
    return SimRobot("robot sim 2D only pos", states, motors, sensors, env2D)


def createSimRobot2DSquare (wallDist:int = 100, noiseLevel: int = 0):
    env2D = Environment2DSquare(wallDist=wallDist)
    # state variables in the absolute coordinate system
    variables = ['ax', 'ay', 'vx', 'vy', 'x', 'y']
    # angular acceleration/rotation/position
    variables.extend(['ao', 'vo', 'o'])
    # relative coordinate system
    variables.extend(['axR', 'ayR', 'vxR', 'vyR'])
    
    relationDict = {'vxR':'axR', 'vyR':'ayR', 'vo':'ao', 'ax':'axR', 'ay':'ayR','vx':'ax', 'vy':'ay', 'x': 'vx', 'y': 'vy', 'o': 'vo'}
    
    def forwardMath2D(states, t, dts):
        # orientation robot
        states.setValue('vo', t, (states['ao'][t] * dts[t] + states['vo'][t - 1]))
        states.setValue('o', t, ((states['vo'][t-1] + states['vo'][t]) / 2 * dts[t] + states['o'][t - 1])%(2*pi))
        # relative coordinate system (aR are independent variables)
        states.setValue('vxR', t, states['axR'][t] * dts[t] + states['vxR'][t - 1]) 
        states.setValue('vyR', t, states['ayR'][t] * dts[t] + states['vyR'][t - 1]) 
        # absolute coordinate system (a depends on aR)
        states.setValue('ax', t, states['axR'][t]*cos(states['o'][t]) - states['ayR'][t]*sin(states['o'][t]))
        states.setValue('ay', t, states['axR'][t]*sin(states['o'][t]) + states['ayR'][t]*cos(states['o'][t]))
        states.setValue('vx', t, states['vxR'][t]*cos(states['o'][t]) - states['vyR'][t]*sin(states['o'][t])) 
        states.setValue('vy', t, states['vxR'][t]*sin(states['o'][t]) + states['vyR'][t]*cos(states['o'][t])) 
        states.setValue('x', t, (states['vx'][t-1] + states['vx'][t]) / 2 * dts[t] + states['x'][t - 1])
        states.setValue('y', t, (states['vy'][t-1] + states['vy'][t]) / 2 * dts[t] + states['y'][t - 1])
        
        
    stateDef = StateDef(variables, relationDict, forwardMath2D)
    
    #two motors determine the acceleration in x and y direction
    ourMotorX = Motors.OurMotor("mX", ['axR', 'vxR'], param1Value=1, param2Value=1)
    ourMotorY = Motors.OurMotor("mY", ['ayR', 'vyR'], param1Value=1, param2Value=1)
    #one motor determines angular acceleration
    ourMotorO = Motors.OurMotor("mO", ['ao', 'vo'], param1Value=1, param2Value=1)
    
    motors = [ourMotorX, ourMotorY, ourMotorO]
    
    from scipy.stats import norm
    noiseDistribution = norm(scale = noiseLevel) if noiseLevel > 0 else None
    
    accelerometerX = Sensors.LineairSensor("sAXr", "axR", 'laX', 10, noiseDistr = noiseDistribution)
    accelerometerY = Sensors.LineairSensor("sAYr", "ayR", 'laY', 10, noiseDistr = noiseDistribution)
    odometerX = Sensors.LineairSensor("sVXr", "vxR", 'lvX', 10, noiseDistr = noiseDistribution)
    odometerY = Sensors.LineairSensor("sVYr", "vyR", 'lvY', 10, noiseDistr = noiseDistribution)
    distanceSensorX = Sensors2D.UnidirectionalDistanceSensor("dstX", env2D, paramName='ldX', maxVal = 2000, orientationOffset=0)
    distanceSensorY = Sensors2D.UnidirectionalDistanceSensor("dstY", env2D, paramName='ldY', maxVal = 2000, orientationOffset=pi/2)
    encoderX = Sensors.LineairSensor("sX", "x", 'lx', 10, noiseDistr = noiseDistribution)
    encoderY = Sensors.LineairSensor("sY", "y", 'ly', 10, noiseDistr = noiseDistribution)
    compass = Sensors.LineairSensor("sO", "o", 'lo', 10, noiseDistr = noiseDistribution)
    touchSensorX = Sensors.EventSensor("tchX", "x", 'lwx', wallDist-1)
    touchSensorY = Sensors.EventSensor("tchY", "y", 'lwy', wallDist-1)
    
    sensors = [accelerometerX, accelerometerY, odometerX, odometerY, encoderX, encoderY, distanceSensorX, compass, distanceSensorY, touchSensorX, touchSensorY]
    
    # global monitor
    createMonitorT([m.name for m in motors], stateDef.variables, [s.name for s in sensors])
    #createMonitorT(stateDef.variables)
    
    initialValues = {var:0 for var in variables}
    initialValues['b'] = False
    states = States(stateDef, initialValues)
    return SimRobot("robot sim 2D", states, motors, sensors, env2D)

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Robot2D.py ***')
    
    FLAG_ACCELERATION = True
    
    def testRobot2D(simRobot, motorInputs):
        simRobot.printConfiguration()
        
        motorNames = [motor.name for motor in simRobot.motors]

        #running
        gMonitorT().printTitle()
        gMonitorT().printLastFrame()
        for i in range(0, len(motorInputs)):
            t = i + 1
            motor_input = {}
            for name, value in zip(motorNames, motorInputs[i]):
                motor_input[name] = value
            gMonitorT().setValues( motor_input, t )
            simRobot.setMotors(motor_input)
            
            sensorValues = simRobot.readSensors()
            gMonitorT().setValues( sensorValues , t)
            gMonitorT().printLastFrame()
        
    if FLAG_ACCELERATION:
        simRobot = createSimRobot2DSquare(wallDist = 15, noiseLevel = 0)
        #[mX, mY, mO]
        motor_input = [[5, 0, 0], [5, 0, 0], [0, 0, pi/2], [0, 0, 0], [0, -4, 0], [0,-6,0], [1,2,0], [3, 3, 0]]  
        #TODO: check weird behaviour when turning: this actually causes a
        # negative acceleration (caused by the speed component in the motor eq)
        # in a direction where the absolute speed is 0
        testRobot2D(simRobot, motor_input)
        
        
    
    if not FLAG_ACCELERATION:
        simRobot = createSimRobot2DSquareOnlyVelocity(wallDist = 80, noiseLevel = 0)
        #[moVx, moVy, moVo]
        motor_input = [[780, 0, 0], [30, 0, 0], [780, 0, pi/2*10], [20, 40, 0], [20, 20, 0], [0, 40, 0], [20, 40, 0], [-50, 0, pi/4*10]]  
        
        testRobot2D(simRobot, motor_input)
    
            
    