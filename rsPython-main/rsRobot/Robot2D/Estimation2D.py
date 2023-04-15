# -*- coding: utf-8 -*-
"""
Created on Fri Apr  8 2022

@author: woute
"""

import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from rsLibrary.Monitor import gMonitorT, createMonitorT
from Environment2D import Environment2D, Environment2DSquare
from Robot2D import createSimRobot2DSquareOnlyVelocity
import RobotEstimation
from math import pi

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 

def dict2str(d:dict) -> str:
    l_f = [str(k)+': %.2f'%(d[k]) for k in d]
    return ', '.join(l_f) 


if __name__== "__main__":
    simRobot = createSimRobot2DSquareOnlyVelocity(80)
    simRobot.printConfiguration()
    
    robotModel = RobotEstimation.RobotModel("robotModel2D", simRobot)
    
    robotModel.getMotorOrSensor('tchX').isCalibrated = True
    robotModel.getMotorOrSensor('tchY').isCalibrated = True
    robotModel.getMotorOrSensor('moVy').isCalibrated = True
    robotModel.getMotorOrSensor('moVx').isCalibrated = True
    #robotModel.getMotorOrSensor('moVo').isCalibrated = True
    robotModel.getMotorOrSensor('sX').isCalibrated = True
    robotModel.getMotorOrSensor('sY').isCalibrated = True
    robotModel.getMotorOrSensor('dstX').isCalibrated = True
    robotModel.getMotorOrSensor('dstY').isCalibrated = True
    #robotModel.getMotorOrSensor('sO').isCalibrated = True
    
    print('Variables to be estimated: ', end='')
    print(str(robotModel.updateableVariableNames()))
    print('Dependent variables     : ', end='')
    print(str(robotModel.stateDef.depVariables))
    print(' == == == == == == == == == == == == == == == == == == == ==')
    
    # global monitor
#    createMonitorT("moVx", "moVy", "moVo", simRobot.stateDef.variables, "ex", "ey", "eo", "evx", "evy", "evxR", "evyR", "er0", "er1", "diff").printTitle()
#    createMonitorT("moVx", "moVy", "moVo", simRobot.stateDef.variables, "dstX", "dstY","sX", "ex", "ey", "eo", "evx", "evy", "evxR", "evyR", "erPRE", "erPOST", "diff").printTitle()
    createMonitorT("vo", "emoVo", "moVo", "moVx", "emoVx", "rmoVx", "eo", "rsO", "vx", "evx", "x", "ex", "sX", "rsX", "dstX", "edstX", "rdstX","tchX", "erPRE", "erPOST", "end", "diff").printTitle()
    
    motorNames = [motor.name for motor in simRobot.motors]
    print(motorNames[0])
#    motorInputs = [[650, 0, 0], [100, 0, 0], [-60, 0, 0], [200, 0, 0], [-200, 680, 0], [0, 400, 0], [-1000, -400, 0], [-800, 0, 0], [0, 0, 0]]
    motorInputs = [[650, 0, 40], [100, 0, 0], [-60, 0, 0], [200, 0, 0], [-200, 0, 0], [0, 0, 0], [-900, -0, 0], [-800, 0, 0], [0, 0, 0]]
    
    # running   
    for i in range(0, min(5, len(motorInputs))):
        t = i + 1
        print(' ============================= FRAME '+str(t)+' ============================')
        print('INPUT ', end='')
        
        motor_input = {}
        for name, value in zip(motorNames, motorInputs[i]):
            motor_input[name] = value
        gMonitorT().setValues( motor_input, t )
        simRobot.setMotors(motor_input)
       
        sensorValues = simRobot.readSensors()
        for name, value in zip(motorNames, motorInputs[i]): sensorValues[name] = value
        
        gMonitorT().setValues( sensorValues, t )
        print(' => OBSERVED ', sensorValues, end='')
        
        # link RobotSim to RobotModel + estimate
        robotModel.addSensorReading(sensorValues, t)
        
        #changes = robotModel.estimate(nbrTries = 20, printLevel = 0) # if t < 3 else 5)
        selectNewEstimate = True
        if not selectNewEstimate:
            changes = robotModel.estimate(nbrTries = 20 if t < 16 else 1, maxNbrDecreases = 10,  printLevel = 0 if t <16 else 5 )
        if selectNewEstimate:
            changes = robotModel.estimateWithValidRegions(nbrTries = 20 if t < 16 else 1, maxNbrDecreases = 10,  printLevel = 0 if t <16 else 5 )
        for k, v in changes.items():
            print('Unknown ', k, ' changed from  %.3f'%(v[0]) , 'to %.3f'%(v[1]))
        
        robotModel.diffWithRef(simRobot)
        
        estimated_variables = robotModel.updateableVariables()
        print(' => ESTIMATED ', dict2str(estimated_variables), end='')
        
        print('')
        gMonitorT().printAllFrames()
    
        if t > 2:
            robotModel.states.startUnknown = robotModel.states.now - 1
