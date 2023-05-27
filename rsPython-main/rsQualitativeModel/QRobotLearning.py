# -*- coding: utf-8 -*-
"""
QRobotLearning.py

Created on Dec  6  2022

@author: Jan Lemeire
"""

import time
import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from Control import RobotControl
from Robots import Robot, createSimRobot1D, createSimRobot1DOnlyVelocity, RobotWithGripper

from rsQualitativeModel.QDataType import Sign
from rsQualitativeModel.QHistory import QHistory
##from rsQualitativeModel.QBehaviorModel import QBehaviorModel

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       RUN ROBOTS        #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###   

np.random.seed(2023)

def simulateRobot(robot: Robot, robotControl: RobotControl) -> QHistory:
    # Based on robot code from File Robots
    robot.printConfiguration()
    gMonitorT().printTitle()
    gMonitorT().printLastFrame()
    
    start = time.time()
    hist = QHistory(*robot.variables())
    
    t = 0      
    while ctrl.hasNext():
         cmd = ctrl.nextCommand()
         t += 1
         if type(cmd) == dict:
             actions = cmd
         elif type(cmd) == list:
             actions = { motor.name: v for motor, v in zip(robot.motors, cmd)}
         else:
             actions = {robot.motors[0].name : cmd}
             
         
         dState = robot.setMotors( actions)
    
         gMonitorT().setValues( actions, t ) # MOVE TO ROBOTS
            
         sensorValues = robot.readSensors()
         
         gMonitorT().setValues( sensorValues , t) # MOVE TO ROBOTS
         gMonitorT().printLastFrame()
         
         state_frame = robot.states.getStateT(t)
         
         hist.add({**actions, **state_frame})
         
         
    stop = time.time()
    print("Time to simulate:", stop-start)
        
   # hist.plotHistory(*hist.allVars)
    return hist

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####         ROBOTS          #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###   
if __name__== "__main__":
    print('*** QRobotLearning.py ***')
    from Control import ControlByArray, ControlByArrays
    from QAnalysis import testForDet2D
    
    ROBOT = 1 # 1: 1D v  2: 1D a  3: RobotWithGripper
    
    if ROBOT == 1:
        
        simRobot = createSimRobot1DOnlyVelocity(walls = (-10, 10), noiseLevel = 0)
        
        motor_input = [10, 10, 0,  20, 0, 20, -10, -10, -20, -10, 0, 0, 10, 20, 20, 20 , 20 , 20, 20, 10, 20]  
        
        # Add extra random motor inputs for more data
        if True:
            for i in range(50):
                rand_val = np.random.uniform(-20, 20)
                motor_input.append(rand_val)

        ctrl = ControlByArray( motor_input )
        hist = simulateRobot(simRobot, ctrl)

        hist.addDerivedVariable('qv')
      #  hist.plot('moV', 'qv')
        
        hist.addDerivedVariable('qmoV')
      #  hist.plot('qmoV', 'qv')
        
        hist.addDerivedVariable('qx')
      #  hist.plot('qx', 'qv')
        
        hist.addDerivedVariable('x_old')
        hist.addDerivedVariable('qdx')
#        hist.plot('x_old', 'x', 'qdx') # identify x-relation
    
      #  testForDet2D(hist['x_old'], hist['x'], hist['qdx'], show=True)
        
        hist.addDerivedVariable('v_old')
        hist.addDerivedVariable('qdv')
        # hist.plot('v', 'v_old', 'qdv') # identify v-relation
       
        
        # print('Toont csi')
        # hist.plot('moV', 'x', 'qv') # mov affects v, except for some x-values
        
        hist.corrcoeffs()
        
        testForDet2D(hist['moV'], hist['x'], hist['qv'], show=True)
        
        if False:
            import pickle
            file = open('robot1History.pkl', 'wb')
            _data = {v.name: hist[v] for v in hist.variables  }
            
            pickle.dump(_data, file)
            print('History of robot 1 written to pickle file '+file.name)
            file.close()
        
    elif ROBOT == 2:
        
        simRobot = createSimRobot1D(walls = (-100, 100), noiseLevel = 0)
        
        motor_input = [1, 1, 2, 2, -2, -2, 0, 0, -1, -2, 1, 1, 0, 2, 2, 1]  
        # Add extra random motor inputs for more data
        for i in range(10):
            rand_val = np.random.random()*4-2
            motor_input.append(rand_val)
        
        ctrl = ControlByArray( motor_input )
        hist = simulateRobot(simRobot, ctrl)
        
        hist.addDerivedVariable('qa')
        
        hist.addDerivedVariable('qv')
        hist.addDerivedVariable('qdv')
        hist.addDerivedVariable('qmX')
        
        hist.addDerivedVariable('qx')
        
        hist.addDerivedVariable('v_old')
        hist.addDerivedVariable('qdv')
        
        hist.addDerivedVariable('x_old')
        hist.addDerivedVariable('qdx')
        
        #hist.plot('x_old', 'x', 'qdx')
        
        hist.plot('mX', 'v_old', 'qa') # het verband zien (a is target)
        testForDet2D(hist['mX'], hist['v_old'], hist['qa'], show=True)
        
        hist.plot('v_old', 'v', 'qdx') # om het verband te zien (is dus niet gewoon dx = v)
        testForDet2D(hist['v_old'], hist['v'], hist['qdx'], show=True)
        
        hist.corrcoeffs() 
       
       # qbehavior.printModel(plotData = True)
       
      ##  for s in qbehavior.qBehaviorFunctions:
      ##      if s in state_frame_old: break
     ##       func = qbehavior.qBehaviorFunctions[s]
    ##        func.printLevel=1
    ##        func.depSet = func.createNewdepSetIAMBMutInf()
    ##        func.plotDepSpace()
    ##        print("DepSet for ", 'd'+func.dstateVar+':', func.depSet)
    ##        func.printLevel=0   

        # qbehavior.qBehaviorFunctions['x'].depSet = [ 'v'] # 'or',
        # qbehavior.qBehaviorFunctions['x'].plotDepSpace()

        # qbehavior.qBehaviorFunctions['v'].depSet = [ 'a'] # 'or',
        # qbehavior.qBehaviorFunctions['v'].plotDepSpace()

        # qbehavior.qBehaviorFunctions['a'].depSet = [ 'v', 'mX'] # 'or',
        # qbehavior.qBehaviorFunctions['a'].plotDepSpace() 
        
    ## RobotWithGripper
    elif ROBOT == 3:
        robotWithGripper = RobotWithGripper('Sabil')
        
        motor_input_left = [10,  10, 3, 4, 8, 5, 3, 4, 3, 5, 10, 4, 2, 0, 0, 0, 2, 0, 4, 5, 2, 1, 0, 1, 3, 7, 1, 3, 0, 5, 2, 0, 0, 2, 4, 3]
        motor_input_right = [10, 10, 0, 1, 1, 0, 3, 0, 1, 1, 10, 4, 4, 3, 3, 7, 4, 4, 9, 5, 5, 3, 8, 3, 4, 7, 1, 7, 5, 5, 2, 5, 5, 5, 4, 3]

        # Add extra random motor inputs for more data
        for i in range(0):
            rand_val_left = int(np.random.random()*21-11)
            rand_val_right = int(np.random.random()*21-11)
            motor_input_left.append(rand_val_left)
            motor_input_right.append(rand_val_right)
        
        zero_array = [0 for i in range(len(motor_input_left))]
        ctrl = ControlByArrays( (motor_input_left, motor_input_right, zero_array, zero_array) )
        
        hist = simulateRobot(robotWithGripper, ctrl)
        
        hist.plot('x', 'y')
        
        hist.addDerivedVariable('qdor')
        hist.addDerivedVariable('qx')
        hist.addDerivedVariable('qy')
        hist.addDerivedVariable('qdx')
        hist.addDerivedVariable('qdy')
        
       # hist.plot('mL', 'mR', 'qdor') # target or OK
        
        hist.plot('or', 'mR', 'qdx') # influence of or on dx
        hist.plot('or', 'mR', 'qdy')

### #### #### #### ### #### ####  END  #### #### #### #### #### ####  
### #### #### #### ### #### #### ##### #### #### #### #### #### ####   