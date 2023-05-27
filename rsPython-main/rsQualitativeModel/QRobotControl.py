# -*- coding: utf-8 -*-
"""
Created on May  4  2023

@author: Jan Lemeire
"""
import time
from numbers import Number
import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
    
from rsLibrary.DataUtils import List, Dict, Tuple, Set
from rsLibrary.Monitor import gMonitorT, createMonitorT
from rsLibrary.Variable import Variable, RobotVariable, DeltaDerived, VariableType, QualType, OrdinalType, DerivedVariable
from Control import RobotControl
from Robots import Robot, SimRobot, createSimRobot1D, createSimRobot1DOnlyVelocity, RobotWithGripper
from Motors import Motor


from rsQualitativeModel.QDataType import Sign
from rsQualitativeModel.QHistory import QHistory
from rsQualitativeModel.QCausalModel import QRobotCausalModel, CondFun, QualFun, CtxtFun

class ControlRobot:
    
    def __init__(self, motors:list[RobotVariable], qrm:QRobotCausalModel, goal:dict[Variable: Number]):
        self.motors = motors
        self.qrm = qrm
        self.goal = goal
        
    def hasNext(self, state: dict[Variable:Number]):
        goals_not_met = [ k for k, v in self.goal.items() if abs(state[k] - v) > 0.2]
        return len(goals_not_met) > 0

    def nextCommand(self, state: dict[Variable:Number]) -> dict[str:Number]:
        #
        # TODO BY MARCO: make intelligent decisions using the qrm
        #
        return { m.name : np.random.uniform(m.minVal, m.maxVal) for m in self.motors }
    
def runRobot(robot: SimRobot, robotControl: ControlRobot):
    # Based on robot code from File Robots
    robot.printConfiguration()
    gMonitorT().printTitle()
    gMonitorT().printLastFrame()
    
    start = time.time()

    t = 0
    MAX_ITERATIONS = 100
    state_frame = robot.states.getStateT(t)
    while robotControl.hasNext(state_frame) and t < MAX_ITERATIONS:
         actions = robotControl.nextCommand(state_frame)
         t += 1
         
      #   print('t', str(t), ':', str(actions))

         robot.setMotors( actions)    
         gMonitorT().setValues( actions, t ) # MOVE TO ROBOTS
         gMonitorT().printLastFrame()
         
         state_frame = robot.states.getStateT(t)
         
         
    stop = time.time()
    if t >= MAX_ITERATIONS: 
        print("Maximal number of iterations reached:", MAX_ITERATIONS)
    else:
        print("Goal reached, congratulations!")
    
    print("Time to run:", stop-start)

def QCausalModelOfRobot1(simRobot: SimRobot) -> QRobotCausalModel:
    # JAN WILL DO THIS: CREATE THE MODEL FOR ROBOT 1
    variables = simRobot.variables()
    moV = variables[0]  # action variable
    v = variables[1]
    x = variables[2]
    
    _v = RobotVariable('_v', varType=VariableType.PREVIOUSVAR);
    dv = DeltaDerived(v);
    _x = RobotVariable('_x', varType=VariableType.PREVIOUSVAR);
    dx = DeltaDerived(x);
    
    # the function for all target variables
    fdv = CondFun( dv, (v, _v),  lambda x, y : Sign.sign(x-y))
    fdx = CondFun( dx, v,  lambda v : Sign.sign(v)) # doc on lambdas: see https://www.w3schools.com/python/python_lambda.asp
    fx = CondFun( x, (_x, dx),  lambda x, y : Sign.sign(x+y))
    
    # motor -> v is contextual because of walls
    # (0) in between walls (1) touching right wall (2) touching left wall
    v_condFuns = ( CondFun(v, moV,  lambda moV: Sign.sign(moV)), CondFun(v,  moV, lambda moV : Sign.sign(moV) if moV < 0 else Sign.ZERO), CondFun(v,  moV, lambda moV : Sign.sign(moV) if moV > 0 else Sign.ZERO) )
    
    wall_left, wall_right = simRobot.environment.walls
    fv = CtxtFun(v, x, lambda x : 1 if x >= wall_right else 2 if x <=wall_left else 0, v_condFuns)
  

    return QRobotCausalModel( List( [moV, v, x, _v, dv, _x, dx] ), List([fdv, fdx, fx, fv]))
    

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####         ROBOTS          #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###   
if __name__== "__main__":
    print('*** QRobotControl.py ***')

    
    ROBOT = 1 # 1: 1D v  2: 1D a  3: RobotWithGripper
    
    if ROBOT == 1:
        
        simRobot = createSimRobot1DOnlyVelocity(walls = (-10, 10), noiseLevel = 0)
        motorVariables = tuple( v for v in simRobot.variables() if v.varType == VariableType.ACTIONVAR )
        
        
        qrm = QCausalModelOfRobot1(simRobot)
        print('Q CAUSAL MODEL OF ROBOT 1')
        qrm.pprint()
        print('\n')
        
        goal_state = { 'x': -5, 'v': 0}
        
        ctrl = ControlRobot(motorVariables, qrm, goal_state)
        
        runRobot(simRobot, ctrl)
        
    elif ROBOT == 2:
         
        simRobot = createSimRobot1D(walls = (-100, 100), noiseLevel = 0)

    elif ROBOT == 3:
        robotWithGripper = RobotWithGripper('Sabil')
    