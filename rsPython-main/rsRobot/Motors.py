#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motors.py
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""
from numbers import Number
from Sensors import MotorOrSensor
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT

#### #### #### ####  Motor Interface #### #### #### #### 
class Motor(MotorOrSensor):
    
    def __init__(self, name, motorType, params):
        super().__init__(name, motorType, params = params )
        
        
    def motorFunc(self, states, t):
        """
        Motor Function
        
        Returns
        ----------
        dict with change of state variable (state variable : new value)
        """
        raise NotImplementedError
    
    def setMotorInput(self, value:Number, t:int):
        self.setObservation(value, t)
    def printMotorFunc(self):     
        raise NotImplementedError
        
#### #### #### ####  Motor Implementations  #### #### #### #### 
class LineairMotor(Motor): 
    """
      motor linearly determines a state variable
      state-veriable = motor-input / lm
    """
    def __init__(self, name, statevars=['a'],  paramName = 'lm', paramValue=1):
        super().__init__(name, motorType = "motor", params = {paramName : paramValue})
        self.statevars = statevars
        self.paramName = paramName
        
    def observFunc(self, states, t):
        return self.params[self.paramName] * states[self.statevars[0]][t]
    
    def motorFunc(self, motorValue, states, t) -> dict:
        # is the inverse of the observ funct 
        
        gMonitorT().setValue(self.name, motorValue, t)
        new_state_value = motorValue  / self.params[self.paramName]
        return {self.statevars[0] : new_state_value}

    def printObserFunc(self):
        print('{}(t) = {} * state.{}(t)'.format(self.name, self.paramName, self.statevar))

    def printObserEq(self, t):
        print('{} = {}*{}({})'.format(self.values[t], self.paramName, self.statevars[0], t))
    def printError(self, states, t):
        print('{}({}): {} = {}*{}({})'.format(self.name, t, self.values[t], self.paramName, self.statevars[0], t) + ' => err = {} - {:.1f}*{:.1f} = {:.1f}'.format(self.values[t], self.params[self.paramName], states[self.statevars[0]][t], self.error(states, t)))
           
    def printMotorFunc(self, t='t'):
        print('{}(t) = {}({}) / {}  with {} = {}'.format(self.statevars[0], self.name, t, self.paramName, self.paramName, self.params[self.paramName]))
 
    
class OurMotor(Motor):
    # motorInput = lm * ax + lp.vx
    # ax = (motorInput - lp.vx[t-1]) / lm
    def __init__(self, name, statevars=["ax", "vx"], param1 = "lm", param1Value = 1, param2 = "lp" , param2Value = 1 ):
        super().__init__(name, motorType = "motor", params = {param1: param1Value, param2 : param2Value })
        self.statevars = statevars
        self.param1 = param1
        self.param2 = param2
        
    def observFunc(self, states, t):
        return self.params[self.param1] * states[self.statevars[0]][t] + self.params[self.param2] * states[self.statevars[1]][t-1]
        
    def motorFunc(self, motorValue, states, t):
        # is the inverse of the observ funct solved
        gMonitorT().setValue(self.name, motorValue, t)
        new_state_value = (motorValue - self.params[self.param2] * states[self.statevars[1]][t-1]) / self.params[self.param1]
        return {self.statevars[0] : new_state_value}
    
    def printObserFunc(self):
        print('{}(t) = {}*state.{}(t) + {}*state.{}(t-1)'.format(self.name, self.param1, self.statevars[0], self.param2, self.statevars[1]))
        
    def printObserEq(self, t):
        print('{} = {}*state.{}(t) + {}*state.{}(t-1)'.format(self.name, self.param1, self.statevars[0], self.param2, self.statevars[1]))

    def printError(self, states, t):
        print('{}({}): {:.2f} = {}*state.{}(t) + {}*state.{}(t-1)'.format(self.name, t, self.values[t], self.param1, self.statevars[0], self.param2, self.statevars[1])  \
              +' => err = {:.2f} - {:.2f}*{:.2f} - {:.2f}*{:.2f} = {:.2f}'.format(self.values[t], self.params[self.param1] , states[self.statevars[0]][t] , self.params[self.param2] , states[self.statevars[1]][t-1], self.error(states, t)))
    
    def printMotorFunc(self):
        print('{}(t) = ({}(t) - {} * state.{}(t-1)) / {}  with {} = {:.3g}, {} = {:.3g}'.format(self.statevars[0], self.name, self.param2, self.statevars[1], self.param1, self.param2, self.params[self.param2], self.param1, self.params[self.param1]))
 
        
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###     
if __name__== "__main__":
    print('*** Demo code Motors.py ***')
    
    ourMotor = OurMotor("motX", ["a", "v" ])
    ourMotor.printMotorFunc()
    ourMotor.printObserFunc()
     
    if True:
        # 1D-world
        from State import StateDef, States
        variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
        relationDict ={'v': 'a', 'x': 'v'}    
        def forwardMath1D(states, t, dts):
            states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
            states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
        stateDef = StateDef(variables, relationDict, forwardMath1D)
        
        # motors
        ourMotor = OurMotor("motX", ["a", "v" ])
        ourMotor.printMotorFunc()
        ourMotor.printObserFunc()
        
        # global monitor
        createMonitorT("motX", variables).printTitle()
        
        # motor input
        motor_input = [1, 1, 2, 2, -2, -2]
        
        # state evolution
        states = States(stateDef, {'a':0, 'v':0, 'x':0})
        gMonitorT().printLastFrame()
        
        for i in range(0, len(motor_input)):
            t = i + 1
            state_change = ourMotor.motorFunc(motor_input[i], states, t)
            print("{motX: "+str(motor_input[i])+ '} => '+str(state_change))
            states.addFrame()
            states.setValues( state_change, t)
            states.calcDepVars(t)
            gMonitorT().printLastFrame()    
    
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### ### #### #### #### #### #### #### ###