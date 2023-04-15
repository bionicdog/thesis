#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created: October 2021

@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
"""

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT

#### #### #### ####  StateDef & States  #### #### #### #### 
class StateDef:
    """
    defines the state of the robot and the environment

    Attributes
    -----------
    variables:      list of state variables
    indepVariables: independent state variables
    depVariables:   dependent state variables, depending on some other state variables
    relations:      dict defining the relations among the variables (dependent : independent)
    forwardMath:    function with equations to calculate the dependent from the independent variables
    """
    def __init__(self, variables:list, relations:dict, forwardMath):
        self.variables = variables
        self.relations = relations  # if key in relations-dict: it is a dependent variable
        self.forwardMath = forwardMath
        
        self.indepVariables = variables.copy()
        self.depVariables = []
        for dep_var in self.relations:
            self.indepVariables.remove(dep_var)
            self.depVariables.append(dep_var)
            
    def addIndependentVars(self, newVariables:list):
        self.variables.extend(newVariables)

    def print(self):
        print('State definition') # TODO
        
class States (dict):
    """
    keeps track of the state evolution

    Attributes
    -----------
    self:      dict with for every state variables a list of its values over all time frames
    stateDef:  state definition
    dts:       time gaps of frame with previous frame
    """
    varPrefix = '' # to distinguish between similar names
    def __init__(self, stateDef:StateDef, initialValues:dict):
        dict.__init__(self)
        self.stateDef = stateDef
        self.dts = [1]  # value is unimportant (there is no previous frame)
        for var in stateDef.variables:
            self[var] = initialValues[var].copy() if type(initialValues[var]) == list else [ initialValues[var] ]
            gMonitorT().setValue(self.varPrefix + var, self[var][0], 0)
        self.now = 0 # current index
        self.T = 0 # current time
        gMonitorT().setValue('T', self.T, self.now)
        self.startUnknown = 1 # start of unknown states
            
    def value(self, var, t = -1):
        return self[var][t]
    
    def setValue(self, var:str, t:int = -1, newValue:float=0):
        self[var][t] = newValue
        gMonitorT().setValue(self.varPrefix + var, newValue, t)

    
    def addFrame(self, dt = 1):
        # initialize all state variables to zero
        for key in self: 
            self[key].append(0)
        self.dts.append(dt)
        self.now += 1 # advance time
        self.T += dt
        gMonitorT().setValue('dt', dt, self.now)
        gMonitorT().setValue('T', self.T, self.now)
        
    def addFrameWithCopy(self, dt = 1):
        for key in self:
            prev_value = self[key][-1]
            self[key].append(prev_value) # copy of previous value
        self.dts.append(dt)
        self.now += 1
        self.T += dt
        gMonitorT().setValue('dt', dt, self.now)
        gMonitorT().setValue('T', self.T, self.now)
        
    def setDt(self, dt):
        current_dt = self.dts[-1]
        self.dts[-1] = dt
        gMonitorT().setValue('dt', dt)
        self.T -= current_dt - dt
        gMonitorT().setValue('T', self.T)

    def setValues(self, varValues:dict, t:int):
        for var in varValues:
            self.setValue(var, t, varValues[var])
    
    def calcDepVars(self, t:int = None):
        if t == None:
            t = self.now
        self.stateDef.forwardMath(self, t, self.dts)
    
    def printFrame(self, t:int):
        print('Frame '+str(t)+': ', end = '')
        for var in self.stateDef.variables:
            print(var+'('+str(t)+')='+str(int(self[var][t])), end = ' ')
        print()
        
    def printDepVariables(self, t:int = -1):
        for var in self.stateDef.depVariables:
            print(var+'('+str(t)+')='+str(int(self[var][t])), end = ' ')
        print()\
    
    def deepcopy(self):
        import copy
        c = copy.deepcopy(self)
        return c
    
    def addVariable(self, var, initialValue):
        self[var] = initialValue.copy() if type(initialValue) == list else [ initialValue ]
        gMonitorT().setValue(self.varPrefix + var, self[var][0], 0)

        
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###     
if __name__== "__main__":
    print('*** Demo code State.py ***')
    
    if False:
        # test code: define & create state & test forward function
        
        # 1D-world
        variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
        relationDict ={'v': 'a', 'x': 'v'}
        
        def forwardMath1D(states, t, dts):
            states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
            states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
            
            
        stateDef = StateDef(variables, relationDict, forwardMath1D)    
        
        # global observer
        createMonitorT(variables).printTitle()
        
        # acceleration evolution (independent variable)
        acceleration = [2, 0, 0, 2, -2, -2]
    
        # state evolution    
        states = States(stateDef, {'a':0, 'v':0, 'x':0})
        gMonitorT().printLastFrame()
        for i in range(0, len(acceleration)):
            t = i + 1
            states.addFrame()
            states.setValue( 'a', t, acceleration[i])
            states.calcDepVars(t)
            gMonitorT().printLastFrame()
            #states.print(t)

    #### #### #### ####  simulation with varying dt  #### #### #### #### 
    if True:
        # 1D-world
        variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
        relationDict ={'v': 'a', 'x': 'v'}
        
        def forwardMath1D(states, t, dts):
            states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
            states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
            
        stateDef = StateDef(variables, relationDict, forwardMath1D)    
        
        # global observer
        createMonitorT('dt', 'T', variables).printTitle()
        
        # acceleration evolution (independent variable)
        acceleration = [2, 0, 0, 2, -2, -2]
        dts = [1, 0.5, 0.1, 1, 1, 0.5]
    
        # state evolution    
        states = States(stateDef, {'a':0, 'v':0, 'x':0})
        gMonitorT().printLastFrame()
        for i in range(0, len(acceleration)):
            t = i + 1
            states.addFrame(dt = dts[i])
            states.setValue( 'a', t, acceleration[i])
            states.calcDepVars(t)
            gMonitorT().printLastFrame()
        
### #### #### #### ### #### #### #### #### #### #### #### ### 