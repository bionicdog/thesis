#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Environment.py: defines the environment of the robot(s)
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from State import States, StateDef
from rsLibrary.Monitor import gMonitorT, createMonitorT

class Environment:
    corrections = {} # dict from variable to tuple of 3 elements: time, old value, new value. see checkState in Environment1D

    def checkState(self, states: States, printit = False):
        '''
        Verify the possibility of the last state given the environment. 
        Make changes to the state if necessary
        '''
        raise NotImplementedError
        
    def rangeOfVariable(self, variable:str) -> tuple: # optional
        return None  # default, results in a Jacobian test change of 1

    def print(self):
        raise NotImplementedError
        

class Environment1D:
    
    def __init__(self, walls: tuple = (-100, 100), posvar:str='x', speedvar:str = 'v', accvar:str = 'a' ):
        self.walls = walls
        self.touchedWall = False
        self.posvar = posvar
        self.speedvar = speedvar
        self.accvar = accvar
        #added by marco to make estimation work on real robot
        self.corrections = {}
        
    def print(self):
        print('Walls at '+str(self.walls))
        
    def checkState(self, states: States, printit = False):
        # in case of a bump, we make sure that v is zero at the end of the period and x is at the wall. 
        # a and dt are changed to achieve this (x & v follow from the forward equations)
        self.touchedWall = False
        self.corrections = {} # reset
        pos_wall = 0
        avg_speed = (states.value(self.speedvar, -1) + states.value(self.speedvar, -2)) / 2
        if states.value(self.posvar) >= self.walls[1] and avg_speed >= 0:
            # bump on the right
            self.touchedWall = True
            pos_wall = self.walls[1]
            
        elif states.value(self.posvar) <= self.walls[0] and avg_speed <= 0:
            # bump on the left
            self.touchedWall = True
            pos_wall = self.walls[0]
            
        if self.touchedWall:
            if self.accvar is not None:
                # system with a, v, x
                # we want v[t] = zero and x[t] = pos_wall  => find at & dt
                # x[t] = (v[t] + v[t-1])/ 2 . dt + x[t-1] = pos_wall
                # => dt = (pos_wall - x[t-1]) * 2 / v[t-1] 
                #  v[t] = a[t].dt + v[t-1] = 0 => a[t] = - v[t-1] / dt
                
                dt = states.dts[-1]
                
                # change = False
                # set_v_to_zero = True
                # if abs(pos_wall - states[self.posvar][-1] )> 0.01:
                #     if states[self.speedvar][-2] == 0:
                #         # print('Going to touch the wall, but previous previous speed = 0; (Environment.checkState())')
                #         # v[t] remains, but we stop right before the wall (10%)
                #         # v[t-1] = 0
                #         # x[t] = v[t] / 2 . dt + x[t-1] = pos_wall - (pos_wall - x[t-1]) / 10
                #         #  => dt = (pos_wall - x[t-1]) * 0.9 * 2 / v[t]
                #         # v[t] = a[t].dt
                #         #  => dt = sqrt( (pos_wall - x[t-1]) * 0.9 * 2 / a[t] )
                #         import math
                #         dt = math.sqrt ( (pos_wall - states[self.posvar][-2] ) * 0.9 * 2 / states[self.accvar][-1] )
                #         set_v_to_zero = False
                #     else:
                #         dt = (pos_wall - states[self.posvar][-2] ) * 2 / states[self.speedvar][-2]
                #     states.setDt(dt)  
                #     change = True
                # if set_v_to_zero and states[self.speedvar][-1] != 0 :
                #     acc = - states[self.speedvar][-2] / dt
                #     states.setValue(self.accvar, newValue = acc)
                #     change = True
                # else:
                #     acc = acc_old
                
                a_t = states[self.accvar][-1]
                v_t = states[self.speedvar][-1] 
                x_t = states[self.posvar][-1] 
                v_prev = states[self.speedvar][-2] 
                x_prev = states[self.posvar][-2] 
                a_t_new, dt, change = adjustAccAndDr(a_t, v_t, x_t, v_prev, x_prev, dt, pos_wall)
                if change:
                    states.setValue(self.accvar, newValue = a_t_new)
                    states.setDt(dt) 
                    
                    states.calcDepVars()                
                    if printit:
                        print('Environment correction: '+self.accvar+' set to {:.3g} (from {:.3g}) and dt = {:.3g} '.format(a_t_new, a_t, dt)+ ' gives '+ self.posvar+' = {:.3g}'.format(states[self.posvar][-1])+' and '+self.speedvar+' = {:.3g}'.format(states[self.speedvar][-1]))
            else:
                # system with only v and x
                # x[t] = x[t-1] + v[t] = pos_wall   =>  v[t] = pos_wall - x[t-1]
                speed = pos_wall - states[self.posvar][-2] 
                states.setValue(self.speedvar, newValue = speed)
                states.calcDepVars()                

    def touchesWall(self, states: States) -> bool:
        x = states.value(self.statevar)
        return x >=  self.walls[1] or x <= self.walls[0]
            
    def rangeOfVariable(self, variable:str) -> tuple: # optional
        return self.walls if variable == self.posvar else None

# returns tuple: new a, new dt and boolean whether changed

def adjustAccAndDr(a_t, v_t, x_t, v_prev, x_prev, dt, pos_wall):
    change = False
    set_v_to_zero = True
    if abs(pos_wall - x_t )> 0.01:
        if v_prev == 0:
            # print('Going to touch the wall, but previous previous speed = 0; (Environment.checkState())')
            # v[t] remains, but we stop right before the wall (10%)
            # v[t-1] = 0
            # x[t] = v[t] / 2 . dt + x[t-1] = pos_wall - (pos_wall - x[t-1]) / 10
            #  => dt = (pos_wall - x[t-1]) * 0.9 * 2 / v[t]
            # v[t] = a[t].dt
            #  => dt = sqrt( (pos_wall - x[t-1]) * 0.9 * 2 / a[t] )
            import math
            dt = math.sqrt ( (pos_wall - x_prev ) * 0.9 * 2 / a_t )
            set_v_to_zero = False
        else:
            dt = (pos_wall - x_prev) * 2 / v_prev
        change = True
    if set_v_to_zero and v_t != 0 :
        acc = - v_prev / dt
        change = True
    else:
        acc = a_t
    return acc, dt, change

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Environment.py ***')
    
    #### #### #### ####  simulation of a, v, x  #### #### #### #### 
    if True:
        from rsLibrary.Monitor import gMonitorT, createMonitorT
        environment = Environment1D(walls = (-10, 90))        
        environment.print()
        
        # 1D-world
        variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
        relationDict ={'v': 'a', 'x': 'v'}
        
        def forwardMath1D(states, t, dts):
            states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
            states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
            
            
        stateDef = StateDef(variables, relationDict, forwardMath1D)    
        
        # global observer
        createMonitorT('dt', 'T', variables, 'tch').printTitle()
        
        # acceleration evolution (dependent variable)
        acceleration = [20, 0, 0, 0, -20,  0, 20, -20, -20, 0, 0, 0, 0, 0, 0]
    
        # state evolution    
        states = States(stateDef, {'a':0, 'v':0, 'x':0})
        gMonitorT().printLastFrame()
        for i in range(0, len(acceleration)):
            t = i + 1
            states.addFrame()
            states.setValue( 'a', t, acceleration[i])
         #   if t > 7:
         #       print('debug')
            states.calcDepVars(t)
            environment.checkState(states, printit = True)
            gMonitorT().setValue('tch', environment.touchedWall) # at both sides
            gMonitorT().printLastFrame()
            
    
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### ### #### #### #### #### #### #### ###