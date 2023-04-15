#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensors.py
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from State import StateDef, States
from numbers import Number
from scipy.stats import rv_continuous as distribution
import math

#### #### #### ####  Sensor #### #### #### #### 
class MotorOrSensor:
    """
    Interface class for sensors and motors

    Attributes
    -----------
    name
    values:         array of sensorvalues
    params:         dict of parameters and values


    """
    varPrefix = '' # to distinguish between similar names
    
    def __init__(self, name:str, sensorType:str, params:dict, noiseDistr: distribution = None):
        """
        Constructor
        """
        self.name = name
        self.sensorType = sensorType
        self.params = params 
        self.noiseDistr = noiseDistr # of type scipy.stats.rv_continuous
        self.values = []  # time series
        self.value = 0
        
        
        self.isCalibrated = False
        self.excludeObservations = [] # time stamps of the observations that should be excluded
        self.enable = True # disable completely for being used in estimation
        self.disableFrom = 10000000 # disable from the given timestamp

        self.residuals = [] # deviation of prediction from real observed value
        self.residualsMean = -1
        self.residualsStddev = -1     
        self.residualsCov = -1
        

    #### ==============  Interface methods to be implemented  ============== ####   
    def observFunc(self, states:States, t:int):
        """
        Observation Function: based on state at time t, return the observation value
        
        """
        raise NotImplementedError
        return None
    
    def printObserFunc(self):
        """  print the Observation Function in a nice way   """
        raise NotImplementedError
    def printObserEq(self, t:int):
        """  print the Observation equation in a nice way   """
        raise NotImplementedError
    def printError(self, states:States, t:int):
        """  print error in a nice way  """
        raise NotImplementedError
    
    #### ==============  Interface methods may be implemented  ============== ####   
    def error(self, states:States, t:int):
        """
        error Function: based on observFunc, return the error with the actual observed value
        positive error: prediction is too low
        
        """
        expected_value = self.observFunc(states, t)
        residual = expected_value - self.values[t]
        while t >= len(self.residuals):
            self.residuals.append(0)
        
        self.residuals[t] = residual
        gMonitorT().setValue('e' + self.name, expected_value, t) #quick fix
        gMonitorT().setValue('r' + self.name, residual, t)
        return residual

    #### ==============  General Methods  ============== ####   
    def setObservation(self, value:Number, t:int):
        self.value = value
        self.values.append(self.value)
        gMonitorT().setValue(self.varPrefix + self.name, value, t)
        
    def generateObservation(self, states:States):
        val = self.observFunc(states, -1)
        if self.noiseDistr is not None:
            noise = self.noiseDistr.rvs(size=1)[0]
            val += noise
            self.residuals.append(noise) # for testing with the simRobot
            
        self.setObservation(val, states.now)

    def setParam(self, param:str, value:Number):
        if param in self.params:
            self.params[param] = value
            gMonitorT().setValue(self.varPrefix + param, value)
        else:
            print("ERROR: Sensor "+self.name+" does not contain parameter "+param)
            
    def adjustParam(self, param:str, change:Number):
        if param in self.params:
            self.params[param] += change
            gMonitorT().setValue(self.varPrefix + param, self.params[param])
        else:
            print("ERROR: Sensor "+self.name+" does not contain parameter "+param)
    def analyzeResiduals(self):
        if len(self.residuals) == 0:
            print('No residuals for sensor '+self.name)
            self.residualsMean = 0
            self.residualsStddev = 0
            self.residualsCov = 0
        else:            
            if self.residuals[0] == -1:
                self.residuals[0] = 0
            import numpy
            avg = 0
            for r in self.residuals:
                avg += r
            avg/=len(self.residuals)
            cov = 0
            for r in self.residuals:
                cov += (r - avg) * (r - avg)
            cov /= len(self.residuals) + 1

            self.residualsMean = avg # numpy.mean(self.residuals)
            self.residualsStddev = math.sqrt(cov) # numpy.std(self.residuals)
            self.residualsCov = cov # numpy.cov(self.residuals)

#           print('MS '+self.name+' avg = {:.2f} cov = {:.2f} stddev = {:.2f}.'.format(avg, cov, math.sqrt(cov)))

#### #### #### ####  Sensor Implementations  #### #### #### #### 
class LineairSensor(MotorOrSensor):
    """
    Sensor which value is proportional to one state variable
    distribution: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.rv_continuous.html
    """
        
    def __init__(self, name, statevar:str="px", paramName:str = 'lx', paramValue=1, noiseDistr: distribution = None):
        super().__init__(name, sensorType = 'lineair', params = {paramName : float(paramValue)}, noiseDistr = noiseDistr )
      #  self.validRange = [0, 40]
        self.input = False
        self.statevar = statevar
        self.paramName = paramName
        
    def observFunc(self, states, t):
        return self.params[self.paramName] * states[self.statevar][t]
    
    def printObserFunc(self):
        if self.noiseDistr is None:
            print('{}(t) = {} * state.{}(t)  with {} = {:.3g}'.format(self.name, self.paramName, self.statevar, self.paramName, self.params[self.paramName]))
        else:
            print('{}(t) = {} * state.{}(t)  with {} = {:.3g} and stddev(noise) = {}'.format(self.name, self.paramName, self.statevar, self.paramName, self.params[self.paramName], self.noiseDistr.std()))
    def printObserEq(self, t:int):
        print('{} = {} * {}({})'.format(self.values[t], self.paramName, self.statevar, t))
    def printError(self, states:States, t:int):
        print('{}({}): {:.1f} = {} * {}({})'.format(self.name, t, self.values[t], self.paramName, self.statevar, t) \
              +' => err = {:.2f} - {:.2f} * {:.2f} = {:.2f}'.format(self.values[t], self.params[self.paramName], states[self.statevar][t], self.error(states, t)))
            
    def errorFunctionString(self, functionNbr:int, states:States, t:int):
        f_str = "def f{}(values:dict): return {} - values['{}'] * values['{}'] ".format(functionNbr, self.values[t], self.paramName, self.statevar)
        return f_str
            
            
    def printValue(self, t):
        print(self.name+'='+str(self.values[t]))
        
class LineairSensorWithConstant(MotorOrSensor):
    """
    Sensor which value is proportional to one state variable
    distribution: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.rv_continuous.html
    """
        
    def __init__(self, name, statevar:str="px", paramName:str = 'lx', paramValue=1, noiseDistr: distribution = None, paramConst:str = 'lxc', paramConstValue=0):
        super().__init__(name, sensorType = 'lineair', params = {paramName : float(paramValue), paramConst : paramConstValue}, noiseDistr = noiseDistr )
      #  self.validRange = [0, 40]
        self.input = False
        self.statevar = statevar
        self.paramName = paramName
        self.paramConst = paramConst
        
    def observFunc(self, states, t):
        return (self.params[self.paramName] * states[self.statevar][t]) + self.params[self.paramConst]
    
    def printObserFunc(self):
        if self.noiseDistr is None:
            print('{}(t) = {} * state.{}(t)  with {} = {:.3g}'.format(self.name, self.paramName, self.statevar, self.paramName, self.params[self.paramName]))
        else:
            print('{}(t) = {} * state.{}(t)  with {} = {:.3g} and stddev(noise) = {}'.format(self.name, self.paramName, self.statevar, self.paramName, self.params[self.paramName], self.noiseDistr.std()))
    def printObserEq(self, t:int):
        print('{} = {} * {}({})'.format(self.values[t], self.paramName, self.statevar, t))
    def printError(self, states:States, t:int):
        print('{}({}): {:.1f} = {} * {}({})'.format(self.name, t, self.values[t], self.paramName, self.statevar, t) \
              +' => err = {:.2f} - {:.2f} * {:.2f} = {:.2f}'.format(self.values[t], self.params[self.paramName], states[self.statevar][t], self.error(states, t)))
            
    def errorFunctionString(self, functionNbr:int, states:States, t:int):
        f_str = "def f{}(values:dict): return {} - values['{}'] * values['{}'] ".format(functionNbr, self.values[t], self.paramName, self.statevar)
        return f_str
            
            
    def printValue(self, t):
        print(self.name+'='+str(self.values[t]))

class EventSensor(MotorOrSensor):
    """
    Sensor which measures when a certain state variable is above the given threshold or below if the threshold is negative
    """
    
    def __init__(self, name, statevar="px", paramName = 'lw', paramValue=100):
        super().__init__(name, sensorType = 'event', params = {paramName : paramValue}  )
      #  self.validRange = [0, 40]
        self.input = False
        self.statevar = statevar
        self.paramName = paramName
        self.threshold = paramValue
        
    def observFunc(self, states, t):
        if self.threshold > 0:
            return 0 if states[self.statevar][t] < self.threshold  else 1
        else:
            return 0 if states[self.statevar][t] > self.threshold  else 1

    def error(self, states, t):
        '''
        default is overwritten!
        error is distance to event ('wall') [since this distance is known, it can be used as information about the error]
        '''        
        if self.values[t] > 0: # there was a bump
            if self.threshold > 0:
                return self.params[ self.paramName ] -  states[self.statevar][t]  # distance to event
            else:
                return states[self.statevar][t] - self.params[ self.paramName ]
        else:
            return 0 # no error
         
    def comp(self):
        return '>=' if self.threshold > 0 else '<='
    def printObserFunc(self):
        print('{}(t) = state.{}(t) '.format(self.name, self.statevar)+self.comp()+' {}   with {} = {}'.format(self.paramName, self.paramName, self.params[self.paramName]))
            
    def printObserEq(self, t='t'):
        print('{} = {}({}) '.format(self.values[t], self.statevar, t)+self.comp()+' {}'.format(self.paramName))

    def printError(self, states, t):
        if self.values[t] > 0:
            print('{}({}): {} = {}({}) '.format(self.name, t, self.values[t], self.statevar, t)+self.comp()+' {}'.format(self.paramName)\
                  +' => err = {:.1f} - {:.1f} = {:.1f}'.format(self.params[ self.paramName ], states[self.statevar][t], self.error(states, t)))
        else:
            print('{}({}): {} = {}({}) '.format(self.name, t, self.values[t], self.statevar, t)+self.comp()+' {}'.format(self.paramName)+' => err = 0')

    def printValue(self, t):
        print(self.name+'='+str(self.values[t]))        
        
class DistanceSensor(MotorOrSensor):
    """
    Sensor which measures distance of a certain state variable to a given value, multiplied with a coefficient
    """
    def __init__(self, name, statevar="x", paramWall = 'lw', paramWallValue=100, paramCoeff = 'ld', paramCoeffValue=5, noiseDistr: distribution = None):
        super().__init__(name, sensorType = 'distance', params = { paramCoeff : paramCoeffValue} , noiseDistr = noiseDistr )
        # we assume that 'lw' is known
        # we only pass param 'ld' to the parent class, so that only that variable will be calibrated
        
        self.input = False
        self.statevar = statevar
        self.paramWall = paramWall
        self.paramWallValue = paramWallValue
        self.paramCoeff = paramCoeff
       
        
    def observFunc(self, states, t):
        if self.paramWallValue > 0:
            return (self.paramWallValue - states[self.statevar][t]) * self.params[self.paramCoeff]
        else:
            return (states[self.statevar][t] - self.paramWallValue) * self.params[self.paramCoeff]
    
    
    def diff(self, _t):
        if self.paramWallValue > 0:
            return '({} - state.{}({}))'.format(self.paramWall, self.statevar, _t)
        else:
            return '(state.{}({}) - {})'.format(self.statevar, _t, self.paramWall)
    def printObserFunc(self):
        print('{}(t) = {} * '.format(self.name, self.paramCoeff)+self.diff('t')+' with {} = {:.3g}, {} = {:.3g}'.format(self.paramCoeff, self.params[self.paramCoeff], self.paramWall, self.paramWallValue),' ' if self.noiseDistr is None else ' and stddev(noise) = {}'.format(self.noiseDistr.std()))
        
    def printObserEq(self, t:int):
        print('{} = {} * '.format(self.values[t], self.paramCoeff)+self.diff(t))
 
    def diff2(self, states:States, t:int):
        if self.paramWallValue > 0:
            return '({} - {:.2f})'.format(self.paramWall, states[self.statevar][t])    
        else:
            return '({:.2f} - {})'.format(states[self.statevar][t], self.paramWall)

    def printError(self, states:States, t:int):
        print('{}({}): {:.1f} = {} * '.format(self.name, t, self.values[t], self.paramCoeff)+self.diff(t) \
              +' => err = {:.2f} - {:.2f} * '.format(self.values[t], self.params[self.paramCoeff])+self.diff2(states, t)+' = {:.2f}'.format(self.error(states, t)))

    def printValue(self, t):
        print(self.name+'='+str(self.values[t]))

class DistanceSensorWithConstant(MotorOrSensor):
    """
    Sensor which measures distance of a certain state variable to a given value, multiplied with a coefficient
    """
    def __init__(self, name, statevar="x", paramWall = 'lw', paramWallValue=100, paramCoeff = 'ld', paramCoeffValue=5, noiseDistr: distribution = None, paramConst = 'ldc', paramConstValue=0):
        super().__init__(name, sensorType = 'distance', params = { paramCoeff : paramCoeffValue, paramConst : paramConstValue} , noiseDistr = noiseDistr )
        # we assume that 'lw' is known
        # we only pass param 'ld' to the parent class, so that only that variable will be calibrated
        
        self.input = False
        self.statevar = statevar
        self.paramWall = paramWall
        self.paramWallValue = paramWallValue
        self.paramCoeff = paramCoeff
        self.paramConst = paramConst
       
        
    def observFunc(self, states, t):
        if self.paramWallValue > 0:
            return ((self.paramWallValue - states[self.statevar][t]) * self.params[self.paramCoeff]) + self.params[self.paramConst]
        else:
            return ((states[self.statevar][t] - self.paramWallValue) * self.params[self.paramCoeff]) + self.params[self.paramConst] 
    
    
    def diff(self, _t):
        if self.paramWallValue > 0:
            return '({} - state.{}({}))'.format(self.paramWall, self.statevar, _t)
        else:
            return '(state.{}({}) - {})'.format(self.statevar, _t, self.paramWall)
    def printObserFunc(self):
        print('{}(t) = {} * '.format(self.name, self.paramCoeff)+self.diff('t')+' with {} = {:.3g}, {} = {:.3g}'.format(self.paramCoeff, self.params[self.paramCoeff], self.paramWall, self.paramWallValue),' ' if self.noiseDistr is None else ' and stddev(noise) = {}'.format(self.noiseDistr.std()))
        
    def printObserEq(self, t:int):
        print('{} = {} * '.format(self.values[t], self.paramCoeff)+self.diff(t))
 
    def diff2(self, states:States, t:int):
        if self.paramWallValue > 0:
            return '({} - {:.2f})'.format(self.paramWall, states[self.statevar][t])    
        else:
            return '({:.2f} - {})'.format(states[self.statevar][t], self.paramWall)

    def printError(self, states:States, t:int):
        print('{}({}): {:.1f} = {} * '.format(self.name, t, self.values[t], self.paramCoeff)+self.diff(t) \
              +' => err = {:.2f} - {:.2f} * '.format(self.values[t], self.params[self.paramCoeff])+self.diff2(states, t)+' = {:.2f}'.format(self.error(states, t)))

    def printValue(self, t):
        print(self.name+'='+str(self.values[t]))                


### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    # test code: creation of sensors and print
    print('*** Demo code Sensors.py ***')
    
    # 1D-world
    
    variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
    relationDict ={'v': 'a', 'x': 'v'}    
    def forwardMath1D(states, t, dts):
        states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
        states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
    stateDef = StateDef(variables, relationDict, forwardMath1D)
    
    # sensors
    from scipy.stats import norm
    noiseDistribution = norm(scale = 5)
    distanceSensor = LineairSensor("dst", "x", 'lx', 1, noiseDistribution)
    print('Sensor '+distanceSensor.name+': ', end='')
    distanceSensor.printObserFunc()
    
    
    touchSensor = EventSensor("touch", "x", 'lw', 100)
    print('Sensor '+touchSensor.name+': ', end='')
    touchSensor.printObserFunc()
    
    # global monitor
    createMonitorT(variables, ["dst", "touch"]).printTitle()
   
    
    
    # acceleration evolution (dependent variable)
    acceleration = [1, 5, 0, 2, 2, 2]

    # state evolution    
    states = States(stateDef, {'a':10, 'v':10, 'x':10})
    gMonitorT().printLastFrame()
    
    for i in range(0, len(acceleration)):
        t = i + 1
        states.addFrame()
        states.setValue( 'a', t, acceleration[i])
        states.calcDepVars(t)
        distanceSensor.generateObservation(states)
        touchSensor.generateObservation(states)
        gMonitorT().printLastFrame()
        
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ### #### #### ###