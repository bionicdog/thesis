#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Corrupters.py
@author: Jan Lemeire from Robotic Sensing lab
Created: October 2021
"""

import matplotlib.pylab as plt
import numpy as np
import math

class Corrupter: 
    def __init__(self, _sensor, _value:float, _value2:float, _activate=True):
        self.name = _sensor.name
        self.value = _value
        self.value2 = _value2
        self.sensor = _sensor
        self.corruptionTimes = []
        self.activate = _activate

        
    def modify(self, varValues: dict, t:int = 0):
        raise NotImplementedError 

    def setCorruptionTimes(self, _corruptionTimes):
        self.corruptionTimes = _corruptionTimes
        
    def getCorruptionTimes(self):
        return self.corruptionTimes
    
    def turnOnCorrupter(self):
        self.activate = True
        
    def turnOffCorrupter(self):
        self.activate = False

class OutlierCorrupter(Corrupter):
    def __init__(self, sensor, value:float = 4000, x:float = 0):
        super().__init__(sensor, value, x)
        self.corrupterType = 'outlier'
        
    def __str__(self):
        return f'sensor: {self.name} | type: outlier | times: {sorted(self.corruptionTimes)}'
        
    def modify(self, varValues:dict, t:int=0):
        if t in self.corruptionTimes:
            if type(varValues[self.name]) is list:
                varValues[self.name][t] = self.value
            else:
                varValues[self.name] = self.value
        
class ConstantCorrupter(Corrupter):
    def __init__(self, sensor, value:float = 50, x:float = 0):
        super().__init__(sensor, value, x)
        self.corrupterType = 'constant'
    
    def __str__(self):
        return f'{self.name} | constant | times: {sorted(self.corruptionTimes)}'
        
    def modify(self, varValues:dict, t:int=0):
        if t > self.value2:
            if type(varValues[self.name]) is list:
                varValues[self.name][t] += self.value
            else:
                varValues[self.name] += self.value

class DriftCorrupterT(Corrupter):
    def __init__(self, sensor, value:float = 2, x:float = 0):
        super().__init__(sensor, value, x)
        self.corrupterType = 'drift'
        
    def __str__(self):
        return f'{self.name} | drift | times: {sorted(self.corruptionTimes)}'
       
    def modify(self, varValues:dict, t:int=0):
        if t > self.value2:
            if type(varValues[self.name]) is list:
                #varValues[self.name][t] += self.value*math.cos(t)
                #varValues[self.name][t] += 4*(math.sin(4*t))+self.value*t
                varValues[self.name][t] += self.value* (t-self.value2)
            else:
                #varValues[self.name] += 4*(math.sin(4*t))+self.value*t
                varValues[self.name] += self.value*(t-self.value2)
                
class DriftCorrupterX(Corrupter):
    def __init__(self, sensor, value:float = 2, x:float = 0):
        super().__init__(sensor, value, x)
        self.corrupterType = 'drift'
        
    def __str__(self):
        return f'{self.name} | drift | times: {sorted(self.corruptionTimes)}'
       
    def modify(self, varValues:dict, t:int=0):
        if t > self.value2:
            if type(varValues[self.name]) is list:
                #varValues[self.name][t] += self.value*math.cos(t)
                #varValues[self.name][t] += 4*(math.sin(4*t))+self.value*t
                varValues[self.name][t] *= self.value
            else:
                #varValues[self.name] += 4*(math.sin(4*t))+self.value*t
                varValues[self.name] *= self.value
                
class OutOfRangeCorrupter(Corrupter):
    def __init__(self, sensor, value:float = 50, value2:float = 900):
        super().__init__(sensor, value, value2)
        self.corrupterType = 'out_range'
        
    def __str__(self):
        return f'{self.name} | out of range | times: {sorted(self.corruptionTimes)}'
       
    def modify(self, varValues:dict, t:int=0):
        if type(varValues[self.name]) is list:
            if varValues[self.name][t] > self.value2:
                varValues[self.name][t] =  self.value2
            
            # take %10 of minimum range vaule
            noiceVal = self.value * 0.1
            
            if varValues[self.name][t] < self.value:
                rand = np.random.randint(-noiceVal, noiceVal)
                varValues[self.name][t] =  self.value #+ rand
        else:
            if varValues[self.name] > self.value2:
                varValues[self.name] =  self.value2
            
            # take %10 of minimum range vaule
            noiceVal = 5
            
            if varValues[self.name] < self.value:
                rand = np.random.randint(-noiceVal, noiceVal)
                varValues[self.name] =  self.value #+ rand
            
class DisconnectCorrupter(Corrupter):
    def __init__(self, sensor, value:float = 0, x:float = 0):
        super().__init__(sensor, value, x)
        self.corrupterType = 'disconnect'
        
    def __str__(self):
        return f'{self.name} | disconnect | times: {sorted(self.corruptionTimes)}'
       
    def modify(self, varValues:dict, t:int=0):
        if t > self.value2:
            if type(varValues[self.name]) is list:
                varValues[self.name][t] = self.value
            else:
                varValues[self.name] = self.value
      
'''
 Apply multiple corrupters
'''
class MultiCorrupter(Corrupter):
    def __init__(self, *_corrupters):
        self.corrupters = []
        for corrupter in _corrupters:
            if issubclass(type(corrupter), Corrupter):
                self.corrupters.append(corrupter)
            else:
                print('Error: expecting corrupter but type of '+str(corrupter)+' is '+str(type(corrupter)));
    
    def __str__(self):
        txt = '\n'
        for corrupter in self.corrupters:
            txt += f'\t- {corrupter}\n'
        return txt
    
    def addCorrupter(self, corrupter):
        if issubclass(type(corrupter), Corrupter):
            self.corrupters.append(corrupter)
        else:
            print('Error: expecting corrupter but type of '+str(corrupter)+' is '+str(type(corrupter)));

    def generateCorruptedTimes(self, endFrame:int, percentage:int):
        for corrupter in self.corrupters:
            corrupter.setCorruptionTimes(np.linspace(1, endFrame, num=endFrame) if percentage == 100  else np.random.randint(endFrame, size = int(percentage/100*endFrame)))
            
    def getCorruptionTimes(self):
        corruptersDict = {}
        for corrupter in self.corrupters:
            corruptersDict[corrupter.name] = corrupter.getCorruptionTimes()
        return corruptersDict
    
    def turnOnCorruptersOfType(self, typeOfCorrupter:str):
        for corrupter in self.corrupters:
            if corrupter.corrupterType == typeOfCorrupter:
                corrupter.turnOnCorrupter()
    
    def turnOffCorruptersOfType(self, typeOfCorrupter:str):
        for corrupter in self.corrupters:
            if corrupter.corrupterType == typeOfCorrupter:
                corrupter.turnOffCorrupter()
    
    def turnOnAllCorrupters(self):
        for corrupter in self.corrupters:
            corrupter.turnOnCorrupter()
    
    def turnOffAllCorrupters(self):
        for corrupter in self.corrupters:
            corrupter.turnOffCorrupter()
    
    def modify(self, varValues: dict, t:int=0):
        for corrupter in self.corrupters:
            corrupter.modify(varValues, t)


class CorrupterModule:
    def __init__(self, _robotModel, commandCorruption=True, sensorsCorruption=True, statesCorruption=True, _activate=True):
        self.robotModel = _robotModel
        self.corrupters = {'motor':MultiCorrupter(),
                           'sensor':MultiCorrupter(),
                           'states':MultiCorrupter()}
        self.corrupterConstructors = {'outlier': OutlierCorrupter,
                                      'constant': ConstantCorrupter,
                                      'driftT': DriftCorrupterT,
                                      'driftX': DriftCorrupterX,
                                      'out_range': OutOfRangeCorrupter,
                                      'disconnect': DisconnectCorrupter}
        self.commandCorruption = commandCorruption
        self.sensorsCorruption = sensorsCorruption
        self.statesCorruption = statesCorruption
        self.activate = _activate
        
    def __str__(self):
        txt = 'Corrupter Module:\n'
        for typeOfCorrupter, multiCorrupter in self.corrupters.items():
            txt += f'{typeOfCorrupter} corrupters: {multiCorrupter}\n'
        return txt

    #def addCorrupter(self, typeVar:str, corrupter):
     #   self.corrupters[typeVar].addCorrupter(corrupter)
        
    def addCorrupter(self, typeOfCorrupter:str, sensorOrState:str, sensorOrStateName:str, value:int, value2:int):
        if sensorOrState == 'states':
            self.corrupters[sensorOrState].addCorrupter(self.corrupterConstructors[typeOfCorrupter](self.robotModel.states[sensorOrStateName], value, value2))
        else:
            self.corrupters[sensorOrState].addCorrupter(self.corrupterConstructors[typeOfCorrupter](self.robotModel.getMotorOrSensor(sensorOrStateName), value, value2))

    def generateCorruptedTimes(self, endFrame:int, percentage:int):
        for multiCorrupter in self.corrupters.values():
            multiCorrupter.generateCorruptedTimes(endFrame, percentage)
            
    def getCorruptionTimes(self):
        corruptionTimesDict = {}
        for typeOfCorrupter, multiCorrupter in self.corrupters.items():
            corruptionTimesDict[typeOfCorrupter] = multiCorrupter.getCorruptionTimes()
        return corruptionTimesDict
    
    def turnOnCorruptersOfType(self, typeOfCorrupter:str):
        for multicorrupter in self.corrupters.values():
            multicorrupter.turnOnCorruptersOfType(typeOfCorrupter)
        
    def turnOffCorruptersOfType(self, typeOfCorrupter:str):
        for multicorrupter in self.corrupters.values():
            multicorrupter.turnOffCorruptersOfType(typeOfCorrupter)
            
    def turnOnAllCorrupters(self):
        for multicorrupter in self.corrupters.values():
            multicorrupter.turnOnAllCorrupters()
            
    def turnOffAllCorrupters(self):
        for multicorrupter in self.corrupters.values():
            multicorrupter.turnOffAllCorrupters()
        
    def turnOnMotorOrStateCorrupters(self, motorOrState:str):
        self.corrupters[motorOrState].turnOnAllCorrupters()

    def turnOffMotorOrStateCorrupters(self, motorOrState:str):
        self.corrupters[motorOrState].turnOffAllCorrupters()
            
    def modify(self, typeVar:str, varValues:dict, t:int=0):
        self.corrupters[typeVar].modify(varValues, t)

        
def createCorrupterModule(robotModel, endFrame:int, percentage:int, sensors:dict = {}, motors:dict = {}, states:dict = {}, activate=True):
    corrupter = CorrupterModule(robotModel, _activate = activate)
    
    for sensor, List in sensors.items():
        print(sensor + ' ' + List[0] + ' '+ str(List[1]) + ' ' + str(List[2]))
        corrupter.addCorrupter(List[0], 'sensor', sensor, List[1], List[2])
    
    for motor, List in motors:
        corrupter.addCorrupter(List[0], 'motor', sensor, List[1], List[2])
        
    for state, List in states:
        corrupter.addCorrupter(List[0], 'states', sensor, List[1], List[2])
    
    corrupter.generateCorruptedTimes(endFrame, percentage)
    
    return corrupter
    


class DemoRobModel:
    def __init__(self):
        self.states = {'x' :[1, 2, 3], 'v':[4, 5, 6]}

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    corruptionTimes = [0]
    corrupter = OutlierCorrupter('a', 10)
    d = {'a': 1, 'b' : 1}
    print(str(d))
    
    corrupter.modify(d, corruptionTimes, 0 )
    print(str(d))
    
    rob = DemoRobModel()
    c = OutlierCorrupter('x', 100)
    c.modify(rob.states, corruptionTimes)
    print(str(rob.states))
    
    multicorrupter = MultiCorrupter(corrupter, OutlierCorrupter('b', 10))
    multicorrupter.modify(d, corruptionTimes)
    print(str(d))
    
    d = {'a': [1, 2, 3], 'b': [4, 5, 6]}
    multicorrupter.modify(d, corruptionTimes)
    print(str(d))
    
    corrupter = DriftCorrupterT('a')
    d = {'a':[]}
    for t in range(100):
        d2 = {'a':1}
        corrupter.modify(d2, corruptionTimes, t)
        d['a'].append(d2['a'])
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(d['a'])
    
    
    