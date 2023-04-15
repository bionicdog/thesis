#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Observer.py: keep track of values present throughout the system and print them nicely
@author: Thibault Thetier from Robotic Sensing lab
Created: March 2022
"""
from numbers import Number
import pandas as pd
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT

"""
       for demo of detection of FDIR look at fhExperiment file
"""

"""
#### #### #### ####  ObservationModule #### #### #### #### 
# keeps values over time for each variable, with time from 0..endTime
# endTime is the maximal time over all calls to setValue
# dict of arrays

class ObservationModule(dict):
    endTime = 0 # there is at least 1 frame
    
    def __init__(self, args):
        #args: str. Names of variables
        #Monitors:
         #   - state variables
         #   - robot sensors values
         "   - motor values
        
        dict.__init__(self)
        self.variables = ['t']
        self['t'] = [ 0 ]
        self.addVariables(args)

        
#        print(self.variables)
        
    def addVariables(self, args):
        #args: str. Names of variables
        
        for _vars in args:
 #           print(str(_vars)+":"+str(type(_vars)))
            if isinstance(_vars, list):
                self.variables.extend(_vars)
            else:
                self.variables.append(_vars)
        # create list for all variables
        for v in self.variables:
            if v not in self:
                self[v] = [-1] * (self.endTime  + 1)
    
    def setValue(self, var:str, value:Number, time:int = -1):
        if var not in self:
            self[var] = [-1] * (self.endTime  + 1)
        if time is None:
            self[var][self.endTime] = value
        else:
            self.enlargeFramesIfNecessary(time)
            self[var][time] = value

    def setValuesForAllT(self, var:str, value:Number):
        if var not in self:
            self[var] = [-1] * (self.endTime  + 1)

        for t in range(0, self.endTime + 1):
            self[var][t] = value

    def setValues(self, varValueDict:dict, time:int):
        self.enlargeFramesIfNecessary(time)
        for var, val in varValueDict.items():
            if var not in self:
                self[var] = [-1] * (self.endTime  + 1)
            self[var][time] = val
            
    def enlargeFramesIfNecessary(self, time:int):
        if (time > self.endTime):
            for var in self:
                for t in range(len(self[var]), time + 1):
                    self[var].append(t if var == 't' else -1)
            self.endTime = time
            
    def printTitle(self):
        print(*[v for v in self.variables], sep = '\t\t')  # 'string with tabs'.expandtabs(8)
        print(*['--' for v in self.variables], sep = '\t\t')
        
    def printLastFrame(self):
        print(*[ int(self[v][self.endTime]) for v in self.variables if v in self], sep = '\t\t')  # FOR NICE PRINTING WE TURN INTO AN INT NOW $$$$$$$$$$
        
    def printAllFrames(self):
        self.printTitle()
        for t in range(0, self.endTime+1):
            print(*[ int(self[v][t]) for v in self.variables if v in self], sep = '\t\t')  # FOR NICE PRINTING WE TURN INTO AN INT NOW $$$$$$$$$$
    # met kleur printen: print("\x1b[31m\"red\"\x1b[0m")  
    
    def exportDataFrame(self):
        df = pd.DataFrame.from_dict(self)#, columns=self.variables, index='t')
        df.to_csv('../rsViz/results.csv', index=False)
        
        
        
_gObservationModule = ObservationModule([])

def createObservationModule(*args):
    global _gObservationModule
    _gObservationModule = ObservationModule(args)
    return _gObservationModule

# use this function to use the global observation module
def gObservationModule():
    global _gObservationModule
    return _gObservationModule
"""
# class for warning objects. these are being mapped to the right isolation object.
class WarningObj:
    def __init__(self, _name : str = '', _warningType : str = '', _sensorObj = '', _time : int = 0, _indexDetect : int = 0):
        self.name = _name
        self.warningType = _warningType
        self.time = _time
        self.sensorObj = _sensorObj
        self.indexDetect = _indexDetect
        self.nameIsolation = ''

# Obeservation module detects possible sensor faults by looking at its residual.
class ObservMod:
    def __init__(self, _robotModel, activate=True, _treshhold:int = 20, _pastFrames:int = 10):
        self.robotModel = _robotModel
        self.activate = activate
        self._treshhold = _treshhold
        self.pastFrames = _pastFrames
        self.blackList = []
        self.listOfWarnings = []
        
    def faultDetectionPerSensor(self, sensObj, time:int, printLevel:int = 0):
        # compare sensor residual with treshhold to trigger the isolation prosess
        
        # Treshhold now in percentage. code below returns right value per type of sensor
        # it should actualy first get the maximum (size) of the sensor values somehow to be generic
        if sensObj.statevar == 'x':
            treshhold = 1000*(self._treshhold/100)
        elif sensObj.statevar == 'v':
            treshhold = 100*(self._treshhold/100)
        elif sensObj.statevar == 'a':
            treshhold = 100*(self._treshhold/100)
            
        residuals = gMonitorT()['r' + sensObj.name][-self.pastFrames:]
        #if sensorName == 'sV':
        #    print(gMonitorT()['r' + sensObj.name])
        if printLevel > 0:
            #print(f'prediction : {prediction}')
            #print(f'sensor value : {self.robotModel.getMotorOrSensor(sensorName).values[time]}')
            print(f'residual {sensObj.name} Value: {residuals[-1:]}')
        i = 0
        for r in reversed(residuals): 
            #print(abs(r))
            treshhold
            if  abs(r) > treshhold:
                templist = []
                for war in self.listOfWarnings:
                    if war.sensorObj.name == sensObj.name:
                            templist.append(war)
                            
                # pastframe and currentframe are two types of warnings (detections) that isolation can use. It is now not used.
                if len(templist) == 0:
                    name = 'Warning' + sensObj.name + str(time)
                    if i > 0:
                        warning = WarningObj(name, 'pastframe', sensObj, time, i)
                    else:
                        warning = WarningObj(name, 'currentframe', sensObj, time, i)
                    return warning
                else:
                    for war in templist:# algo voor sterke verandering??
                        timelast = (war.time - war.indexDetect)
                            
                        if timelast != (time - i):
                            name = 'Warning' + sensObj.name + str(time)
                            if i > 0:
                                warning = WarningObj(name, 'pastframe', sensObj, time, i)
                            else:
                                warning = WarningObj(name, 'currentframe', sensObj, time, i)
                            return warning
            i = i + 1
        return True

    def faultDetectionAllSensors(self, time:int):

        for sens in self.robotModel.sensors:
            if sens.name not in self.blackList:
                
                result = self.faultDetectionPerSensor(sens, time)
    
                if result is not True:
                    self.listOfWarnings.append(result)
                    

def createObservMod(robotModel, activate, _treshhold, _pastFrames):
    return ObservMod(robotModel, activate, _treshhold, _pastFrames)

