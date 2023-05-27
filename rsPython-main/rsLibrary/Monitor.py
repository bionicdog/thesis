#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Monitor.py: keep track of values present throughout the system and print them nicely
@author: Jan Lemeire from Robotic Sensing lab
Created: October 2021
"""
from numbers import Number
import pandas as pd


#### #### #### ####  MONITOR #### #### #### #### 
# keeps one value per variable   
class Monitor (dict):
    colWidth = 10
    
    def __init__(self, args):
        dict.__init__(self)
        self.variables = []
        self.addVariables(args)
#        print(self.variables)
        
    def addVariables(self, args):
        for _vars in args:
 #           print(str(_vars)+":"+str(type(_vars)))
            if isinstance(_vars, list):
                self.variables.extend(_vars)
            else:
                self.variables.append(_vars)
    
    def setValue(self, var: str, value: Number):
        self[var] = value

    def setValues(self, varValueDict: dict):
        for k, v in varValueDict.items():
            self[k] = v
        
    def setColumnWidth(self, columnWidth:int):
        self.colWidth = columnWidth

    def printTitle(self):
        print(*[v.ljust(self.colWidth) for v in self.variables], sep = '')  # 'string with tabs'.expandtabs(8)
        print(*['--'.ljust(self.colWidth) for v in self.variables], sep = '\t\t')
        
    def printValues(self):
        print(*[ str(int(self[v])).ljust(self.colWidth) for v in self.variables if v in self], sep = '')  # FOR NICE PRINTING WE TURN INTO AN INT NOW $$$$$$$$$$
        
        
_gMonitor = Monitor([])

def createMonitor(*args):
    global _gMonitor
    _gMonitor = Monitor(args)
    return _gMonitor

# use this function to use the global monitor
def gMonitor():
    global _gMonitor
    return _gMonitor


#### #### #### ####  MONITORT #### #### #### #### 
# keeps values over time for each variabe, with time from 0..endTime
# endTime is the maximal time over all calls to setValue
# dict of arrays

class MonitorT (dict):
    endTime = 0 # there is at least 1 frame
    colWidth = 10
    nbrSignificantDigits = 4
    _sign_str = '.'+str(nbrSignificantDigits)+'g'
    _max_not_e = pow(10, colWidth)
    _max_int_sign_digits = pow(10, nbrSignificantDigits)
    
    def __init__(self, args):
        dict.__init__(self)
        self.variables = ['t']
        self['t'] = [ 0 ]
        self.addVariables(args)
        
#        print(self.variables)
        
    def addVariables(self, args):
        for _vars in args:
 #           print(str(_vars)+":"+str(type(_vars)))
            if isinstance(_vars, list):
                self.variables.extend(_vars)
            else:
                self.variables.append(_vars)
        # create list for all variables
        for v in self.variables:
            if not v in self:
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

    def setValues(self, varValueDict:dict, time:int= -1):
        self.enlargeFramesIfNecessary(time)
        for var, val in varValueDict.items():
            if var not in self:
                self[var] = [-1] * (self.endTime  + 1)
            self[var][time] = val

    def setValuesWithPrefix(self, prefix:str, varValueDict:dict, time:int= -1):
        self.enlargeFramesIfNecessary(time)
        for var, val in varValueDict.items():
            pvar = prefix + var
            if pvar not in self:
                self[pvar] = [-1] * (self.endTime  + 1)
            self[pvar][time] = val
            
    def enlargeFramesIfNecessary(self, time:int):
        if (time > self.endTime):
            for v in self:
                for t in range(len(self[v]), time + 1):
                    self[v].append(t if v == 't' else -1)
            self.endTime = time
            
    def setColumnWidth(self, columnWidth:int):
        self.colWidth = columnWidth
        self._max_not_e = pow(10, self.colWidth)

    def setNbrSignificantDigits(self, nbrSignificantDigits:int):
        self.nbrSignificantDigits = 3
        self._max_int_sign_digits = pow(10, nbrSignificantDigits)
        self._sign_str = '.'+str(nbrSignificantDigits)+'g'
        
    def printTitle(self):
        if len(self.variables) > 1:
            print(*[v.ljust(self.colWidth) for v in self.variables], sep = '')  # 'string with tabs'.expandtabs(8)
            print(*['--'.ljust(self.colWidth) for v in self.variables], sep = '')
        
    def value2string(self, v) -> str:
        if isinstance(v, Number):
            if abs(v) < 0.00000001:
                return '0'.ljust(self.colWidth)
            if v >= self._max_int_sign_digits and v < self._max_not_e:
                return str(int(v)).ljust(self.colWidth)
            else:
                return format(v, self._sign_str).ljust(self.colWidth)
        else: 
            return str(v).ljust(self.colWidth)

    def showAllVariables(self):
        self.addVariables(list(self.keys()))
    def printLastFrame(self):
        if len(self.variables) > 1:
            print(*[ self.value2string(self[v][self.endTime]) for v in self.variables if v in self], sep = '') 
    def printAllFrames(self):
        if len(self.variables) > 1:
            self.printTitle()
            for t in range(0, self.endTime+1):
                print(*[ self.value2string(self[v][t]) for v in self.variables if v in self], sep = '')
    # met kleur printen: print("\x1b[31m\"red\"\x1b[0m")     
        
    def exportDataFrame(self):
        df = pd.DataFrame.from_dict(self)#, columns=self.variables, index='t')
        df.to_csv('../rsViz/results.csv', index=False)
        
    def showOnTimeLine(self, *variables:str):
        import matplotlib.pyplot as plt
        for var in variables:
            plt.plot(self['t'], self[var], label = var)
        plt.xlabel(xlabel='time')
        plt.legend(loc='upper center')
        plt.show()
        
_gMonitorT = MonitorT([])

def createMonitorT(*args):
    global _gMonitorT
    _gMonitorT = MonitorT(args)
    return _gMonitorT

# use this function to use the global monitor
def gMonitorT():
    global _gMonitorT
    return _gMonitorT


### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    if False:
        createMonitorT('a', 'b', 'c')
        gMonitorT().setValue('a', 12345, 0)
        gMonitorT().setValue('b', 12.34433, 0)
        
        gMonitorT().printAllFrames()
        
        gMonitorT().setValue('a', 0.02434, 0)
        gMonitorT().setValue('b', 12.345, 0)
        
        gMonitorT().setValue('a', 360404054050, 1)
        gMonitorT().setValue('b', 6)
        gMonitorT().setValue('c', 3, 1)
        print('print everything again')
        gMonitorT().printAllFrames()
    if True: # read from file with list of dictionaries
        import pickle
        file = open('dataEstWithMotor.pkl', 'rb')
        data = pickle.load(file)
        file.close()
        createMonitorT()
        t=0
        for frame in data:
            gMonitorT().setValues(frame, t)
            t = t + 1
            
        gMonitorT().showAllVariables()
        gMonitorT().printAllFrames()