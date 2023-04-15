"""
FaultHandling.py
@author: Thibault Thétier from Robotic Sensing lab
Created: April 2022
"""
import numpy as np
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from scipy.stats import linregress

"""
       for demo of isolation of FDIR look at fhExperiment file
"""

"""
class FHModule:
    def __init__(self, _robotModel, activate=True):
        self.robotModel = _robotModel
        self.activate = activate
        
    def compareWithDatafusion(self, sensorName:str, time:int, printLevel:int = 1):
        # compare sensor data with datafusion to see if need to re-estimate
        
        threshold = 100
        prediction = self.robotModel.getMotorOrSensor(sensorName).observFunc(self.robotModel.states, time)
        comparison = abs(prediction - self.robotModel.getMotorOrSensor(sensorName).values[time])
        
        if printLevel > 0:
            #print(f'prediction : {prediction}')
            #print(f'sensor value : {self.robotModel.getMotorOrSensor(sensorName).values[time]}')
            print(f'error {sensorName} Value: {comparison}')
        
        if  comparison > threshold:
            #self.robotModel.getMotorOrSensor(sensorName).excludeObservations.append(time)
            return True
        else:
            return False

    def outlierDetection(self, time:int):
        outlierDetected = False
        for motsens in self.robotModel.motorsAndSensors:
            if self.compareWithDatafusion(motsens.name, time):
                outlierDetected = True
                #exclude
                self.robotModel.getMotorOrSensor(motsens.name).excludeObservations.append(time)
                #re-estimate
        if outlierDetected:
            newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)



def createFaultHandlingModule(robotModel, activate=True):
    return FHModule(robotModel, activate)
"""

# class for Isolation objects. these are being mapped to the right warning and recovery objects.
class IsoObj:
    def __init__(self, _name : str = '', _isoType : str = '', _sensorObj = '', _time : int = 0):
        self.name = _name
        self.isoType = _isoType
        self.time = _time
        self.sensorObj = _sensorObj
        self.nameWarning = ''
        self.nameRecovery = ''
        self.i = 0


class IsoModule:
    def __init__(self, _robotModel, _ObservMod, activate=True, driftTresh:int=1, constVar:int = 6, constTresh:int = 0.5, outTresh:int = 10):
        self.robotModel = _robotModel
        self.ObservMod = _ObservMod
        self.activate = activate
        self.sensAlarmCount = {}
        self.sensNoImmFault = {}
        self.constVar = constVar
        self.constTresh = constTresh
        self.driftTresh = driftTresh
        self.outTresh = outTresh
        
        # sensor eigenschappen, moet eigenlijk megeggeven worden bij het aanmaken van sensor
        self.outRangeMin = 0
        self.outRangeMax = 0
        self.disconnectValue = 0
        
        self.listOfIsolations = []
        
        for sens in self.robotModel.sensors:
            self.sensAlarmCount[sens.name] = [0]
            
        for sens in self.robotModel.sensors:
            self.sensNoImmFault[sens.name] = False
        
    def Isolation(self):

        for detect in self.ObservMod.listOfWarnings:
            
            if detect.sensorObj.statevar == 'x':
                self.outTresh = 10*self.outTresh
                self.driftTresh = 10*self.driftTresh
                self.constTresh = 10*self.constTresh
                
            if detect.sensorObj.statevar == 'a':
                self.outTresh = self.outTresh/2
                self.driftTresh = self.driftTresh/2
                self.constTresh = self.constTresh/2
            
            if detect.nameIsolation == '':
                time = detect.time
                    
                if detect.warningType == 'currentframe': # 2 soorten warnings???
                    
                    # bij /1 is 1 eigenlijk dt. bij de realrobot is dit momenteel 100ms dus 0.1s
                    rico = ((gMonitorT()['r' + detect.sensorObj.name][time])-(gMonitorT()['r' + detect.sensorObj.name][time-1]))/1
                    

                    # outlier check
                    if abs(rico) > self.outTresh:
                        r = gMonitorT()['r' + detect.sensorObj.name][time]
                        print(f'Outlier isolated with residual: {r}')
                        
                        name = 'Isolation' + detect.sensorObj.name + str(time)
                        Iso = IsoObj(name, 'Outlier', detect.sensorObj, time)
                        Iso.nameWarning = detect.name
                        self.listOfIsolations.append(Iso)
                        
                        detect.nameIsolation = name
                        self.sensAlarmCount[detect.sensorObj.name].append(time)
                            
                    else:  
                        #checks for disconnect, out of range min and out of range max fault. If not one of these then check drift.
                        if detect.nameIsolation == '' and self.sensNoImmFault[detect.sensorObj.name] is False:
                            self.disconnectOrOutOfRangeCalc(detect.sensorObj.name, detect, -20)
                            if detect.nameIsolation == '':
                                self.driftCalc(detect.sensorObj.name, detect, -20) #10 kan generieker
                            if detect.nameIsolation == '':
                                self.sensNoImmFault[detect.sensorObj.name] = True
                        if detect.nameIsolation == '':
                        
                            alarmArr = self.sensAlarmCount[detect.sensorObj.name][-10:]
                            for ind, x in enumerate(alarmArr):
                                if x < time - 10:
                                    alarmArr.pop(ind)
    
                            #check for faults again if past 10 poinst had more then 5 warnings (excluding outliers).
                            if len(alarmArr) > self.constVar-1:
                                start = 0
                                for warn in self.ObservMod.listOfWarnings:
                                    if warn.sensorObj == detect.sensorObj and warn.nameIsolation == 'no Isolation found' and warn.time in alarmArr:
                                        start = warn.time-1
                                        break
                                if start != 0:
                                    self.sensNoImmFault[detect.sensorObj.name] = False
                                    #constand and noice check
                                    self.disconnectOrOutOfRangeCalc(detect.sensorObj.name, detect, -20)
                                    if detect.nameIsolation == '':
                                        self.driftCalc(detect.sensorObj.name, detect, -20)
                                    if detect.nameIsolation == '':
                                        self.constandCalc(detect.sensorObj.name, detect, -20)
                                        
                            #if detect.nameIsolation == '':
                            #    self.driftCalc(detect.sensorObj.name, detect)
                        
                elif detect.warningType == 'pastframe':
                    self.disconnectOrOutOfRangeCalc(detect.sensorObj.name, detect, detect.indexDetect)
                    if detect.nameIsolation == '':
                        self.driftCalc(detect.sensorObj.name, detect, detect.indexDetect)
                
                 
                    
                if detect.nameIsolation == '':
                    detect.nameIsolation = 'no Isolation found'
                    print('no isolation found')
                    if detect.warningType == 'currentframe':
                        self.sensAlarmCount[detect.sensorObj.name].append(time)
            
            if detect.sensorObj.statevar == 'x':
                self.outTresh = self.outTresh/10
                self.driftTresh = self.driftTresh/10
                self.constTresh = self.constTresh/10
            
            if detect.sensorObj.statevar == 'a':
                self.outTresh = self.outTresh*2
                self.driftTresh = self.driftTresh*2
                self.constTresh = self.constTresh*2
        
        
        
    def driftCalc(self, nameSensor, detect, j = -10):
        time = detect.time
        
        resArr = gMonitorT()['r' + nameSensor]
        #index values checken
        smallResArr = resArr[(j if time >= j else time) :]
        arrLeng = len(smallResArr)
        count = 0
        ind = 0
        totalRico = 0
        outlierFlag = 0
        driftRico = 0
        for i in range(arrLeng-5):
            count = 0
            ind = 0
            while count < 4 :
                rico = (smallResArr[arrLeng-2-i-ind]-smallResArr[arrLeng-1-i-ind])/1
                ind = ind + 1
                if abs(rico) < self.outTresh:
                    count = count + 1
                    
                    totalRico = totalRico + rico
                elif outlierFlag == 0:
                    outlierFlag = count + i
            averageRico = totalRico/(count if count != 0 else 1)
            totalRico = 0
            if abs(averageRico) > self.driftTresh:
                if driftRico == 0:
                    driftRico = abs(averageRico)
                if outlierFlag > 0:
                    break
            elif abs(averageRico) < 0.5 and driftRico > 0:
                outlierFlag = i-1
                break
                
        if driftRico > 0:
           # print(f'Drift isolated with slope: {driftRico}')
            name = 'Isolation' + detect.sensorObj.name + str(time)
            Iso = IsoObj(name, 'Drift', detect.sensorObj, time)
            Iso.nameWarning = detect.name
            Iso.i = ((time - outlierFlag) if outlierFlag != 0 else (time -arrLeng if time-arrLeng>-1 else 0))
            print(f'Drift isolated with beginning: {Iso.i}')
            self.listOfIsolations.append(Iso)
            
            detect.nameIsolation = name
        #else:
           # print('no drift')
    
    def constandCalc(self, nameSensor, detect, j):
        time = detect.time
 
        resArr = gMonitorT()['r' + nameSensor]
        #index values checken
        smallResArr = resArr[(j if time >= j else time) :]
        arrLeng = len(smallResArr)
        count = 0
        ind = 0
        totalRico = 0
        outlierFlag = 0
        constRico = 0
        for i in range(arrLeng-5):
            count = 0
            ind = 0
            while count < 4 :
                rico = (smallResArr[arrLeng-2-i-ind]-smallResArr[arrLeng-1-i-ind])/1
                ind = ind + 1
                if abs(rico) < self.outTresh:
                    count = count + 1
                    
                    totalRico = totalRico + rico
                elif outlierFlag == 0:
                    outlierFlag = count + i
            averageRico = totalRico/(count if count != 0 else 1)
            totalRico = 0
            if abs(averageRico) < self.constTresh:
                if constRico == 0:
                    constRico = abs(averageRico)
                if outlierFlag > 0:
                    break

        if constRico > 0:
            #print(f'Constant isolated with slope: {constRico}')
            name = 'Isolation' + detect.sensorObj.name + str(time)
            Iso = IsoObj(name, 'Constant', detect.sensorObj, time)
            Iso.nameWarning = detect.name
            Iso.i = ((time - outlierFlag) if outlierFlag != 0 else (time -arrLeng if time-arrLeng>-1 else 0))
            print(f'Constant isolated with beginning: {Iso.i}')
            #print(Iso.i)
            self.listOfIsolations.append(Iso)
            
            detect.nameIsolation = name
            
    def disconnectOrOutOfRangeCalc(self, nameSensor, detect, i):
        time = detect.time
        
        resArr = gMonitorT()['r' + nameSensor]
        
        smallResArr = resArr[(i if time >= i else time) :]
        arrLeng = len(smallResArr)
        count = 0
        ind = 0
        totalRico = 0
        outlierFlag = 0
        for y in range(arrLeng-5):
            count = 0
            ind = 0
            while count < 4 :
                rico = (smallResArr[arrLeng-2-y-ind]-smallResArr[arrLeng-1-y-ind])/1
                ind = ind + 1
                if abs(rico) < self.outTresh:
                    count = count + 1
                    
                    totalRico = totalRico + rico
                elif outlierFlag == 0:
                    outlierFlag = count + y
            averageRico = totalRico/(count if count != 0 else 1)
            totalRico = 0
            if outlierFlag > 0:
                break    
            elif abs(averageRico) < 0.5:
                outlierFlag = y-1
                break
        StartPoint = time-outlierFlag
        #print(StartPoint)
        data = gMonitorT()[nameSensor][-(outlierFlag+1):]
        
        #is data constant without potential outliers
        count = 0
        totalRico = 0
        
        for j in range(len(data)-1):
            rico = (data[j+1]-data[j])/1
            #print(rico)
            #0.3 en 10 zijn aan te passen values. hangt eigenlijk af van u noice eigenschappen van u sensor
            if abs(rico) < self.outTresh:
                count = count + 1
                totalRico = totalRico + rico
                
        averageRico = totalRico/(count if count != 0 else 1)
        #print("average rico:" + str(averageRico))
        
        if abs(averageRico) < self.constTresh:
            average = np.average(data)
            
            #list without outliers
            data2 = []
            for y in data:
                if abs(y-average) < 15: #15 is een gekozen parameter
                    data2.append(y)
            
            average2 = np.average(data2)
            
            if abs(average2-self.disconnectValue) < 5:
                
                name = 'Isolation' + detect.sensorObj.name + str(time)
                Iso = IsoObj(name, 'Disconnect', detect.sensorObj, time)
                Iso.nameWarning = detect.name
                Iso.i = (StartPoint if outlierFlag != 0 else (time -arrLeng if time-arrLeng>-1 else 0))
                self.listOfIsolations.append(Iso)
                print(f'Disconnect isolated with beginning: {Iso.i}')
                detect.nameIsolation = name
                
            elif abs(average2-self.outRangeMin) < 5:
                
                name = 'Isolation' + detect.sensorObj.name + str(time)
                Iso = IsoObj(name, 'Out_Of_Range_Min', detect.sensorObj, time)
                Iso.nameWarning = detect.name
                Iso.i = (StartPoint if outlierFlag != 0 else (time -arrLeng if time-arrLeng>-1 else 0))
                self.listOfIsolations.append(Iso)
                print(f'Out of range minimum isolated with beginning: {Iso.i}')
                detect.nameIsolation = name
                
            elif abs(average2-self.outRangeMax) < 5:
                 
                 name = 'Isolation' + detect.sensorObj.name + str(time)
                 Iso = IsoObj(name, 'Out_Of_Range_Max', detect.sensorObj, time)
                 Iso.nameWarning = detect.name
                 Iso.i = (StartPoint if outlierFlag != 0 else (time -arrLeng if time-arrLeng>-1 else 0))
                 self.listOfIsolations.append(Iso)
                 print(f'Out of range maximum isolated with beginning: {Iso.i}')
                 detect.nameIsolation = name   
            #else:
                #print('average: ' + str(average2))
        
        
def createIsolationModule(robotModel, _ObservMod, activate, driftTresh, constVar, constTresh, outTresh):
    return IsoModule(robotModel, _ObservMod, activate, driftTresh, constVar, constTresh, outTresh)







