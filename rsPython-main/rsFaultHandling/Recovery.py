"""
Recovery.py
@author: Marco Van Cleemput
Created: june 2022
"""
import numpy as np
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from scipy.stats import linregress
from rsRobot.Control import ControlBySpeed, Control1DForwardUntilWall, Control1DTurn180
from rsLibrary.Monitor import gMonitorT

"""
       for demo of recovery of FDIR look at fhExperiment file
"""

# class for recovery objects. these are being mapped to the right isolation objects.
class RecObj:
    def __init__(self, _oldControl, _name : str = '', _recType : str = '', _sensorObj = '', _time : int = 0):
        self.name = _name
        self.recType = _recType
        self.time = _time
        self.sensorObj = _sensorObj
        self.nameIsolation = []
        self.oldControl = _oldControl
        self.step = 0
        self.Counter = 0
        self.i = 0
        self.x = 0
        self.deltaTime = 0
        self.touchedWall = False
        

class RecoveryModule:
    def __init__(self, _robotModel, _ObservMod, _IsoMod, _environment, activate=True, speed: int = 2):
        self.robotModel = _robotModel
        self.ObservMod = _ObservMod
        self.IsoMod = _IsoMod
        self.environment = _environment
        self.activate = activate
        self.changeControl = False
        self.speed = speed
        
        self.listOfRecoverings = []
        self.listOfRecoveringsForever = []
        self.listOfRecoveries = []
        
    def Recovery(self, oldControl, time):
        newControl = oldControl
        for iso in self.IsoMod.listOfIsolations:
            if iso.nameRecovery == '':
                
                for rec in self.listOfRecoverings:
                    if iso.sensorObj == rec.sensorObj:
                        iso.nameRecovery = rec.name 
                        rec.nameIsolation.append(iso.name)
                        break
                    
                for rec in self.listOfRecoveringsForever:
                    if iso.sensorObj == rec.sensorObj:
                        iso.nameRecovery = rec.name 
                        rec.nameIsolation.append(iso.name)
                        break
                    
            if iso.nameRecovery == '':
                startTime = time - 20
                # 6 Isolations: Outlier, Drift, Constand, Disconnect, Out_Of_Range_Min, Out_Of_Range_Max
                if iso.isoType == 'Outlier':
                    #exclude
                    iso.sensorObj.excludeObservations.append(time)
                    
                    if time > 2:
                        self.robotModel.states.startUnknown = time - 2
                    #re-estimate
                    newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
                    
                    print(f'Outlier recovered at time: {time}')
                    
                    name = 'Recovery' + iso.sensorObj.name + str(time)
                    Rec = RecObj(oldControl, name, 'Outlier', iso.sensorObj, time)
                    Rec.nameIsolation.append(iso.name)
                    self.listOfRecoveries.append(Rec)
                    
                    iso.nameRecovery = name
                    
                elif iso.isoType == 'Drift' :
                    print(f'Drift recovered at time: {time}')
                    
                    iso.sensorObj.isCalibrated = False
                    
                    #print(iso.sensorObj.params)
                    #print(iso.i)
                    self.robotModel.states.startUnknown = (iso.i if iso.i > 0 else (time + iso.i if time + iso.i > 0 else 0))
                    #re-estimate
                    newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
                    
                    iso.sensorObj.isCalibrated = True
                    #print(iso.sensorObj.params)
                    
                    name = 'Recovery' + iso.sensorObj.name + str(time)
                    Rec = RecObj(oldControl, name, 'Drift', iso.sensorObj, time)
                    Rec.nameIsolation.append(iso.name)
                    Rec.recType = 'Drift'
                    self.listOfRecoveries.append(Rec)
                    
                    iso.nameRecovery = name
                    
                elif iso.isoType == 'Constant':
                    #recalibrating with constand
                    
                    print(f'Constand recovered at time: {time}')
                    
                    iso.sensorObj.isCalibrated = False
                    
                    #print(iso.sensorObj.params)
                    self.robotModel.states.startUnknown = (iso.i if iso.i > 0 else (time + iso.i if time + iso.i > 0 else 0))
                    #re-estimate
                    newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
                    
                    iso.sensorObj.isCalibrated = True
                    #print(iso.sensorObj.params)
                    name = 'Recovery' + iso.sensorObj.name + str(time)
                    Rec = RecObj(oldControl, name, 'Constant', iso.sensorObj, time)
                    Rec.nameIsolation.append(iso.name)
                    self.listOfRecoveries.append(Rec)
                    
                    iso.nameRecovery = name
                    
                elif iso.isoType == 'Disconnect':
                    self.ObservMod.blackList.append(iso.sensorObj.name)
   
                    iso.sensorObj.disableFrom = (iso.i if iso.i > 0 else (time + iso.i if time + iso.i > 0 else 0))
                    
                    if time > 20:
                        self.robotModel.states.startUnknown = (iso.i if iso.i > 0 else (time + iso.i if time + iso.i > 0 else 0))
                    
                    #re-estimate
                    newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)    
   
                    print(f'Sensor disconnected recovered at time: {time}')
                    
                    name = 'Recovery' + iso.sensorObj.name + str(time)
                    Rec = RecObj(oldControl, name, 'Disconnect', iso.sensorObj, time)
                    Rec.nameIsolation.append(iso.name)
                    Rec.recType = 'Disconnect'
                    Rec.i = iso.i
                    Rec.deltaTime = 5
                    Rec.Counter = Rec.deltaTime/0.5
                    self.listOfRecoveringsForever.append(Rec)
                    
                    iso.nameRecovery = name
                        
                elif iso.isoType == 'Out_Of_Range_Min':
                    self.ObservMod.blackList.append(iso.sensorObj.name)
                    

                    iso.sensorObj.disableFrom = (iso.i)
                    
                    if time > 20:
                        self.robotModel.states.startUnknown = (iso.i)
                    
                    #re-estimate
                    newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)         
   
                    print(f'Under sensor range recovered at time: {time}')
                    
                    name = 'Recovery' + iso.sensorObj.name + str(time)
                    Rec = RecObj(oldControl, name, 'Disconnect', iso.sensorObj, time)
                    Rec.nameIsolation.append(iso.name)
                    Rec.recType = 'Out_Of_Range_Min'
                    Rec.i = iso.i
                    Rec.deltaTime = 5
                    Rec.Counter = Rec.deltaTime/0.5
                    self.listOfRecoveringsForever.append(Rec)
                    
                    iso.nameRecovery = name
                    
                elif iso.isoType == 'Out_Of_Range_Max':
                    self.ObservMod.blackList.append(iso.sensorObj.name)
                    #exclude
                    
                    iso.sensorObj.disableFrom = (iso.i if iso.i > 0 else (time + iso.i if time + iso.i > 0 else 0))
                    
                    if time > 20:
                        self.robotModel.states.startUnknown = (iso.i if iso.i > 0 else (time + iso.i if time + iso.i > 0 else 0))
                    
                    #re-estimate
                    newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
                    
                    print(f'Over sensor range recovered at time: {time}')
                    
                    name = 'Recovery' + iso.sensorObj.name + str(time)
                    Rec = RecObj(oldControl, name, 'Out_Of_Range_Max', iso.sensorObj, time)
                    Rec.nameIsolation.append(iso.name)
                    self.listOfRecoveringsForever.append(Rec)
                    
                    iso.nameRecovery = name
        
        # in listOfRecoverings you can find recovery objects that are busy with their recovery action.
        if len(self.listOfRecoverings) != 0:
            rec = self.listOfRecoverings[0]
            # 2 Recovering actions: Disconnect of dist sensor, Out_Of_Range_Min of dist sensor 

            if rec.recType == 'Out_Of_Range_Min' :
                    #nu welke distance: normaal met orientatie in 2d
                    
                    if rec.step == 0:
                        print('Changing control to out range min recovery action')
                        rec.step += 1
                        self.changeControl = True
                        rec.oldControl = oldControl

                        x = gMonitorT()['ex'][len(gMonitorT()['ex'])-1]
                        
                        if rec.sensorObj.name == 'dist': #eig niet de sensor maar de orientatie
                            newControl = Control1DForwardUntilWall(self.environment, 200, self.speed) 
                            rec.x = (x-1) + 3
                            rec.smaller = True
                        elif rec.sensorObj.name == 'dist2':
                            newControl = Control1DForwardUntilWall(self.environment, 200, -self.speed)
                            rec.x = x+1
                            rec.smaller = False
                            
                    if rec.step == 1: 
                        if self.environment.touchedWall:
                            rec.touchedWall = True
                        #print(rec.touchedWall)
                        if rec.smaller:
                            if gMonitorT()['ex'][len(gMonitorT()['ex'])-1] < rec.x and rec.touchedWall:
                                newControl = rec.oldControl
                                self.changeControl = True
                                self.listOfRecoverings.pop(0)
                                rec.step = 0
                                rec.touchedWall = False
                                print('Changing control from out range min recovery action back to normal')
                        else:
                            if gMonitorT()['ex'][len(gMonitorT()['ex'])-1] > rec.x and rec.touchedWall:
                                newControl = rec.oldControl
                                self.changeControl = True
                                self.listOfRecoverings.pop(0)
                                rec.step = 0
                                rec.touchedWall = False
                                print('Changing control from out range min recovery action back to normal')
                                
            elif rec.recType == 'Disconnect' :
                    #nu welke distance: normaal met orientatie
                    
                    if rec.step == 0:
                        rec.step += 1
                        if rec.sensorObj.name == 'dist':
                            self.robotModel.getMotorOrSensor('dist2').disableFrom = time
                            self.ObservMod.blackList.append('dist2')
                        elif rec.sensorObj.name == 'dist2':
                            self.robotModel.getMotorOrSensor('dist').disableFrom = time
                            self.ObservMod.blackList.append('dist')
                            
                        self.changeControl = True
                        rec.oldControl = oldControl
                        print("Changing control to disconnect recovery action")
                        print('turning...')
                        newControl = Control1DTurn180(80, 100)
                        rec.x = time
                    if time > rec.x:
                        if rec.step == 1 and oldControl.step == 2:
                            
                            rec.Counter += 1
                            if rec.Counter >= 10:
                                print("standing still...")
                                if rec.sensorObj.name == 'dist':
                                    for i in range(1,9):
                                        self.robotModel.getMotorOrSensor('dist').values[time-i] = 1000-self.robotModel.getMotorOrSensor('dist2').values[time-i] #eig rekening houden met verschillende parameters
                                        
                                elif rec.sensorObj.name == 'dist2':
                                    for i in range(1,9):
                                        self.robotModel.getMotorOrSensor('dist2').values[time-i] = self.robotModel.getMotorOrSensor('dist').values[time-i]
                                #re estimate:
                                disableT = rec.sensorObj.disableFrom
                                rec.sensorObj.disableFrom = 10000000
                                
                                if time > 10:
                                    self.robotModel.states.startUnknown = time - 10

                                newchanges = self.robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
                                
                                rec.sensorObj.disableFrom = disableT
                                
                                rec.step += 1
                                oldControl.step = 0
                                print('turning...')

                    if rec.step == 2 and oldControl.step == 2:
                            newControl = rec.oldControl
                            self.changeControl = True
                            self.listOfRecoverings.pop(0)
                            rec.step = 0
                            rec.Counter = 0
                            print('Changing control from disconnect recovery action back to normal')
                            if rec.sensorObj.name == 'dist':
                                self.robotModel.getMotorOrSensor('dist2').disableFrom = 10000000
                                for ind, black in enumerate(self.ObservMod.blackList):
                                    if black == 'dist2':
                                        self.ObservMod.blackList.pop(ind)
                                        break
                            elif rec.sensorObj.name == 'dist2':
                                self.robotModel.getMotorOrSensor('dist').disableFrom = 10000000
                                
                                for ind, black in enumerate(self.ObservMod.blackList):
                                    if black == 'dist':
                                        self.ObservMod.blackList.pop(ind)
                                        break
                                    
        # in listOfRecoveringsForever you can find recovery objects that are not busy with their recovery action. It also checks if the sensor still has the fault.
        if len(self.listOfRecoveringsForever) != 0:
            
               for x, rec in enumerate(self.listOfRecoveringsForever):
                   if rec not in self.listOfRecoverings:
                       minus = rec.i - (time+1)
                       data = gMonitorT()[rec.sensorObj.name][minus:]
                       
                       average = np.average(data)
                       
                       #list without outliers
                       data2 = []
                       for y in data:
                           if abs(y-average) < 15:
                               data2.append(y)
                       
                       average2 = np.average(data2)
                       #print(average2)
                       # 2 Recovering actions: Disconnect of dist sensor, Out_Of_Range_Min of dist sensor 
                       if rec.recType == 'Disconnect' :
                           if abs(average2-self.IsoMod.disconnectValue) < 5:
                               if rec.sensorObj.sensorType == 'distance':
    
                                   if rec.Counter >= rec.deltaTime/0.5:
                                       rec.Counter = 0
                                       
                                       self.listOfRecoverings.append(rec)
        
                                   else:
                                       rec.Counter += 1
                           else:
                               for ind, black in enumerate(self.ObservMod.blackList):
                                   if black == rec.sensorObj.name:
                                       self.ObservMod.blackList.pop(ind)
                                       break
                               self.listOfRecoveries.append(rec)
                               self.listOfRecoveringsForever.pop(x)
                               rec.sensorObj.disableFrom = 10000000
                               for i in range(1,self.ObservMod.pastFrames):
                                   rec.sensorObj.excludeObservations.append(time -i)
                               
                       elif rec.recType == 'Out_Of_Range_Min' :
                           if abs(average2-self.IsoMod.outRangeMin) < 5:
                               if rec.sensorObj.sensorType == 'distance':
    
                                   if rec.Counter >= rec.deltaTime/0.5:
                                       rec.Counter = 0
                                       self.listOfRecoverings.append(rec)
        
                                   else:
                                       rec.Counter += 1
                           else:
                               for ind, black in enumerate(self.ObservMod.blackList):
                                   if black == rec.sensorObj.name:
                                       self.ObservMod.blackList.pop(ind)
                                       break
                               self.listOfRecoveries.append(rec)
                               self.listOfRecoveringsForever.pop(x)     
                               rec.sensorObj.disableFrom = 10000000
                               print('no out range')
                               for i in range(1,self.ObservMod.pastFrames):
                                   rec.sensorObj.excludeObservations.append(time -i)
                              
                       elif rec.recType == 'Out_Of_Range_Max' :
                           if abs(average2-self.IsoMod.outRangeMax) > 4.9:
                              print('nimmeer boven max aver: ' +str(average2))
                              for ind, black in enumerate(self.ObservMod.blackList):
                                  if black == rec.sensorObj.name:
                                      self.ObservMod.blackList.pop(ind)
                                      break
                              self.listOfRecoveries.append(rec)
                              self.listOfRecoveringsForever.pop(x)   
                              rec.sensorObj.disableFrom = 10000000
                              for i in range(1,self.ObservMod.pastFrames):
                                  rec.sensorObj.excludeObservations.append(time -i)
          
        return newControl
    
def createRecoveryModule(robotModel, _ObservMod, _IsoMod, _environment, activate, speed):
    return RecoveryModule(robotModel, _ObservMod, _IsoMod, _environment, activate, speed)



