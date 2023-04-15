#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fhExperiment.py
@author: Thibault Thétier & Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: March 2022
"""

import time
import copy
import random
import sys
import numpy as np
import pandas as pd
import matplotlib.pylab as plt
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from rsFaultHandling.Observer import gObservationModule, createObservationModule, createObservMod
from rsFaultHandling.Corrupters import createCorrupterModule
from rsLibrary.Estimation import EstimationProblem, EstimationAlg, RandomChangeEstimation
import rsLibrary.Jacobian as Jacobian
from rsRobot.RobotEstimation import RobotModel
from Environment import Environment, Environment1D
from rsFaultHandling.FaultHandling import createFaultHandlingModule, createIsolationModule
from rsFaultHandling.Recovery import createRecoveryModule
from rsRobot.Control import Control1DUntilWallWithAcc

from State import StateDef, States
import Motors
import Sensors
import Robots

class Viz:
    def __init__(self, varValues:dict, turning:list):
        # create dataframes
        frame = pd.DataFrame.from_dict(varValues)
        
        robotFrame = frame[['t', 'ex', 'ev', 'ea', 'edist', 'edist2', 'sV', 'sA', 'tch', 'tch2', 'mX']]
        
        groundTruthFrame = frame[['t', 'x', 'v', 'a', 'dist', 'dist2']]
        
        errFrame = frame[['rdist', 'rsV', 'rsA']].rename(columns={"rdist": "ResidualDist", "rsV": "ResidualV", 'rsA': 'ResidualA'})
        errFrame['t'] = frame['t']
        errFrame = errFrame[['t', 'ResidualDist', 'ResidualV', 'ResidualA']]

        # visualize data
        fig, axs = plt.subplots(nrows=4, ncols=3)
        k,f = 1, 1
        variables = list(robotFrame.keys())
        states_variables = list(groundTruthFrame.keys())
        errVariables = list(errFrame.keys())
        
        axs[0,0].set_title('X')
        axs[0,0].plot(groundTruthFrame['t'], groundTruthFrame['x'], label='x', color='red')
        axs[0,0].plot(robotFrame['t'], robotFrame['ex'], label='ex')
        axs[0,0].set_xlabel('tijd (s)')
        axs[0,0].set_ylabel('afstand (cm)')
        #axs[1,0].set_title('V')
        #axs[1,0].plot(groundTruthFrame['t'], groundTruthFrame['v'], label='v', color='red')
        #axs[1,0].plot(robotFrame['t'], robotFrame['ev'], label='ev')
        #axs[1,0].set_xlabel('tijd (s)')
        #axs[1,0].set_ylabel('snelheid (cm/s)')
        axs[2,0].set_title('A')
        axs[2,0].plot(groundTruthFrame['t'], groundTruthFrame['a'], label='a', color='red')
        axs[2,0].plot(robotFrame['t'], robotFrame['ea'], label='ea')
        axs[2,0].set_xlabel('tijd (s)')
        axs[2,0].set_ylabel('versnelling (cm/s²)')
        
        axs[1,0].set_title('Draai')
        axs[1,0].plot(groundTruthFrame['t'], turning, label='turning', color='red')
        axs[1,0].set_xlabel('tijd (s)')
        axs[1,0].set_ylabel('aan het draaien')        
        
        axs[0,1].set_title('Residual afstandssensor')
        axs[0,1].plot(errFrame['t'], errFrame['ResidualDist'])
        axs[0,1].set_xlabel('tijd (s)')
        axs[0,1].set_ylabel('residual')
        axs[1,1].set_title('errV')
        axs[1,1].plot(errFrame['t'], errFrame['ResidualV'], label='resV')
        axs[1,2].set_title('errA')
        axs[1,2].plot(errFrame['t'], errFrame['ResidualA'], label='resA')
        
        axs[0,2].set_title('Afstandssensor')
        axs[0,2].plot(groundTruthFrame['t'], groundTruthFrame['dist'], label='afst', color='red')
        axs[0,2].plot(robotFrame['t'], robotFrame['edist'], label='eafst')
        axs[0,2].set_xlabel('tijd (s)')
        axs[0,2].set_ylabel('afstand')
        axs[2,1].set_title('sV')
        axs[2,1].plot(robotFrame['t'], robotFrame['sV'], label='sV')
        axs[2,2].set_title('sA')
        axs[2,2].plot(robotFrame['t'], robotFrame['sA'], label='sA')
        
        axs[3,0].set_title('Afstandssensor 2')
        axs[3,0].plot(groundTruthFrame['t'], groundTruthFrame['dist2'], label='afst2', color='red')
        axs[3,0].plot(robotFrame['t'], robotFrame['edist2'], label='eafst2')
        axs[3,0].set_xlabel('tijd (s)')
        axs[3,0].set_ylabel('afstand')
        axs[3,1].set_title('mX')
        axs[3,1].plot(robotFrame['t'], robotFrame['mX'], label='mX')
        axs[3,2].set_title('tch')
        axs[3,2].plot(robotFrame['t'], robotFrame['tch'], label='tch')
        axs[3,2].plot(robotFrame['t'], robotFrame['tch2'], label='tch2')
        
        for i in range(len(axs)):
            for j in range(len(axs[0])):
                axs[i,j].grid()
                axs[i,j].legend()
        
        """ 
        for i in range(len(axs)):
            for j in range(len(axs[0])):
                axs[i,j].grid()
                
                # show corruption time
                #if corruptionTimes != []:
                    #axs[i,j].annotate('corruption',xy=(time,20))
                    #axs[i,j].axvline(corruptionTimes[0], color='k', linewidth=1.5)
                    #for time in corruptionTimes:
                        
                        #axs[i,j].annotate('corruption',xy=(time,20))
                        #axs[i,j].axvline(time, color='k', linewidth=1.5)
                
                  
                if i != 1 and k<len(variables):
                    axs[i,j].plot(robotFrame['t'], robotFrame[variables[k]], label=variables[k])
                    
                    
                    #if i == 0:
                    if k < len(states_variables):
                        axs[i,j].plot(groundTruthFrame['t'], groundTruthFrame[states_variables[k]], label=states_variables[k])
                        
                    if variables[k] == 'edist' or variables[k] == 'tch':
                        k+=1
                        axs[i,j].plot(robotFrame['t'], robotFrame[variables[k]], label=variables[k])
                        axs[i,j].plot(groundTruthFrame['t'], groundTruthFrame[states_variables[k]], label=states_variables[k])
                    
                    axs[i,j].set_title(variables[k])
                    axs[i,j].legend()
                    k+=1
                    
                else:
                    if f < len(errVariables):
                        axs[i,j].plot(errFrame['t'], errFrame[errVariables[f]], label = errVariables[f])
                        axs[i,j].set_title(errVariables[f])
                        axs[i,j].legend()
                        f+=1
                """
                    
        
        '''
        for axi in axs:
            for axj in axi:
                if i<len(variables):
                    axj.grid()
                    axj.set_title(variables[i])
                    
                    #axj.plot(groundTruthDict['t'], groundTruthDict[states_variables[i]], label='Truth')
                    #axj.plot(noCorruptionDict['t'], noCorruptionDict[variables[i]], label='Base')
                    axj.plot(robotFrame['t'], robotFrame[variables[i]], label='Robot Model')
                    
                    #ticks = axj.set_xticks(valuesDict['t'])
                    axj.legend()
                    i+=1
        i = 1
        for axi in axs:
            for axj in axi:
                if i<len(states_variables):
                    axj.plot(groundTruthFrame['t'], groundTruthFrame[states_variables[i]], label='Truth')
                    axj.legend()
                    i+=1
        #frame.plot(x= 't',subplots=True, layout=(4, 4), figsize=(10,6), sharex=False)
        '''

def dict2str(d:dict) -> str:
    l_f = [str(k)+': %.2f'%(d[k]) for k in d]
    return ', '.join(l_f)               

def createSimRobot1D(walls:tuple = (-100, 100), noiseLevel:int = 0):

    environment1D = Environment1D(walls)
    variables = ['a', 'v', 'x'] # acceleration holds during the whole period, v and x is the value at the end of the period
    relationDict ={'v': 'a', 'x': 'v'}    
    def forwardMath1D(states, t, dts):
        states.setValue('v', t, states['a'][t] * dts[t] + states['v'][t - 1]) 
        states.setValue('x', t, (states['v'][t-1] + states['v'][t]) / 2 * dts[t] + states['x'][t - 1])
    stateDef = StateDef(variables, relationDict, forwardMath1D)
    
    ourMotor = Motors.OurMotor("mX", ["a", "v"], param1Value = 1, param2Value = 1)
    
    from scipy.stats import norm
    # size of data of sensors are different so the size of simulated noice is also different
    noiseDistribution = norm(scale = noiseLevel*0.25) if noiseLevel > 0 else None
    noiseDistributionSmall = norm(scale = noiseLevel*0.0416) if noiseLevel > 0 else None
    noiseDistributionVerrySmall = norm(scale = noiseLevel*0.0025) if noiseLevel > 0 else None
    
    #WithConstant is added for recovering constant faults
    accelerometer = Sensors.LineairSensorWithConstant("sA", "a", 'la', 10, noiseDistr = noiseDistributionVerrySmall)
    odometer = Sensors.LineairSensorWithConstant("sV", "v", 'lv', 10, noiseDistr = noiseDistributionSmall)
    #distanceSensor = Sensors.LineairSensor("sX", "x", 'lx', 10, noiseDistr = noiseDistribution)
    touchSensor = Sensors.EventSensor("tch", "x", 'lw', environment1D.walls[1])
    touchSensor2 = Sensors.EventSensor('tch2', 'x', 'lw', walls[0])
    distanceSensor = Sensors.DistanceSensorWithConstant('dist', 'x', 'lw', walls[1], noiseDistr = noiseDistribution)
    distanceSensor2 = Sensors.DistanceSensorWithConstant('dist2', 'x', 'lw', walls[0], noiseDistr = noiseDistribution)
    
    
    states = States(stateDef, {'a':0, 'v':0, 'x':0})
    return Robots.SimRobot("robot sim 1D", states, [ourMotor], [accelerometer, odometer, distanceSensor, distanceSensor2, touchSensor, touchSensor2], environment1D)
    
# calculating difference between two graphs (arrays). it calculates mean difference and maximum difference   
def Results(a, b):
    TotDiff = 0
    AverageDiff = 0
    MaxDiff = 0
    for i in range(len(a)):
        diff = abs(a[i] - b[i])
        TotDiff += diff
        if diff > MaxDiff:
            MaxDiff = diff
    AverageDiff = TotDiff/len(a)

    return round(AverageDiff,4), round(MaxDiff,4) 

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####           MAIN          #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    
    #hoe veel is estimation ernaast? met en zonder recovery voor elke fout elke sensor met data van enlke sensor (wel meerdere groottes van fout want elk ander effect)
    
    #### #### #### ####  1D-robot with a, v, x  #### #### #### #### 
    
    """
        Simulation Parameters
    """
    walls = [-100, 100]
    endFrame = 100
    percentage = 10
    speed = 2
    pastFrames = 1
    startFault = 40
    
    """
        Objects Instantiation
    """    
    simRobot = createSimRobot1D(walls = (-100, 100), noiseLevel = 2)
    simRobot.printConfiguration()
    
    # create control WithAcc
    ctrl = Control1DUntilWallWithAcc(simRobot.environment, endFrame, speed)
    
    # Motor(s)AndSensors naam verandert
    robotModel = RobotModel("robotModel", simRobot)
    
    # Observation Module for alarm triggering / treshhold is in percentage
    ObservMod = createObservMod(robotModel, activate=True, _treshhold=7, _pastFrames = pastFrames)
    
    # Isolation 
    IsoModule = createIsolationModule(robotModel, ObservMod, activate=True, driftTresh=0.3, constVar = 6, constTresh = 0.3, outTresh = 4)
    
    # Recovery
    RecModule = createRecoveryModule(robotModel, ObservMod, IsoModule, simRobot.environment, activate=True, speed = speed)
    
    #out of range values:
    Min = 100
    Max = 100000
    #disconnect value:
    disconnectVal = 0
    
    IsoModule.outRangeMin = Min
    IsoModule.outRangeMax = Max
    IsoModule.disconnectValue = disconnectVal
    
    #name corrupters: outlier, constant, drift, out_range, disconnect
    #Dictionaries of corruptors: syntax: sensors/motors/states = {'name object1' : ['name corruption', value1, value2],
    #                                                             'name object2' : ['name corruption', value1, value2], ...}
    sensors = {}#'dist' : ['disconnect',disconnectVal,startFault]}
    #           'dist2' : ['out_range', Min, Max]}
    
    # corrupters
    corrupter = createCorrupterModule(robotModel, endFrame=endFrame, percentage=percentage, sensors=sensors)

    # motor input
    motor_input = []
    
    
    """
        Robot Calibration
    """
    #robotModel.getMotorOrSensor('tch').isCalibrated = True
    #robotModel.getMotorOrSensor('mX').isCalibrated = True
    #robotModel.randomizeParameters(seed = 23) # only those not calibrated
    
    for motsens in robotModel.motorsAndSensors:
        motsens.isCalibrated = True
        
    robotModel.printConfiguration()
    
    print('Variables to be estimated: ', end='')
    print(str(robotModel.updateableVariableNames()))
    print('Dependent variables     : ', end='')
    print(str(robotModel.stateDef.depVariables))
    print(' == == == == == == == == == == == == == == == == == == == ==')

    
    """
        Data log
    """
    # global monitor
    createMonitorT("mX", simRobot.stateDef.variables, [s.name for s in simRobot.sensors], 'ea', "ev", "ex", 'elp', 'elm', 'ela', 'elv', "elx", 'elw', "it", 'end', 'rmX', 'rsA','rsV', 'rtch', 'rtch2', 'rdist', 'rdist2', "diff").printTitle()
    # global observation module
    # createObservationModule("mX",simRobot.stateDef.variables, [s.name for s in simRobot.sensors])
    
    
    """
        Main loop
    """
    a = 1
    motor = simRobot.motors[0].name
    start_time = time.time()
    t = 0
    turning = []
    while ctrl.hasNext() and t < 101:
    #for i in range(0, max(2, len(motor_input))):
        
        t += 1
        #print(' ============================= FRAME '+str(t)+' ============================')
        
        
        
        # back-propagate only to the last 10 frames
        if t > pastFrames:
            robotModel.states.startUnknown = t - pastFrames
        
        """
            Command Generation
        """
        nextCommand = ctrl.nextCommand(t)
        
        #simulating a turning command in 1D
        if nextCommand > 999:
            turning.append(1)
            nextCommand = 0
        else:
            turning.append(0)
            
        motor_input.append(nextCommand)
        key_value = {motor : motor_input[t-1]}
        
        
        """
            Motor values
        """
        # corrupter: motor values u->u*
        if corrupter.commandCorruption:
            corrupter.modify('motor', key_value, t)
        
        # observer: motor values u*
        gMonitorT().setValues(key_value , t)
        #gObservationModule().setValues(key_value, t)
        
        #print(key_value, end='')
        simRobot.setMotors(key_value)
        
        """
            Sensor values
        """
        sensorValues = simRobot.readSensors()
        sensorValues[motor] = key_value[motor]  # also motor inputs!
        
        # corrupter: sensor values sv->sv*
        if corrupter.sensorsCorruption:
            corrupter.modify('sensor', sensorValues, t)
        
        # observer: sensor values sv*
        gMonitorT().setValues(sensorValues, t)
        #gObservationModule().setValues(sensorValues, t)
        # print(' =>', sensorValues, end='')
    
        """
            Estimation
        """
        # link RobotSim to RobotModel
        robotModel.addSensorReading(sensorValues, t)
        #robotCopy = copy.deepcopy(robotModel)
        
        #if t==10:
            #robotModel.getMotorOrSensor('dist').excludeObservations.append(t)
        # estimate
        changes = robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0)
        
        
        """
            Fault Handling:
                Alarm-Triggering
        """
        if ObservMod.activate:
            ObservMod.faultDetectionAllSensors(t)
        
        """
                Isolation
        """
        if IsoModule.activate:
            IsoModule.Isolation()
            
        """
                Recovery
        """
        if RecModule.activate:
            newctrl = RecModule.Recovery(ctrl, t)
            
        if RecModule.changeControl:
            print('changing Control...')
            ctrl = newctrl
            RecModule.changeControl = False
            

            #print(f"truth: {simRobot.states['x'][t]} \nx estimation: {robotModel.states['x'][-1]}\nx re-estimation: {robotCopy.states['x'][-1]}")        
               
        #for k, v in changes.items():
            #print('Unknown ', k, ' changed from  %.3f'%(v[0]) , 'to %.3f'%(v[1]))

        """
            corrupter
        """
        # corrupter: state estimation st->st*
        if corrupter.statesCorruption:
            corrupter.modify('states', robotModel.states, t)
        
        robotModel.diffWithRef(simRobot)
        
        #for estimating every point twice, removes error in which some state estimations are 1
        if t == startFault -1 :
            robotModel.states.startUnknown = 1
            #re-estimate
            newchanges = robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
        if t == 100 :
            robotModel.states.startUnknown = startFault+1
            #re-estimate
            newchanges = robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0, newFrame=False)
        
        
        #gMonitorT().printAllFrames()
        #gObservationModule().printAllFrames()
        
        '''if simRobot.environment.touchedWall:
            # we assume that estimation is OK now
            robotModel.states.startUnknown = robotModel.states.now + 1
            for motsens in robotModel.motorsAndSensors():
                motsens.isCalibrated = True'''
            
    # viz data
    turning.append(0)
    Viz(gMonitorT(),turning)
    print("--- %s seconds ---" % (time.time() - start_time))
    for iso in IsoModule.listOfIsolations:
        print('Isolation: name sensor: ' + iso.sensorObj.name + ' \t time: ' + str(iso.time) + '\t type: ' + iso.isoType + '\n')
        
    for rec in RecModule.listOfRecoveries:
        print('Recoverie: name sensor: ' + rec.sensorObj.name + ' \t time: ' + str(rec.time) + '\t type: ' + rec.recType + '\n')
    #gMonitorT().printAllFrames()
    
    points = endFrame - startFault
    
    xValues = gMonitorT()['x'][-points:]
    vValues = gMonitorT()['v'][-points:]
    aValues = gMonitorT()['a'][-points:]
    exValues = gMonitorT()['ex'][-points:]
    evValues = gMonitorT()['ev'][-points:]
    eaValues = gMonitorT()['ea'][-points:]
    
    print(Results(xValues, exValues))
    print(Results(vValues, evValues))
    print(Results(aValues, eaValues))
    
    
            