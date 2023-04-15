#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Estimation.py
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""

import copy
import random
import sys
import math
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT
from rsFaultHandling.Observer import gObservationModule, createObservationModule
from rsFaultHandling.Corrupters import Corrupter, MultiCorrupter
from rsLibrary.Estimation import EstimationProblem, EstimationAlg, RandomChangeEstimation
import rsLibrary.Jacobian as Jacobian
from rsLibrary.Monitor import gMonitorT, createMonitorT

from State import StateDef, States
import Motors
import Sensors
import Robots


def dict2str(d:dict) -> str:
    l_f = [str(k)+': %.2f'%(d[k]) for k in d]
    return ', '.join(l_f) 

#### #### #### ####  ROBOT STATE ESTIMATION #### #### #### #### 
     

class RobotModel (Robots.Robot, EstimationProblem):
            
    def __init__(self, name, robot):
        super().__init__(name, [], [])
        self.robot = robot
        self.stateDef = robot.stateDef  # copy necessary?
        self.states = States(self.stateDef, robot.states) # we copy initial state
        self.states.varPrefix = 'e'
        self.calculatedCovariances = False
        
        for m in robot.motors:
            m_copy = copy.deepcopy(m)
            m_copy.varPrefix = 'e'
            self.motorsAndSensors.append(m_copy)
            self.motors.append(m_copy)
            
        for s in robot.sensors:
            s_copy = copy.deepcopy(s)
            s_copy.varPrefix = 'e'
            self.motorsAndSensors.append(s_copy)
            self.sensors.append(s_copy)
        
        for motorSensor in self.motorsAndSensors:
            motorSensor.setObservation(0, 0) # for initial frame - MIGHT BE DIFFERENT IN THE FUTURE!!
            
    def randomizeParameters(self, seed = 3):
        random.seed(seed)
        for motorSensor in self.motorsAndSensors:
            if not motorSensor.isCalibrated:
                for k, v in motorSensor.params.items():
                    change = random.uniform(-v, v)
                    motorSensor.params[k] += change
        
    def addSensorReading(self, sensorValues, t):
        self.sensorValues = sensorValues
        for motorSensor in self.motorsAndSensors:
            motorSensor.setObservation( sensorValues[motorSensor.name], t )
        
    '''
      return a dict with all variables to be estimated and for each a tuple with old and new value
    '''
    def estimate(self, dt = 1, nbrTries = 10000, printLevel = 1, UPDATE_FACTOR = 0.5, maxNbrDecreases = 10, newFrame=True):
       if newFrame:
           self.states.addFrameWithCopy(dt = dt) # TODO: could provide better estimate for state variables of new frame
#       self.states.setValue('a', self.states.now, 1)
       self.states.calcDepVars(self.states.now)
       if (printLevel > 2):
           print('Initialization of ', end ='')
           self.states.printFrame(self.states.now)     
        
      # est = RandomChangeEstimation(self, nbrTries, printLevel)
       est = Jacobian.EstimationWithJacobian(self, nbrTries, UPDATE_FACTOR = UPDATE_FACTOR, maxNbrDecreases = maxNbrDecreases, printLevel = printLevel)
       
       variables_before_update = self.updateableVariables().copy()  
       
       est.solve() # method defined in Estimation.IterativeEstimationAlg
       
       errors, totalError_end = self.errors()

       variables_after_update =  self.updateableVariables()
       
       changes = {} # dict from variable to tuple with old and new value
       for k in variables_before_update:
           changes[k] = (variables_before_update[k], variables_after_update[k])
       return changes
   
    def estimateWithValidRegions(self, nbrTries = 10000, printLevel = 1, UPDATE_FACTOR = 0.5, maxNbrDecreases = 10, newFrame=True):
       if newFrame:
           self.states.addFrameWithCopy() # TODO: could provide better estimate for state variables of new frame
       self.states.calcDepVars(self.states.now)
       if (printLevel > 2):
           print('Initialization of ', end ='')
           self.states.printFrame(self.states.now)     
        
      # est = RandomChangeEstimation(self, nbrTries, printLevel)
       est = Jacobian.EstimationWithJacobian(self, nbrTries, UPDATE_FACTOR = UPDATE_FACTOR, maxNbrDecreases = maxNbrDecreases, printLevel = printLevel)
       
       variables_before_update = self.updateableVariables().copy()  
       variables_after_update =  self.updateableVariables()
       states_before_update = self.states.deepcopy()
       print('\nBEFORE')       
       print('\tvariables:', variables_before_update)
       print('\tstates', states_before_update)
       
       # self.states.setValue('o', -1, 50)
       # print(id(self.states), id(states_before_update))
      
       best_states = self.states.deepcopy()
       
       totalError_end = []
       
       # to save the errors of the best solution
       errors = self.errors()[0]
       lowest_error = self.errors()[1]
       
       if printLevel%0.7==0:
           print("Nr of possible regions: ", len(self.robot.constraintGen.getConstraints().valid_regions))
           for region in self.robot.constraintGen.getConstraints().valid_regions:
               print(region.intervals[0])
       for region in self.robot.constraintGen.getConstraints().valid_regions:
           # orientation is set to the middle of the monotone region
           i = region.intervals[0]
           start_est_at = (i.upper_bound+i.lower_bound)/2
           if printLevel%0.7==0:
               print('Checking for region: ', i)
               print('\tStarting estimation at: ', i.var, '=', start_est_at)
           self.states.setValue(i.var, self.states.now, start_est_at)
           self.states.setValue('vo', self.states.now, start_est_at-self.states['o'][self.states.now-1])
           
           print("before solving", variables_before_update)
           print(self.states)
           
           errors = self.errors()[0]
           
           if printLevel%0.7==0:
               print("errors:\n")
               i=0
               for motorSensor in self.motorsAndSensors:
                   if motorSensor.enable:
                       for t in range(self.states.startUnknown, self.states.now+1):
                           print(motorSensor.name, t, errors[i], end = '\t')
                           i+=1
                           if i%3==0:
                               print('\n')
           
           est.solve() # method defined in Estimation.IterativeEstimationAlg
           
           if printLevel%0.7==0:
               print("states[o]: ", self.states['o'])
           err = self.errors()
           totalError_end.append(err[1])
           
           #save only best solution
           if err[1]<lowest_error:
               errors = err[0]
               lowest_error = err[1]
               variables_after_update =  self.updateableVariables()
               best_states = self.states.deepcopy()
               if printLevel%0.7==0:
                   print("New better values with error=", totalError_end[-1])
           else:
               self.update(variables_before_update)
               if printLevel%0.7==0:
                   print("Not better with error=", totalError_end[-1])
           self.states = states_before_update.deepcopy()
           
           
       if printLevel%0.7==0:
           print(variables_after_update)
          
       gMonitorT().setValue("erPRE", self.errors()[1])
       self.states = best_states
       gMonitorT().setValue("erPOST", self.errors()[1])
       if printLevel%0.7==0:
           print("errors:\n")
           i=0
           for motorSensor in self.motorsAndSensors:
               if motorSensor.enable:
                   for t in range(self.states.startUnknown, self.states.now+1):
                       print(motorSensor.name, t, errors[i], end = '\t')
                       i+=1
                       if i%3==0:
                           print('\n')
       self.update(variables_after_update)
       changes = {} # dict from variable to tuple with old and new value
       for k in variables_before_update:
           changes[k] = (variables_before_update[k], variables_after_update[k])
       return changes
      
    def constructJacobian(self, printLevel = 1):
        est = Jacobian.EstimationWithJacobian(self, printLevel = printLevel)
        updateableVariables = self.updateableVariables()
        updateableVariables_list = list(updateableVariables) # dict to list of keys
        errors, totalError = self.errors()
        
        # transform to arrays
        import numpy as np
        error_array = np.array( [v for v in errors] )
        val_array = np.array( [v for k, v in updateableVariables.items()] , dtype = float)

        return est.constructJacobian(updateableVariables, updateableVariables_list, val_array, error_array)
      
    #  +++++ IMPLEMENTATION OF METHODS OF EstimationProblem +++++
    def updateableVariableNames(self) -> list:
        updateableVariables = self.stateDef.indepVariables.copy()
        
        for motorSensor in self.motorsAndSensors:
            if not motorSensor.isCalibrated:
                for mosens_param in motorSensor.params:
                    updateableVariables.append(mosens_param)
            
        return updateableVariables
    
    def updateableVariables(self) -> dict:
        # collect updateableVariables
        # loop over all states& sensors/motios : TODO  loop over frames 
        
        updateableVariables = { (v, t) : self.states[v][t] for v in  self.stateDef.indepVariables for t in range(self.states.startUnknown, self.states.now+1)}
      #  updateableVariables.extend(self.stateDef.indepVariables)
        
        updateableParameters = { (m.name, k) : v for m in self.motorsAndSensors if not m.isCalibrated  for k, v in m.params.items() }
 #       for motorSensor in self.motorsAndSensors:
  #          for mosens_param in motorSensor.params:
   #             updateableVariables[mosens_param] = motorSensor.paramValues[ motorSensor.params.index(mosens_param) ]
            
        updateableVariables.update(updateableParameters) # adds parameters-dict to the general dict
        return updateableVariables

    def printEvidence(self): # the evidence
        print('Evidence: observations = observation equations => errors')
        for motorSensor in self.motorsAndSensors:
            for t in range(self.states.startUnknown, self.states.now+1):
                #motorSensor.printObserEq(t) # gives error x with values
                if motorSensor.enable and (not t in motorSensor.excludeObservations) and t < motorSensor.disableFrom: # list of timestamps to be ignored
                    motorSensor.printError(self.states, t) # gives error x with values
        
    def errors(self) -> tuple:
        # collect errors
        #   loop over all sensors & frames
        errors = []
        totalError = 0
        for motorSensor in self.motorsAndSensors:
            if motorSensor.enable:
                for t in range(self.states.startUnknown, self.states.now+1):
                    if (not t in motorSensor.excludeObservations) and t < motorSensor.disableFrom: # list of timestamps to be ignored
                        error =  motorSensor.error(self.states, t)
                        errors.append(error)
                        totalError += abs( error )
        return errors, totalError
    
    def calculateCovariances(self, printit:bool = False):
        for motorSensor in self.motorsAndSensors:
            motorSensor.analyzeResiduals()
            if printit:
                print('Sensor '+motorSensor.name+": mean = {:.2f}, stddev = {:.2f}, cov = {:.2f}.".format(motorSensor.residualsMean, motorSensor.residualsStddev, motorSensor.residualsCov) )
        self.calculatedCovariances = True
    def covarianceMatrixDiagonal(self) -> list:
        if self.calculatedCovariances:
            covariances = []
            for motorSensor in self.motorsAndSensors:
                if motorSensor.enable:
                    for t in range(self.states.startUnknown, self.states.now+1):
                        if (not t in motorSensor.excludeObservations) and t < motorSensor.disableFrom: # list of timestamps to be ignored
                            covariances.append(motorSensor.residualsCov)        
            return covariances
        else:
            return None

    def update(self, updates) -> None:
        self.updates = updates
        for upd_var, upd_value in updates.items():
            if upd_var[0] in self.stateDef.variables:
                self.states.setValue(upd_var[0], upd_var[1], upd_value)
            else:
                for motorSensor in self.motorsAndSensors:
                    if upd_var[0] == motorSensor.name:
                        motorSensor.setParam(upd_var[1], upd_value)
        for t in range(self.states.startUnknown, self.states.now + 1):
            self.stateDef.forwardMath(self.states, t, self.states.dts)
       
            # CHECK THE STATE TODO: FOR ALL STATES UNTIL startUnknown
            for variable in self.stateDef.variables:
                range_of_variable = self.rangeOfVariable(variable)
                if range_of_variable != None: 
                    v = self.states.value(variable, t)
                    if v < range_of_variable[0] or v > range_of_variable[1]:
                        raise ValueError("Variable "+variable +" out of range with value "+str(v))
        self.errors() #recalcuate errors at the end of each update

            
    #  +++++ +++++ +++++ ++++ +++++ +++++
    
    def diffWithRef(self, robotOfReference) -> float:
        diff = 0
        for v in  self.stateDef.variables:
            for t in range(self.states.startUnknown, self.states.now+1):
                diff += abs(self.states[v][t] - robotOfReference.states[v][t])
    
        for m in self.motorsAndSensors:
            if not m.isCalibrated:
                m_ref = robotOfReference.getMotorOrSensor(m.name)
                for param, value in m.params.items():
                    diff += abs(value - m_ref.params[param])
        gMonitorT().setValue("diff", diff)
       
        for t in range(self.states.startUnknown, self.states.now+1):
            dstate = 0
            for v in  self.stateDef.variables:
                dstate += abs(self.states[v][t] - robotOfReference.states[v][t])
            gMonitorT().setValue("dstate", dstate, t)
        
        return diff               
    
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code RobotEstimation.py ***')
    print('Parameters of the estimation algorithm: ')
    print(' * nbrTries: maximal number of iterations')
    print(' * printLevel: increase to see more details')
    print('Variables of the estimation algorithm: ')
    print(' * erPRE: error of observation functions before the estimation')
    print(' * it: number of iterations of the estimation algorithm')
    print(' * end: reason of algorithm termination. 0: below threshold; 1: maximum iterations reached; 2: no improvement of score')
    print(' * erPOST: error of observation functions after the estimation')
    print(' * diff: difference of estimated values and true values (over whole time range of the estimation)')
    print(' * dstate: sum of absolute differences of estimated state values and true values (only current frame)')
    
    
    #### #### #### ####  1D-robot with a, v, x  #### #### #### #### 
    if True: 
        # create RobotSim
        simRobot = Robots.createSimRobot1D(walls = (-10, 25), noiseLevel = 4)
        simRobot.printConfiguration()

        robotModel = RobotModel("robotModel",simRobot)
        robotModel.getMotorOrSensor('tch').isCalibrated = True
       # robotModel.getMotorOrSensor('mX').isCalibrated = True
        robotModel.randomizeParameters(seed = 23) # only those not calibrated
        robotModel.printConfiguration()
        
        print('Variables to be estimated: ', end='')
        print(str(robotModel.updateableVariableNames()))
        print('Dependent variables     : ', end='')
        print(str(robotModel.stateDef.depVariables))
        print(' == == == == == == == == == == == == == == == == == == == ==')

        
        # global monitor
        createMonitorT('dt', 'T', 'mX', simRobot.stateDef.variables, [s.name for s in simRobot.sensors], 'ea', "ev", "ex", 'elp', 'elm', 'ela', 'elv', "eld", "erPRE", "it", 'end', "erPOST", 'rmX', 'rsA','rsV','rsX','diff', 'dstate').printTitle()
        gMonitorT().setColumnWidth(8)
        
        motor = simRobot.motors[0].name
        #motor_input = [10, 15, 5, 10, -20, -20, -20, -20, -20, -20]
        motor_input = [10, 20, 8, -20, -17, -10, 10, 10] + [0] * 20
        
        # running   
        for i in range(0, min(33, len(motor_input))):
            t = i + 1
            print(' ============================= FRAME '+str(t)+' ============================')
            
            key_value = {motor : motor_input[i]}
            

            gMonitorT().setValues( key_value , t)
            
          #  print(key_value, end='')
            simRobot.setMotors(key_value)
            sensorValues = simRobot.readSensors()
            sensorValues[motor] = key_value[motor]  # also motor inputs!
            dt = simRobot.states.dts[-1] # might be different than 1
          
            gMonitorT().setValues( sensorValues, t )
          #  print(' =>', sensorValues, end='')
        
            # link RobotSim to RobotModel + estimate
            robotModel.addSensorReading(sensorValues, t)
            changes = robotModel.estimate(dt = dt, nbrTries = 20 if t < 36 else 2, maxNbrDecreases = 10,  printLevel = 1 if t < 33 else 3 )
            #for k, v in changes.items():
               # print('Unknown ', k, ' changed from  %.3f'%(v[0]) , 'to %.3f'%(v[1]))
            
            robotModel.diffWithRef(simRobot)

            gMonitorT().printAllFrames()
            
            if simRobot.environment.touchedWall:
                # we assume that estimation is OK now
                robotModel.states.startUnknown = robotModel.states.now + 1
                for motsens in robotModel.motorsAndSensors:
                    motsens.isCalibrated = True

    
    
    #### #### #### ####  1D-robot with only v and x  #### #### #### ####             
    if False: # 1D only position robot - noisy distance sensor + touch sensor
        # create RobotSim
        simRobot = Robots.createSimRobot1DOnlyVelocity( walls = (-100, 80) )
        
        # create RobotModel (prefix 'e' is used for variabeles to indicate estimated)
        robotModel = RobotModel("robotModel",simRobot)
        
        robotModel.getMotorOrSensor('tch').isCalibrated = True
        robotModel.randomizeParameters(seed = 23)
        
        print('Variables to be estimated: ', end='')
        print(str(robotModel.updateableVariableNames()))
        print('Dependent variables     : ', end='')
        print(str(robotModel.stateDef.depVariables))
        print(' == == == == == == == == == == == == == == == == == == == ==')

        # global monitor
        createMonitorT("moV", simRobot.stateDef.variables, "sX", "tch", "ev", "ex", 'elm', "elx", "erPRE", "it", "end", "erPOST", "diff", 'dstate').printTitle()
        
        motor = simRobot.motors[0].name
        motor_input = [200, 400, 200, -100, -100, -200, 200]  
        
        # running   
        for i in range(0, min(20, len(motor_input))):
            t = i + 1
            print(' ============================= FRAME '+str(t)+' ============================')
            key_value = {motor : motor_input[i]}
            gMonitorT().setValues( key_value, t )
            
            print('INPUT ', key_value, end='')
            simRobot.setMotors(key_value)
            
            sensorValues = simRobot.readSensors()
            sensorValues[motor] = motor_input[i]  # also motor inputs!
            
            # apply corrupter/observer here
            
            gMonitorT().setValues( sensorValues, t )
            print(' => OBSERVED ', sensorValues, end='')
        
            # link RobotSim to RobotModel + estimate
            robotModel.addSensorReading(sensorValues, t)
            
            changes = robotModel.estimate(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 1 )
            for k, v in changes.items():
                print('Unknown ', k, ' changed from  %.3f'%(v[0]) , 'to %.3f'%(v[1]))
            
            robotModel.diffWithRef(simRobot)
            
            estimated_variables = robotModel.updateableVariables()
            print(' => ESTIMATED ', estimated_variables, end='')
            
            print('')
            gMonitorT().printAllFrames()
            
    #### #### #### ####  TESTING MAX LIKELIHOOD - SAME 1D-robot with a, v, x AS FIRST EXAMPLE #### #### #### #### 
    if False: # 1D robot
     
        simRobot = Robots.createSimRobot1D(walls = (-100, 50), noiseLevel = 1)
        from scipy.stats import norm
        simRobot.getMotorOrSensor('sX').noiseDistr = norm(scale = 50) # THIS SENSOR GETS A FARGE LARGER NOISE LEVEL -> LESS PRECISE
        simRobot.printConfiguration()

        robotModel = RobotModel("robotModel",simRobot)
        for ms in robotModel.motorsAndSensors:
            ms.isCalibrated = True

        
        robotModel.printConfiguration()
        
        print('Variables to be estimated: ', end='')
        print(str(robotModel.updateableVariableNames()))
        print('Dependent variables     : ', end='')
        print(str(robotModel.stateDef.depVariables))
        print(' == == == == == == == == == == == == == == == == == == == ==')

        
        # global monitor
        createMonitorT('dt', 'T', "mX", simRobot.stateDef.variables, [s.name for s in simRobot.sensors], 'ea', "ev", "ex", "erPRE", "it", 'end', "erPOST", 'rmX', 'rsA','rsV','rsX','diff', 'dstate').printTitle()
        gMonitorT().setColumnWidth(9)
        
        motor = simRobot.motors[0].name
        
        from Control import Control1DForwardUntilWall, Control1DForwardUntilWallRandom
        ctrl = Control1DForwardUntilWallRandom(simRobot.environment, nbrSteps = 20, motorInputMin = 5, motorInputMax = 15)
        
        t=0
        while ctrl.hasNext() and t < 21:
            t += 1
            print(' ============================= FRAME '+str(t)+' ============================')
            

            cmd = ctrl.nextCommand()
            key_value = {motor : cmd}
            
            gMonitorT().setValues( key_value , t)
            
            simRobot.setMotors(key_value)
            sensorValues = simRobot.readSensors()
            sensorValues[motor] = key_value[motor]  # also motor inputs!
            dt = simRobot.states.dts[-1] # might be different than 1
          
            gMonitorT().setValues( sensorValues, t )
        
            # link RobotSim to RobotModel + estimate
            robotModel.addSensorReading(sensorValues, t)
            changes = robotModel.estimate(dt = dt, nbrTries = 20 , maxNbrDecreases = 10,  printLevel = 0  if t != 100 else 3 )         
            
            robotModel.diffWithRef(simRobot)
                        
            print('')
            gMonitorT().printAllFrames()

            if t == 11:
                jac = robotModel.constructJacobian()
                print('Jacobian= ')
                print(jac)                     
                print(' Jacobian rows = observations: ')
                robotModel.printEvidence()
                print(' Errors = ', robotModel.errors() )
                print(' Jacobian columns = unknowns:', robotModel.updateableVariables())
                print('')
                print('Covariance diagonal ', robotModel.covarianceMatrixDiagonal())                

            if t > 3:
                robotModel.states.startUnknown = robotModel.states.now
            
            if t == 10:
                # 
                print(' *** We assume that estimation is OK now => apply Max Likelihood with the following variances (stddev): *** ')
                robotModel.calculateCovariances(True)
                
    
        print(' *** measured noise of simulator *** ')
        simRobot.calculateCovariances(True)
        print(' *** estimated noise of model ***')
        robotModel.calculateCovariances(True)

### #### #### #### ### #### #### #### #### #### #### #### ###  
