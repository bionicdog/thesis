# -*- coding: utf-8 -*-
"""
Created on April 2022

@author: Jan April

2D estimation requires specific solutions because:
    - position should remain within bounds
    - walls: discontinous derivative
"""
import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from rsLibrary.Monitor import gMonitorT, createMonitorT
from Environment2D import Environment2D, Environment2DSquare
from Constraints import Simple2DConstraintGenerator, Constraints
from rsLibrary.Estimation import EstimationProblem
import rsLibrary.Jacobian as Jacobian
from copy import copy, deepcopy

class Test2Destimation (EstimationProblem):
    def __init__(self, x:float, y:float, orientation:float, distance:float, env2D:Environment2D):    
        self.x = float(x)
        self.y = float(y)
        self.o = float(orientation)
        self.distance = float(distance) # distance sensor output
        self.env2D = env2D
        
    def states(self):
        return [self.x, self.y, self.o]
    def updateableVariableNames(self) -> list:
        return ['x', 'o']        
    def updateableVariables(self) -> dict:
        return {'x': self.x, 'o': self.o}
    def errors(self) -> tuple:
        dist2wall = self.env2D.distance2wall(self.x, self.y, self.o)
        err = dist2wall - self.distance
        return [err], abs(err)
    def update(self, updates:dict):
        self.x = updates['x']
        self.o = updates['o']
    def rangeOfVariable(self, variable:str) -> tuple: # optional
        MIN_DIST_TO_WALL = 0.1
        if variable == 'o':
            return [-1.0, 1.0]
        elif variable == 'x':
            return [-100.0 + MIN_DIST_TO_WALL, 100.0 - MIN_DIST_TO_WALL]
        else:
            return [-10.0, 10.0]
        
    def dist2Wall(self):
        return self.env2D.distance2wall(self.x, self.y, self.o)
    def print(self):
        print('(x={:.1f}, y={:.1f}, o={:.2f}) gives distance {:.1f} - should be {:.2f}'.format(self.x, self.y, self.o, self.dist2Wall(), self.distance))
    
    
   
class Test2DestimationWithoutOrientation (Test2Destimation):
    def __init__(self, x:float, y:float, orientation:float, distance:float, env2D:Environment2D):    
        super().__init__(x, y, orientation, distance, env2D)
        
    def updateableVariableNames(self) -> list:
        return ['x']        
    def updateableVariables(self) -> dict:
        return {'x': self.x}
    def update(self, updates:dict):
        self.x = updates['x']

class Test2DestimationMultipleRegions(Test2Destimation):
    def __init__(self, x:float, y:float, orientation:float, distance:float, env2D:Environment2D, maxDeltas):
        super().__init__(x, y, orientation, distance, env2D)
        self.constraintGen = Simple2DConstraintGenerator([x], [y], [o], env2D, maxDeltas)
    
    #convergence restricted by range of 'o'
    #put error related to maxDeltas in Jacobian?
    def rangeOfVariable(self, variable:str):
        MIN_DIST_TO_WALL = 0.1
        if variable == 'o':
            r = [-1.0, 1.0]
            if self.regionID() is not None:
                _constr = self.constraintGen.getConstraints()
                _i = _constr.valid_regions[self.regionID()].getVarInterval(variable)
                r = [_i.lower_bound, _i.upper_bound]
            return r
        elif variable == 'x':
            return [-100.0 + MIN_DIST_TO_WALL, 100.0 - MIN_DIST_TO_WALL]
        else:
            return [-10.0, 10.0]
    def estimate(self, nbrTries = 10000, printLevel = 1, UPDATE_FACTOR = 0.01, maxNbrDecreases = 10, newFrame=True):   
        """
        Check for a solution in every possible region
        """
        variables_before_update = self.updateableVariables().copy()  
        variables_after_update =  self.updateableVariables()
        states_before_update = self.states()
        print('\nBEFORE')       
        print('\tvariables:', variables_before_update)
        print('\tstates:', states_before_update)
        print('\tmaxDeltas:', self.constraintGen.maxDeltas)
       
        best_states = deepcopy(self.states())
        
        totalError_end = []
        
        # to save the errors of the best solution
        lowest_error = self.errors()[1]
       
        # get valid regions
        regions = self.constraintGen.getPossibleRegions()
        
        print("Nr of possible regions: ", len(regions))
        for region in regions:
            print(region.intervals[0])
            
        for region in regions:
            est = Jacobian.EstimationWithJacobian(self, nbrTries, UPDATE_FACTOR = UPDATE_FACTOR, maxNbrDecreases = maxNbrDecreases, printLevel = printLevel)
            # orientation is set to the middle of the monotone region
            i = region.intervals[0]
            start_est_at = (i.upper_bound+i.lower_bound)/2
            if printLevel>1:
                print('Checking for region: ', i)
                print('\tStarting estimation at: ', i.var, '=', start_est_at)
            
            self.o =  start_est_at
            
            if printLevel>1:
                print("before solving", variables_before_update)
                print('/tstates:', self.states())
            
            est.solve() # method defined in Estimation.IterativeEstimationAlg
            
            if printLevel>1:
                print("states[o]: ", self.o)
            err = self.errors()
            totalError_end.append(err[1])
            
            #save only best solution
            if err[1]<lowest_error:
                lowest_error = err[1]
                variables_after_update =  self.updateableVariables()
                best_states = self.states()
                if printLevel>1:
                    print("New better values with error=", totalError_end[-1])
            else:
                self.update(variables_before_update)
                if printLevel>1:
                    print("Not better with error=", totalError_end[-1])
            self.x, self.y, self.o = states_before_update
            
            
        if printLevel>1:
            print(variables_after_update)
           
        gMonitorT().setValue("erPRE", self.errors()[1])
        self.states = best_states
        gMonitorT().setValue("erPOST", self.errors()[1])
        self.update(variables_after_update)
        changes = {} # dict from variable to tuple with old and new value
        for k in variables_before_update:
            changes[k] = (variables_before_update[k], variables_after_update[k])
        return changes
    def regionID(self):
        #gives slightly different results than global regionID attribute for each region
        return self.constraintGen.getConstraints().getValidRegionID(self.updateableVariables())
        
###  
if __name__== "__main__":
    if True: # look to the right
        wallDist = 100
        x = 60
        y = 0
        o = 0.2
        dist = 22 #dist = 25, x=60, o=0.2 doesn't work
    else: # look to the left
        wallDist = -100
        x = -99.5
        y = 0
        o = -3.14
        dist = 5

    #print('Wall is at '+str(wallDist)+' I see dist of '+str(dist))
    env2D = Environment2DSquare(wallDist = wallDist, accvars = None)
    #test2Dest = Test2Destimation(x, y, o, dist, env2D)
    print('TESTING WITH ORIENTATION')
    if True:
        test2Dest = Test2Destimation(x, y, o, dist, env2D)
        
        
        test2Dest.print()
        print('error = ',str(test2Dest.errors()))
        
        # test2Dest.estimateWithValidRegions(nbrTries = 20, maxNbrDecreases = 10,  printLevel = 0.7)
        
        est = Jacobian.EstimationWithJacobian(test2Dest, printLevel = 5, maxNbrDecreases=30)
        
        #   variables_before_update = self.updateableVariables().copy()  
           
        est.solve() # method defined in Estimation.IterativeEstimationAlg
    
        print("RESULT: ", end='')
        test2Dest.print()
        
    else:
        # test2Dest = Test2Destimation(x, y, o, dist, env2D, True)
        # test2Dest.print()
        # print('error = ',str(test2Dest.errors()))
        
        # test2Dest.estimateWithValidRegions(nbrTries = 10000, maxNbrDecreases = 30,  printLevel = 2)
        
        test2Dest = Test2DestimationMultipleRegions(x, y, o, dist, env2D, maxDeltas={'o':0.5})
        test2Dest.print()
        
        test2Dest.estimate(nbrTries = 10000, maxNbrDecreases = 30,  printLevel = 0)
        
        print("RESULT: ", end='')
        test2Dest.print()
