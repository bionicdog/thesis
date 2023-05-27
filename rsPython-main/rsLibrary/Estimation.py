#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Estimation.py
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Monitor import gMonitorT, createMonitorT

#### #### #### ####  ESTIMATION PROBLEM Interface #### #### #### #### 
class EstimationProblem:
    """
    Interface class all functions need to be overwritten by child classes

    """
    def updateableVariables(self) -> dict:
        raise NotImplementedError
        return {}
    def errors(self) -> tuple:
        raise NotImplementedError
        return [], 0
    
    '''
    updates: dictionary from updateablevariable to new value
    '''
    def update(self, updates:dict):
        raise NotImplementedError

    # *** OPTIONAL METHODS ***
    def rangeOfVariable(self, variable:str) -> tuple: # optional
        return None  # default, results in a Jacobian test change of 1
        
    def printEvidence(self): # optional
        return 0 

    def calculateCovariances(self):
        pass
    
    def covarianceMatrixDiagonal(self) -> list:
        return None

def l2str(l) -> str:
    if type(l) == list:
        l_f = ['%.3g'%k for k in l]
        return ', '.join(l_f) 
    else:
        '%.3g'%l
def dict2str(d:dict) -> str:
    l_f = [str(k)+': %.2f'%(d[k]) for k in d]
    return ', '.join(l_f) 

#### #### #### ####  ESTIMATION ALGORITHM Interface #### #### #### #### 
class EstimationAlg:
    def __init__(self, estimationProblem:EstimationProblem):
         self.estimationProblem = estimationProblem
 
    #### ==============  Interface method to be implemented  ============== ####   
    def solve(self):
        raise NotImplementedError

class IllegalStateError(Exception):
    ''' Raised when estimation reaches an impossible state '''
    pass

#### #### #### ####  ITERATIVE ESTIMATION ALGORITHM #### #### #### #### 
class IterativeEstimationAlg (EstimationAlg):
    def __init__(self, estimationProblem:EstimationProblem, maxNbrIterations:int = 10000, errorThreshold:int = 0.1, printLevel:int = 1):
         super().__init__(estimationProblem)
         self.maxNbrIterations = maxNbrIterations
         self.errorThreshold = errorThreshold
         self.printLevel = printLevel
         
    #### ==============  Interface method to be implemented  ============== ####   
    def changeEstimate(self, updateableVariables:dict) -> bool:
        raise NotImplementedError
        
    ''' OPTIONAL: subclass can make the iteration stop '''
    def increaseOfError(self):
        return False # return whether estimation should continue
    
    ''' OPTIONAL: override to add parameters '''
    def printParameters(self):
        print('Estimation parameters: maxNbrIterations = %d, errorTrheshold = %.2f, printLevel = %d '%(self.maxNbrIterations, self.errorThreshold, self.printLevel))

    #### ==============  Implemented Interface method  ============== ####   
    def solve(self):
        updateableVariables = self.estimationProblem.updateableVariables()
        errors, totalError = self.estimationProblem.errors()
        gMonitorT().setValue("erPRE", totalError)
        if self.printLevel > 0:
            self.printParameters()
            print('To be estimated: '+dict2str(updateableVariables)+ ' gives total error %.5f'%(totalError)+' (residuals = '+l2str(errors)+')')
            
            self.estimationProblem.printEvidence()
            print('Gives total error of %.2f (=erPRE)'%(totalError))
        endStr = None
        endShortStr = None
        it = 0
        continue_estimation = True
        while abs(totalError) > self.errorThreshold and it < self.maxNbrIterations: # and not self.stopEstimation()
            it += 1
            if self.printLevel > 1:
                print('\n --- Estimation iteration {} ---'.format(it))
                
         #   if it == 4:
         #       print('DEBUG FROM HERE')
         
            new_values = updateableVariables.copy()
            
            continue_estimation = self.changeEstimate(new_values) # HOOK METHOD (see above)
            if not continue_estimation:
                endShortStr = 2
                endStr = 'no improvement of score'
                break
            
            self.estimationProblem.update(new_values)   
            
            errors, new_totalError = self.estimationProblem.errors()
            if abs(new_totalError) < abs(totalError):
                totalError = new_totalError
                updateableVariables = new_values
                if self.printLevel > 1:
                    #print('Found better values: '+dict2str(new_values)+" gives error %.5f"%(new_totalError)+' (errors = '+l2str(errors)+')')
                    print('Found better values: gives error %.5f'%(new_totalError)+' (residuals = '+l2str(errors)+')')
                    if len(gMonitorT().variables) > 1:
                        gMonitorT().printAllFrames()
                    else:
                        print('  Values: '+dict2str(updateableVariables))
               
            else:
                if self.printLevel > 2: 
                    print('NOT BETTER with '+dict2str(new_values)+' gives total error %.5f'%(new_totalError)+' (residuals = '+l2str(errors)+'), I do a reset to '+dict2str(updateableVariables))
                    if len(gMonitorT().variables) > 1:
                        gMonitorT().printAllFrames()
                        self.estimationProblem.printEvidence()
                self.estimationProblem.update(updateableVariables) # reset
                   
                if not self.increaseOfError(): # hook method
                    endShortStr = 2
                    endStr = 'no improvement of score'
                    if self.printLevel > 2: 
                        print('NOT BETTER, WE STOP WITH THIS ALGO')
                    break
                
              #  errors, totalError_check = self.estimationProblem.errors()
              #  print(' Errors should be the same again: totalError = '+str(totalError_check)+' (errors = '+str(errors)+')')    
              
        if endStr is None:
            if abs(totalError) <= self.errorThreshold:
                endShortStr = 0
                endStr = 'below error threshold ('+str(self.errorThreshold)+')'
            else:
                endShortStr = 1
                endStr = 'maximum iterations reached ('+str(self.maxNbrIterations)+')'
        #print("AFTER EST: updateableVariables =  "+str(updateableVariables)   )     
        varValues = self.estimationProblem.updateableVariables()
        errors, totalError_end = self.estimationProblem.errors()
        if self.printLevel > 0:
            print("AFTER EST: "+dict2str(varValues) + ' gives total error %.5f'%(totalError_end)+' (residuals = '+l2str(errors)+') #iterations = '+str(it)+' ('+endStr+')')    
            self.estimationProblem.printEvidence()
            print('Gives total error of %.2f (=erPOST)'%(totalError_end))
        gMonitorT().setValue("erPOST", totalError_end)
        gMonitorT().setValue("it", it)
        gMonitorT().setValue("end", endShortStr)




#### #### #### ####  Example Estimation Problem #### #### #### #### 
class ZeroOfFunction(EstimationProblem):
    """
    Find x that nullifies the given one-parameter function

    """
    def __init__(self, function, initialGuess = 0):
        self.function = function
        self.x = initialGuess
    def updateableVariableNames(self):
        return ['x']
    
    def updateableVariables(self):
        return {'x': self.x}
    
    def errors(self):
        f = self.function(self.x)
        return [f], abs(f)
    
    def update(self, updates):
        self.x = updates['x']

class ZeroOfTwoParametersFunctions(EstimationProblem):
    """
    Find x and y that nullify the given two-parameter functions

    """
    def __init__(self, functions, initialGuess = [0, 0]):
        self.functions = functions
        self.x = initialGuess[0]
        self.y = initialGuess[1]
    def updateableVariableNames(self):
        return ['x', 'y']
    
    def updateableVariables(self):
        return {'x': self.x, 'y': self.y}
    
    def errors(self):
        errors = []
        total_error = 0
        for f in self.functions:
            err = f(self.x, self.y)
            errors.append(err)
            total_error += abs(err)
        return errors, total_error
    
    def update(self, updates):
        self.x = updates['x']
        self.y = updates['y']

class ZeroOfNParameterFunctions(EstimationProblem):
    """
    Find values that nullify the given functions
    functions: take n parameters
    variableNames: list/tuple of n names
    initialGuess: list/tuple of n initial values - should be of equal length as variableNames

    """
    def __init__(self, functions, variableNames, initialGuess):
        self.functions = functions
        self.variableNames = variableNames
        self.values = tuple(initialGuess)
        
    def updateableVariableNames(self):
        return self.variableNames
    
    def updateableVariables(self):
        return { v : x for v, x in zip(self.variableNames, self.values)}
    
    def errors(self):
        errors = []
        total_error = 0
        for f in self.functions:
            err = f(*self.values)  # asterisk is for unpacking the list
            errors.append(err)
            total_error += abs(err)
        return errors, total_error
    
    def update(self, updates):
        self.values = tuple( [updates[k] for k in updates] )
        
#### #### #### ####  Estimation algorithm Implementations  #### #### #### #### 

import random
class RandomChangeEstimation(IterativeEstimationAlg):
    def __init__(self, _estimationProblem, _maxNbrIterations = 10000, _errorThreshold = 1, _maxChange = 5, _useInt = False, printLevel = 1):
        super().__init__(_estimationProblem, printLevel)
        self.maxChange = _maxChange
        self.useInt = _useInt
        
    def changeEstimate(self, updateableVariables):
        for v in updateableVariables:
            if self.useInt:
                updateableVariables[v] += random.randint(-self.maxChange, self.maxChange)
            else:
                updateableVariables[v] += random.uniform(-self.maxChange, self.maxChange)  # floating point

               
        

# MY RANDOM EST ALG D:\references\robotics\camera and image processing\python code old        


    
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Estimation.py ***')
    
    if False:
        def function(x):
            return 20 - 8*x - 2 * x * x
        initial_guess = 0
        zeroOfFunction = ZeroOfFunction(function, initial_guess)
        print('Function('+str(initial_guess)+') gives error '+str(zeroOfFunction.errors()[1]))
        
        est = RandomChangeEstimation(zeroOfFunction, _maxNbrIterations=100000, _maxChange = 0.5)
        est.solve()
        print('Function('+str(zeroOfFunction.x)+') gives error '+str(zeroOfFunction.errors()[1]))
 
    if True:
        def function1(x, y):
            return -5+ 20 * y + 8*x*y - 2 * x * x
        def function2(x, y):
            return 5 - 10 * y - 3 *x*y + 2 * y * y
        initial_guess = [0, 0]
        functions = [function1, function2]
        zeroOfFunctions = ZeroOfTwoParametersFunctions(functions, initial_guess)
        print('Functions('+str(initial_guess)+') gives errors '+str(zeroOfFunctions.errors()[0]))
        
        est = RandomChangeEstimation(zeroOfFunctions, _maxNbrIterations=100000)
        est.solve()
        print('Functions with ('+str(zeroOfFunctions.x)+', '+str(zeroOfFunctions.y)+') gives total error of '+str(zeroOfFunctions.errors()[1])+'('+str(zeroOfFunctions.errors()[0])+')')



### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ### #### #### ###