#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jacobian.py
@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""
import numpy as np
import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Estimation import EstimationAlg, IllegalStateError, IterativeEstimationAlg, ZeroOfFunction, ZeroOfTwoParametersFunctions, ZeroOfNParameterFunctions
from rsLibrary.Monitor import gMonitorT
from math import pi

def mat2str(mat):
    return str([[ '%.3f'%k for k in row] for row in mat])
def l2str(l):
    if type(l) == list or type(l) == tuple or type(l) == np.ndarray:
        l_f = ['%.3f'%k for k in l]
        return '[' + ', '.join(l_f) +']'
    else:
        '%.2f'%l
        
def dict2str(d):
    l_f = [str(k)+': %.3f'%(d[k]) for k in d]
    return ', '.join(l_f) 
       
#### #### #### ####  ESTIMATION ALGORITHM Interface #### #### #### #### 
class EstimationWithJacobian (IterativeEstimationAlg):

    def __init__(self, estimationProblem, maxNbrIterations = 10000, errorThreshold = 0.1, UPDATE_FACTOR = 0.8, maxNbrDecreases = 10, useInt = False,printLevel = 1):
         super().__init__(estimationProblem, maxNbrIterations, errorThreshold, printLevel)
         self.useInt = useInt
         self.UPDATE_FACTOR = UPDATE_FACTOR
         self.maxNbrDecreases = maxNbrDecreases
         self.ctrOfUpdateFactorChanges = 0

         float_formatter = "{:.2f}".format
         np.set_printoptions(formatter={'float_kind':float_formatter})
 
    def applyNewValues(self, updateableVariables, val_array):
        # convert
        i = 0
        for k in updateableVariables:
            updateableVariables[k] = val_array[i]
            i+=1
            
        # apply
        self.estimationProblem.update(updateableVariables)  # in RobotModel
        
        if self.printLevel > 4:
            print()
            gMonitorT().printAllFrames()
        
        # retrieve errors
        errors, totalError = self.estimationProblem.errors()
        return np.array( [v for v in errors] ), totalError
    
    def chooseDelta(self, variable:str, currentValue):
        var_range = self.estimationProblem.rangeOfVariable(variable)
        # if variable == 'o':
        #     var_range = [0, pi/4]
        DELTA = 1 # default
        if var_range != None:
            DELTA =  min(1, var_range[1]/10)
            if currentValue + DELTA >= var_range[1] or currentValue + DELTA <= var_range[0]:
                if DELTA==0:
                    DELTA = var_range[0]/10 # go to the other side
                else:
                    DELTA = DELTA/10
                if self.printLevel > 4:        
                    print('*** REDUCING DELTA to '+str(DELTA)+' (JACOBIAN.chooseDelta) ***')
                
        return DELTA
    
    # def chooseDelta(self, variable:str, currentValue):
    #     var_range = self.estimationProblem.rangeOfVariable(variable)
    #     # if variable == 'o':
    #     #     var_range = [0, pi/4]
    #     DELTA = 1 # default
    #     if var_range != None:
    #         DELTA =  min(1, var_range[1]/10)
    #         if currentValue + DELTA >= var_range[1]:
    #             if currentValue + DELTA/10 > var_range[1]:
    #                 DELTA = var_range[0]/10 # go to the other side
    #             else:
    #                 DELTA = DELTA/10
    #             if self.printLevel > 4:        
    #                 print('*** REDUCING DELTA to '+str(DELTA)+' (JACOBIAN.chooseDelta) ***')
    #     return DELTA

    
    def checkIfUpdateFactorWillBeWithinRange(self, updateableVariables_list, val_array, update_array ):
        ''' check if variables within range, otherwise decrease update_factor - if 0, stop estimation '''
        new_update_factor = self.UPDATE_FACTOR
        for i in range(len(updateableVariables_list)):
            var_range = self.estimationProblem.rangeOfVariable(updateableVariables_list[i])
            if var_range != None:
                new_value = val_array[i] + new_update_factor * update_array[i]
                if new_value > var_range[1]:
                    new_update_factor = (var_range[1] - val_array[i]) / update_array[i]
                    if self.printLevel > 3:
                        print('*** REDUCING UPDATE_FACTOR to '+str(new_update_factor)+'(JACOBIAN.checkIfUpdateFactor) ***')
                elif new_value < var_range[0]:
                    new_update_factor = ( var_range[0] - val_array[i]) / update_array[i]
                    if self.printLevel > 3:
                        print('*** REDUCING UPDATE_FACTOR '+str(new_update_factor)+'(JACOBIAN.checkIfUpdateFactor) ***')
                
                new_value = val_array[i] + new_update_factor * update_array[i]
                
#                if min( abs(new_value - var_range[1]), abs(new_value - var_range[0])) <= self.errorThreshold:
#                    if self.printLevel > 3:
#                        print('*** stop estimation (below threshold of '+str(self.errorThreshold)+'JACOBIAN.checkIfUpdateFactor) ***')

        return new_update_factor
    
    
    def rowOfJacobianForVariable(self, updateableVariables, updateableVariables_list, val_array, idx:int, error_array):
        DELTA = self.chooseDelta(updateableVariables_list[idx], val_array[idx])
        val_array[idx] += DELTA
        val_array_cp = val_array.copy()
        try:
            new_error_array, new_total_error = self.applyNewValues(updateableVariables, val_array) # JL KAN LEIDEN TOT ONGELDIGE STATE
            #val_array[idx] -= DELTA
        except IllegalStateError as err:
            if self.printLevel > 4:
                print("IllegalStateError when applying DELTA "+str(DELTA)+" on "+str(updateableVariables_list[idx]) +": "+str(err)+" => try inverse DELTA")
            val_array[idx] -= DELTA
            # try other DELTA             
            DELTA = -DELTA
            val_array[idx] += DELTA
            try:
                new_error_array, new_total_error = self.applyNewValues(updateableVariables, val_array) # JL KAN LEIDEN TOT ONGELDIGE STATE
                #val_array[idx] -= DELTA
            except IllegalStateError as err:
                if self.printLevel > 4:
                    print("IllegalStateError when applying second DELTA "+str(DELTA)+" on "+str(updateableVariables_list[idx]) +": "+str(err)+" => return zero array")
                val_array[idx] -= DELTA
                d_error = np.zeros( len(error_array) )
                return d_error
        
        d_error = (new_error_array - error_array) / DELTA

        if self.printLevel > 4:
            print(' ++ Jac contr: update of '+str(updateableVariables_list[idx])+ ' to {:.1f} (DELTA={:.1f})'.format(val_array_cp[idx], DELTA)) #+' gives new error '+l2str(new_error_array)+' => d_error '+l2str(d_error))
            self.estimationProblem.printEvidence() ################
            print('Jac: update '+str(val_array_cp)+' gives new error '+l2str(new_error_array)+' => d_error '+l2str(d_error))
        
        return d_error
        
    def constructJacobian(self, updateableVariables, updateableVariables_list, val_array, error_array):
        error_length = len(error_array)
        update_length = len(val_array)
        
        if self.printLevel > 4:
            print(' ++++++++++++ Jacobian construction ++++++++++++++')
            print(dict2str(updateableVariables)) ################
            self.estimationProblem.printEvidence()
        jac = np.zeros([error_length, update_length])
        
        
        for idx in range(update_length):
            d_error = self.rowOfJacobianForVariable(updateableVariables, updateableVariables_list, val_array, idx, error_array)
            jac[:, idx] = d_error # row of Jacobian
            
        if self.printLevel > 4:
            print(' ++++++++++++ Jacobian construction END ++++++++++++++')
        return jac

    #### ==============  Interface method of IterativeEstimationAlg  ============== ####   
    def changeEstimate(self, updateableVariables:dict) -> bool:
        updateableVariables_list = list(updateableVariables) # dict to list of keys        
        errors, totalError = self.estimationProblem.errors()
        
        # transform to arrays
        error_array = np.array( [v for v in errors] )
        val_array = np.array( [v for k, v in updateableVariables.items()] , dtype = int if self.useInt else float)
        
        
        jac = self.constructJacobian(updateableVariables, updateableVariables_list, val_array, error_array)
        if self.printLevel > 3:        
            print('Jacobian = \n'+str(jac))
        
       
        # apply jac
        cov_mat_diag = self.estimationProblem.covarianceMatrixDiagonal()
        if cov_mat_diag is None:
            pinvJac = np.linalg.pinv(jac)
            update_array = -np.dot(pinvJac, error_array)
        else:
            cov = np.array(cov_mat_diag) * np.eye(len(cov_mat_diag))
            omega = np.linalg.pinv(cov)
            jac_transposed = np.transpose(jac)
            H = np.dot(np.dot(jac_transposed, omega), jac)
            b = np.dot(np.dot(jac_transposed, omega), error_array)
            pinvH = np.linalg.pinv(H)
            update_array = -np.dot(pinvH, b)

           # pinvJac = np.linalg.pinv(jac)
        
        
        if self.printLevel > 3: 
             print(' Jac gives update_array of '+l2str(update_array))
             
        
        if False: # optional check of jac
            self.checkIfJacobianIsOK( updateableVariables, updateableVariables_list, val_array, update_array, totalError, jac)
            
        self.UPDATE_FACTOR = self.checkIfUpdateFactorWillBeWithinRange(updateableVariables_list, val_array, update_array )
        
        ctr_changes = 1
        while ctr_changes < self.maxNbrDecreases:
            ctr_changes += 1
            try:
                update_array *= self.UPDATE_FACTOR
                if self.printLevel > 3:  
                    print(' => update of '+str(updateableVariables_list)+' with '+l2str(update_array)+'  (factor '+str(self.UPDATE_FACTOR)+')' )
                val_array += update_array
               
                new_error_array, new_total_error = self.applyNewValues(updateableVariables, val_array)   # JL KAN LEIDEN TOT ONGELDIGE STATE - verander UPDATE_FACTOR INDIEN NODIG

                if new_total_error < totalError:
                    return True

            except IllegalStateError as err:
                if self.printLevel > 1:  
                    print("IllegalStateError when applying self.UPDATE_FACTOR "+str(self.UPDATE_FACTOR)+" (err="+str(err)+")  => we reduce update_factor")
            
            val_array -= update_array
            self.UPDATE_FACTOR = self.UPDATE_FACTOR / 2
            if self.printLevel > 1:  
                print('Increase of error: update_factor is reduced to '+str(self.UPDATE_FACTOR))
        
        return False
                
    def checkIfJacobianIsOK(self, updateableVariables, updateableVariables_list, val_array, update_array, totalError, jac):
        update_array_cp = update_array.copy()
        update_array_cp *= 0.01
        
        val_array_cp = val_array.copy()
        val_array_cp += update_array_cp
        new_error_array, new_total_error = self.applyNewValues(updateableVariables, val_array_cp)
        self.applyNewValues(updateableVariables, val_array) # reset values
        if new_total_error > totalError:
            print(' **** Jac is NOK **** with update '+l2str(update_array_cp)+' update_factor = '+str(self.UPDATE_FACTOR)+', err increases from %.4f'%(totalError)+' to %.4f'%(new_total_error))
            print('Jacobian = \n'+str(jac))
            if False and self.printLevel < 5: # to prevent endless loop
                old_level = self.printLevel
                self.printLevel = 5
                old_update_factor = self.UPDATE_FACTOR
                self.UPDATE_FACTOR = 0.01
                print('***************************   RECHECK JAC ********************************')
                self.changeEstimate(updateableVariables) # run it again
                self.printLevel = old_level
                self.UPDATE_FACTOR = old_update_factor
                print('***************************   END OF RECHECK JAC ********************************')
        elif self.printLevel > 4: 
            print(' **** Jac is OK!! ****  err decreases from %.4f'%(totalError)+' to %.4f'%(new_total_error))



    def increaseOfError(self):
        # first time: check with a very small UPDATE_FACTOR

        
       # self.ctrOfUpdateFactorChanges += 1
       # if self.ctrOfUpdateFactorChanges < self.maxNbrDecreases:
       #     self.UPDATE_FACTOR = self.UPDATE_FACTOR / 2
       #     if self.printLevel > 1:  
       #         print('Increase of error: update_factor is reduced to '+str(self.UPDATE_FACTOR))
               # self.printLevel = 5
               # print('**** printLevel increased to '+str(printLevel))
                
       #     return True # continue with the estimation
        
        return False # return whether estimation should continue


    def printParameters(self):
        super().printParameters()
        print(' update_factor = %.2f'%(self.UPDATE_FACTOR))
        


### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Estimation.py ***')
    
    if True:
        def function(x):
            return 20 - 8*x - 2 * x * x
        initial_guess = 0
        zeroOfFunction = ZeroOfFunction(function, initial_guess)
        print('Function(%.2f) gives error %.2f'%(initial_guess, zeroOfFunction.errors()[1]))
        
        est = EstimationWithJacobian(zeroOfFunction, maxNbrIterations = 20,  errorThreshold = 0.01)
        est.solve()
        print('Function(%.2f) gives error %.2f'%(zeroOfFunction.x, zeroOfFunction.errors()[1]))
 
    if False:
        def function1(x, y):
            return -5+ 20 * y + 8*x*y - 2 * x * x
        def function2(x, y):
            return 5 - 10 * y - 3 *x*y + 2 * y * y
        initial_guess = [0, 0]
        functions = [function1, function2]
        zeroOfFunctions = ZeroOfTwoParametersFunctions(functions, initial_guess)
        print('Functions('+l2str(initial_guess)+') gives errors '+l2str(zeroOfFunctions.errors()[0]))
        
        # does not improves with bigger _UPDATE_FACTOR!!!
        est = EstimationWithJacobian(zeroOfFunctions, maxNbrIterations = 2000,  errorThreshold = 0.01, UPDATE_FACTOR = 0.01) 
        est.solve()
        print('Functions(%.2f, %.2f) gives total error of %.4f ('%(zeroOfFunctions.x, zeroOfFunctions.y, zeroOfFunctions.errors()[1])+l2str(zeroOfFunctions.errors()[0])+')')
    if False:
        variableNames = ['a', 'la', 'lv' , 'lx']
        def function1(a, la, lv , lx):
            return 10 - a
        def function2(a, la, lv , lx):
            return 100 - la * a
        def function3(a, la, lv , lx):
            return 100 - lv * a
        def function4(a, la, lv , lx):
            return 100 - lx * a

        initial_guess = [2.7, 18, 19, 17]
        functions = [function1, function2, function3, function4]
        zeroOfFunctions = ZeroOfNParameterFunctions(functions, variableNames, initial_guess)
        print('Functions('+str(initial_guess)+') gives errors '+str(zeroOfFunctions.errors()[0]))
        
#        est = RandomChangeEstimation(zeroOfFunctions, _maxNbrIterations=100000)
        est = EstimationWithJacobian(zeroOfFunctions, maxNbrIterations = 100, errorThreshold = 1, UPDATE_FACTOR = 0.5, printLevel = 3)
        est.solve()
        print('Functions with '+str(zeroOfFunctions.updateableVariables())+ ' give a total error of '+str(zeroOfFunctions.errors()[1])+'('+str(zeroOfFunctions.errors()[0])+')')

    if False: # during robot estimation did gave no solution, here it does!
        variableNames = ['a1', 'a2', 'lm', 'lp', 'la', 'lv' , 'lx'] # 7 unknowns
#        initial_guess = [3.99, 2.31, 2.13, 2.21, 24.5, 24.27, 24.15]
        initial_guess = [2.351, 0.906, 4.260, 2.878, 40.376, 42.803, 40.827]
        
        def function1(a1, a2,  lm, lp,la, lv , lx):
            return 10 - lm * a1
        def function2(a1, a2,  lm, lp,la, lv , lx):
            return 15 - lm * a2 - lp * a1   # v1 = a1
        def function3(a1, a2,  lm, lp,la, lv , lx):
            return 100 - la * a1
        def function4(a1, a2,  lm, lp,la, lv , lx):
            return 50 - la * a2
        def function5(a1, a2,  lm, lp,la, lv , lx):
            return 100 - lv * a1
        def function6(a1, a2,  lm, lp,la, lv , lx):
            return 150 - lv * (a1 + a2)  # v2 = a1 + a2
        def function7(a1, a2,  lm, lp,la, lv , lx):
            return 100 - lx * a1 # x1 = v1 = a1
        def function8(a1, a2,  lm, lp,la, lv , lx):
            return 250 - lx * (a1 * 2 + a2) # x2 = v1 + v2 = 2 * a + a2
        def function9(a1, a2,  lm, lp,la, lv , lx):
            return 25 - (a1 * 2 + a2) # x2 = v1 + v2 = 2 * a + a2

        
        functions = [function1, function2, function3, function4, function5, function6, function7, function8, function9]
        zeroOfFunctions = ZeroOfNParameterFunctions(functions, variableNames, initial_guess)
        print('Functions('+str(initial_guess)+') gives errors '+l2str(zeroOfFunctions.errors()[0]))
        
#        est = RandomChangeEstimation(zeroOfFunctions, _maxNbrIterations=100000)
        est = EstimationWithJacobian(zeroOfFunctions, maxNbrIterations = 20, errorThreshold = 1, UPDATE_FACTOR = 0.2, printLevel = 3)
        est.solve()
        print('Functions with '+dict2str(zeroOfFunctions.updateableVariables())+ ' give a total error of %.5f'%zeroOfFunctions.errors()[1]+' ('+l2str(zeroOfFunctions.errors()[0])+')')
    