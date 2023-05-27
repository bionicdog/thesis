# -*- coding: utf-8 -*-
"""
Created on November 14 2022

@author: Nick Wouters & Jan Lemeire
"""

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
    
from rsQualitativeModel.QDataType import Sign
from rsLibrary.DataUtils import flattenDict
from rsLibrary.StatisticsUtils import JointDistribution, ProbabilityDistribution
from rsQualitativeModel.QHistory import QHistory
from typing import Callable
from sklearn import tree
from numbers import Number

import numpy as np

class QBehaviorModel(): 
    """
    Data structure for the qualitative behaviour of an MDP
    """
    def __init__(self, stateVars:list[str], actionVars:list[str], actionFunc:Callable=None, printLevel:int = 0):
        self.stateVars = stateVars
        self.actionVars = actionVars if type(actionVars) is list else [actionVars]
        self.actionFunc = actionFunc # used for sampling, can be None
        
        # history contains pairs of (s, a) state, action
        self.globalHistory = [(0) * (len(self.stateVars) + len(self.actionVars))] 
        #  links to globalHistory on index
        self.qDeltasHistory = [{s:Sign.UNKNOWN for s in self.stateVars}]
        
        self.history = QHistory(self.stateVars, self.actionVars)
        
        self.qBehaviorFunctions = { s : QBehaviorFunction(
            self.stateVars, self.actionVars, s, self.history, printLevel=printLevel
            ) for s in self.stateVars}
        
     
    def infer(self, state:dict, actions:dict) -> dict[str,Sign]:
        """
        returns dstate
        """
        # todo: call infer of all functions and group
        pass
    
    def add(self, state:dict, actions:dict, dState:dict[str,Sign]):
        """
        add triple and updates model
        """
        self.history.add(state, actions, dState)
        # *1* check for conflict
        for key in self.qBehaviorFunctions:
            func = self.qBehaviorFunctions[key]
            if func.conflictWithModel(state, actions, dState[func.dstateVar]):
                
                # *2* check dependencies
                
                # old way
                # self.depSet = self.createNewdepSetProjection()
                newDeps = func.createNewdepSetIncremental(state, actions, dState)
    
                # if self.printLevel>0 and len(newDeps-func.depSet) != 0:
                #     print('New deps found:', newDeps)
                
                func.depSet = func.depSet | newDeps
                
                # if self.printLevel > 0:
                #     print('New depSet = '+str(func.depSet))
                    
                # *3* train model
                func.model = func.createModel()

    
    def sample(self, state:dict, actions:dict): 
        """
        After action is done, add new data to histories, and update 
        the appropriate ActionStateCells
        """
        # sample, get start state and add to hist
        dState = self.actionFunc(actions)
        self.add(state, actions, dState)
      
        
    def printModel(self, plotData: bool = False):
        """
        Prints the qualitative model
        """
       # print('\t', end='')
        for s in self.stateVars:
            print(self.qBehaviorFunctions[ s ])
            if plotData:
                self.qBehaviorFunctions[ s ].plotDepSpace()
        #    f = self.qBehaviorFunctions[ s ]
        #    f.printDeps()
        #    print('\t', end='')
        #print('\n')

    def printModelForCase(self, state:dict, actions:dict):
        """
        Prints the Model for a given case
        """
        for s in self.stateVars:
            print(s, end='\t')
            f = self.qBehaviorFunctions[ s ]
            f.printFunctionForCase(state, actions)
            print('\t', end='')
        print('\n', end='')

        
        

class QBehaviorFunction():
    """
    Data structure to store the qualitative relation of function actions, state -> dState 
    """
    def __init__(self, stateVars:list[str], actionVars:list[str], dstateVar:str, history:QHistory=None, printLevel:int = 0):
        self.stateVars = stateVars
        self.actionVars = actionVars
        self.dstateVar = dstateVar # output of function
        
        if history==None:
            self.newHist = QHistory(self.stateVars, self.actionVars)
        else:
            self.newHist = history
        
        self.depSet = set() # set so no duplicates are possible
        self.depSetMaybe = set(stateVars) | set(actionVars) # | = union operator
        # https://docs.python.org/3/tutorial/datastructures.html#sets
        
        self.model = None #model(s, a) -> q
        self.printLevel = printLevel
        
        # multiple local models, dependent on context? + model selector
        self.models = []
        # self.model_ranges = []
        # self.modelSelector = QModelDep(cell_test, qBehaviour.stateVars, tree.DecisionTreeClassifier())
        
    ### Reset
    
    def reset(self):
        """Reset to initial state"""
        self.newHist = QHistory(self.stateVars, self.actionVars)
        self.depSet = set()
        self.depSetMaybe = set(self.stateVars) | set(self.actionVars)
        self.model = None
        self.models = []
        
    ### Views
    
    # def histFromView(self, view:set[int]) -> list[list[Number]]:
    #     """Returns history with only variables that correspond to idxs in view"""
    #     return [[val for idx, val in enumerate(entry) if idx in view] for entry in self.history] 
    
    def depView(self, someDepSet:set[str]) -> list[Number]:
        """ Returns index's of the dependent vars in someDepSet"""
        return [idx for idx, var in enumerate (self.stateVars + self.actionVars)\
                if var in someDepSet]
    
    def depHistory(self) -> list[Number]:
        """Return the history with only dependent variables"""
        view = self.depView(self.depSet) 
        return self.newHist.histFromView(view)
    
    ### Update model + conflict methods
    
    def add(self, state:dict[str, Number], actions:dict[str, Number], dState:dict[str,Sign]):
        """
        Update the cell with new information. Save the states where the qualitative 
        change of stateVar corresponds to sign in history. Update dependencies
        and contradictActions and choose a new model if necessary 
        """
        self.__addToHist(state, actions, dState)
        q = dState[self.dstateVar]
        
        if self.printLevel > 0:
            print(' [d'+self.dstateVar+'] ', end ='')
        
        # *1* check for conflict
        if self.conflictWithModel(state, actions, q):
            
            # *2* check dependencies
            
            # old way
            # self.depSet = self.createNewdepSetProjection()
            newDeps = self.createNewdepSetIncremental(state, actions, dState)

            if self.printLevel>0 and len(newDeps-self.depSet) != 0:
                print('New deps found:', newDeps)
            
            self.depSet = self.depSet | newDeps
            
            if self.printLevel > 0:
                print('New depSet = '+str(self.depSet))
                
            # *3* train model
            self.model = self.createModel()
        else:
            if self.printLevel > 0:
                print('No conflicts found (q='+str(q)+')')
            
    def __addToHist(self, state:dict[str, Number], actions:dict[str, Number], dStates:dict[str,Sign]):
        """Concat state and actions to new dict sa, add to corresponding sign history"""
        # sa = {**state, **actions} # concat 2 dicts
        # q = dStates[self.dstateVar] if self.dstateVar in dState else Sign.UNKNOWN
        # self.history.append(flattenDict(sa))
        # self.qDeltas.append(q)
        # NEW CODE BELOW
        self.newHist.add(state, actions, dStates)
        
    #### STEPS OF MODEL UPDATE ####    
    # *1*     
    def conflictWithHist(self, state:dict[str, Number], actions:dict[str, Number], qActual:Sign) -> bool:
        return self.newHist.conflict(state, actions, qActual)
    
    def conflictWithDepHistory(self, state:dict[str, Number], actions:dict[str, Number], qActual:Sign) -> bool:
        depHist = self.depHistory(self.depView(self.depSet))
        return depHist.conflict(state, actions, qActual)
    
    def conflictWithModel(self, state:dict[str, Number], actions:dict[str, Number], qActual:Sign) -> bool:
        q_predicted = self.infer(state, actions)
        if q_predicted == Sign.UNKNOWN:
            if self.printLevel > 0:
                print('Conflict since prediction returns UNKNOWN')
            return True
        elif q_predicted != qActual:
            if self.printLevel > 0:
                print('Conflict since predicted ('+str(q_predicted)+') <> actual ('+str(qActual)+')')
            return True
        else:
            return False
    
    # *2*  
    def createNewdepSetMI(self) -> set[str]:
        """Get MI for each var and add to deps if greater than threshold"""
        thresh = 0.5
        deps = []
        self.newHist.createBeliefs()
        for var in self.stateVars+self.actionVars:
            mutInf = self.newHist.beliefs[self.dstateVar].mutualInformation(var)
            if self.printLevel>0: 
                print(f"MI({'d'+self.dstateVar};{var}) =", mutInf)
            if mutInf>thresh:
                deps.append(var)
        return set(deps)
    
    def createNewdepSetIAMBMutInf(self, thresh=0.1) -> set[str]:
        return self.newHist.createNewdepSetIAMBMutInf(self.dstateVar)
            
    
    def createNewdepSetIncremental(self, state:dict[str, Number], actions:dict[str, Number], dState:dict[str,Sign]) -> set[str]:
        """
        Track dependencies based on the history.
        Forward: adding all vars to potential depSet
        Backward: by taking complete potential new depSet and pruning if 
        removing from depSet results in more conflicts in validatedepSet
        """
        # # Forward
        potentialdepSet = self.depSet.copy()
        # if self.printLevel>0:
        #     print("Potential depSet:", potentialdepSet)
        # # Nr of conflicts with current depSet and sample (state + action, dState)
        # nr_conflicts = len(self.checkForConflictsOnSample(state, actions, dState, potentialdepSet))
        # for var in self.stateVars+self.actionVars:
        #     # if self.printLevel>0:
        #     #     print("Var =", var, ", conflicts with:", 
        #     #           len(self.checkForConflictsOnSample(state, actions, dState, self.depSet|{var})), 
        #     #           ", conflicts without:", 
        #     #           len(nr_conflicts))
        #     #     print("Conflict points:", self.checkForConflictsOnSample(state, actions, dState, self.depSet|{var}))
            
        #     # # Check if nr of conflicts is lower when var gets added to depSet
        #     # if len(self.checkForConflictsOnSample(state, actions, dState, self.depSet|{var}))<nr_conflicts:
        #     #     if self.printLevel>0:
        #     #         print("add", var)
        #         potentialdepSet.add(var)
        
        # # Pruning
        # for var in potentialdepSet.copy():
        #     if self.printLevel>0:     
        #         print("Var =", var, ", conflicts with:", 
        #               len(self.checkForConflictsOnSample(state, actions, dState, potentialdepSet)), 
        #               ", conflicts without:", 
        #               len(self.checkForConflictsOnSample(state, actions, dState, potentialdepSet-{var})))
           
        #     # Check if ignoring var in depSet results in more conflicts
        #     if len(self.checkForConflictsOnSample(state, actions, dState, potentialdepSet))\
        #         ==len(self.checkForConflictsOnSample(state, actions, dState, potentialdepSet-{var})):
        #             if self.printLevel>0: 
        #                 print("remove", var)
        #             # If nr of conflicts stays the same, then independent from var
        #             potentialdepSet.remove(var)
                    
        # # Check whole history for single variable changes
        # # Should be integrated in conflict check, but optimised for changed var
        # sa_flat = flattenDict({**state, **actions})
        # for sa_hist, q_del in zip(self.newHist.stateActions, self.newHist.qDeltas):
        #     changedVar = self.checkIfOnlyOneVarHasChanged(sa_hist, sa_flat)
        #     if changedVar!=None and q_del!=dState[self.dstateVar]:
        #         # Certainly in depSet if only changed variable
        #         self.depSetMaybe.discard(changedVar)
        #         potentialdepSet.add(changedVar)
        #         # self.depSet.add(changedVar)
                    
        # Add potentialDepSet to self.depSet?
        return potentialdepSet
    
    def createNewdepSetFullHistory(self) -> set[str]:
        """
        Track dependencies based on the history.
        Forward: adding a var to potential depSet and see if conflicts get resolved
        Backward: by taking complete potential new depSet and pruning if 
        removing from depSet results in more conflicts in validatedepSet
        """
        # Forward
        potentialdepSet = self.depSet.copy()
        if self.printLevel>0:
            print("Potential depSet:", potentialdepSet)
        # Nr of conflicts with current depSet and sample (state + action, dState)
        nr_conflicts = len(self.checkForConflicts(potentialdepSet))
        for var in self.stateVars+self.actionVars:
            if self.printLevel>0:
                print("Var =", var, ", conflicts with:", 
                      len(self.checkForConflicts(self.depSet|{var}), 
                      ", conflicts without:", len(nr_conflicts)))
                print("Conflict points:", self.checkForConflicts(self.depSet|{var}))
                
            # Check if nr of conflicts is lower when var gets added to depSet
            if len(self.checkForConflicts(self.depSet|{var}))<nr_conflicts:
                if self.printLevel>0:
                    print("add", var)
                potentialdepSet.add(var)
                
        # Ignore forward check and consider all variables in pruning step?
        
        # Pruning
        for var in potentialdepSet.copy():
            if self.printLevel>0:     
                print("Var =", var, ", conflicts with:", 
                      len(self.checkForConflicts(potentialdepSet)), 
                      ", conflicts without:", 
                      len(self.checkForConflicts(potentialdepSet-{var})))
            # Check if ignoring var in depSet results in more conflicts
            if len(self.checkForConflicts(potentialdepSet))\
                ==len(self.checkForConflicts(potentialdepSet-{var})):
                # If nr of conflicts stays the same, then independent from var
                potentialdepSet.remove(var)
        return potentialdepSet
    
    # OLD
    # def createNewdepSetProjection(self) -> set[str]:
    #     """Track dependencies based on cell history,
    #     for now just a 1D projection check for each var (not ideal)"""         
    #     depVars = {var for var in self.stateVars if self.dependentOnVarConflict(var)}
    #     return depVars
    
    # # OLD
    # def dependentOnVarConflict(self, var:str) -> bool:
    #     if len(self.checkForConflicts(set(self.stateVars+self.actionVars)))\
    #         <len(self.checkForConflicts(set(self.stateVars+self.actionVars)-{var})):
    #             return True
    #     else:
    #         return False
    
    # TODO: use checkForConflictsOnSample for all points in history?
    # TODO: MOVE TO QHIST
    def checkForConflicts(self, somedepSet:set[str], method='bin') -> list[tuple[list[Number, Number]]]:
        """
        Returns conflictPoints if only variables in somedepSet are considered.
        If no conflictPoints are found somedepSet is thus valid.
        method => 'exact', 'bin' or 'KDE'
        A list is returned with tuples of conflicting points
        """
        # Initialise output
        conflictPoints = []
        # Find indexes of vars in somedepSet
        all_vars = self.stateVars+self.actionVars
        var_idxs = [all_vars.index(var) for var in somedepSet]
        if self.printLevel>0:
            print("Checked depSet =", somedepSet, "var_idxs =", var_idxs)
        # Projected potential depHistory
        projected = [[val for idx, val in enumerate(entry[0]) if idx in var_idxs]\
                        for entry in self.newHist]
        if self.printLevel>0: 
            print("Projected =", projected)
        # Loop through all entries in projected
        for idx, (entry, qDelta) in enumerate(zip(projected, self.newHist.qDeltas)):
            # Store indexes where projected points in state space are the same
            other_idx_list = [i for i, other_entry\
                             in enumerate(projected)\
                                 if self.inBucketRange(entry, other_entry, 1)\
                                     and i!=idx] 
            if len(other_idx_list)>0:
                # If qDeltas differ, a conflict is found
                for other_idx in other_idx_list:
                    if self.newHist.qDeltas[other_idx]!=self.newHist.qDeltas[idx]:
                        # Conflict found -> dependent on var
                        if self.printLevel>0:
                            print("conflict for points:", self.newHist.stateActions[idx], self.newHist.stateActions[other_idx])
                        # Don't add the pair if duplicate
                        if (self.newHist.stateActions[other_idx], self.newHist.stateActions[idx]) not in conflictPoints:
                            conflictPoints.append((self.newHist.stateActions[idx], self.newHist.stateActions[other_idx]))
        # Return conflictPoints
        return conflictPoints   
    
    def checkForConflictsOnSample(self, state:dict[str, Number], actions:dict[str, Number], dState:dict[str,Sign], someDepSet:set[str]) -> list[tuple[list[Number, Number]]]:
        """
        Check if new information corresponds to beliefs with someDepSet, if not 
        there are new potential dependencies. Probably to be used after an
        exploration action.
        """
        # Flatten input
        sa_flat = flattenDict({**state, **actions})
        # Initialise output
        conflictPoints = []
        # Find indexes of vars in depSet
        all_vars = self.stateVars+self.actionVars
        var_idxs = [all_vars.index(var) for var in someDepSet]
        if self.printLevel>0:
            print("Checked depSet =", self.depSet, "var_idxs =", var_idxs)
        sa_projected_dep = [val for idx, val in enumerate(sa_flat) if idx in var_idxs]
        # Store indexes where point collides with depHistory
        idx_list = [i for i, (entry, q)\
                         in enumerate(self.newHist.histFromView(self.depView(someDepSet)))\
                             if self.inBucketRange(entry, sa_projected_dep, 1)] 
        if len(idx_list)>0:
            # If qDeltas differ, a conflict is found
            for idx in idx_list:
                if self.newHist.qDeltas[idx]!=dState[self.dstateVar]:
                    # Conflict found -> dependent on var
                    if self.printLevel>0:
                        print("conflict for points:", self.newHist.stateActions[idx], sa_flat)
                    # Don't add if duplicate
                    if (sa_flat, self.newHist.stateActions[idx]) not in conflictPoints:
                        conflictPoints.append((self.newHist.stateActions[idx], sa_flat))
                    # If only one var has changed => reason for conflict = this var
                    # if len(self.changedVars(sa_flat, self.newHist.stateActions[idx]))==1: 
                    #     changedVar = self.changedVars(sa_flat, self.newHist.stateActions[idx])[0]
                    #     print("conflict for points:", self.newHist.stateActions[idx], sa_flat)
                    #     print("changed_var:", changedVar)
                    #     print(sa_flat, someDepSet)
                    #     self.depSetMaybe.discard((self.stateVars+self.actionVars)[changedVar])
                    #     self.depSet.add((self.stateVars+self.actionVars)[changedVar])
                    
        # Return conflictPoints
        return conflictPoints
    
    def changedVars(self, sa1:list[Number], sa2:list[Number]) -> list[Number]:
        """Returns the indexes(!) of the changed variables in sa1 vs sa2"""
        if self.printLevel>0:
            for idx, (val1, val2) in enumerate(zip(sa1, sa2)):
                print(f"val1={val1}, val2={val2}, val1==val2:{val1==val2}")
        return [idx for idx, (val1, val2)\
                    in enumerate(zip(sa1, sa2)) if val1!=val2]
    
    def checkIfOnlyOneVarHasChanged(self, sa1:list[Number], sa2:list[Number]) -> str:
        """Returns the changed variable name if only one variable has changed, 
        else it returns None"""
        varList = self.changedVars(sa1, sa2)
        if self.printLevel>0:    
            print("changed vars:", varList)
        if len(varList)==1:
            # Return the variable that has changed
            return (self.stateVars+self.actionVars)[varList[0]]
    
    def inBucketRange(self, sample:list[Number], other_sample:list[Number], granularity=1, printLevel=0) -> bool:
        """
        Returns True if the samples are close enough to each other
        """
        #TODO: list of granularities for each state
        if printLevel==1:
            print("Comparing", sample, "and", other_sample, "with granularity =", granularity)
        for s, so in zip(sample, other_sample):
            if not s-granularity <= so <= s+granularity:
                return False
        if printLevel==1:
            print("In bucket!")
        return True
     
    # *3*  
    def createModel(self):
        """Create QModel based on depSet"""
        #unites expand and reduce
        if len(self.depSet) == 0:
            # find the const value
            if len(self.newHist.qDeltas)==0:
                const=Sign.UNKNOWN
            else:
                const = self.newHist.qDeltas[0][self.dstateVar]
            self.model = QModelConst(self, const)
            if self.printLevel > 0:
                print("Create new QModelConst: ", self.model)
        else:
            # self.plotDepSpace()
            classifier = tree.DecisionTreeClassifier()
            self.learn(classifier)
            self.model = QModelDep(self, classifier)
            if self.printLevel > 0:
                print("Create new QModelDep: ", self.model)
            # self.model = None
        return self.model
    
    def learn(self, classifier):
        """Flatten inputs (X) and labels (y) and fit classifier"""
        X = self.depHistory().stateActions
        if self.printLevel>0:
            print("DepHistory =", X)
        y = [str(sign[self.dstateVar]) for sign in self.newHist.qDeltas]
        
        if self.printLevel>0:
            print('X = ', X, '; y = ', y)
        classifier.fit(X, y)
        
    ### Forward prediction
        
    def infer(self, state:dict[str, Number], actions:dict[str, Number]) -> Sign:
        """ Return the inferred state delta sign for a given state """
        # l = []
        # for s in self.qDeltas:
        #     if s not in l:
        #         l.append(s)
        # if len(l)>1:
        #     return Sign.UNKNOWN
        if self.model==None:
            return Sign.UNKNOWN
        else:
            return self.model.infer(state, actions)
        
    ### Methods for exploration
    
    def normalisedDirectionVector(self, sa1:list[Number], sa2:list[Number]) -> tuple[Number]:
        """Returns the normalised difference between sa1 and sa2, 
        a unit vector in the direction sa1->sa2"""
        dirVec = [val2-val1 for val1, val2 in zip(sa1, sa2)]
        no = np.sqrt(sum([val**2 for val in dirVec]))
        return tuple([val/no for val in dirVec])
    
    def negDirection(self, dirVec) -> tuple[Number]:
        """Returns the opposite/negative direction"""
        return tuple([-val for val in dirVec])
    
    def allDirs(self, someDepSet:set[str]) -> set[tuple[Number]]:
        """Returns all possible axis parallel directions, that have to be
        explored when they are in someDepSet"""
        sa_vars = self.stateVars + self.actionVars
        # Generate 2*N axis parallel directions: vectors with only one non-zero entry
        exploreDirsPos = {tuple([0 if i!=idx or not sa_vars[i] in someDepSet else 1 for i in range(len(sa_vars))]) for idx in range(len(sa_vars))}
        exploreDirsNeg = {tuple([0 if i!=idx or not sa_vars[i] in someDepSet else -1 for i in range(len(sa_vars))]) for idx in range(len(sa_vars))}
        exploreDirs = exploreDirsPos | exploreDirsNeg
        return exploreDirs
        
    def unexploredAxisDirs(self, state:dict[str, Number], action:dict[str, Number], someDepSet:set[str]) -> set[tuple[Number]]:
        """
        Returns the unexplored axis parallel directions (included in someDepset)
        for one specific point compared to the history by checking if any points
        in history correspond to these dirs
        """
        sa_flat = flattenDict({**state, **action})
        sa_vars = list({**state, **action}.keys())
        # Generate 2*N axis parallel directions: vectors with only one non-zero entry
        exploreDirsPos = {tuple([0 if i!=idx or not sa_vars[i] in someDepSet else 1 for i in range(len(sa_flat))]) for idx in range(len(sa_flat))}
        exploreDirsNeg = {tuple([0 if i!=idx or not sa_vars[i] in someDepSet else -1 for i in range(len(sa_flat))]) for idx in range(len(sa_flat))}
        exploreDirs = exploreDirsPos | exploreDirsNeg
        # Remove all full-zero vectors that were generated by ignoring vars not in someDepSet
        exploreDirs = {el for el in exploreDirs if el!=tuple([0 for i in range(len(el))])}
        for point in self.newHist.stateActions:
            # If point is the same as sa_flat, skip, because dist=0 (divide by 0)
            if point==sa_flat:
                continue
            # Direction from sa_flat to point
            nDir = self.normalisedDirectionVector(sa_flat, point)
            if nDir in exploreDirs:
                exploreDirs.remove(nDir)
        return exploreDirs
    
    def pointsToExplore(self, exploreMagnitude=1) -> list[list[Number]]:
        """
        Generates the points to explore, based on the depSetMaybe
        """
        s_len = len(self.stateVars)
        a_len = len(self.actionVars)
        # Start with all possible directions
        possibleDirs = self.allDirs(self.depSetMaybe)
        interestingPoints = []
        for sa in self.newHist.stateActions:
            # Isolate a state dict from history
            state = {var:val for var, val in zip(self.stateVars, sa[:s_len])}
            # Isolate an action dict from history
            action = {var:val for var, val in zip(self.actionVars, sa[s_len:])}
            # Exploration directions, based on depSetMaybe
            expDirs = self.unexploredAxisDirs(state, action, self.depSetMaybe)
            # Intersect with current possibleDirs
            possibleDirs &= expDirs
            if self.printLevel>0:
                print(f"State: {state}, Action: {action}, expDirs: {expDirs}")
        for direction in possibleDirs:
            # Start from last point in history and go each direction to explore
            interestingPoints.append([val + d*exploreMagnitude for val, d in zip(sa, direction)])
        return interestingPoints
    
        
    # LATER
    def selectLocalModel(self, state:dict, actions:dict):
        """
        Select a local model to infer the qualitative delta sign, based on the 
        state and action. There can be different regions in state-action space 
        where other models apply. 
        """
        # sa = {**state, **actions}
        # for model, mrange in zip(self.models, self.model_ranges):
        #     inRange = True
        #     for var in mrange:
        #         if mrange[var][0]<sa[var]<=mrange[var][1]:
        #             continue
        #         else:
        #             inRange = False
        #             break
        #     if inRange: return model
        # return None
        return self.modelSelector.infer(state, actions)
    
    
    ### Print methods
    def plotDepSpace(self):
        self.newHist.plotSpace(self.depSet, self.dstateVar)
    
    def __str__(self) -> str:
        return 'd' + self.dstateVar + ' = f(' + str(list(self.depSet)) +')' + ( ' = ' + self.model.__str__() if self.model is not None else ' = ?')

    def printFunctionForCase(self, state:dict, actions:dict):
        if self.model==None:
            print(Sign.UNKNOWN, end='')
        else:
            print(self.model.infer(state, actions), end='')
            
    def printDeps(self):
        print(str([dep for dep in self.depSet]), end='')
        

class QModel():
    """ Qualitative Model interface class"""
    def __init__(self, qBehaviorFunction: QBehaviorFunction):
        # keep circular dependency to actionStateCell, easy to change model type
        self.qBehaviorFunction = qBehaviorFunction 
        self.depSet = self.qBehaviorFunction.depSet # depSet on which model is based - depSet can change in qBehaviorFunction
        
    def infer(self, state:dict, actions:dict) -> Sign:
        """ Returns the inferred state delta sign for a given state """
        raise NotImplementedError

    def __str__(self) -> str:
        """ prints the model """
        raise NotImplementedError

    def scores(self, state:dict, actions:dict):
        """ Return certainty scores for each qualitative class"""
        raise NotImplementedError
        
    def proba(self, state:dict, actions:dict):
        def softmax(x):
            """Compute softmax values for each sets of scores in x."""
            return np.exp(x) / np.sum(np.exp(x), axis=0)
        return softmax(self.scores(state, actions))
        
class QModelConst(QModel):
    """
    Constant behaviour, simplest assumption
    """
    def __init__(self, qBehaviorFunction: QBehaviorFunction, const: Sign):
        super().__init__(qBehaviorFunction)
        self.const = const
        
    def infer(self, state:dict, actions:dict) -> Sign:
        return self.const
    
    def __str__(self) -> str:
        return str(self.const)

    def scores(self, state:dict, actions:dict):
        s = {Sign.MINUS:0, Sign.ZERO:0, Sign.PLUS:0, Sign.UNKNOWN:0}
        s[self.infer(state, actions)] = 10
        return flattenDict(s)
    
class QModelDep(QModel):
    """
    Different state/action values can change the behaviour. This model takes 
    dependencies with the state into account. Can be implemented in different
    ways.
    """
    def __init__(self, qBehaviorFunction, classifier):
        super().__init__(qBehaviorFunction)
        self.classifier = classifier
        
    def infer(self, state:dict, actions:dict) -> dict[str,Sign]:
        if self.classifier is not None:
            depState = {var:state[var] for var in state if var in self.depSet}
            depActions = {var:actions[var] for var in actions if var in self.depSet}
            flat = [flattenDict(depState) + flattenDict(depActions)]
            return self.classifier.predict(flat)[0]
        else:
            return Sign.UNKNOWN
    
    def __str__(self) -> str:
        return str(self.classifier)

    
    def proba(self, state:dict, actions:dict):
        if self.classifier is not None:
            depState = {var:state[var] for var in state if var in self.depSet}
            depActions = {var:actions[var] for var in actions if var in self.depSet}
            flat = [flattenDict(depState) + flattenDict(depActions)]
            return self.classifier.predict_proba(flat)[0]
    
class RangeTree():
    """Select hypercube where model applies"""
    def __init__(self, var, stateVars, actionVars):
        self.var = var
        self.stateVars = stateVars
        self.actionVars = actionVars
        self.children = {(float('-inf'), float('inf')):LeafNodeTest("test")}
    
    def choose(self, state:dict, action:dict):
        val = {**state, **action}[self.var]
        for low, high in self.children:
            if low<val<high:
                # recursively call choose, leaf node should be a model
                # with choose function that returns itself
                return self.children[(low, high)].choose(state, action)
            
class LeafNodeTest():
    def __init__(self, testString:str):
        self.testString = testString
    
    def choose(self, state, action):
        return self.testString
    
            


if __name__== "__main__":
    print('*** Demo code QBehaviorModel.py ***')
    
    TEST_FLAG = 0
    
    if TEST_FLAG==0:  # QBehaviorModel test code
        actionVars = ['a1', 'a2']
        stateVars = ['s1', 's2']
        qbehavior = QBehaviorModel( stateVars, actionVars, actionFunc = None, printLevel=1)
        qbehavior.printModel()
        
        # samples
       
        triples = [ ({'s1':1, 's2':2}, {'a1': 1}, {'s1': Sign.PLUS, 's2': Sign.ZERO}),
                   ({'s1':2, 's2':1}, {'a1': -1}, {'s1': Sign.MINUS, 's2': Sign.ZERO})
                   ]
        for triple in triples:
            state = triple[0]
            actions = triple[1]
            dState = triple[2]
            print(' *** SAMPLE: state = '+str(state)+' a = '+str(actions)+' => '+str(dState)+' ***')
            qbehavior.add(state, actions, dState)
            qbehavior.printModel()
        
    if TEST_FLAG==1: #elementary tests
        from sklearn import tree
        hist = {
                Sign.MINUS:[
                    {'s0':50, 's1':20, 's2':0},
                    {'s0':40, 's1':16, 's2':0}, 
                    {'s0':30, 's1':10, 's2':-1}
                    ]  ,
                Sign.ZERO:[
                    {'s0':50, 's1':50, 's2':-2},
                    {'s0':40, 's1':40, 's2':1}, 
                    {'s0':30, 's1':25, 's2':-1}
                    ],
                Sign.PLUS:[
                    {'s0':50, 's1':20, 's2':-40},
                    {'s0':40, 's1':17, 's2':-30}, 
                    {'s0':30, 's1':12, 's2':-48}
                    ]
                }
        
        print("\nTest QBehaviorFunction:")
        qBehaviorFunction = QBehaviorFunction(
            ['s0', 's1', 's2'], [], 's0'
            )
        # qBehaviorFunction.history = hist
        for s in hist:
            for sample in hist[s]:
                print(sample)
                qBehaviorFunction.add(sample, {}, {'s0':s})
        print("dstateVar =", qBehaviorFunction.dstateVar)
        # qBehaviorFunction.depSet = qBehaviorFunction.createNewdepSetProjection()
        print("Function depSet:", qBehaviorFunction.depSet)
        print("Validate depSet on history conflicts:", qBehaviorFunction.checkForConflicts(['s1']))
        print("Conflicts on samples:", qBehaviorFunction.checkForConflictsOnSample(
            state={'s0':50, 's1':50, 's2':-2}, actions={},
            dState={'s0':Sign.PLUS}, someDepSet=qBehaviorFunction.depSet
        ))
        qBehaviorFunction.depSet = set() # reset depSet
        print("Create depSet from history:", qBehaviorFunction.createNewdepSetFullHistory())
        qBehaviorFunction.depSet = qBehaviorFunction.createNewdepSetFullHistory()
        print("Update depSet:", qBehaviorFunction.checkForConflictsOnSample(
            state={'s0':50, 's1':40, 's2':1}, actions={},
            dState={'s0':Sign.PLUS}, someDepSet=qBehaviorFunction.depSet
        ))
        
        print("\nTest Incremental DepSet creation:")
        qBehaviorFunction.depSet = set()
        print("Created depSet with new sample:", qBehaviorFunction.createNewdepSetIncremental(
            state={'s0':50, 's1':40, 's2':1}, actions={}, dState={'s0':Sign.PLUS}))
        print("Certain depSet based on single var change(s):", qBehaviorFunction.depSet)
        
        print("\nTest QModelDep:")
        qBehaviorFunction.createModel()
        
        #  model_test = QModelDep(cell_test, tree.DecisionTreeClassifier())
        #  model_test.learn()
        print("Dep variables:", qBehaviorFunction.depSet)
        # tree.plot_tree(model_test.classifier)
        qBehaviorFunction.depSet = {'s2', 's1'}
        qBehaviorFunction.plotDepSpace()
        print("Inference:", qBehaviorFunction.infer({'s0':30, 's1':17.5, 's2':-2}, {}))
        print("Probabilities:", qBehaviorFunction.model.proba({'s0':30, 's1':17.5, 's2':-2}, {}))    
        print("DepHistory:", qBehaviorFunction.depHistory())
        
        print("\nTest Mutual Information deps")
        # qBehaviorFunction.newHist.createBeliefs()
        # print("Probability distribution:", qBehaviorFunction.newHist.beliefs)
        # qBehaviorFunction.newHist.beliefs.marginalise("ds0", "s1", "s2").plotProbs()
        print("Based on Mutual Information & threshold:", qBehaviorFunction.createNewdepSetMI())
        print("With IAMB-like algorithm:", qBehaviorFunction.createNewdepSetIAMBMutInf())
        print("Conflict-based:", qBehaviorFunction.createNewdepSetFullHistory())
        
        qBehaviorFunction.newHist.createBeliefs(bandwidth=5)
        # qBehaviorFunction.newHist.beliefs["s0"].marginalise("ds0", "s2", "s1").plotProbs()
        
        print("\nTest maybe deps, dropDeps and exploration points:")
        qBehaviorFunction.reset()
        print("depSetMaybe at creation:", qBehaviorFunction.depSetMaybe)
        print("Changed var idxs:", qBehaviorFunction.changedVars([44, 18, -30], [44, 17, -30]))
        print("depSetMaybe:", qBehaviorFunction.depSetMaybe)
        
        print("\nTest ModelConst:")
        model_const_test = QModelConst(qBehaviorFunction, const='+')
        print("Inference:", model_const_test.infer({'s0':40, 's1':40, 's2':1}, {"a0":50}))
        
        print("\nTest QBehaviorModel")
        from rsRobot.Robots import createSimRobot1D
        rob = createSimRobot1D()
        actions = [m.name for m in rob.motors]
        matr_test = QBehaviorModel(rob.states.stateDef.variables, actions, rob.setMotors)
        action = {'mX': 10}
        #print(rob.setMotors(action))
        matr_test.sample(rob.states.getStateT(-1), action)
        matr_test.printModelForCase(rob.states.getStateT(-1), action)
        action = {'mX': 10}
        # rob.setMotors(action) 
        matr_test.sample(rob.states.getStateT(-1), action)
        matr_test.printModel()
        
        
        
    if TEST_FLAG==2:
        print("\nTest model selection")
        tree = RangeTree('s0', ['s1', 's2'], ['a0'])
        print(tree.choose({'s0':5, 's1':2, 's2':22}, {'a0':-44}))
        sub1=RangeTree('s1', ['s2'], ['a0'])
        sub1.children = {(0, 10):LeafNodeTest("a"), (10, 100):LeafNodeTest("b")}
        tree.children = {(0, 50):sub1, (50, 100):LeafNodeTest("c")}
        
        print(tree.choose({'s0':5, 's1':20, 's2':22}, {'a0':-44}))
        # print("\nTest model selector:")
        # cell_test.models.append(model_test)
        # cell_test.model_ranges.append({'s0':[20, 60], 's1':[0, 50], 's2':[-50, 10]})
        # cell_test.models.append(model_const_test)
        # cell_test.model_ranges.append({'s0':[float('-inf'), float('inf')], 's1':[float('-inf'), float('inf')], 's2':[float('-inf'), float('inf')]})
        # print(cell_test.selectLocalModel({'s0':40, 's1':40, 's2':1}, {"a0":50}))
        # print(cell_test.selectLocalModel({'s0':-80, 's1':40, 's2':1}, {"a0":50}))
    
    if TEST_FLAG==3:
        # Some more tests
        qBehaviorFunction = QBehaviorFunction(
            ['s0', 's1', 's2'], [], 's0'
            )
        print(qBehaviorFunction.normalisedDirectionVector([4, 0, 4], [4, 4, 0]))
        
        qBehaviorFunction.history=[[4, 0, 0], [-2, 0, 0], [0, 2, 0]]
        print(qBehaviorFunction.unexploredAxisDirs({'s0':0, 's1':0, 's2':0}, {}, {'s0', 's1', 's2'}))
        print("Interesting points:", qBehaviorFunction.pointsToExplore())
    
        

        
    
    
    
    