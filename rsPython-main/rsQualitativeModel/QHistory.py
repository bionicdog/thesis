# -*- coding: utf-8 -*-
"""
Created on Apr 6, 2023

@author: Nick Wouters en Jan Lemeire
"""
from enum import Enum
from numbers import Number
from numpy import sign
import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code


from rsLibrary.DataUtils import flattenDict, List, Dict
from rsLibrary.Variable import Variable, RobotVariable, DerivedVariable, DeltaDerived, VariableType, QualType, OrdinalType
from rsQualitativeModel.QDataType import Sign
from rsLibrary.DataSet import DataSet

class QHistory(DataSet):
    """Data structure for storing the state history """
    def __init__(self, *variables:RobotVariable):
        super().__init__(*variables)
        self.stateVars = List()
        self.actionVars = List()
        self.derivedVars = List()
        
        for var in variables:
            # Priority if derived variable (which can be a state or action var, but doesn't get included)
            if type(var) == DerivedVariable:
                self.derivedVars.append(var)
            elif var.varType == VariableType.STATEVAR:
                self.stateVars.append(var)
            elif var.varType == VariableType.ACTIONVAR:
                self.actionVars.append(var)
        
        if len(self.derivedVars)==0:
            self.setDefaultDerivedVars()
        
        # self.stateActions = [] # List of lists
        self.qDeltas = [] # List of dicts
        
    @staticmethod
    def fromDataSet(dataSet:DataSet, stateVars, actionVars, derivedVars):
        len_data = len(dataSet.data[dataSet.variables[0]])
        hist = QHistory(stateVars, actionVars, derivedVars)
        
        dat = [[dataSet.data[var][t] for var in hist.allVars] for t in range(len_data)]
        hist.stateActions = dat
        return hist

    @property
    def allVars(self):
        """Combines all variables into one list, so indexing is easier"""
        return List(self.stateVars+self.actionVars+self.derivedVars) # List for nice print
    
    @property
    def stateActions(self):
        """Return the stateAction values as a list of samples over time"""
        # Create from self.data
        nr_samples = len(self.data[self.variables[0]])
        # Return the samples per timestep, together witch calculated forwardMath values
        return [[self.data[v][t] for v in self.data] for t in range(nr_samples)]
        
    def __iter__(self):
        """Dunder method to return a built-in iterator object"""
        return ((h, q) for h, q in zip(self.stateActions, self.qDeltas))
    
    def __next__(self):
        """Dunder method to implement the next method for the iterator"""
        return (next(self.stateActions), next(self.qDeltas))
    
  #  def __str__(self):
  #      return f"QHistory with vars:{[str(v) for v in self.allVars]}"
    
    def varIdx(self, var:str):
        """Return the index of var in allVars"""
        return self.allVars.index(var)
    
    def setDefaultDerivedVars(self):
        """Default derived vars are the deltas from all stateVars"""
        for s in self.stateVars:
            
     #       def forwardMath(data, t=-1):
     #           if len(data[s][:t])>0:
     #               return data[s][t]-data[s][t-1]
     #           else:
     #               return 0
#            self.derivedVars.append(DerivedVariable('d'+str(s), VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL, forwardMath=forwardMath))
            self.derivedVars.append(DeltaDerived(s))
        
    # def Q(self, var:str, t=-1):
    #     """Basic operator to obtain qualitative value"""
    #     return Sign.sign(self.stateActions[t][self.allVars.index(var)])
    
    def add(self, stateActionDict: dict):
        """Add new information to self.data"""
        # add to self.data!
        for v in stateActionDict:
            if v in self.variables or v in self.varNames:
                self[v].append(stateActionDict[v])
        for dv in self.derivedVars:
            if dv in self.data:
                # Append to corresponding
                self[dv].append(dv.calculate(self))
            else:
                # Create new list
                self[dv] = [dv.calculate(self)]
        
    def forwardMath(self, t=-1):
        """Calculate all derived variable values, at time t, return them as a dict"""
        derivedVarDict = {}
        for s in self.derivedVars:
            derivedVarDict[s] = s.forwardMath(self.data, t)
        return derivedVarDict
    
    def getStateActionAtIdx(self, idx:int):
        """Return the entry at idx as two dicts: state and action"""
        ent = self.stateActions[idx]
        state = {sVar:sVal for sVar, sVal in zip(self.stateVars, ent)}
        actions = {aVar:aVal for aVar, aVal in zip(self.actionVars, ent[len(self.stateVars):])}
        return state, actions
        
    def view(self, *viewVars:str) -> list[list[Number]]:
        """Returns stateActions with only variables that correspond to idxs in view"""
        # Make new History, with only vars from view
        viewDat = super().view(*viewVars)
        viewHist = QHistory(*viewDat.variables)
        # Set properties correspondingly
        viewHist.data = viewDat.data
        viewHist.variables = viewDat.variables
        return viewHist
        # return [[val for idx, val in enumerate(entry) if idx in view] for entry in self.stateActions] 
    
    ## JL move to alg
    def conflict(self, state:dict[str, Number], actions:dict[str, Number], qActual:dict[str, Sign]) -> bool:
        """Check for conflicts between inputs and full stateActions"""
        sa = {**state, **actions}
        sa = {key:sa[key] for key in sa if key in self.stateVars+self.actionVars}
        sa_flat = flattenDict(sa)
        if sa_flat in self.stateActions:
            idx = self.stateActions.index(sa_flat)
            for var in qActual:
                if self.qDeltas[idx][var]!=qActual[var]:
                    return True
        return False
    
    ## JL move to alg   
    def createNewdepSetIAMBMutInf(self, var:str, thresh=0.1, printLevel=0) -> set[str]:
        """Based on IAMB, with forward and backward pass"""
        if printLevel>0:
            print("DepSet creation with IAMB for", 'd'+var)
        # Forward
        # Start with empty deps and all eligible vars
        deps = []
        eligible = self.stateVars+self.actionVars
        distr = self.createJointDistribution()
        # Repeat until eligible is empty
        while len(eligible)>0:
            if printLevel>0:
                print("Eligible set:", eligible)
            # Find var that has highest association
            mutInfs = [distr[var].conditionalMutualInformation(
                'd'+var, 
                var, 
                *deps
                ) for var in eligible]
            if printLevel>0:
                print("List of MI:", mutInfs)
            argmax = mutInfs.index(max(mutInfs))
            y = eligible[argmax]
            # Remove from eligible
            eligible = eligible[:argmax] + eligible[argmax+1:]
            # If higher than thresh, add to deps
            if distr[var].conditionalMutualInformation(
                    'd'+var,
                    y, 
                    *deps
                    )>thresh:
                deps.append(y)
                eligible = [e for e in list(set(self.stateVars+self.actionVars)) if e not in deps]
            if printLevel>0:
                print("Intermediate depSet during forward step:", deps)
            
        # Backward
        for x in deps:
            if distr[var].conditionalMutualInformation(
                    'd'+var,
                    x,
                    *[d for d in deps if d!=x]
                    )<thresh:
                deps = [d for d in deps if d!=x]
            if printLevel>0:
                print("Intermediate depSet during backward step:", deps)
        return set(deps)       
        
    def plotHistory(self, *variables:str):
        import matplotlib.pyplot as plt
        
        t = range(len(self[variables[0]]))
        
        if len(variables)==0:
            print("Nothing to plot")
        elif len(variables)==1:
            plt.xlabel(xlabel="t")
            plt.ylabel(ylabel=variables[0])
            plt.title(f"Plot of {variables[0]} over time")
            
            #JL from dataset
            data = self[variables[0]]
            plt.plot(t, data, 'o')
      #  elif len(variables)==2:
      #    import seaborn as sns 
      #      sns.set(style = "darkgrid")
      #      ax = plt.axes(projection='3d')
      #      ax.set_xlabel("time")
      #      ax.set_ylabel(variables[0])
      #      ax.set_zlabel(variables[1])
      #      plt.title(f"Plot of {variables[0]} and {variables[1]} over time")
      #      data = [[self.stateActions[i][self.allVars.index(var)] for var in variables] for i in t]
      #      y = [float(p[0]) for p in data]
      #      z = [float(p[1]) for p in data]
      #      ax.scatter(t, y, z, 'o')
        else: 
             for i, var in enumerate(variables):
                 data = self[var]
                 plt.plot(t, data, label = str(var))
             plt.xlabel(xlabel='time')
             plt.legend(loc='upper center')
             plt.show()
           
    def toDataSet(self, oldVars=False, dVars=False):
        """Create DataSet from History"""      
        # dat = DataSet(*self.allVars)
        # for idx, var in enumerate(self.allVars):
        #     dat.data[var] = [self.stateActions[t][idx] for t in range(len(self.stateActions))]
        # if oldVars:
        #     for var in self.allVars:
        #         # print(var.old())
        #         dat[var.old()] = dat.old(var)
        # if dVars:
        #     for var in self.allVars:
        #         dat[var.d()] = dat.d(var)
        # return dat
        return self
           

    ## To be completed


### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###
if __name__ == "__main__":
    testFlag = 1
    if testFlag==1:
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)

        print("Tests QHistory:")
        hist = QHistory(s0, a0)
        print(hist)
        hist.add({"s0":1, "a0":5})
        hist.add({"s0":2, "a0":3})
        hist.add({"s0":4, "a0":4})
        hist.add({"s0":3, "a0":3})
        hist.add({"s0":4, "a0":1})
        hist.add({"s0":4, "a0":2})
        print(hist.stateActions)
        distr = hist.createJointDistribution()
        print("ranges:", distr.ranges)
        print("Mutual information", distr.mutualInformation("s0", "a0"))
        # Test iterator
        for s in hist:
            print(s)
        print("Test new methods for full stateActions")
        print(hist.getStateActionAtIdx(2))
        # print(hist.histFromView({1}).qDeltas) # OLD
      #  hist.plotSpace(hist.allVars, "s0")
        if True:      
            import pickle
            file = open('TestHistoryWriting.pkl', 'wb')
            pickle.dump(hist, file)
            print('History of test 1 written to pickle file '+file.name)
            file.close()
    elif testFlag==2:
        print("Tests derived Vars and Q operator")
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        
        hist = QHistory(s0, a0)
        hist.add({s0:1, a0:5})
        hist.add({s0:2, a0:3})
        hist.add({s0:4, a0:4})
        hist.add({s0:3, a0:3})
        hist.add({s0:4, a0:1})
        hist.add({s0:4, a0:2})
        print("derivedVars:", hist.derivedVars)
        print("forwardMath, t=3:", hist.forwardMath(t=3))
        print("history:", hist.stateActions)
        print("Q(ds0, t=-1) =", hist.Q("ds0", -1))
        print("Q(s0, t=-1) =", hist.Q("s0", -1))
        hist.plot("s0", "a0")
    elif testFlag==3:
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        # Add qs0 as a variable to QHistory
        def forwardMath(data, t=-1):
            if len(data[s0][:t])>0:
                # data["ds0"] doesn't work, so a workaround
                return Sign.sign(data[s0][t]-data[s0][t-1])
            else:
                return Sign.sign(0)
        qs0 = DerivedVariable("qds0", VariableType.STATEVAR, QualType.QUALITATIVE, OrdinalType.ORDINAL, forwardMath=forwardMath)
        
        hist = QHistory(s0, a0, qs0)
        print("variables", hist.allVars)
        hist.add({s0:1, "a0":5})
        hist.add({"s0":2, "a0":3})
        hist.add({"s0":4, a0:4})
        hist.add({"s0":3, "a0":3}) 
        hist.add({s0:4, a0:1}) # Doesn't matter if string or object is used
        hist.add({"s0":4, "a0":2})
        print(hist.stateActions)
    
        distr = hist.createJointDistribution()
        print(distr.probabilities.shape)
        
        hist.plot(s0)

        dat = hist.toDataSet()
        print(dat)
        
        distr = hist.createJointDistribution()
        print(distr.probabilities.shape)
        
    elif testFlag==4:
        print("Test DataSet methods as superclass of History")
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        
        hist = QHistory(s0, a0)
        hist.add({s0:1, a0:5})
        hist.add({s0:2, a0:3})
        hist.add({s0:4, a0:4})
        hist.add({s0:3, a0:3})
        hist.add({s0:4, a0:1})
        hist.add({s0:4, a0:2})
        
        print(hist)
        print(hist.orderedVariables())
        print(hist["s0"])
        print(hist.d(s0))
        
        print(hist.view("a0"))
        
        
        
