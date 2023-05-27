# -*- coding: utf-8 -*-
"""
Created on Mar 28 2023

@author: Nick
"""

from enum import Enum
from rsQualitativeModel.QDataType import Sign
    
import numpy as np

class VariableType(Enum):
    STATEVAR = 1
    ACTIONVAR = 2
    DERIVEDVAR = 3
    PREVIOUSVAR = 4
    
class QualType(Enum):
    QUANTITATIVE = 1
    QUALITATIVE = 2
    
class OrdinalType(Enum):
    ORDINAL = 1
    NON_ORDINAL = 2


class Variable:
    def __init__(self, name, qualType:QualType=QualType.QUANTITATIVE, ordinalType:OrdinalType=OrdinalType.ORDINAL, dataType:type=float, minVal=0, maxVal=10, bins=10):
        self.name = name
        self.qualType = qualType
        self.ordinalType = ordinalType
        self.dataType = dataType
        self.minVal = minVal
        self.maxVal = maxVal
        self.bins = bins
        
    def long_str(self):
        return f"Variable:\n\t{vars(self)}"    
    
    def __str__(self):
        return self.name
    
    def __eq__(self, o):
        """Used when the == operator is called"""
        # If id or name is the same as o, return True
        if id(o)==id(self) or o==self.name:
            return True
        else:
            return False
    
    def __hash__(self):
        """To be used as a key in a dict"""
        return hash(self.name)//2
    
    @property
    def space(self):
        """Create full space for this variable (alternative to range)"""
        if self.dataType not in [int, float]:
            return list(self.dataType)
        else:    
            return np.linspace(self.minVal, self.maxVal, self.bins)
        
    def Q(self):
        """Create a qualitative version of a variable"""
        myType = type(self)
        properties = vars(self).copy()
        properties['name'] = 'q'+properties['name']
        properties['qualType'] = QualType.QUALITATIVE
        properties['ordinalType'] = OrdinalType.ORDINAL
        properties['dataType'] = Sign
        properties['minVal'] = min(list(Sign))
        properties['maxVal'] = max(list(Sign))
        properties['bins'] = len(list(Sign))
        return myType(**properties)
    
    def D(self):
        """Create a first order derivative var"""
        # Should this be a DerivedVariable with default forwardMath?
        myType = type(self)
        properties = vars(self).copy()
        properties['name'] = 'd'+properties['name']
        properties['ordinalType'] = OrdinalType.ORDINAL
        properties['dataType'] = Sign
        properties['minVal'] = self.minVal-self.maxVal
        properties['maxVal'] = self.maxVal-self.minVal
        return myType(**properties)
    
    def old(self):
        """Create a old version of a variable"""
        # DerivedVariable?
        myType = type(self)
        properties = vars(self).copy()
        # print(properties, myType)
        # properties = vars(self).copy()
        properties['name'] = properties['name']+"_old"
        return myType(**properties)
    
    @staticmethod
    def createSimpleVariables(*varStrings, ranges=None):
        """Utility to create some simple state and action vars from a list"""
        varList = []
        for idx, var in enumerate(varStrings):
            varT, qualT, ordT, datT = None, None, None, None
            # Set default ranges
            minVal, maxVal = 0, 100
            # 'q' represents Qualitative var
            if var[0]=='q':
                qualT = QualType.QUALITATIVE
                ordT = OrdinalType.ORDINAL
                # Assume type is Sign in this case
                datT = Sign
                robVar = True
                if var[1]=='s':
                    varT = VariableType.STATEVAR
                elif var[1]=='a':
                    varT = VariableType.ACTIONVAR
                # If not a state or action var, then no RobVar
                else:
                    robVar = False
                minVal, maxVal, bins = min(list(Sign)), max(list(Sign)), len(list(Sign))
            else:
                qualT = QualType.QUANTITATIVE
                ordT = OrdinalType.ORDINAL
                # Use float as default for quantitative vars
                datT = float
                robVar = True
                if var[0]=='s':
                    varT = VariableType.STATEVAR
                elif var[0]=='a':
                    varT = VariableType.ACTIONVAR
                # If not a state or action var, then no RobVar
                else:
                    robVar = False
                if ranges is not None:
                    minVal, maxVal = ranges[idx][0], ranges[idx][-1]
            if robVar:
                varList.append(RobotVariable(var, varT, qualT, ordT, datT, minVal, maxVal))
            else:
                varList.append(Variable(var, qualT, ordT, datT, minVal, maxVal))
        return varList

class RobotVariable(Variable):
    def __init__(self, name, varType:VariableType, qualType:QualType=QualType.QUANTITATIVE, ordinalType:OrdinalType=OrdinalType.ORDINAL, dataType:type=float, minVal=0, maxVal=10, bins=10):
        super().__init__(name, qualType, ordinalType, dataType, minVal, maxVal, bins)
        self.varType = varType

class DerivedVariable(RobotVariable):
    def __init__(self, name, varType:VariableType, qualType:QualType, ordinalType:OrdinalType=OrdinalType.ORDINAL, dataType:type=float, minVal=0, maxVal=10, bins=10):    
        super().__init__(name, varType, qualType, ordinalType, dataType, minVal, maxVal, bins)
      #  self.forwardMath = forwardMath # as a string? Then use eval? or as a function?
    def calculate(self, data, t=-1):
        raise NotImplementedError
        
class DeltaDerived(DerivedVariable):
    def __init__(self, parent:Variable, minVal=0, maxVal=10, bins=10):    
        super().__init__('d'+parent.name, varType=VariableType.DERIVEDVAR, qualType=QualType.QUANTITATIVE, ordinalType=OrdinalType.ORDINAL, dataType=float, minVal=minVal, maxVal=maxVal, bins=bins)
        self.parent = parent
    def calculate(self, data, t=-1):
        if len(data[self.parent][:t])>0:
            return data[self.parent][t]-data[self.parent][t-1]
        else:
            return 0
        
        
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###     
if __name__ == "__main__":
    # Example qualitative and non-ordinal type
    class Color(Enum):
        BLUE = 1
        RED = 2
        
    d = Variable("d", QualType.QUALITATIVE, OrdinalType.NON_ORDINAL, Color)
    print("Variable created:", d)
    
    s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL, float)
    print("Variable created:", s0)
    
    print("Create variables")
    variables = Variable.createSimpleVariables("s0", "s1", "a0", "qs0", "qds0")
    for v in variables:
        print("\t new var created", v)
    
  #  def forwardMath(data, t=-1):
  #      return data["s0"][t] + data["s1"][t]
        
  #  der0 = DerivedVariable("der0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL, float, forwardMath=forwardMath)
  #  print(der0.long_str())
  #  print("forwardMath execution = ", der0.forwardMath({"s0":[7,5,6], "s1":[4, 3, 6]}))
    
    
        