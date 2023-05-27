 # -*- coding: utf-8 -*-
"""
Created on Mon Nov 14 12:49:31 2022

@author: Nick
"""

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from enum import Enum
from numpy import sign

class Sign(Enum):
    """
    Enum representing the qualitative change sign
    """
    MINUS = -1
    ZERO = 0
    PLUS = 1
    UNKNOWN = 2
    
    def __str__(self):
        return ["-", "0", "+", "?"][self.value+1]
    
    def __mul__(self, other):
        new_val = self.value*other.value
        if new_val==-2:
            return Sign.UNKNOWN
        else:
            return Sign(self.value*other.value)
    
    def __hash__(self):
        # has to be hashable to be used as a dict key
        return hash(str(self))
    
    def __float__(self):
        return float(self.value)
    
    def __int__(self):
        return self.value
    
    def __gt__(self, other):
        return True if float(self) > float(other) else False
    
    def __lt__(self, other):
        return True if float(self) < float(other) else False
    
    def __ge__(self, other):
        return True if float(self) >= float(other) else False
    
    def __le__(self, other):
        return True if float(self) <= float(other) else False
    
    @staticmethod
    def listAll():
        return [Sign.MINUS, Sign.ZERO, Sign.PLUS, Sign.UNKNOWN]
    
    @classmethod 
    def sign(cls, val):
        return cls(sign(val))
 
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__ == "__main__":
    testFlag = 0
    if testFlag==0:
        print("Tests Sign Enum:")
        print(Sign(0)*Sign(2))
        print(Sign.MINUS)
        print(Sign(Sign.MINUS))
        print(Sign.sign(-5))
        print(Sign(0)==Sign(1))
        s = set()
        s.add(Sign(0))
        s.add(Sign(2))
        s.add(Sign(2))
        print(len(s))
        
        
