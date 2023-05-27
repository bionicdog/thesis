# -*- coding: utf-8 -*-
"""
General utility functions for data

@author: Jan Lemeire
Created on Fri Jan 15 2021

"""

# Sum of squared errors
def sse(v):
    score = 0
    for x in v:
        score += x * x
    return score

def flattenDict(myDict):
    """Utility: flatten a dict, returns a list with only the values (without keys)"""
    # ordering always preserved? Yes! => https://medium.com/junior-dev/python-dictionaries-are-ordered-now-but-how-and-why-5d5a40ee327f
    return [myDict[var] for var in myDict]


### #### #### ####   CONTROL HOW THEY ARE PRINTED    #### #### #### ###
class List(list):
  def __str__(self):
#    return str([str(v) for v in self])
    l = [str(v) for v in self]
    return '[' + ', '.join(l) + ']'
#  def __hash__(self):
     # has to be hashable to be used as a dict key
 #    return super().__hash__()

class Set(set):
  def __str__(self):
    l = [str(v) for v in self]
    return '{' + ', '.join(l) + '}'

class Tuple(tuple):
  def __str__(self):
    l = [str(v) for v in self]
    return '(' + ', '.join(l) + ')'

class Dict(dict):
  def __init__(self, d:dict, newLine:bool = False):
    super().__init__(d)
    self.newLine = newLine
    
  def __str__(self):
    l = [str(k)+':' +str(self[k]) for k in self]
    if self.newLine:
        return '[' + ',\n '.join(l) + ']'
    else:
        return '[' + ', '.join(l) + ']'

# previously used - to be integrated with the above
def arr2str(arr):
    s = '['
    for a in arr:
        if abs(a) > 10:
            s += str(int(a)) + ' '
        elif abs(a) > 0.1:
            s += str( int(a*100)/100) + ' '
        else:   
            s += str( int(a*10000)/10000) + ' '
    s += ']'
    return s

def l2str(l) -> str:
    return str([str(v) for v in l])
        

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###
if __name__ == "__main__":
    
    class C:
        def __init__(self, name:str):
            self.name = name
        def __str__(self):
            return self.name
                     
    l0 = [C('a'), C('b')]
    print('dirty print:',  l0)
    l = List(l0)
    print('nice print:',l)
    
    d0 = {C('a'):1, C('b'):2}
    print('dirty print:',  d0)
    d = Dict(d0)
    print('nice print:',d)
    
    l2 = [1, 2]
    L2 = List(l2)
    print('Numbers are printed ', L2)
    
    print('Numbers with printed ', L2)
    
    s = {C('a'), C('b')}
    S = Set(s)
    print('dirty print:',  s)
    print('nice print :', S)
    
    t = ( C('a'), C('b') )
    T = Tuple(t)
    print('dirty print:',  t)
    print('nice print :', T)