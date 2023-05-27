# -*- coding: utf-8 -*-
"""
Created on May  4  2023

@author: Jan Lemeire

The qualitative causal model structures the variables in a causal contextual DAG.
Non-input variables are called target variables. 
Each target variable has a Qualitative Function (QualFun) attributed to it.
 - In case of context-independence, it is a Conditional Function (CondFun) which states the 
qualitative function given the causal parents.
 - In case of context-dependence, it is a Contextual Function (CtxtFun) which holds a condition
 defined over some context variables. To each outcome of the condition, a CondFun is attached defining
 the qualitative function that holds in that context. The parents that are not present in all this CondFun
 are the contextual parents of the target variable
The edges of the model may depend on context variables. The contextual DAG keeps a dict of the edges 
mapped onto its context variables (empty list if context-independent)
  
 
abbreviations:
    var(s) = variable(s)
    fun = function
    ctxt = context
    cond = conditional
    qual = qualitative
    quan = quantitative
"""

from numbers import Number
from typing import Callable, Iterable, Iterator

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
    
from rsQualitativeModel.QDataType import Sign
from rsLibrary.Variable import Variable, RobotVariable
from rsLibrary.DataUtils import List, Dict, Tuple, Set

# function prototype definition: https://docs.python.org/3/library/typing.html#callable

class QualFun:
    def __init__(self, variable: Variable):
        self.variable = variable
        
    def qualFun(self, state: dict[Variable: Number]) -> Sign:
        raise NotImplementedError
    def allParents(self) -> Set[Variable]:
        raise NotImplementedError
    def contextualParents(self) -> Set[Variable]:
        raise NotImplementedError

class CondFun(QualFun): # conditionalFunction
    def __init__(self, variable: Variable, parents: Variable | tuple[Variable],
                    qualFun: Callable[tuple[Number], Sign]): # takes the values of the parents as input
        super().__init__(variable)
        self.parents = parents if type(parents) == tuple else tuple( [parents] )
        self.qualFunction = qualFun
    
    def qualFun(self, state: dict[Variable: Number]) -> Sign:
        if not self.parents:
            return self.qualFunction()
        elif len(self.parents) == 1:
            return self.qualFunction( state[self.parents[0]] )
        else:
            values = tuple(state[v] for v in self.parents)
            return self.qualFunction(values)
    def allParents(self) -> Set[Variable]:
        return Set(self.parents)
    def contextualParents(self) -> tuple[Variable]:
        return Set()
    def pprint(self):
        printFun(self.qualFunction, self.parents, self.variable)

class CtxtFun(QualFun):
    '''
 
    '''
    def __init__(self, variable: Variable, ctxtVars: Variable | tuple[Variable], 
                 decisionFun: Callable[tuple[Number], int], # function of the condition: takes the values of the contextVariables as input, decides on which ConditionalFunction is valid
                 condFuns:tuple[CondFun]
                 ):
        super().__init__(variable)
        self.ctxtVars = Tuple(ctxtVars) if type(ctxtVars) == tuple else Tuple( [ctxtVars] )
        self.decisionFun = decisionFun
        self.condFuns = condFuns
        
    def decision(self, state: dict[Variable: Number]) -> int: 
        if len(self.ctxtVars) == 1:
            return self.decisionFun( state[self.ctxtVars[0]] )
        else:
            values = tuple(state[v] for v in self.ctxtVars)
            return self.decisionFun(values)
   
    def qualFun(self, state: dict[Variable: Number]) -> Sign:
        idx = self.decision(state)
        actual_cond_fun= self.condFuns[idx]
        return actual_cond_fun.qualFun(state)
    
    def allParents(self) -> Set[Variable]:
        # union of all parents via set
        all_parents = { p for cf in self.condFuns for p in cf.parents }
        return Set(all_parents)
    
    def contextualParents(self) -> Set[Variable]:
        # parents that are not present in all sets
        all_parents = { p for cf in self.condFuns for p in cf.parents }
        ctxt_parents = None
        for cf in self.condFuns:
            _ctxt = {v for v in all_parents if v not in cf.parents}
            if ctxt_parents == None:
                ctxt_parents = _ctxt
            else:
                ctxt_parents.update(_ctxt)
        return Set(ctxt_parents)
    
    def pprint(self):
        printFun(self.decisionFun, self.ctxtVars, 'context')
        for i, cf in enumerate(self.condFuns):
            print('Context', i)
            cf.pprint()

# General qualitative causal model defined over a set of variables        
class QCausalModel():
    def __init__(self, variables: list[Variable], targetVariables: list[QualFun] ):
        self.variables = variables
        self.targetVariables = targetVariables
        # create list of possibly conditional edges
        self.edges = Dict( {}, newLine = True ) # Dict[Tuple[Variable, Variable]: Tuple[Variable]]
        for tv in self.targetVariables:
            all_parents = tv.allParents()
            ctxt_parents = tv.contextualParents()
            non_ctxt_parents = all_parents.difference(ctxt_parents)
            self.edges.update( { Tuple( (v, tv.variable) ) : None for v in non_ctxt_parents } )
            self.edges.update( { Tuple( (v, tv.variable) ) : tv.ctxtVars for v in ctxt_parents } )
    def pprint(self):       
        print('Edges:\n'+str(self.edges))
        for target in self.targetVariables:
            print("Target "+target.variable.name+': all parents = '+str(target.allParents())+' contextual parents = '+str(target.contextualParents()))
            target.pprint()
     
# Qualitative causal model defined for a robot (is like a dynamic Bayesian network)    
#   robot specific code comes here 
class QRobotCausalModel(QCausalModel):
    def __init__(self, variables: list[RobotVariable], targetVariables: list[QualFun] ):        
        super().__init__(variables, targetVariables)
        # VariableType gives us the 4 possibilities: STATEVAR, ACTIONVAR, DERIVEDVAR, PREVIOUSVAR
    
def printFun(fun, inputVars: Variable | list[Variable], outputVar: Variable):
    COULUMN_WIDTH = 8
    if isinstance(inputVars, Iterable) and len(inputVars) == 1:
        inputVars = inputVars[0]
    if isinstance(inputVars, Variable):
        r = range(-10, +20, 5)
        _input = [str(i) for i in r]
        output = [ str(fun(x)) for x in r]
        print( '      '+(str(inputVars)+':\t'+ '\t'.join(_input)).expandtabs(COULUMN_WIDTH) )
        print( '      '+(str(outputVar)+'\t'+ '\t'.join(output)).expandtabs(COULUMN_WIDTH) )
    elif len(inputVars) == 0:
        print('     '+str(outputVar)+' = '+str(fun()))
    elif len(inputVars) == 2:
        r1 = range(-10, +20, 5)
        r2 = range(-10, +20, 5)
        _input1 = [str(i) for i in r1]
        _input2 = [str(i) for i in r2]
        output = [ [ str(fun(j, i)) for j in r1] for i in r2]
        
        print( (' '+str(outputVar)+'\t\t'+ '\t'.join(_input1)+'\t <= ' + str(inputVars[0])).expandtabs(COULUMN_WIDTH) )
        for row, arr in enumerate(output):
            if row == 2:
                print( ('  '+str(inputVars[1])+'\t'+_input2[row]+':\t' + '\t'.join(arr)).expandtabs(COULUMN_WIDTH) )
            else:
                print( ('\t'+_input2[row]+':\t' + '\t'.join(arr)).expandtabs(COULUMN_WIDTH))
    else:
        print('Printing of function with '+ len(inputVars)+' input variables not supported yet.')
    
    

    
if __name__== "__main__":
    print('*** Demo code QCausalModel.py ***')
    
    if True:
        # a -> b -ctxt-> c
        a, b, c, ctxt =  ( Variable(v) for v in ('a', 'b', 'c', 'ctxt') )
        
        _b = CondFun( b, a,  lambda x : Sign.sign(-x)) # doc on lambdas: see https://www.w3schools.com/python/python_lambda.asp
        
        condFuns = ( CondFun(c, (),  lambda : Sign.ZERO), CondFun(c,  b, lambda x : Sign.PLUS if x > 0 else Sign.ZERO) )
        _c = CtxtFun(c, ctxt, lambda x : 1 if x >= 10 else 0, condFuns)
                     
        qcm = QCausalModel( [a, b, c, ctxt], [_b, _c] )
        
        qcm.pprint()
           
        
        print("\n ** Let's execute the model **")
        states = [ Dict({a : 1, b : -1, c: 0, ctxt: 0}),
                   Dict({a : 5, b : -5, c: 0, ctxt: 5}),
                   Dict({a : -5, b : 5, c: 1, ctxt: 10}),
                  ]
        for state in states:
            print('With state =', state, ' we have the following model predictions:')
            print(' -> b gives      :', _b.qualFun(state))
            print(' -> context of c :', _c.decision(state))
            print(' -> c gives      :', _c.qualFun(state))
        
        
    