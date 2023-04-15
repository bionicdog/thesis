# -*- coding: utf-8 -*-
"""
Created on Fri May 13 15:30:48 2022

@author: woute
"""
from math import pi

from Environment2D import Environment2D

#TODO: clean

class RobotConstraintGenerator():
    """class to generate spatial constraints for a robot in a 2D environment"""
    def __init__(self, robot, maxDeltas:dict):
        self.robot = robot
        self.maxDeltas = maxDeltas #the maximum change in 1 timestep
        self.generators = [Simple2DConstraintGenerator(robot.states['x'], robot.states['y'], robot.states['o'], robot.environment, maxDeltas)]
    
    def getConstraints(self):
        regionList = []
        for gen in self.generators:
            regionList+=gen.getPossibleRegions()
        return Constraints(regionList)
        
    
      
                    
class Simple2DConstraintGenerator():
    """
    generates orientation constraints in a 2D environment, based on x, y and o
    The constraints depend on the states, the idea is that ConstraintGenerator
    objects hold references to the states and generate the Constraints
    """
    def __init__(self, x:list, y:list, o:list, environment:Environment2D, maxDeltas:dict):
        #dict of lists to be compliant with States structure
        self.states = {'x':x, 'y':y, 'o':o}
        self.environment = environment
        self.maxDeltas = maxDeltas
        
    def __getCritAngles(self):
        """
        Based on the field of view and stationary points(angles), it calculates
        the spatial monotone regions.
        """
        #critical angles don't depend on the orientation
        pos = (self.states['x'][-1], self.states['y'][-1])
        critAngles = []
        #corners
        for angles in self.environment.fieldOfView(self.states):
            critAngles.append(angles[0])
            critAngles.append(angles[1])
        #stationary points
        for wall in self.environment.walls:
            critAngles.append(wall.angleOfPerpendicularIntersectPoint(pos, 0))
        
        #remove duplicates and sort
        critAngles=list(set(critAngles))
        critAngles.sort()
        #circularity by adding the smallest angle + 360°
        critAngles.append(critAngles[0]+2*pi)
        return critAngles
    
    def getPossibleRegions(self):
        """
        Constructs the monotone regions from the critical angles. The regions 
        are currently only dependent of 'o' but this should be extensible to 
        other variables as well.
        """ 
        critAngles = self.__getCritAngles()
        regions = []
        for var in self.maxDeltas:
            delta = self.maxDeltas[var]
            val = self.states[var][-1]
            max_val = (val+delta)
            min_val = (val-delta)
            for i in range(1, len(critAngles)):
                #normal
                if max_val%(2*pi)>min_val%(2*pi) or delta>pi:
                    #check if possible
                    if min_val<critAngles[i] and max_val>critAngles[i-1]:
                        interval = Interval(var, max(critAngles[i-1], min_val), min(critAngles[i], max_val))
                        regions.append(ValidRegion([interval]))
                #wraparound
                else:
                    if min_val%(2*pi)<critAngles[i]:
                        interval = Interval(var, max(critAngles[i-1], min_val%(2*pi)), critAngles[i])
                        regions.append(ValidRegion([interval]))
                    elif max_val%(2*pi)>critAngles[i-1]:
                        interval = Interval(var, critAngles[i-1], min(critAngles[i], max_val%(2*pi)))
                        regions.append(ValidRegion([interval]))
        return regions  
    
    def getConstraints(self):
        return Constraints(self.getPossibleRegions())
    
    # def getRangeMaxDelta(self, var:str):
    #     maxDeltaRange = None
    #     if var in self.maxDeltas.keys():
    #         currentVal = self.states[var][-1]
    #         maxDeltaRange = [currentVal+self.maxDeltas[var], currentVal-self.maxDeltas[var]]
    #     return maxDeltaRange



#TODO: unify ValidRegion constraints with IllegalState?
class Constraints():
    """
    Class to define all the constraints. Goal is to check if a boundary is
    crossed after update of the variables. A transition would
    invalidate the previously constructed Jacobian.
    """
    def __init__(self, valid_regions:list):
        self.valid_regions = valid_regions
        
    def getValidRegionID(self, variables:dict):
        for idx, r in enumerate(self.valid_regions):
            if r.inRegion(variables):
                return idx
        else: return None
        
    def getBoundaryVars(self, variables_old:dict, variables_new:dict):
        region_old = self.valid_regions[self.getValidRegionID(variables_old)]
        region_new = self.valid_regions[self.getValidRegionID(variables_new)]
        variables = variables_new
        for b1 in region_old.intervals:
            if (region_new.getVarInterval(b1.var).lower_bound == b1.upper_bound):
                variables[b1.var]=b1.upper_bound
            elif (region_new.getVarInterval(b1.var).upper_bound == b1.lower_bound):
                variables[b1.var]=b1.lower_bound
        return variables
    
     #TODO: add illegal overlapping ValidRegions
                


class ValidRegion():
    """
    Class to define a valid region, where the Jacobian is continuously
    valid. Defined by one interval per variable
    """
    def __init__(self, intervals:list):
        self.intervals = intervals
        
    def inRegion(self, variables:dict):
        for b in self.intervals:
            if b.var in variables.keys():
                if not b.inBounds(variables[b.var]):
                    return False
        else: return True
    
    def getVarInterval(self, var:str):
        for b in self.intervals:
            if b.var == var:
                return b
        else: return None
        


class Interval():
    def __init__(self, var:str, lower_bound:float, upper_bound:float):
        self.var = var
        self.lower_bound = lower_bound  
        self.upper_bound = upper_bound
        
    def inBounds(self, val):
        if self.var=='o':
            val%=2*pi
        if self.lower_bound<val<self.upper_bound:
            return True
        else:
            return False
        
    def setUpperBound(self, new_value):
        self.lower_bound = new_value
        
    def setLowerBound(self, new_value):
        self.upper_bound = new_value
    
    def __str__(self):
        return ("var "+self.var +": interval="+ str((self.lower_bound, self.upper_bound)))
    
#Maybe use this later    
#class MappedInterval(Interval):
#    """
#    Class to define a relationship between two intervals: for example variable 
#    x has an interval [0:100], and variable y is x/2 in this interval. The 
#    mapping is passed as a string that is executed with 'eval'.
#    """
#    def __init__(self, var:str, mapping:str, other_interval:Interval):
#        self.mapping = mapping
#        super().__init__(var, self.__mappedValue(other_interval.lower_bound), self.__mappedValue(other_interval.upper_bound))
#        self.other_interval = other_interval
#        
#    def __mappedValue(self, val):
#        expr = str(val) + self.mapping
#        return(eval(expr))
#        
#    def inBounds(self, vals:tuple):
#        return super().inBounds(vals[1]) and self.other_interval.inBounds(vals[0])
                
        
            

if __name__ == "__main__":
#    #Validregion construction from intervals
#    aX = Interval('x', -20, 40)
#    aY = Interval('y', 0, 100)
#    a = ValidRegion([aX, aY])
#    print(aY.inBounds(20))
#    
#    bX = Interval('x', 40, 80)
#    bY = Interval('y', 0, 100)
#    b = ValidRegion([bX, bY])
#    
#    test = MappedInterval('y', '/2+50', bX)
#    print(test.inBounds((50, 75)))
#    
#    #Regions to check
#    d1 = {'x':30, 
#         'y':20,
#         'o':10
#         }
#    d2 = {'x':75, 
#         'y':30,
#         'o':10
#         }
#    print(a.inRegion(d1))
#    print(b.inRegion(d2))
#    
#    
#    constr = Constraints([a, b])
#    print("ID=", constr.getValidRegionID(d1))
#    print("ID=", constr.getValidRegionID(d2))
#    
#    print("Boundary at ", constr.getBoundaryVars(d1, d2))
    
    from Robot2D import createSimRobot2DSquareOnlyVelocity
    robot = createSimRobot2DSquareOnlyVelocity()
    robot.states['x'][-1] = -42.486835356134776
    robot.states['y'][-1] = -49.192162195015335
    robot.states['o'][-1] = 1.0
    print(robot.states['o'][-1])
    cg = Simple2DConstraintGenerator(robot.states['x'], robot.states['y'], robot.states['o'], robot.environment, maxDeltas={'o':2})
    cg = RobotConstraintGenerator(robot, maxDeltas={'o':1.5})
    robot.states['x'][-1] = -20
    robot.states['y'][-1] = -10
    robot.states['o'][-1] = 1.0
    test = cg.getConstraints()
    for region in test.valid_regions: 
        print(region.intervals[0])
        pass
        