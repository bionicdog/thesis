# -*- coding: utf-8 -*-
"""
Created on Fri Apr  8 09:51:27 2022

@author: woute
"""
import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from State import States, StateDef
from Environment2D import Environment2DSquare, Environment2D
import Sensors

from scipy.stats import rv_continuous as distribution
from math import pi, cos, sin, sqrt

class UnidirectionalDistanceSensor(Sensors.MotorOrSensor):
    """
    Sensor which gives the distance to the closest object or wall in a 2D 
    environment in the direction of it's orientation
    
    Distance to the closest wall is calculated based on the statevars and the
    environment. maxVal determines the max distance the sensor can 'see'
    something, orientationOffset how the sensor is oriented relative to the 
    Robot's orientation (in radians).
    """
    def __init__(self, name, environment:Environment2D, statevars=['x', 'y', 'o'], paramName='ld', paramValue=10, noiseDistr: distribution = None, maxVal=100, orientationOffset = 0):
        super().__init__(name, sensorType = "distance", params = {paramName:paramValue}, noiseDistr = noiseDistr)
        self.statevars = statevars
        self.environment = environment
        self.paramName = paramName
        self.paramValue = paramValue
        self.maxVal = maxVal
        self.orientationOffset = orientationOffset
        
    def __distance(self, x1, y1, x2, y2):
        return sqrt((x1-x2)**2 + (y1-y2)**2)
        
    def observFunc(self, states, t):
        x = states[self.statevars[0]][t]
        y = states[self.statevars[1]][t]
        o = states[self.statevars[2]][t]+self.orientationOffset
        
        observedWallDist = -self.environment.distance2wall(x, y, o)
        # TODO: call function distance2wall of env2D! JAN: I copied the code
      #  visionEndX = x + self.maxVal*cos(o)
      #  visionEndY = y + self.maxVal*sin(o)
      #  observedWallDist = self.maxVal
        # problem here!: if the position estimation overshoots to behind the 
        # wall, the distancesensor will see no more wall and return it's max
        # value
      #  for wall in self.environment.walls:
            #find intersect point of each wall with sensor vision line
      #      p = wall.touchingPoint([x, y], [visionEndX, visionEndY])
      #      if p!=None:
      #          dist = self.__distance(p[0], p[1], x, y)
       #         observedWallDist = min(observedWallDist, dist)
        return observedWallDist*self.params[self.paramName]
    
    def printObserFunc(self):
        print('{}(t) = dist(t) *'.format(self.name)+\
              ' {}   with {} = {}'.format(self.paramName, self.paramName, self.params[self.paramName])+\
        ' and \n dist(t) = min( \n\tdistance(wall1, robotPos(t))\n\t...\n\tdistance(wallN, robotPos(t)) \n)')
 
    def printObserEq(self, t:int):
        pass
    
    def printError(self, states:States, t:int):
        print('{}({}): {:.1f} = {} * {}({})'.format(self.name, t, self.values[t], self.paramName, 'dist', t) \
              +' => err = {:.2f} - {:.2f} * {:.2f} = {:.2f}'.format(self.values[t], self.params[self.paramName], self.observFunc(states, t)/self.params[self.paramName], self.error(states, t)))
                

### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Demo code Sensors2D.py ***')
    env = Environment2DSquare(100)
    sensor = UnidirectionalDistanceSensor("distSensor0", env, maxVal = 2000)
    variables = ['vx', 'vy', 'x', 'y']
    variables.extend(['vo', 'o'])
    variables.extend(['vxR', 'vyR'])
    variables.extend(['b'])
    
    relationDict ={'vx':'vxR', 'vy':'vyR', 'x': 'vx', 'y': 'vy', 'o': 'vo'}

    def forwardMath2D(states, t, dts):
        states.setValue('o', t, (states['vo'][t] * dts[t] + states['o'][t - 1])%(2*pi))
        if states['b'][t] == True:
            states.setValue('vxR', t, states['vx'][t]*cos(states['o'][t]) + states['vy'][t]*sin(states['o'][t]))
            states.setValue('vyR', t, -states['vx'][t]*sin(states['o'][t]) + states['vy'][t]*cos(states['o'][t]))
        # relative CS
        states.setValue('vx', t, states['vxR'][t]*cos(states['o'][t]) - states['vyR'][t]*sin(states['o'][t]))
        states.setValue('vy', t, states['vxR'][t]*sin(states['o'][t]) + states['vyR'][t]*cos(states['o'][t]))
        # absolute CS
        states.setValue('x', t, states['vx'][t] * dts[t] + states['x'][t - 1])
        states.setValue('y', t, states['vy'][t] * dts[t] + states['y'][t - 1])
        
         
    stateDef = StateDef(variables, relationDict, forwardMath2D)
    states = States(stateDef, {'vx':0, 'vy':0, 'vo':0, 'x':70, 'y':-30, 'o':pi/4, 'vxR':0, 'vyR':0, 'b':False})
    sensor.observFunc(states,0)
    sensor.printObserFunc()
    sensor.generateObservation(states)
    sensor.printError(states, 0)