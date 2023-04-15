# -*- coding: utf-8 -*-
"""
Created on Wed Jul 13 16:17:14 2022

@author: Nick
"""

import sys

from rsRobot.Robot2D.Environment2D import Environment2D, Wall
from rsRobot.State import States, StateDef
from rsLibrary.Geometry2D import line3Coef, distance, angle
from rsLibrary.Monitor import gMonitorT, createMonitorT

class Environment2DWithObstacles(Environment2D):
    """
    Represent a 2D environment with walls and obstacles in simulation. This
    will result in multiple possible situations, which the robot has to learn.
    """
    def __init__(self, walls:list, objects:list, posvars:tuple = ('x', 'y'),
                 speedvars:tuple = ('vxR', 'vyR'), accvars:tuple = ('axR', 'ayR'),
                 orvar:str = 'o', gripvar:str = 'g', gripForceVar:str = 'p', proxVar:str = 'prox'):
        super().__init__(walls, posvars, speedvars, accvars, orvar)
        self.gripvar = gripvar
        self.gripForceVar = gripForceVar
        self.proxVar = proxVar
        self.objects = objects
        self.touchedObject = False
        self.goalRegion = None #TODO
        self.situations = None #TODO
        
    #TODO
    def getSituation(self, states):
        driving = True
        if states[self.speedvars[0]][-1]<=0.01 and states[self.speedvars[1]][-1]<=0.01:
            driving = False
        situationConditionList = [self.touchedObject, self.touchedWall, driving]
        situationIdx = 0
        for i, cond in enumerate(situationConditionList):
            situationIdx+=2**i*cond
        return situationIdx
    
    def __str__(self):
        string = super().__str__()
        for obj in self.objects:
            string+='\t'
            string+=str(obj)+'\n'
        return string
    
    def checkState(self, states: States, printit = False):
        #call checkState from Environment2D
        super().checkState(states, printit)
        self.touchedObject=False
        oldX = states[self.posvars[0]][-2]
        oldY = states[self.posvars[1]][-2]
        oldPos = (oldX, oldY)
        newPos = (states.value(self.posvars[0]), states.value(self.posvars[1]))
        #check for collision with objects
        for o in self.objects:
            if o.touched(newPos, oldPos):
                self.touchedObject=True
                pTouch = (o.x, o.y)
                xDir = (pTouch[0]-oldPos[0])/abs(pTouch[0]-oldPos[0]) if not (abs(pTouch[0]-oldPos[0])<0.01) else 0
                yDir = (pTouch[1]-oldPos[1])/abs(pTouch[1]-oldPos[1]) if not (abs(pTouch[1]-oldPos[1])<0.01) else 0
                #object is pushed to new position
                o.x = newPos[0]+xDir
                o.y = newPos[1]+yDir
                # print('BOTS')
                #TODO: take into account mass -> robot will slow down
        # first check if object is gripped, then move it, eventually update proximity
        self.checkGripState(states)
        self.moveGrippedObjects(states)
        self.checkProximityState(states)
        
    def checkProximityState(self, states, proxThreshold=2):
        minDist = min([distance(o.x, o.y, states[self.posvars[0]][-1], states[self.posvars[1]][-1]) for o in self.objects])
        #a bit of cheating on the proximity event sensor
        if minDist<proxThreshold:
            states.setValue(self.proxVar, newValue=1) #set eventsensor high
        else:
            states.setValue(self.proxVar, newValue=0) #set low
                
    def checkGripState(self, states, gripToleranceAngle=0.05, gripMoveThreshold=10, gripOpeningThreshold=25, proxThreshold=2):
        """
        sets MovableObject.gripped to true if gripped
        """
        # if the gripper made a grip movement...
        gripped = states[self.gripvar][-2] - states[self.gripvar][-1]>gripMoveThreshold
        # and the final position is closed enough
        closed_enough = states[self.gripvar][-1]<gripOpeningThreshold
        for o in self.objects:
            dist = distance(o.x, o.y, states[self.posvars[0]][-1], states[self.posvars[1]][-1])
            if gripped and closed_enough:
                if self.orvar is not None:
                    #line of robot to object
                    dirLine = line3Coef((states[self.posvars[0]][-1], states[self.posvars[1]][-1]),
                                        (o.x, o.y))
                    #orientation relative to horizontal robot-axis
                    x_axis = line3Coef((0, states[self.posvars[1]][-1]), 
                                       (100, states[self.posvars[1]][-1]))
                    objectDir = angle(x_axis, dirLine)
                    if abs(states[self.orvar][-1]-objectDir)>gripToleranceAngle:
                        continue #this object is not in range, go to next object
                if dist<=proxThreshold:
                    o.gripped=True
                    print('Grip')
            elif not closed_enough and o.gripped:
                #previously gripped object is dropped when gripper is open
                o.gripped=False
                print('drop')
    
    def moveGrippedObjects(self, states):
        objectGripped=False
        for o in self.objects:
            if o.gripped:
                o.x = states[self.posvars[0]][-1]
                o.y = states[self.posvars[1]][-1]
                #the robot can 'feel' the exerted force
                newValue=25-states[self.gripvar][-1]
                states.setValue(self.gripForceVar, newValue=newValue) #proportional to gripper value
                objectGripped=True
        if not objectGripped:
            states.setValue(self.gripForceVar, newValue=0) #reset to 0
                    
                    
            
        
class MoveableObject():
    def __init__(self, x, y, weight):
        self.x = x
        self.y = y
        self.weight = weight
        self.gripped = False
        
    def __str__(self):
        return 'MoveableObject at x='+str(self.x)+', y='+str(self.y) + ', weight='+str(self.weight)
      
    def touched(self, pointOld, pointNew):
        """
        returns true if the object lies on the trajectory of the line segment 
        from pointOld to pointNew
        """
        if pointOld==pointNew:
            return False
        A, B, C = line3Coef(pointOld, pointNew)
        xInBounds=min(pointOld[0], pointNew[0])<=self.x<=max(pointOld[0], pointNew[0])
        yInBounds=min(pointOld[1], pointNew[1])<=self.y<=max(pointOld[1], pointNew[1])
        return A*self.x + B*self.y + C <= 0.01 and xInBounds and yInBounds
    
    
def createSquareEnvironmentWithRandomObstacles(wallDist:int, nrObjects:int):
    wallDist = 100
    walls = []
    #create four walls
    for x in [-1, 1]:
        wall = Wall(x*wallDist, -wallDist, x*wallDist, wallDist)
        walls.append(wall)
    for y in [-1, 1]:
        wall = Wall(-wallDist, y*wallDist, wallDist, y*wallDist)
        walls.append(wall)
    #create random objects
    import random
    objects = []
    for i in range(nrObjects):
        x = random.randint(-wallDist, wallDist)
        y = random.randint(-wallDist, wallDist)
        objects.append(MoveableObject(x, y, 0))
    return Environment2DWithObstacles(walls, objects, speedvars=('vx', 'vy'), accvars=None)
        
    
    


if __name__ == "__main__":
    
    print('*** Demo code EnvironmentWithObstacles.py ***')
    
    wallDist = 100
    env = createSquareEnvironmentWithRandomObstacles(wallDist, 2)
    env.objects[0].x = 45
    env.objects[0].y = 45

    print(env)
    
    pointA = (48, 48)
    pointB = (52, 52)
    for o in env.objects:
        print(o.touched(pointA, pointB))
        
        
    # 2D-world (x and y), no acceleration variables
    variables = ['vx', 'vy', 'x', 'y', 'o', 'g', 'p', 'prox'] 
    relationDict ={'x': 'vx', 'y': 'vy'}
    
    #No acceleration, vx/vy -> independent variables
    def forwardMath2D(states, t, dts):
        states.setValue('x', t, states['vx'][t] * dts[t] + states['x'][t - 1])
        states.setValue('y', t, states['vy'][t] * dts[t] + states['y'][t - 1])
        
        
    stateDef = StateDef(variables, relationDict, forwardMath2D)    

    # state evolution    
    states = States(stateDef, {'vx':0, 'vy':0, 'x':0, 'y':0, 'o':0, 'g':0, 'p':0, 'prox':0})
    #FIXME: initial states don't show on the monitor
    
    # global observer
    createMonitorT(variables).printTitle()
    
    # velocity evolution (dependent variable)
    velocityX = [40, 15, 5, 10, 15, -5, -10, 39, 39]
    velocityY = [40, 15, 10, 10, 15, -5, 1, 39, 39]
    #FIXME: faulty behavior when touching multiple walls, see checkState
    g = [130, 15, 10, 10, 15, 80, 50, 39, 39]
    o = [0.78, 0.74, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    gMonitorT().printLastFrame()
    for i in range(0, len(velocityX)):
        t = i + 1
        states.addFrame()
        states.setValue( 'vx', t, velocityX[i])
        states.setValue( 'vy', t, velocityY[i])
        states.setValue( 'g', t, g[i])
        states.setValue( 'o', t, o[i])
        states.calcDepVars(t)
        env.checkState(states)
        # env.checkGripState(states)
        # env.moveGrippedObjects(states)
        gMonitorT().printLastFrame()
        print("Situation ID =", env.getSituation(states))
        
    print(env)
        
    
    