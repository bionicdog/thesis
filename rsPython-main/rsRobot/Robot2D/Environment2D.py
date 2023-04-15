# -*- coding: utf-8 -*-
"""
Created on Thu Apr  7 17:08:21 2022

@author: woute
"""
import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code


from Environment import Environment, adjustAccAndDr
from State import States, StateDef
from math import atan2, pi, cos, sin, sqrt
from rsLibrary.Geometry2D import distance, angle, line3Coef, intersect

class Environment2D(Environment):
    """
    Base class to represent 2-dimensional environments, characterized by Walls 
    """
    def __init__(self, walls:list, posvars:tuple = ('x', 'y'), speedvars:tuple = ('vxR', 'vyR'), accvars:tuple = ('axR', 'ayR'), orvar:str = 'o'):
        self.walls = walls
        self.touchedWall = False
        self.posvars = posvars
        self.speedvars = speedvars
        self.accvars = accvars
        self.orvar = orvar
        self.maxVal = 1000 # TODO: length of diagonal
        
    def print(self):
        print('Walls at:')
        for wall in self.walls:
            print('\t', end='')
            print(wall)
            
    def __str__(self):
        string = 'Walls at:\n'
        for wall in self.walls:
            string+='\t'
            string+=str(wall)+'\n'
        return string

    def distance2wall(self, x, y, o) -> float:
        visionEndX = x + self.maxVal*cos(o)
        visionEndY = y + self.maxVal*sin(o)
        observedWallDist = self.maxVal
        # problem here!: if the position estimation overshoots to behind the 
        # wall, the distancesensor will see no more wall and return it's max
        # value
        for wall in self.walls:
            #find intersect point of each wall with sensor vision line
            p = wall.touchingPoint([x, y], [visionEndX, visionEndY])
            if p!=None:
                dist = distance(p[0], p[1], x, y)
                observedWallDist = min(observedWallDist, dist)
        if False and observedWallDist == self.maxVal:
            print('outside environment with x='+str(x)+' y='+str(y)+" o="+str(o))
         #   wall_dist = self.distance2wall(x, y, o)
         #   return wall_dist
        
        return observedWallDist
    
    def checkState(self, states: States, printit = False):
        self.touchedWall = False    #to signal if a wall is touched
        blockingWall = None         #to store the particular Wall object
        #use current position and previous position
        if len(states[self.posvars[0]])<2:
            print(len(states[self.posvars[0]]))
            return
        oldX = states[self.posvars[0]][-2]
        oldY = states[self.posvars[1]][-2]
        oldPos = (oldX, oldY)
        newPos = (states.value(self.posvars[0]), states.value(self.posvars[1]))
        self.corrections = {} # reset
        
        #check if a wall has been 'crossed'
        for wall in self.walls:
            if wall.touched(newPos, oldPos):
                self.touchedWall = True
                blockingWall = wall #TODO: multiple touched walls
        
        
        
        if self.touchedWall:
            # print("touch")
            #find the contact point with the wall
            pTouch = blockingWall.touchingPoint(newPos, oldPos)
           
            # if one of the pTouch coordinates is the same as the
            # corresponding oldPos coordinate, there can still be a very small 
            # difference 
            xDir = (pTouch[0]-oldPos[0])/abs(pTouch[0]-oldPos[0]) if not (abs(pTouch[0]-oldPos[0])<0.01) else 0
            yDir = (pTouch[1]-oldPos[1])/abs(pTouch[1]-oldPos[1]) if not (abs(pTouch[1]-oldPos[1])<0.01) else 0
           
           
            pTouch = blockingWall.touchingPoint(newPos, oldPos)
           
            # if one of the pTouch coordinates is the same as the
            # corresponding oldPos coordinate, there can still be a very small 
            # difference 
            xDir = (pTouch[0]-oldPos[0])/abs(pTouch[0]-oldPos[0]) if not (abs(pTouch[0]-oldPos[0])<0.01) else 0
            yDir = (pTouch[1]-oldPos[1])/abs(pTouch[1]-oldPos[1]) if not (abs(pTouch[1]-oldPos[1])<0.01) else 0
            
            
             #HINT: better way to find direction?
           #  if self.accvars is not None:
           #      # system with a, v, x
           #      # we want v[t] = zero and x[t] = pos_wall
           #      #  remark: this cannot by following the dep var equations, we have to force it
           #      # Assumption: constant acceleration in each frame
                
           #      # v[t] = a[t] + v[t-1] = 0 => a[t] = - v[t-1]            
           #      accX, accY = (- states[speedvar][-2] for speedvar in self.speedvars)
           #      states.setValue(self.accvars[0], newValue = accX)
           #      states.setValue(self.accvars[1], newValue = accY)
           #      #TODO: check if logical this way: both directions adjusted
           #      states.calcDepVars()
           #      # we force x and y to be at the wall
                
           #      current_value_pos0 = states.value(self.posvars[0])
           #      current_value_pos1 = states.value(self.posvars[1])
           #      states.setValue(self.posvars[0], newValue = pTouch[0]-xDir) 
           #      states.setValue(self.posvars[1], newValue = pTouch[1]-yDir) 
              
           #      self.corrections[self.posvars[0]] =  (states.now, current_value_pos0, states.value(self.posvars[0])) # to be reapplied later: see RobotEstimation.update
           #      self.corrections[self.posvars[1]] =  (states.now, current_value_pos1, states.value(self.posvars[1])) # to be reapplied later: see RobotEstimation.update
           #      if printit:
           #          print('Environment correction: '+self.accvars[0]+' set to '+str(accX)+'; '+self.posvars[0]+' from '+str(current_value_pos0)+' to '+str(states.value(self.posvars[0])))
           #          print('                    and '+self.accvars[1]+' set to '+str(accY)+'; '+self.posvars[1]+' from '+str(current_value_pos1)+' to '+str(states.value(self.posvars[1])))

           #  else:
           #      # system with only v and x
           #      # x[t] = x[t-1] + v[t] = pos_wall - dir   =>  v[t] = pos_wall - dir - x[t-1]
           #      # the -dir term is to ensure the point is not 'in' the wall
           #      speedX = pTouch[0]-xDir - states[self.posvars[0]][-2]
           #      speedY = pTouch[1]-yDir - states[self.posvars[1]][-2] 
           #      relSpeedX = speedX*cos(states[self.orvar][-1]) + speedY*sin(states[self.orvar][-1])
           #      relSpeedY = -speedX*sin(states[self.orvar][-1]) + speedY*cos(states[self.orvar][-1])
           #      states.setValue(self.speedvars[0], newValue = relSpeedX)
           #      states.setValue(self.speedvars[1], newValue = relSpeedY)
           #      states.calcDepVars() 
           #      if printit:
           #          print('Environment correction: '+self.speedvars[0]+' set to '+str(relSpeedX)+' and '+self.speedvars[1]+' set to '+str(relSpeedY))
 
            
            #HINT: better way to find direction?
            if self.accvars is not None:  # system with a, v, x
                # similar as 1D-code
                dt = states.dts[-1]
            
                aX_t = states[self.accvars[0]][-1]
                vX_t = states[self.speedvars[0]][-1] 
                xX_t = states[self.posvars[0]][-1] 
                vX_prev = states[self.speedvars[0]][-2] 
                xX_prev = states[self.posvars[0]][-2] 
                pos_wallX = pTouch[0]-xDir #Added -xDir to stop right before wall, not IN it
                aX_t_new, dtX, changeX = adjustAccAndDr(aX_t, vX_t, xX_t, vX_prev, xX_prev, dt, pos_wallX)

                aY_t = states[self.accvars[1]][-1]
                vY_t = states[self.speedvars[1]][-1] 
                xY_t = states[self.posvars[1]][-1] 
                vY_prev = states[self.speedvars[1]][-2] 
                xY_prev = states[self.posvars[1]][-2] 
                pos_wallY = pTouch[1]-yDir #Added -yDir to stop right before wall, not IN it
                aY_t_new, dtY, changeY = adjustAccAndDr(aY_t, vY_t, xY_t, vY_prev, xY_prev, dt, pos_wallY)
                
                change = changeX and changeY
                if change:
                    states.setValue(self.accvars[0], newValue = aX_t_new)
                    states.setValue(self.accvars[1], newValue = aY_t_new)
                    if abs(dtX - dtY) > 0.1:
                        print('Expecting dtX ('+str(dtX)+') and dtY ('+str(dtY)+') to be similar in Environment2D.checkState()...')
                    dt = min(dtX, dtY)
                    states.setDt(dt) 
                    
                    states.calcDepVars()                
                    if printit:
                        print('Environment correction: '+self.accvars[0]+' set to {:.3g} (from {:.3g}) and dt = {:.3g} '.format(aX_t_new, aX_t, dt)+ ' gives '+ self.posvars[0]+' = {:.3g}'.format(states[self.posvars[0]][-1])+' and '+self.speedvar+' = {:.3g}'.format(states[self.speedvars[0]][-1]))
                        print('                        '+self.accvars[1]+' set to {:.3g} (from {:.3g}) and dt = {:.3g} '.format(aY_t_new, aY_t, dt)+ ' gives '+ self.posvars[1]+' = {:.3g}'.format(states[self.posvars[1]][-1])+' and '+self.speedvar+' = {:.3g}'.format(states[self.speedvars[1]][-1]))
            
            
                # # we want v[t] = zero and x[t] = pos_wall
                # #  remark: this cannot by following the dep var equations, we have to force it
                # # Assumption: constant acceleration in each frame
                
                # # v[t] = a[t] + v[t-1] = 0 => a[t] = - v[t-1]            
                # accX, accY = (- states[speedvar][-2] for speedvar in self.speedvars)
                # states.setValue(self.accvars[0], newValue = accX)
                # states.setValue(self.accvars[1], newValue = accY)
                # #TODO: check if logical this way: both directions adjusted
                # states.calcDepVars()
                # # we force x and y to be at the wall
                
                # current_value_pos0 = states.value(self.posvars[0])
                # current_value_pos1 = states.value(self.posvars[1])
                # states.setValue(self.posvars[0], newValue = pTouch[0]-xDir) 
                # states.setValue(self.posvars[1], newValue = pTouch[1]-yDir) 
              
                # self.corrections[self.posvars[0]] =  (states.now, current_value_pos0, states.value(self.posvars[0])) # to be reapplied later: see RobotEstimation.update
                # self.corrections[self.posvars[1]] =  (states.now, current_value_pos1, states.value(self.posvars[1])) # to be reapplied later: see RobotEstimation.update
                # if printit:
                #     print('Environment correction: '+self.accvars[0]+' set to '+str(accX)+'; '+self.posvars[0]+' from '+str(current_value_pos0)+' to '+str(states.value(self.posvars[0])))
                #     print('                    and '+self.accvars[1]+' set to '+str(accY)+'; '+self.posvars[1]+' from '+str(current_value_pos1)+' to '+str(states.value(self.posvars[1])))

            else:
                # system with only v and x
                # x[t] = x[t-1] + v[t] = pos_wall - dir   =>  v[t] = pos_wall - dir - x[t-1]
                # the -dir term is to ensure the point is not 'in' the wall
                speedX = pTouch[0]-xDir - states[self.posvars[0]][-2]
                speedY = pTouch[1]-yDir - states[self.posvars[1]][-2] 
                relSpeedX = speedX*cos(states[self.orvar][-1]) + speedY*sin(states[self.orvar][-1])
                relSpeedY = -speedX*sin(states[self.orvar][-1]) + speedY*cos(states[self.orvar][-1])
                states.setValue(self.speedvars[0], newValue = relSpeedX)
                states.setValue(self.speedvars[1], newValue = relSpeedY)
                states.calcDepVars() 
                if printit:
                    print('Environment correction: '+self.speedvars[0]+' set to '+str(relSpeedX)+' and '+self.speedvars[1]+' set to '+str(relSpeedY))
 
        
    def fieldOfView(self, states):
        # point of view
        point = (states[self.posvars[0]][-1], states[self.posvars[1]][-1])
        #print(point)
        wallAngles = []
        for wall in self.walls:
            # return angles with respect to the relative CS 
            wallAngles.append(wall.fieldOfViewAngles(point, 0))
        return wallAngles
    def rangeOfVariable(self, variable:str) -> tuple: 
        raise NotImplementedError


class Environment2DSquare(Environment2D):
    """
      Simple square 2D environment: walls are located at
      x = wallDist, x = -wallDist, y=wallDist and y = -wallDist
    """
    def __init__(self, wallDist: int = 100, posvars:tuple = ('x', 'y'), speedvars:tuple = ('vxR', 'vyR'), accvars:tuple = ('axR', 'ayR'), orvar:str = 'o'):
        super().__init__(self.__generateWalls(wallDist), posvars, speedvars, accvars, orvar)
        self.wallDist = wallDist
        
    def __generateWalls(self, wallDist:int):
        """
        Function to generate two vertical and two horizontal walls.
        """
        walls = []
        #vertical walls
        for x in [-1, 1]:
            wall = Wall(x*wallDist, -wallDist, x*wallDist, wallDist)
            walls.append(wall)
        #horizontal walls
        for y in [-1, 1]:
            wall = Wall(-wallDist, y*wallDist, wallDist, y*wallDist)
            walls.append(wall)
        return walls
    
    def rangeOfVariable(self, variable:str) -> tuple: # optional
        MIN_DIST_TO_WALL = 0.1
        print('test')
        return [-self.wallDist + MIN_DIST_TO_WALL, self.wallDist - MIN_DIST_TO_WALL] if variable == self.posvars[0] or variable == self.posvars[1] else None

        

class Wall:
    """
    Representation of a wall in a 2D environment. Defined as a line between
    two points A and B.
    """
    def __init__(self, x1, y1, x2, y2):
        self.pointA = (x1, y1)
        self.pointB = (x2, y2)
        
    def __str__(self):
        return 'Wall from '+str(self.pointA)+' to '+str(self.pointB)
        
    def touched(self, pointC, pointD):
        """
        checks the intersection of the wall with a line segment from 
        pointC to pointD.
        returns a boolean
        """
        return intersect(self.pointA, self.pointB, pointC, pointD)
    
    def touchingPoint(self, pointC, pointD):
        if not self.touched(pointC, pointD): return None
        #Cramer's rule
        L1 = line3Coef(self.pointA, self.pointB)
        L2 = line3Coef(pointC, pointD)
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        x = Dx / D
        y = Dy / D
        return x,y
    
    def fieldOfViewAngles(self, point, offset):
        """
        calculates the two angles of the field of view from a given point
        to the Wall object, in the absolute CS
        """
        # lines to the edges of the wall
        line1 = line3Coef(point, self.pointA)
        #line1 = self.__line3Coef((0, 0), (-1, 1))
        line2 = line3Coef(point, self.pointB)
        # the x-axis
        xLine = line3Coef((0, 0), (1,0))
        angleA = angle(xLine, line1)
        angleB = angle(xLine, line2)
        return (angleA-offset)%(2*pi), (angleB-offset)%(2*pi)
    
    def perpendicularIntersectPoint(self, point):
        """
        https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
        """
        Ax, Ay = self.pointA
        Bx, By = self.pointB
        Cx, Cy = point
        t=((Cx-Ax)*(Bx-Ax)+(Cy-Ay)*(By-Ay))/((Bx-Ax)**2+(By-Ay)**2)
        Dx=Ax+t*(Bx-Ax)
        Dy=Ay+t*(By-Ay)
        return (Dx, Dy)
    
    def angleOfPerpendicularIntersectPoint(self, point, offset):
        D = self.perpendicularIntersectPoint(point)
        xLine = line3Coef((0, 0), (1,0))
        CD = line3Coef(point, D)
        return (angle(xLine, CD)-offset)%(2*pi)
        
        
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ###        
if __name__== "__main__":
    from rsLibrary.Monitor import gMonitorT, createMonitorT
    
    print('*** Demo code Environment2D.py ***')
    
    FLAG_ACCELERATION = True
    
    if FLAG_ACCELERATION:
        env = Environment2DSquare(wallDist = 80, speedvars = ('vx', 'vy'), accvars = ('ax', 'ay'), orvar = 'o')
        env.print()
        
        # 2D-world + acceleration
        variables = ['ax', 'ay', 'vx', 'vy', 'x', 'y', 'o'] 
        relationDict ={'vx': 'ax', 'vy': 'ay','x': 'vx', 'y': 'vy'}
        
        #With acceleration variables in both directions
        def forwardMath2D(states, t, dts):
            states.setValue('vx', t, states['ax'][t] * dts[t] + states['vx'][t - 1]) 
            states.setValue('vy', t, states['ay'][t] * dts[t] + states['vy'][t - 1]) 
            states.setValue('x', t, (states['vx'][t-1] + states['vx'][t]) / 2 * dts[t] + states['x'][t - 1])
            states.setValue('y', t, (states['vy'][t-1] + states['vy'][t]) / 2 * dts[t] + states['y'][t - 1])
            
            
        stateDef = StateDef(variables, relationDict, forwardMath2D)    
    
        # state evolution    
        states = States(stateDef, {'ax':0, 'ay':0, 'vx':0, 'vy':0, 'x':0, 'y':0, 'o':0})
        #FIXME: initial states don't show on the monitor
        
        # global observer
        createMonitorT(variables).printTitle()
        
        # velocity evolution (dependent variable)
        accelerationX = [20, 0, 0, 20, 0, -20, -20, -60, 0, 0]
        accelerationY = [10, 5, 4, -19, 20, 0, 0, 0, 0, 0]
        #FIXME: faulty behavior when touching multiple walls, see checkState
    
        gMonitorT().printLastFrame()
        for i in range(0, len(accelerationX)):
            t = i + 1
            states.addFrame()
            states.setValue( 'ax', t, accelerationX[i])
            states.setValue( 'ay', t, accelerationY[i])
            states.calcDepVars(t)
            env.checkState(states, printit = False)
            gMonitorT().printLastFrame()
            
    
    if not FLAG_ACCELERATION:
        env = Environment2DSquare(wallDist = 80, speedvars = ('vx', 'vy'), accvars = None)
        env.print()
        print("Field of view: ", [x*180/pi for wall in env.walls for x in wall.fieldOfViewAngles((0,0), 0)])
        #comment test Jarne
        # 2D-world (x and y), no acceleration variables
        variables = ['vx', 'vy', 'x', 'y', 'o'] 
        relationDict ={'x': 'vx', 'y': 'vy'}
        
        #No acceleration, vx/vy -> independent variables
        def forwardMath2D(states, t, dts):
            states.setValue('x', t, states['vx'][t] * dts[t] + states['x'][t - 1])
            states.setValue('y', t, states['vy'][t] * dts[t] + states['y'][t - 1])
            
            
        stateDef = StateDef(variables, relationDict, forwardMath2D)    
    
        # state evolution    
        states = States(stateDef, {'vx':0, 'vy':0, 'x':0, 'y':0, 'o':0})
        #FIXME: initial states don't show on the monitor
        
        # global observer
        createMonitorT(variables).printTitle()
        
        # velocity evolution (dependent variable)
        velocityX = [-20, -15, -40, -20, 12, -20, -20, 0, 36, 0]
        velocityY = [60, 2, -8, 9, 32, -2, 0, 5, -10, 120]
        #FIXME: faulty behavior when touching multiple walls, see checkState
    
        gMonitorT().printLastFrame()
        for i in range(0, len(velocityX)):
            #print(env.fieldOfView())
            t = i + 1
            states.addFrame()
            states.setValue( 'vx', t, velocityX[i])
            states.setValue( 'vy', t, velocityY[i])
            states.calcDepVars(t)
            env.checkState(states)
            gMonitorT().printLastFrame()
    