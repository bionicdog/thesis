# -*- coding: utf-8 -*-
"""
Created on Wed Aug 24 09:59:04 2022

@author: Nick
"""

import sys
if '../..' not in sys.path:
    sys.path.append('..')
    sys.path.append('../..')
    
import random

from rsRobot.State import States, StateDef
from rsRobot.Robot2D.Robot2D import createSimRobot2DSquareOnlyVelocity
from rsQualitativeModel.QBehaviorModel import QBehaviorModel
from gym.spaces import Space, Box
from numpy import sign
import numpy as np

class ExplorationAlg():
    def __init__(self, possibleActionDict:dict[str, Space]):
        """
        possibleActionDict{key = actionID : value = [ActionSpace] 
        Note that the value corresponds to an openAI gym Space object
        """
        self.possibleActionDict = possibleActionDict
            
    def exploreAction(self) -> dict:
        """
        Returns an exploratory action
        """
        return NotImplementedError()
    
    
class RandomExploration(ExplorationAlg):
    def __init__(self, possibleActionDict):
        super().__init__(possibleActionDict)
        
    def exploreAction(self) -> dict: 
        actionVar = random.choice([var for var in self.possibleActionDict])
        actionVal = self.possibleActionDict[actionVar].sample()[0] #method from gym.Space
        return actionVal
    
class DirectedExploration(ExplorationAlg):
    def __init__(self, possibleActionDict, qBehaviorModel:QBehaviorModel):
        """
        qBehavior: Forward model of qualitative behavior s_t+1 = f(s_t, a_t)
        Select action from possibleActionDict based on the qBehavior that
        directs to the goalState
        """
        super().__init__(possibleActionDict)
        self.qBehaviorModel=qBehaviorModel
        if [key for key in possibleActionDict]!=self.qBehaviorModel.actionVars:
            print("Mismatch between actions and qBehaviorModel!")
    
    def exploreAction(self, goalState:list, currentState:list, actionVar):
        """
        Select action to go from goalState to currentState for a single actionVar,
        based on qBehaviorModel
        """
        goalDeltas = sign(goalState - currentState)
        possibleActions = [key for key in self.possibleActionDict]
        #save bounds for restoring later
        old_low = possibleActions.low
        old_high = possibleActions.high
        for var in self.goalState:
            #combine information about qualitative direction (s_del) and
            #current qualitative behaviour
            s_del = sign(self.goalState[var]-qBehaviour.states[var][-1])
            s_act = qBehaviour.getStateDelta(var)*s_del
            #constrain possibleActions, dependent of goal direction
            if s_act>-1:
                possibleActions.low = np.asarray([0.])
            if s_act<1:
                possibleActions.high = np.asarray([0.])
        #sample an action with the right qualitative properties
        act_val = possibleActions.sample()[0]
        #restore bounds
        possibleActions.low = old_low
        possibleActions.high = old_high
        return act_val
    
    def exploreStep(self): #herorden dit in explore
        validActions = []
        for action in self.possibleActionDict:
            qBehaviour = self.qBehaviours[action]
            for var in self.goalState:
                #if action influences var, we can use it to manipulate var
                if qBehaviour.getStateDelta(var)!=0:
                    validActions.append(action)
        #choose random action from validActions to get closer to goalState
        action = random.choice(validActions)
        action_val = self.exploreAction(action)
        return {action:action_val}
        
    def explore(self, states, actuatorFunc, observFunc=None, steps=10):
        """
        actuatorFunc: executes the action when called
        """
        goalReached = False
        while not goalReached and steps>0:
            #take an exploration step
            actionDict = self.exploreStep()
            action = list(actionDict.keys())[0]
            action_val = actionDict[action]
            actuatorFunc(actionDict) #this executes the action!
            #save known QVectors to compare
            known = QBehaviourVector.knownQVectors.copy()
            currentQBehaviour = QBehaviourVector(states, (action, action_val))
            print(QBehaviourVector.knownQVectors)
            #check if behaviour has changed, to correctly select actions
            if currentQBehaviour!=self.qBehaviours[action]:
                self.qBehaviours[action] = currentQBehaviour
            #if a new one is found, explore more (currently at random)
            if currentQBehaviour not in known:
                print("NEW")
                pass
                #explore more here!
            if observFunc is not None:
                print(observFunc())
            steps-=1
            
            # select best action
            # when new qBehaviour is found, sample extra
            # return {action : self.exploreAction(action) for action in self.possibleActionDict}

class QExploration():
    def __init__(self, behaviorModel:QBehaviorModel, explorationAlg:ExplorationAlg=None):
        self.behaviorModel = behaviorModel
        self.explorationAlg = explorationAlg
        self.priorityQueue = []
    
    def nextAction(self):
        if self.priorityQueue:
            return self.priorityQueue.pop(0)
        else:
            return explorationAlg.exploreAction()
    
    # def verifyDepList(self, depList):
    #     # Which states do we want to explore?
    #     conflictPoints = self.behaviorModel.
    #     # Exploratory action
    #     action = self.explorationAlg.exploreAction()
        
    #     # Check for conflict
        
    #         # 
    #     pass
    
    def constructHyperMeshToExplore(self, pointList:list, varList:list) -> list:
        pointsToExplore = []
        for v_idx in range(len(varList)):
            for p_idx in range(len(pointList)-1):
                if pointList[p_idx][v_idx]!=pointList[p_idx+1][v_idx]:
                    # Change only one var to the value of next point in list
                    meshPoint = pointList[p_idx].copy()
                    meshPoint[v_idx] = pointList[p_idx+1][v_idx]
                    pointsToExplore.append(meshPoint)
        return pointsToExplore
            
    
if __name__ == "__main__":
    robot = createSimRobot2DSquareOnlyVelocity()
    print(robot.states)
    print(robot.motors[0].name)
    motTest = robot.motors[0]
    
    print(robot.stateDef.variables)
    
    testFlag=1
    if testFlag==0:
        explorationAlg = RandomExploration({motTest.name : Box(low=-100, high=100, shape=(1,), seed=2022)})
        for i in range(10):
            print(explorationAlg.exploreAction())
            
    if testFlag==1:
        # State and action variables from robot
        stateVars = ["x", "y", "vx", "vy"]
        actionVars = ["moVx", "moVy"]
        
        # QBehavior based on history
        hist = [[0, 0, 0, 10, 0, 100], [0, 10, 20, 0, 200, 0], [20, 10, 10, 10, 100, 100], [30, 20, 0, 0, 0, 0]]
        behavior = QBehaviorModel(stateVars, actionVars)
        behavior.globalHistory = hist
        actionDict = {motor.name: Box(low=-100, high=100, shape=(1,), seed=2022) for motor in robot.motors}
        explorationAlg = RandomExploration(actionDict)
        explorer = QExploration(behavior, explorationAlg)
        # print(explorer.__dict__)
        pointList = [[80, 20], [100, 60], [40, 30]]
        print(explorer.constructHyperMeshToExplore(pointList, ['x', 'y']))
        
    elif testFlag==2:
        pass
        
        
        