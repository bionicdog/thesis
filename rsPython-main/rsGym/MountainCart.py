# -*- coding: utf-8 -*-
"""
Created on Tue Oct  4 10:39:50 2022

@author: Nick
"""

import gym
import time 
import matplotlib.pyplot as plt 
from sklearn import tree
from rsQualitativeModel.QualitativeState import QController, QBehaviourVector
from rsRobot.State import States, StateDef
from rsQualitativeModel.QStateClassifier import train_qualitative_tree
env = gym.make('MountainCar-v0')

# Observation and action space 
obs_space = env.observation_space
action_space = env.action_space
print("The observation space: {}".format(obs_space))
print("The action space: {}".format(action_space))

# reset the environment and see the initial observation
obs = env.reset()
print("The initial observation is {}".format(obs))

# Number of steps you run the agent for 
num_steps = 1500

obs = env.reset()

print(env.action_space)

# Create QController 
actuatorVars = ["motor"]
QBehaviourVector.reset()
controller = QController(actuatorVars)

#Define states
def forward(states, t, dts):
    pass
variables = ['x', 'v']
stateDef = StateDef(variables, {}, forward)
states = States(stateDef, {'x':obs[0], 'v':obs[1]})

#Hardcoded policy

#
start_exploitation = num_steps*0.8
exploration=True
clf = class_names = feature_names = None
for step in range(num_steps):
    # take random action, but you can also do something more intelligent
    # action = my_intelligent_agent_fn(obs) 
    if exploration==True:
        action = env.action_space.sample()
    else:
        pass
        # tree.plot_tree(clf, class_names=class_names, feature_names=feature_names)
        # plt.show()
        # break
        
    # print(action-1)
    
    # apply the action
    obs, reward, done, info = env.step(action)
    
    # add new state frame
    states.addFrame()
    # print({var:val for var, val in zip(variables, obs)})
    states.setValues({var:val for var, val in zip(variables, obs)}, -1)
    
    #sample qualitative behaviour
    controller.sampleBehaviour(states, ("motor", action-1))
    
    # Render the env
    # env.render()

    # Wait a bit before the next frame unless you want to see a crazy fast video
    time.sleep(0.001)
    if start_exploitation==step:
        clf, class_names, feature_names = train_qualitative_tree('motor', controller.actuatorBehaviourMapping, controller.behaviourToStateActionMapping)
        exploration = False
    # If the epsiode is up, then start another one
    if done:
        env.reset()

# Close the env
env.close()

print(controller.behaviourToStateActionMapping)
print(QBehaviourVector.knownQVectors)