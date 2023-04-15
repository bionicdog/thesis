# -*- coding: utf-8 -*-
"""
Created on Wed Aug 24 13:46:39 2022

@author: Nick
"""

import gym
import time 
import matplotlib.pyplot as plt 
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

for step in range(num_steps):
    # take random action, but you can also do something more intelligent
    # action = my_intelligent_agent_fn(obs) 
    action = env.action_space.sample()
    
    # apply the action
    obs, reward, done, info = env.step(action)
    
    # Render the env
    env.render()

    # Wait a bit before the next frame unless you want to see a crazy fast video
    time.sleep(0.001)
    
    # If the epsiode is up, then start another one
    if done:
        env.reset()

# Close the env
env.close()