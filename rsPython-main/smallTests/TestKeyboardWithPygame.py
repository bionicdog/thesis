# -*- coding: utf-8 -*-
"""
Created on Apr 22 2023

@author: Jan Lemeire
"""

import pygame

#### #### #### ####  a Pygame window should be created #### #### #### #### 


clock = pygame.time.Clock()

def waitForKey():# -> key:
    key = None
    while key == None:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # top right X of the window
                return None
            if event.type == pygame.KEYDOWN:
                print(pygame.key.name(event.key))
                print(type(event))
                print(type(event.key))
                key = event.key
    return key

pygame.init()

window = pygame.display.set_mode((300, 300))
k = waitForKey()

pygame.quit()