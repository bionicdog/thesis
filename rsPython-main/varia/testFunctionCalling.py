# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 13:44:48 2020

@author: Jan Lemeire

"""

from threading import Thread

  
def handleKeyboardInput(handleMessage=None):
    while True:
        key = input('Give a text or "q" to quit')
        if callable(handleMessage):
            handleMessage(key)
        if key == 'q':
            break

def printMsg(msg):
    print('Message: '+msg)
       
listen_thread = Thread(target = handleKeyboardInput, args = [printMsg] )
listen_thread.start()
print('Waiting for thread to finish')
listen_thread.join()