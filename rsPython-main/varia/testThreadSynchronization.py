# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 20:41:34 2020

@author: Jan Lemeire
"""
import threading

cv = threading.Condition()
msg = ''
def waitForReady(readyMessage = 'READY'):
    global cv, msg
    msg = ''
    with cv:
        while msg != readyMessage:
            cv.wait()
                
def doAction():
    waitForReady()
    print('THREAD: Next step')
    waitForReady()
    print('THREAD: Step 2')
    waitForReady()
    print('THREAD: Finished')
    
other_thread = threading.Thread(target = doAction, args = [])
other_thread.start()
        
while(True):
    _msg = input('say something, READY if ready, q to quit: ')
    if _msg == 'q':
        break
    with cv: # protects critical section by locking the section
        msg = _msg
        cv.notifyAll()
        
other_thread.join()