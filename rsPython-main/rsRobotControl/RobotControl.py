#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobotControl.py

template for creating the main robot control software

@author: Jan Lemeire & Nikolai Devolder from Robotic Sensing lab
Created: May - October 2021
"""

# Python libraries
import datetime
import threading
from enum import Enum

# our libraries
import serial2arduino
import UserPreferences


class Mode(Enum):
    MANUAL_DRIVING = 1
    AUTOMATIC = 2
    ACTIONS = 3
    
###### global variables ######
gMode = Mode.MANUAL


def resetEverything():
    global gMode
    gMode = Mode.MANUAL
    
    
# define Motors that returns robotAction into motor input

#### #### #### ####  FUNCTIONS #### #### #### #### 

###### ACTIONS ######
char2action = {}


def showHelp():
    global char2action
    print('Possible Actions:')
    print(str(char2action))
    
def doAction(charCode):
    action = char2action.get(charCode, None)
    if action == None:
        print('Non-existing action code: '+str(charCode))
        showHelp()
        return False
    else:
        action()
        return True
def doActions():
    global cv, selectedThing, things
    cv = threading.Condition()
    
    gotoThing()
    waitForReady('finished')
    grabThing()
    waitForReady('finished')
    moveThing()
    waitForReady('finished')
    openClaw()
    waitForReady('finished')
    goHome()
    print('Finished do actions')
    cv = None
    
def doSequence():
    seq_thread = threading.Thread(target = doActions)
    seq_thread.start()
    
char2action['h'] = showHelp
char2action['x'] = doSequence

#char2action['u'] = extractBlueU
#char2action['a'] = detectAllThings
#char2action['d'] = detectThing # 'k' voor zwart
# select thing: geef een nummer
#char2action['c'] = selectObjectAndTarget

#char2action['t'] = gotoThing
#char2action['g'] = grabThing
#char2action['m'] = moveThing
#char2action['o'] = openClaw

#char2action['p'] = goHome1
#char2action['z'] = goHome2


###### END OF ACTIONS ######    

cv = None # condition variable to synchronize threads
arduinoMessage = ''
def messageFromArduino(msg):
    global cv, arduinoMessage
    print('message From Arduino: '+str(msg))
    if cv is not None:
        with cv: # protects critical section by locking the section
            arduinoMessage = msg
            cv.notifyAll()
    else:
        arduinoMessage = msg

# call this function to wait for a specific message from arduino
def waitForReady(readyMessage = 'READY'):
    global cv, arduinoMessage
    arduinoMessage = ''
    with cv:
        while not(arduinoMessage in readyMessage):
            cv.wait()
            
#### #### #### ####  INITIALIZATION #### #### #### #### 

# initialize sensors etc here

#### #### #### ####  THREAD CREATION #### #### #### #### 
serial2arduino.initSerial( handleMessage=messageFromArduino)

# reading sensors & reflexes (e.g. wall check)

# automatic driving will also be in seperate thread


#### #### #### ####  MAIN LOOP #### #### #### #### 

ctr = 1
i = 0
while(True):
    i+=1

    key = cv2.waitKey() # 1000
#    if key > -1:
#        print("Key  "+str(key))
    key = key & 0xFF
    key_char = chr(key)
    if key == 32 and (USE_PICAM or USE_CAMERA): # space bar
        #print('next frame')
        if USE_PICAM:
            rawCapture = PiRGBArray(camera)
            camera.capture(rawCapture, format="rgb")
            img_rgb = rawCapture.array
            frame = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        elif USE_CAMERA:
            ret, frame = cap.read()  # usb-camera
        height, width = frame.shape[:2]
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if key == 32 or key == ord('z'): # reset
        resetEverything()
    
    if key == ord('q'): # quit
        break
        
     # Our operations on the frame come here
    elif key_char in char2action.keys():
        #print("Action Key  "+str(key))
        doAction(chr(key))
            

    elif key >= ord('0') and key <= ord('9'):
        digit = key - ord('0')
        print('Pressed digit '+str(digit))
        
        
    elif key == ord('s'):
        
        _now = datetime.datetime.now()
        # str(time.time())+
       # file_name = 'finishfotos/test.jpg'  #  '+ str(_now) +'
        #file_name = 'finishfotos/test'+str(ctr)+'_exposure'+str(exposure)+'.jpg'  #  '+ str(_now) +'
        file_name = 'pictures/image_'+str(_now)[0:16]+'_'+str(ctr)+'.jpg'  #  '+ str(_now) +'
        file_name = file_name.replace(':', '_')
        file_name = file_name.replace(' ', '_')
        ctr+=1
        
        #file_name = ImageUtils.selectNewFile()
        #print('saving to '+file_name)
        cv2.imwrite(file_name, frame)
        print('Saved to '+file_name)
        UserPreferences.setUserPref('currentImageFile', file_name)
    
    # Display the resulting frame
    cv2.imshow(WINDOW_NAME,img_copy)


#### #### #### ####  TERMINATION #### #### #### #### 
serial2arduino.continueListening = False
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ### #### #### ###