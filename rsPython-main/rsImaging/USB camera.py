# -*- coding: utf-8 -*-
"""
USB camera.py: view the camera images and save them

@author: Jan Lemeire
Created on Wed Feb 26 11:58:47 2020

"""
import datetime
import time
import numpy as np
import cv2

def dateTimeString(): # date & time format: '2020-12-22_15.38'
    return str(datetime.datetime.now())[0:16].replace(':', '.').replace(' ', '_')

cap = cv2.VideoCapture(1)  # Camera ID (0 is internal camera)
ctr = 1
i = 0
#prop = cap.get(cv2.CAP_PROP_FPS ) #  )
#print( 'CAP_PROP_FPS = ' + str(prop) +' type = ' +str(type(prop) ))
#for i in range(0, 20):
#    prop = cap.get(i)
#    print('prop ' +str(i)+': '+str(prop)+' type = ' +str(type(prop)))

# supports 640.0 x 480   1280 x 720   1920 x 1080
width = 1920
height = 1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH , width ) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT , height ) 

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH  ) 
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT  )
print('images '+str(width) + ' x ' + str(height) + ' = '+str(width*height))

prop = cap.get(cv2.CAP_PROP_EXPOSURE ) 
print( 'CAP_PROP_EXPOSURE = ' + str(prop) +' type = ' +str(type(prop)))


gain_prev = cap.get(cv2.CAP_PROP_GAIN ) 
ret_val = cap.set(cv2.CAP_PROP_GAIN, 20) 
gain = cap.get(cv2.CAP_PROP_GAIN ) 
print( 'CAP_PROP_GAIN '+str(gain_prev)+ ' => ' + str(gain) +' type = ' +str(type(gain)) + ' return = '+str(ret_val) )


#ret_val = cap.set(cv2.CAP_PROP_EXPOSURE, -10) 
exposure = cap.get(cv2.CAP_PROP_EXPOSURE ) 
print( 'CAP_PROP_EXPOSURE = ' + str(exposure) +' type = ' +str(type(exposure)) + ' return = '+str(ret_val) )

# CHECK THIS: https://stackoverflow.com/questions/45713327/is-there-a-way-to-adjust-shutter-speed-or-exposure-time-of-a-webcam-using-python
# exposure = cap.get(cv2.cv.CV_CAP_PROP_EXPOSURE)   DOES NOT WORK ON MY SYSTEM

#prop = cap.get(cv2.CAP_PROP_FORMAT  ) 
#print( 'CAP_PROP_FORMAT = ' + str(prop) +' type = ' +str(type(prop) ))

_now = dateTimeString()

while(True):
    i+=1
    # Capture frame-by-frame
    ret, frame = cap.read()

    if i % 20 == 21:
        prop = cap.get(cv2.CAP_PROP_FPS ) #  )
        print( 'CAP_PROP_FPS = ' + str(prop) +' type = ' +str(type(prop) ))
        prop = cap.get(cv2.CAP_PROP_EXPOSURE ) 
        print( 'CAP_PROP_EXPOSURE = ' + str(prop) +' type = ' +str(type(prop) ))
        prop = cap.get(cv2.CAP_PROP_FORMAT  ) 
        print( 'CAP_PROP_FORMAT = ' + str(prop) +' type = ' +str(type(prop) ))
        
    # Our operations on the frame come here
   # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',frame)
    key = cv2.waitKey(1)
  #  if key > -1:
   #     print("Key  "+str(key))
    if key & 0xFF == ord('q'):
        break
    if key & 0xFF == ord('w'):
        gain_prev = cap.get(cv2.CAP_PROP_GAIN ) 
        gain += 2
        ret_val = cap.set(cv2.CAP_PROP_GAIN, gain) 
        gain = cap.get(cv2.CAP_PROP_GAIN ) 
        print( 'CAP_PROP_GAIN '+str(gain_prev)+ ' => ' + str(gain) +' type = ' +str(type(gain)) + ' return = '+str(ret_val) )

    if key & 0xFF == ord('e'):           
        gain_prev = cap.get(cv2.CAP_PROP_GAIN ) 
        gain -= 2
        ret_val = cap.set(cv2.CAP_PROP_GAIN, gain) 
        gain = cap.get(cv2.CAP_PROP_GAIN ) 
        print( 'CAP_PROP_GAIN '+str(gain_prev)+ ' => ' + str(gain) +' type = ' +str(type(gain)) + ' return = '+str(ret_val) )

    if key & 0xFF == ord('s'):
        file_name = 'robotfotos/image_'+_now+'_'+str(ctr)+'_'+str(width)+'x'+str(height)+'.jpg'
        ctr+=1
        print('saving to '+file_name)
        cv2.imwrite(file_name, frame)


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()