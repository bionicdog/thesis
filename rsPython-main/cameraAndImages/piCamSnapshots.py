#!/usr/bin/python
#
#  

import time
import datetime
import os
from picamera import PiCamera
#import numpy as np

def dateTimeString():
    _now = datetime.datetime.now()
    _str = str(_now)[0:16]
    _str = _str.replace(':', '.')
    _str = _str.replace(' ', '_')
    return _str

_now = dateTimeString()
folder = '/home/pi/Pictures/'+str(_now)+'/'
print("Images are saved to "+folder)
os.mkdir(folder)

camera = PiCamera()

#try:
#    camera.start_preview()
#    camera.annotate_text = 'any key to capture image, q to quit'
#finally:
 #   camera.stop_preview()
  #  camera.close()
# overlay
 #   a = np.zeros(720, 1280, 3)
 #   a[360, :, :] = 0xff
 #   a[:, 640, :] = 0xff
#    camera_overlay = camera.add_overlay(np.getbuffer(a), layer=3, alpha=64)

camera.start_preview()
camera.annotate_text = 'any key to capture image, q to quit'
i=1

while True:
    key = input('Enter to capture, q to quit');
    print('key='+key+ '\n')
    if key == 'q':
        break;
    
    start_time = time.time()
    
   # try:
    camera.capture((folder + 'image%s '+key+'.jpg') % i)
    #finally:
        #camera.stop_preview()
        #camera.close()
    elapsed = time.time() - start_time
    print('picture '+str(i)+' captures in %2.3f seconds\n' % elapsed)
    i += 1
print('I am quitting\n')
doContinue = False
#    camera.remove_overlay(camera_overlay)
camera.stop_preview()
camera.close()
