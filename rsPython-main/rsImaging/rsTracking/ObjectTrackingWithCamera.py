# -*- coding: utf-8 -*-
"""
ObjectTrackingWithCamera: tracks the flyer object in the images of the camera

@author: Jan Lemeire
Created on Wed Feb 26 2020

"""
import datetime
import time
import math
import cv2

import sys
sys.path.append('C:\\Users\\Cedric\\Documents\\Spyder\\thesis\\rsPython-main') # repository of Robotic Sensing Lab's Python code

from rsImaging.FilterAndContours import contoursOnHSV
from rsImaging.ImageUtils import Color, resizeImage
import ObjectLocalization
   
def dateTimeString(): # date & time format: '2020-12-22_15.38'
    return str(datetime.datetime.now())[0:16].replace(':', '.').replace(' ', '_')

def checkParams(params):
    frho,  x, z, azimuth = params
    #frho, pitch, x, z, azimuth = params
    return frho > 500 and frho < 1500 and x > -1000 and x < 1500 and z > -2500 and z < 2500 

def createoutfile():
    t = time.gmtime()
    h = t[3]+1
    m=t[4]
    s=t[5]
    timestring = str(h)+'_'+str(m)+'_'+str(s)
    outfile = open('camera_log_'+timestring+'.csv', 'w')
    return outfile

################ MAIN ################
if __name__== "__main__":
    cap = cv2.VideoCapture(1)  # Camera ID (0 is internal camera)
    image_ctr = 1
    i = 0
    _now = dateTimeString()
    time0 = time.time()
    
    outfile = None
    # supports 640.0 x 480   1280 x 720   1920 x 1080
    width = 1920
    height = 1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH , width )  # necessary!!
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT , height ) 
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH  ) )
    height = int( cap.get(cv2.CAP_PROP_FRAME_HEIGHT  ) )
    print('Camera captures images of '+str(width) + ' x ' + str(height) + ' = '+str(width*height)+' pixels')
    
    frho = 1100
    pitch = -math.pi*0.5
    x = -100
    z = 300
    azimuth = -1.11
    ctr_ok = 0
    ctr_nok = 0
    
#    params = (frho, pitch, x, z, azimuth)
    params = (frho, x, z, azimuth)
    prev_params = params
    SCALE_PERCENTAGE = 70
    while(True):
        i+=1
        # Capture frame-by-frame
        ret, img = cap.read()
    
        # Our operations on the frame come here
       # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        # Detect contours the resulting frame
        #color = Color.ORANGE
        #contoursOnHSV(img, color.rangeLower(), color.rangeUpper(), minimal_size = 2500, show_filtered_image= False)
        
        #height, width = img.shape[:2]
        
        
        
        #cv2.imshow('frame',resizeImage(img, SCALE_PERCENTAGE))
        
        if True:
            start_time = time.time()
            img_original = img.copy()
            params_guessed, ok, ppixels = ObjectLocalization.identifyAndLocate(img,  pitch, params_init = params,cameraHeight = -1600, doPrint = False, doImage = True)
            
                
            elapsed = time.time() - start_time
            #print('Position robot (x='+str(int(x))+', z = '+str(int(z))+'). picture '+str(i)+' analyzed in %2.3f seconds\n' % elapsed)
            if ok and checkParams(params_guessed):
                ctr_ok+=1
                prev_params = params
                params = params_guessed # for the next round
                frho, x, z, azimuth = params
#                frho, pitch, x, z, azimuth = params
                out_str =str(int( (time.time()-time0)*1000))+';'+str(int(x))+','+str(int(z))+','+str(ppixels[0])+','+str(ppixels[1])+','+str(ppixels[2])+','+str(ppixels[3])+','+str(ppixels[4])+','+str(ppixels[5])+','+str(ppixels[6])+','+str(ppixels[7])
                print(out_str)
                if outfile is not None:
                    outfile.write(out_str+'\n')
                if ctr_ok > 2:
                    ctr_nok=0
            else:
                ctr_nok+=1
                if ctr_nok > 3:
                    ctr_ok = 0
                    params = prev_params
            
        cv2.imshow('frame C',resizeImage(img, SCALE_PERCENTAGE))
        key = cv2.waitKey(1) & 0xFF
    
        if key == ord('q'):
            break
       
        if key == ord('o'):
            if outfile is None:
                print('start writing to file')
                outfile = createoutfile()
            else:
                print('STOP writing to file')
                outfile.close()
                outfile = None
                
        if key == ord('i'):
            frho = 1000
            pitch = -0.73
            x = -100
            z = 700
            azimuth = -0.31
            params = (frho, pitch, x, z, azimuth)
            print('Params reinitialized')
            
        if key == ord('c'):
            start_time = time.time()
            img_processed = img.copy()
            params_guessed, ok = cameraOnTommysGripper.identifyAndTrack(img_processed, params_init = params, cameraHeight = -1600, doPrint = False, doImage = True)
            if ok:
                params = params_guessed # for the next round
            elapsed = time.time() - start_time
            print('picture '+str(i)+' analyzed in %2.3f seconds\n' % elapsed)
            cv2.imshow('frame Calib',resizeImage(img_processed, SCALE_PERCENTAGE))
           
            
            file_name = 'robotfotos/image_'+_now+'_'+str(image_ctr)+'_'+str(width)+'x'+str(height)+'_original.jpg' 
            print('saving to '+file_name)
            cv2.imwrite(file_name, img_original)

            
        if key == ord('s') or key == ord('c'):
            file_name = 'robotfotos/image_'+_now+'_'+str(image_ctr)+'_'+str(width)+'x'+str(height)+'.jpg' 
            image_ctr+=1
            print('saving to '+file_name)
            cv2.imwrite(file_name, img)
    
    
    # When everything done, release the capture
    if outfile is not None:
        outfile.close()
    cap.release()
    cv2.destroyAllWindows()