# -*- coding: utf-8 -*-
"""
ObjectLocalization.py: localization of objects with camera

@author: Jan Lemeire
Created on Fri May 15 2020

"""
import numpy as np
import math

import sys
if '..\..' not in sys.path:
    sys.path.append('..\..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

import rsImaging.perspective as Perspective
import rsLibrary.DataUtils as DataUtils


### Functions defining the flyer object ###

# rotation counterclockwise
def rotation2DinXZplane(point, center, azimuth):
    dx = point[0] - center[0]
    dz = point[2] - center[2]
    return ( center[0] + dx * math.cos(azimuth) - dz * math.sin(azimuth), point[1], center[2] + dx * math.sin(azimuth) + dz * math.cos(azimuth) )

# calculate four corners from (x,z)-position and azimuth of flyer
W = 80 # in mm
H = 0
cornerPositionFunctions = []  # returns 3D tuple
#cornerPositionFunctions.append(lambda x, z, azimuth : rotation2DinXZplane ( (x , H, z), (x , H, z), azimuth)) # center
cornerPositionFunctions.append(lambda x, z, azimuth : rotation2DinXZplane ( (x   , H, z + 2 * W), (x , H, z), azimuth)) # top (= tail of flyer)
cornerPositionFunctions.append(lambda x, z, azimuth : rotation2DinXZplane ( (x + W, H, z), (x , H, z), azimuth)) # right 
cornerPositionFunctions.append(lambda x, z, azimuth : rotation2DinXZplane ( (x   , H, z - W), (x , H, z), azimuth)) # bottom
cornerPositionFunctions.append(lambda x, z, azimuth : rotation2DinXZplane ( (x - W, H, z), (x , H, z), azimuth)) # left



### parameters/state <> observed pixels ###
class CameraLocalization:
    # parameters = frho [],  pitch (of the camera)], x, z and azimuth (robot) 
    cornerPositionFunctions = 0
    def __init__(self, cameraHeight, pitch, cornerPosFunctions = None):
        self.cameraHeight = cameraHeight
        self.cornerPositionFunctions = cornerPositionFunctions if cornerPosFunctions is None else cornerPosFunctions
        self.pitch  = pitch
    
    def observationFunction(self, params):        
        ppixels = np.zeros(0)
        for ppixel in self.pixelList(params): # transform to numpy array
            ppixels = np.append(ppixels, ppixel)
        return ppixels
       # return np.array(self.pixelList(params))
    
    def pixelList(self, params):
        x = params[1] 
        z = params[2]
        azimuth = params[3]
        l = []
        for cornerF in self.cornerPositionFunctions:
            pglobal = cornerF(x, z, azimuth)
            ppixel = self.global2image(pglobal, params)
            l.append(ppixel)
        return l # projection frame
    
    def setObservedPixels(self, pixels): # image frame, expected numpy array
        self.pixels = pixels
    
    def calcErrors(self, params):
        ppixels = self.observationFunction(params) # rows of 2 columns
        return ppixels - self.pixels #(ppixels - self.pixels).flatten() # to 1D
    
    def totalError(self, params):
        return DataUtils.sse(self.calcErrors(params))
     
    def global2image(self, pglobal, params): # returns pixel
        frho = params[0]
        pos = np.array(( 0, self.cameraHeight, 0) ) # height of camera
#        attitude = np.array( ( params[1], 0) ) # pitch, yaw
        attitude = np.array( ( self.pitch, 0) ) # pitch, yaw
        persp = Perspective.Perspective(frho, pos, attitude)
        return persp.global2image(pglobal)

    def global2camera(self, pglobal, params): # camera system
        frho = params[0]
        pos = np.array(( 0, 0, self.cameraHeight) ) 
 #       attitude = np.array( ( params[1], 0) ) # pitch, yaw
        attitude = np.array( ( self.pitch, 0) ) # pitch, yaw
        persp = Perspective.Perspective(frho, pos, attitude)
        return persp.global2local(pglobal)


    def printBeaconPixels(self, params):
        x = params[1] 
        z = params[2]
        azimuth = params[3]
        for cornerF in self.cornerPositionFunctions:
            pglobal = cornerF(x, z, azimuth)
            pcamera = self.global2camera(pglobal, params)
            ppixel = self.global2image(pglobal, params)
            print('Corner '+str(pglobal) + ' => '+str(pcamera)+' results in pixel '+str(ppixel))

    def createParams(self, frho, pitch, x, z, azimuth):
#        return np.array(  (frho, pitch, x, z, azimuth) )
        return np.array(  (frho, x, z, azimuth) )
    
    def printParams(self, params):
        frho = params[0]
        height = self.cameraHeight
        pitch = self.pitch # params[1]
        x = params[1] 
        z = params[2]
        azimuth = params[3] 
        print('frho = {:.0f}, H = {:.0f}, pitch = {:.2f}, x = {:.0f}, z = {:.0f}, azimuth = {:.2f}'.format(frho, height, pitch, x, z, azimuth))


### Interface for Esimation ###

import sys
sys.path.append('..\\..\\..\\studentwork\\2021Nikolai')
from algoritme import Algoritmes

class CameraEstim:
    # param = frho, y (height of the camera), pitch (of the camera), x, z and azimuth (robot)
    params = None
    f = 1 # needed to be compliant with interface
    
    def __init__(self, cameraLocalization, params):
        self.camLoc = cameraLocalization
        self.params = params
    
    def totalError(self):
        return self.camLoc.totalError(self.params)
    
    def printSolution(self):
        self.camLoc.printParams(self.params)
        
    # the 4 interface functions for the estimation
    def getUpdateArray_full_length(self, fend=0, fstart=1):
        return 4 # 5 # nbr parameters

    def getErrorArray_full(self, fend=0, fstart=1):
        return self.camLoc.calcErrors(self.params)
    
    def applyUpdateArray_full(self, updateArray, fend=0, fstart=1):
       # updateArray[1] /= 10 # for the angles
        updateArray[3] /= 10
        self.params += updateArray
    
    def getJacobian_full(self, fend=0, fstart=1):
        return Algoritmes.genJac(self)

### Estimate the parameters based on the retrieved corner pixels ###
# pixels in image frame
def estimateParametersAndPosition(pixels, params = None, frho=1200, cameraHeight=-1000, pitch= -math.pi*0.5, x=-270, z=300, azimuth=-0.1, doPrint = True):
    # 1. object that will do the conversions between parameters and frames
    camLoc = CameraLocalization(cameraHeight, pitch)
    params_init = camLoc.createParams(frho, pitch, x, z, azimuth) if params is None else params
    camLoc.setObservedPixels(pixels)
    
    #print('cameraHeight = '+str(cameraHeight))
    
    camLoc.printParams(params_init)
    
    total_error = int(camLoc.totalError(params_init))
    #print(DataUtils.arr2str(camLoc.calcErrors(params_init)) +" => "+str(total_error))
    
    # 2. estim contains the functions for the estimation algorithm
    estim = CameraEstim(camLoc, params_init)
    guessList = [] # we keep the different guesses
    guessList.append(camLoc.pixelList(estim.params) ) # first guess
    ctr=0
    
    while total_error > 100 and (ctr < 3 or total_error < 10000000) and ctr < 20:
        ctr += 1
        Algoritmes.full_newtonExt(estim, printJac=False, printUpdateA=doPrint)
        guessList.append(camLoc.pixelList(estim.params) )
         
        total_error = int(estim.totalError())
        #print(DataUtils.arr2str(estim.getErrorArray_full()) +" => "+str(total_error))
        print('('+str(ctr)+'): error = '+str(total_error))
        camLoc.printParams(estim.params)
        
    #print(str(estim.getErrorArray_full()) +" => "+str(estim.totalError())+ ' after '+str(ctr)+' iterations')
    if doPrint:
        camLoc.printParams(estim.params)    
     
    return estim.params, guessList, total_error

### Estimate the parameters based on an image ###
import cv2
import rsImaging.ImageUtils as ImageUtils
import rsImaging.Contours as Contours
import rsImaging.ImageThings as ImageThings
# 2D
def distance(p1, p2):
    return math.sqrt( (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def identifyAndLocate(img_copy, pitch, cameraHeight=-1000, params_init = None, doPrint = True, showIterativeProcess = True):
    # 1. identify object by color => contour
    height, width = img_copy.shape[:2]
    img_hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    contour_objects, mask = Contours.contourObjectsOfColor(img_hsv, color = ImageUtils.Color.PINK)
    biggest_object = Contours.maxObject(contour_objects)
    if biggest_object is None:
        print('Beacon could not be found in image...')
        return params_init, False
    
    # 2. extract points of objects
    scrpoints, uPoints = ImageThings.extractFlyer(biggest_object.polygon, width, height) # projection and image frame
    if doPrint:
        print('Retrieved scrpoints (projection frame): '+ str(scrpoints)) #pixel and image frame
        print('Retrieved Pixels         (image frame): '+ str(uPoints))
        
    cv2.polylines(img_copy, [biggest_object.polygon], False, (0, 255, 0), 2, lineType=cv2.LINE_AA) 
    ImageUtils.drawPoints(img_copy, scrpoints, radius = 10)
    
    # 3. estimate camera parameters and position
    # convert to numpy array
    ppixels = np.zeros(0)
    for p in uPoints:
        ppixels = np.append(ppixels, p)
        
    params, guessList, totalError = estimateParametersAndPosition(ppixels, params = params_init, cameraHeight = cameraHeight, doPrint = doPrint)
   
    ok = True if totalError < 1000 else False
    
    if showIterativeProcess:
        i = 0
        ctr=0
        prev_pixel = (-100000, -100000)
        for guess in guessList:
            scrpixels = []
            for pix in guess:
                scrpixels.append(Perspective.pix2scr(pix, width, height))
            i += 1
            if i < 4 or (i % 5 == 0 and i < len(guess) - 5) or i == len(guess) or distance(scrpixels[0], prev_pixel) > 40:
                _color = i * 30 % 256
                if doPrint:
                    print('pixelList = '+str(scrpixels)+' color = '+str(_color))
               
                ImageUtils.drawPoints(img_copy, scrpixels, color = (_color, _color, _color), radius = 10)
                cv2.putText(img_copy, str(i), ImageThings.zwaartepunt(scrpixels), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
                prev_pixel = scrpixels[0]
    
#        frho, pitch, x, z, azimuth = params   
        frho, x, z, azimuth = params   
       
        cv2.putText(img_copy, 'frho = '+str(int(frho)), (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, 'frho = '+str(int(frho)), (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(img_copy, 'height = '+str(int(cameraHeight)), (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, 'height = '+str(int(cameraHeight)), (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(img_copy, 'pitch = '+str(int(pitch*180/math.pi)), (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, 'pitch = '+str(int(pitch*180/math.pi)), (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(img_copy, 'x = '+str(int(x)), (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, 'x = '+str(int(x)), (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(img_copy, 'z = '+str(int(z)), (20, 165), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, 'z = '+str(int(z)), (20, 165), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(img_copy, 'azimuth = '+str(int(azimuth*180/math.pi)), (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, 'azimuth = '+str(int(azimuth*180/math.pi)), (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        _str = 'totalError = '+str(totalError) + ('   OK' if ok else '    NOK')
        cv2.putText(img_copy, _str, (20, 235), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),6,cv2.LINE_AA)
        cv2.putText(img_copy, _str, (20, 235), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        
        if x > -100000 and x < 100000 and z > -100000 and z < 100000:
            L = 400
            H = 200
            MAX_X = 2000
            MAX_Z = 1500
            _x = L/2 * x / MAX_X  # from the center
            _z = z * H / MAX_Z
            p = (int(_x), int(_z))
            p_img = (int(width-L/2+_x), int(H-_z))
            if doPrint:
                print('Coordinates = '+str( (int(x), int(z)) )+' => '+str( p ))
            cv2.rectangle(img_copy, (width-L, 0), (width, H), (255, 255, 255), thickness = -1) # fill with negative thickness
            cv2.circle(img_copy, p_img , 5, (0, 0, 0), thickness = -1)
        
    return params, ok, ppixels

################ MAIN ################    
if __name__== "__main__":
    
    ### VALIDTION BY SIMULATION: TRUTH -> ESTIMATION -> COMPARE ###
    if False:
        # THE TRUE PARAMETERS
        frho = 1000
        height = -300
        pitch = -math.pi/2 # downwards = negative
        x = 10
        z = 300
        #azimuth = 0
        azimuth = math.pi/8 # counterclockwise
        
        camLoc = CameraLocalization(height, pitch)
        true_params = camLoc.createParams(frho, pitch, x, z, azimuth)
        camLoc.printParams(true_params)
        camLoc.printBeaconPixels(true_params)
        
        # THE OBSERVED PIXELS
        ppixels = camLoc.observationFunction(true_params)
        camLoc.setObservedPixels(ppixels)
        
        # ESTIMATION
        params_init = camLoc.createParams(frho*1.4, pitch*0.8, x*2, z*1.4, azimuth*1.5)
        print(str(camLoc.calcErrors(params_init)) +" => "+str(camLoc.totalError(params_init)))
        
        estim = CameraEstim(camLoc, params_init)
        
        Algoritmes.full_newton(estim, True, True)
        total_error = estim.totalError()
        print(str(estim.getErrorArray_full()) +" => "+str(estim.totalError()))
        ctr=0
        while total_error > 10 and total_error < 1000000 and ctr < 100:
            ctr += 1
            Algoritmes.full_newton(estim, False, False)
            total_error = estim.totalError()
            print('['+str(ctr)+'] '+DataUtils.arr2str(estim.getErrorArray_full()) +" => "+str(estim.totalError()))
        
        print('Solution: ')
        estim.printSolution()
        print('Truth: ')
        camLoc.printParams(true_params)
        print('Estimation error = '+str (true_params - estim.params))
        
    ### FROM IMAGE ###
    if True:
        import time
        img, img_file = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/mycode/smartdot/imaging/robotfotos/image_2021-01-05_15.13_3_1920x1080_original.jpg')

        #img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/robotics/camera and image processing/robotfotos_NOK/image_2020-12-22_19.09_6_1920x1080_original.jpg')
        cv2.namedWindow('camera image', cv2.WINDOW_NORMAL)  
    
        height, width = img.shape[:2]
        print('Image of '+str(height)+' x ' +str(width))
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        pitch = -math.pi/2 # downwards = negative
        
        start_time = time.time()
        identifyAndLocate(img, pitch, doPrint = False, showIterativeProcess = True)
        elapsed = time.time() - start_time
        print('picture analyzed in %2.3f seconds\n' % elapsed)
        print("Press a key to quit or 's' to save image\n")
        cv2.imshow('camera image', img)  
    
        # wait until a key is pressed   
        key = cv2.waitKey(0)
        c = chr(key)
        if c == 's':
            file_name = img_file[0:-4] + '_processed' + img_file[-4: len(img_file)] 
            print('saving to '+file_name)
            cv2.imwrite(file_name, img)
            
        cv2.destroyAllWindows()
