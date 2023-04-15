"""
Contours.py

Identify objects through thresholding on color ranges

Created on Thu Mar 12 2020

@author: Jan Lemeire & Jonathan Ntumba Kanyinda

"""

import cv2
import math
import numpy as np
import sys
sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder)

import rsImaging.ImageUtils as ImageUtils

    
################ Contour objects: Contour Objects ################
thing_ctr = 0

class ContourObject:
    def __init__(self, color, contour):
        global thing_ctr
        self.id = thing_ctr
        thing_ctr += 1
        self.color = color
        self.contour = contour
        self.area = cv2.contourArea(contour)
        self.roi = cv2.boundingRect(contour)
        
        self.x,self.y,self.w,self.h = cv2.boundingRect(contour)
        self.width = self.w
        self.height = self.h
        self.o_points = [ (self.x , self.y + self.h//2) , (self.x + self.w//2, self.y) , (self.x + self.w, self.y + self.h//2) , (self.x + self.w//2, self.y + self.h) ] # left top right bottom
        self.centrum = (self.x + self.w//2, self.y + self.h//2)
        self.filling = self.area/(self.w * self.h)
        
        epsilon = 0.01*cv2.arcLength(self.contour,True)
        self.polygon = cv2.approxPolyDP(self.contour,epsilon,True)
        self.selected = False
        self.backpoint = None
        self.backpointGlobal = None
        
    def print(self):
        print("("+str(self.id)+"): pos  = ("+str(self.x)+", "+str(self.y)+") area = "+str(self.area))
        print("Left x = "+str(self.x)+", right x = "+str(self.x+self.w))

    def draw(self, img, color = (0,0,255), radius = 5):
#        cv2.circle(img, th.centrum , 1, color, 20)
       # if self.backpointGlobal is None:
        #else:
         #   cv2.putText(img, str(self.id)+": "+str(self.backpointGlobal),self.centrum, cv2.FONT_HERSHEY_SIMPLEX, 4,(255,255,255),4,cv2.LINE_AA)
        cv2.drawContours(img, self.contour, -1, (0, 255, 0), 2 if self.selected else 1) # draws points
        if False:
            cv2.putText(img, str(self.id),self.centrum, cv2.FONT_HERSHEY_SIMPLEX, 4,(255,255,255),4,cv2.LINE_AA)
            ImageUtils.drawPoints(img, self.o_points , color, radius, connect = False)
            cv2.rectangle(img, (int(self.x), int(self.y)), (int(self.x+self.w), int(self.y+self.h )), (0,140,255) if self.selected else (0,0,255), 8 if self.selected else 3)
            cv2.polylines(img, [self.polygon], False, (0, 0, 255), 2, lineType=cv2.LINE_AA) 
         
            ## OLD DRAW CODE
       #  print('('+str(i)+') rel size =  {:.2f}% filling = '.format(relSize) +str(filling)+'pct, expanse = {:.1f}x, polygon with {:d}corners'.format(expanse, len(polygon)))
       # ImageUtils.drawPoints(img_contour, polygon, color = (0, 255, 0))
       # cv2.circle(img_contour, (int(x), int(y)), 1, (0,0,255), 2)
       # cv2.rectangle(img_contour, (int(x), int(y)), (int(x+w), int(y+h )), (0,0,255), 3)
       # text = "{:d}: {:d}corners".format(i, len(polygon))
       # cv2.putText(img_contour, text,(x+w//2,y+25), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),4,cv2.LINE_AA)
       # cv2.drawContours(img_contour, contour, -1, (0, 255, 0), 1) # draws points

    def convertToMask(self, imageShape:tuple):
        mask = np.zeros(imageShape, np.uint8)
        cv2.drawContours(mask, self.contour, -1, (255),1)
        return mask
        
    def isInside(self, other):
        return self.x > other.x and self.y > other.y and (self.x + self.w) < (other.x + other.w + 10) and (self.y + self.h) < (other.y + other.h + 10)

    # THIS CODE IS SPECIFIC FOR OBJECT LOCALIZATION/TRACKING
    def projectPoint(self, persp, width, height):
        
        self.backpoint = np.array( ( self.o_points[1][0] - width/2, height/2 - self.o_points[1][1]) )
        co =  persp.locatePixelOnZplane(self.backpoint)
        self.backpointGlobal = np.array( ( int(-co[0]), int(-co[1]), int(co[2]) ) )



################ Create Contours ################

# pass result of openCV findContours()
def contours2contourObjects(openCVcontoursTuple, color = None, contourFilter = None):
    return [ContourObject(color, contour) for contour in openCVcontoursTuple[0] if (contourFilter is None or contourFilter(contour))]

def contoursOfColor(img_hsv, color):
    if type(color) is str:
        color = ImageUtils.char2color.get(color, None)
        if color is None:
            print('Unknown color character: '+str(color))
            return []
    elif type(color) is ImageUtils.Color:
        return contoursOfHSVRange(img_hsv, color.rangeLower(), color.rangeUpper())
    else:
        print('Unhadled type '+str(type(color))+' of color parameter '+str(color))
        return []
    
def contourObjectsOfColor(img_hsv, color, contourFilter = None):    
    contours, mask = contoursOfColor(img_hsv, color)
    if contours is None or len(contours) == 0:
        return [], mask
    contourObjects = [ContourObject(color, contour) for contour in contours if (contourFilter is None or contourFilter(contour))]
    return contourObjects, mask

def contoursOfHSVRange(img_hsv, range_lower, range_upper):
    if range_lower[0] < range_upper[0]: # Hue-component
        mask = cv2.inRange(img_hsv, range_lower, range_upper)
    else:
        range_upper2 = np.array([256, range_upper[1], range_upper[2]])
        mask1 = cv2.inRange(img_hsv, range_lower, range_upper2)
        range_lower2 = np.array([0, range_lower[1], range_lower[2]])
        mask2 = cv2.inRange(img_hsv, range_lower2, range_upper)
        mask = cv2.bitwise_or(mask1, mask2)
    contours_tuple = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # returns tuple of 2 or 3 elements: contours and the hierarch (https://docs.opencv.org/3.4/d9/d8b/tutorial_py_contours_hierarchy.html)
    return contours_tuple[0], mask  # we only return the contours, not the rest 

def contourObjectsOfHSVRange(img_hsv, rangesTuple, color = None, contourFilter = None):    
    contours, mask = contoursOfHSVRange(img_hsv, rangesTuple[0], rangesTuple[1])
    if contours is None or len(contours) == 0:
        return [], mask
    contourObjects = [ContourObject(color, contour) for contour in contours if (contourFilter is None or contourFilter(contour))]
    return contourObjects, mask

################ Operations on ContourObjects ################
    
def showContours(img, img_hsv = None, color = None, rangesTuple = None, minimalSize = None, showFilteredImage = False):
    if color is None and rangesTuple is None:
        print('Function showContours requires color or ranges-tuple')
    sizeFilter = None if minimalSize is None else createSizeContourFilter(minimalSize)
    if color is not None:
        rangesTuple = (color.rangeLower(), color.rangeUpper())
    if img_hsv is None:
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   # print('Chosen color = '+str(color)+': '+str(color.rangeLower())+' - '+str(color.rangeUpper()))
    print('rangesTuple = '+str(rangesTuple) )
    contourObjects, mask = contourObjectsOfHSVRange(img_hsv, rangesTuple, color, sizeFilter)
    print('Found '+str(len(contourObjects))+' contours in HSV range '+str(rangesTuple[0])+' - '+str(rangesTuple[1])+ ('' if color is None else ' (color '+str(color)+')')+('' if minimalSize is None else ' (minimalSize = '+str(minimalSize)+')'))
    img_contour = img.copy()
    printContourObjects(contourObjects)
    drawContourObjects(img_contour, contourObjects)
    cv2.imshow('image with contours', img_contour)
    if showFilteredImage:
        #cv2.namedWindow('mask', cv2.WINDOW_NORMAL) 
        #cv2.imshow('mask', mask)
        img_filter = cv2.bitwise_and(img, img, mask = mask)
        cv2.namedWindow('filtered image', cv2.WINDOW_NORMAL) 
        cv2.imshow('filtered image', img_filter)
    return contourObjects, mask
                    
    
def drawContourObjects(img, things, color = (0,0,255), radius = 5):
    for th in things:
        th.draw(img, color, radius)

def printContourObjects(things):
    for th in things:
        th.print()
        
def renumberThings(things):
    id = 0
    for thing in things:
        thing.id = id
        id = id + 1
        
################ Contour filters ################

def createSizeContourFilter(minimalSize):
    def hasSize(contour):
        area = cv2.contourArea(contour)
        return area > minimalSize
    return hasSize

def createContourFilter(minRelSize, imageWidth, imageHeight, minY = 500, minFillingPct = 25, maxExpanse = 2.8):
    def contourFilter(ContourObject):
        relSize = ContourObject.area * 100 / ( imageWidth * imageHeight)
        expanse = ContourObject.width / ContourObject.height if ContourObject.width > ContourObject.height else ContourObject.height / ContourObject.width 
        return relSize > minRelSize and ContourObject.y > minY and ContourObject.filling > minFillingPct and expanse < maxExpanse
    return contourFilter

def filterContours(ContourObjects, contourFilter):
    return [contourTh for contourTh in ContourObjects if contourFilter(contourTh)]

# bounding rect not inside other

# TODO rename: filter
def extractEncapsulatingThings(things):
    things_copy = list(things)
    for thing1 in things_copy:
        for thing2 in things_copy:
            if thing1 is not thing2 and thing2.isInside(thing1):
                if thing2.color != thing1.color:
                    thing1.innerColor = thing2.color
                if thing2 in things:
                    things.remove(thing2)
    return things


            
################ Contour Selectors ################
def maxObject(things):
    _max = 0
    _max_thing = None
    for thing in things:
        if _max < thing.area:
            _max = thing.area
            _max_thing = thing
    return _max_thing
def selectThing(things, color): # selects biggest object with color
    max_area = 0
    max_th = None
    for th in things:
        if th.color is color:
            if th.area > max_area:
                max_th = th
                max_area = th.area
    return max_th    



################ MAIN ################
if __name__== "__main__":
    
 #   img, img_file = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/mycode/smartdot/imaging/robotfotos/image_2021-01-05_15.13_5_1920x1080_original.jpg')
 #   img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/research/robotics/grijper Arion Tommy/legobricks/image8.jpg')
    img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE,'D:/research/vision/coloredObjectIdentification/IMG_6220_small_part.jpg')

    if False:
        cv2.namedWindow('image with contours', cv2.WINDOW_NORMAL)
        cv2.imshow('image with contours', img)
    
        print('** Identify objects based on thresholding on the HSV colors **')
        print('Choose a color and regions (called contours) are identified. Only the bigger ones are selected (minimalSize parameter).')
        print('Press h to see all predefined colors and their corresponding character')
        print('Press q to quit')
        
        show_filtered_image = True
        img_contour = None
        rangesTuple = None
    
        while True:
            print("Choose color or 'q' to quit or 'h' for help")
            key = cv2.waitKey()
            if key > 0:
                c = chr(key)
                #print("key = "+str(key)+" char = "+c)
                
                if c == 'q' or key==27:
                    break
                if c == 's' and img_contour is not None:
                    file_name = img_file[0:-4] + '_processed' + img_file[-4: len(img_file)] 
                    print('saving to '+file_name)
                    cv2.imwrite(file_name, img)  
            
                if c == 't':
                    import ThresholdingOnHSV
                    ThresholdingOnHSV.showThresholdingGUI(img, rangesTuple)
                    
                if c == 'h':
                    print('Color map: \n'+str(ImageUtils.char2color))
                    
                color = ImageUtils.char2color.get(c, None)
                if color is not None:
                    showContours(img, color = color, minimalSize=50, showFilteredImage = show_filtered_image)
                    rangesTuple = (color.rangeLower(), color.rangeUpper())
        
        cv2.destroyAllWindows()
    else:
        c = 'r'
        color = ImageUtils.char2color.get(c, None)
        contourObjects, mask = showContours(img, color = color, minimalSize=50, showFilteredImage = False)
        key = cv2.waitKey() # to show the image, press a key to continue
        max_contour = maxObject(contourObjects)
        img_contour = max_contour.convertToMask(img.shape)
        img_contour_dil = cv2.dilate(img_contour, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        cv2.imshow('image with contours', img_contour_dil)
        key = cv2.waitKey()
        
        
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ### #### #### ###