# -*- coding: utf-8 -*-
"""
ContourThings: analyzing images based on 'things' identified in the image. A thing is identified by its contour.

@author: Jan Lemeire
Created on Wed May 27 2020
"""
import cv2
import math
import ImageUtils
import Contours

import sys
sys.path.append('D:\\mycode\\smartdot\\rsPython') # repository of Robotic Sensing Lab

from Perspective import scr2pix, pix2scr

###### UTILITY FUNCTIONS ######
def zwaartepunt(d2points):
    x = 0
    y=0
    for p in d2points:
         x += p[0]
         y += p[1]
    return (int(x / len(d2points)), int(y / len(d2points)))

# rotation counterclockwise FOR 2D POINTS!
def rotation2DinXYplane(point, center, azimuth):
    dx = point[0] - center[0]
    dz = point[1] - center[1]
    return ( center[0] + dx * math.cos(azimuth) - dz * math.sin(azimuth), center[1] + dx * math.sin(azimuth) + dz * math.cos(azimuth) )
 

###### SPECIAL FUNCTIONS ######
    

   
def contourFilter(contour, allContours, minRelSize, minY = 500, minFillingPct = 25, maxExpanse = 2.8):
    area = cv2.contourArea(contour)
    x,y,w,h = cv2.boundingRect(contour)
    # bounding rect not inside other
    for contour2 in allContours:
        x2,y2,w2,h2 = cv2.boundingRect(contour)
        if x > x2 and y > y2 and (x + w) < (x2 + w2 + 10) and (y + h) < (y2 + h2 + 10):
            return False
    relSize = area * 100 / ( width * height)
    filling = int(area / w * 100 / h)
    expanse = w / h if w > h else h / w 
    # TODO FILTER
    return relSize > minRelSize and y > minY and filling > minFillingPct and expanse < maxExpanse
    
   

###### BLUE U ######
    
# extract 1 thing based on score_function => BestThing
def extractBlueU(img_hsv, color = ImageUtils.Color.BLUE):
    def identifyU(contours):
        _max = 0
        contour_max = None
        for contour in contours[1]:
            area = cv2.contourArea(contour)
            x,y,w,h = cv2.boundingRect(contour)
            #print('contour at '+str(x)+','+str(y)+': '+str( (x,y,w,h) ) )
            filling = area * 100.0 / w / h
            roi_size = w*h
            
            # TODO: score function  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            score = (roi_size / filling) if filling > 0 else  0
            
            if _max < score:
                _max = score
                contour_max = contour
                #print('contour at '+str(x)+','+str(y)+': score = '+str(score))
        return contour_max

  #  contours, mask = contoursOfHSVRange(img_hsv, color.rangeLower(), color.rangeUpper(), )
  #  contour = identifyU(contours)
  #  new_thing = ContourThing(color, contour)
  #  return new_thing


################ Analysis of contour polygon ################
def extractOuterPoints(polygon, width, height):
    leftscore = 10000
    rightscore = -10000
    bottomscore = 10000
    topscore = -10000
    for points in polygon:
        x,y=points[0]
        if (x  < leftscore):
            leftscore = x
            left = (x, y)
        if (x > rightscore):
            rightscore = x
            right  = (x, y)
        if (y < bottomscore):
            bottomscore = y
            bottom = (x, y)
        if (y >  topscore):
            topscore = y
            top  = (x, y)
    return [left, bottom, right, top],  [scr2pix (left, width, height) , scr2pix (bottom, width, height), scr2pix(right, width, height), scr2pix(top, width, height)]

def extractCornerPoints(polygon, width, height):
    topleftscore = 10000
    toprightscore = 10000
    bottomleftscore = 10000
    bottomrightscore = 10000
    for points in polygon:
        x,y=points[0]
        if (x + y < topleftscore):
            topleftscore = x+y
            topleft = (x, y)
        if (y - x < toprightscore):
            toprightscore = y-x
            topright  = (x, y)
        if (x - y < bottomleftscore):
            bottomleftscore = x - y
            bottomleft = (x, y)
        if (-x -y < bottomrightscore):
            bottomrightscore = -x-y
            bottomright  = (x, y)
    return [bottomleft, topleft , topright, bottomright], [scr2pix (bottomleft, width, height), scr2pix (topleft, width, height) , scr2pix(topright, width, height), scr2pix(bottomright, width, height)]

# returns 4 coordinates in projection and image frame
def extractFlyer(polygon, width, height):
    n = len(polygon)
    # zwaartepunt
    zx = 0
    zy = 0
    for points in polygon:
        x,y=points[0]
        zx += x
        zy += y
    zx /= n
    zy /= n
    # tail point
    score = 0
    for points in polygon:
        x,y=points[0]
        dist = (x - zx) * (x - zx) + (y - zy) * (y - zy)
        if dist > score:
            score = dist
            tail = (x, y)
    
    perpRight = rotation2DinXYplane(tail, (zx, zy), -math.pi/2)
    
    leftscore = 1000000
    rightscore = -1000000
    bottomscore = 10000000
   
    for points in polygon:
        x,y=points[0]
        perpScore = (x - zx) * (perpRight[0] - zx) + (y - zy) * (perpRight[1] - zy)
        tailScore = (x - zx) * (tail[0] - zx) + (y - zy) * (tail[1] - zy)
        if (perpScore  < leftscore):
            leftscore = perpScore
            left = (x, y)
        if (perpScore > rightscore):
            rightscore = perpScore
            right  = (x, y)
        if (tailScore < bottomscore):
            bottomscore = tailScore
            bottom = (x, y)
       
    return [tail, left, bottom, right ],  [scr2pix(tail, width, height), scr2pix(left, width, height) , scr2pix (bottom, width, height), scr2pix (right, width, height)  ]

     
def detectShape(contour):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(contour, True)
        poly = cv2.approxPolyDP(contour, 0.04 * peri, True)
                # if the shape is a triangle, it will have 3 vertices
        if len(poly) == 3:
            shape = "triangle"
 
        # if the shape has 4 vertices, it is either a square or a rectangle
        elif len(poly) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(poly)
            ar = w / float(h)
 
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.8 and ar <= 1.2 else "rectangle"
 
        # if the shape is a pentagon, it will have 5 vertices
        elif len(poly) == 5:
            shape = "pentagon"
 
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
 
        # return the name of the shape
        return shape
  

    
################ MAIN ################
if __name__== "__main__":

#    img=cv2.imread('D:/research/robotics/grijper Arion Tommy/grijper tommy/U-vorm - 480 op 640.jpg')
    #img=cv2.imread('D:/research/robotics/grijper Arion Tommy/grijper tommy/image4_blauwe U - 15 op 10cm.jpg')
    #img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/robotics/camera and image processing/finishfotos/test5_1920.0x1080.0.jpg')
    #img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/robotics/camera and image processing/robotfotos/image_2020-12-22_19.17_13_1920x1080_original.jpg')

   # img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/mycode/smartdot/imaging/robotfotos/image_2021-01-05_12.45_1_1920.0x1080.0.jpg')
    
   # img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/mycode/smartdot/imaging/robotfotos/image_2021-01-05_12.45_1_1920.0x1080.0.jpg')
    img, img_file = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/mycode/smartdot/imaging/robotfotos/image_2021-01-05_15.13_3_1920x1080_original.jpg')

    #img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/robotics/camera and image processing/robotfotos_NOK/image_2020-12-22_19.09_6_1920x1080_original.jpg')
    cv2.namedWindow('camera image', cv2.WINDOW_NORMAL)  

    height, width = img.shape[:2]
    print('Image of '+str(height)+' x ' +str(width))
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    object_color = ImageUtils.Color.PINK
    
    if True:
        
        contour_objects, mask = Contours.contourObjectsOfColor(img_hsv, color =object_color)
        biggest_object = Contours.maxObject(contour_objects)
        scrpoints, uPoints = extractFlyer(biggest_object.polygon, width, height)
        
        print('Retrieved scrpoints: '+ str(scrpoints))
        print('Retrieved Pixels   : '+ str(uPoints))
        cv2.polylines(img, [biggest_object.polygon], False, (0, 255, 0), 2, lineType=cv2.LINE_AA) 
        ImageUtils.drawPoints(img, scrpoints, radius = 10)
    
    
        if True:
            L= 100.0
            W = 150.0
            pix = uPoints # [ ( -981,330), (-788,978), (652,934), (857,292)]
            ranges = ([80, 120], [-5, 5], [-100, -50], [-100, -10], [100, 250], [2400, 2600])
            
           # solution = findZero(ranges, projEqPixels)
           # print('Solution = ', prsol(solution))
           # print('Score    = {:.4f}'.format(sse(projEqPixels(solution))))
        else:    
            solution = ()
            
       # alfa, beta, x, y, z, frho = solution
       # pos = np.array(( 0, 0, y) ) 
       # attitude = np.array( (-alfa, beta) )
        
        #persp = perspective(frho, pos, attitude)
        #persp.locatePixelOnZplane(v)
    

    
    cv2.imshow('camera image', img)  
    
    c = chr(cv2.waitKey(0)) # wait until a key is pressed   
    if c == 's':
        file_name = img_file[0:-4] + '_processed' + img_file[-4: len(img_file)] 
        print('saving to '+file_name)
        cv2.imwrite(file_name, img)
    
    if c == 'c': # check contours
        cv2.namedWindow('image with contours', cv2.WINDOW_NORMAL)
        Contours.showContours(img, color = object_color, minimalSize=50, showFilteredImage = False)
        c = chr(cv2.waitKey(0)) # wait until a key is pressed  
        
    if c == 't':
        import ThresholdingOnHSV
        ThresholdingOnHSV.showThresholdingGUI(img, rangesTuple = (object_color.rangeLower(), object_color.rangeUpper()))   
        c = chr(cv2.waitKey(0)) # wait until a key is pressed  
        
    cv2.destroyAllWindows()
    
################ #### ################