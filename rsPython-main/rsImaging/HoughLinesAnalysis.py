# -*- coding: utf-8 -*-
"""
Created on Fri Aug 13 2021

@file hough_linesAnalysis.py
@brief analysis of the results of the Hough transform
@author Jan Lemeire
"""

import sys
import math
import cv2
import numpy as np

# the parameters
canny_kernel_size = 3
canny_low_threshold = 30
canny_ratio = 2

rho = 1
theta = np.pi / 180
hough_threshold = 60
minLineLength = 50
maxLineGap = 10
img_file = 'D:/references/vision/lego brick images/lego-bricks-1x1-3d.jpg'  #  LegoBricks - small.jpg' #  
#img_file = 'D:/references/vision/lego brick images/Neal/Real/3002/1588081323.94.jpg'
img=cv2.imread(img_file)
height, width = img.shape[:2] 
print('Image of width = '+str(width)+" height = "+str(height))


def startAndDirVector(width, height, _rho, _theta): 
   alpha = _theta - math.pi / 2 # direction (clockwise)
   
   if math.isclose(_theta, 0, abs_tol=10**-3): # parallel to Y-axis
       pt_start = (_rho, 0)
       v_dir = (0, 1)
       perp_dir = (1, 0)
     #  print('Line '+str(i)+' parallel to Y-axis')
   elif math.isclose(_theta, math.pi/2, abs_tol=10**-3): # parallel to Y-axis
       pt_start = (0, _rho)
       v_dir = (1, 0)
       perp_dir = (0, 1)
    #   print('Line '+str(i)+' parallel to X-axis')
   
   else:
       if alpha > 0:
           if _rho > 0:
               pt_start = (0, _rho / math.sin(_theta)) # crosses X-axis
             #  print('1. Crosses Y-axis in '+str(pt_start)+' alpha = '+str(int(alpha * 180.0 / math.pi)))
           else:
               pt_start = ( _rho /math.cos(_theta), 0) # crosses Y-axis
             #  print('2. Crosses X-axis in '+str(pt_start)+' alpha = '+str(int(alpha * 180.0 / math.pi)))
               v_dir = ( math.cos(alpha), math.sin(alpha))
       
       else:
           pt_start = (0, _rho / math.sin(_theta))
           if pt_start[1] >= height:   
               pt_y = height - 1
               pt_x = math.tan(_theta) * ( _rho / math.sin(_theta) - pt_y )
               pt_start = ( pt_x, pt_y )
            #   print('4. Line '+str(i)+' crosses X-axis in '+str(pt_start)+' alpha = '+str(int(alpha * 180.0 / math.pi)))
           #else:
               #  print('3. Crosses Y-axis in '+str(pt_start)+' alpha = '+str(int(alpha * 180.0 / math.pi)))
      
       v_dir = ( math.cos(alpha), math.sin(alpha))
       alpha = alpha + math.pi/2
       perp_dir = (math.cos(alpha), math.sin(alpha) )
   return pt_start, v_dir, perp_dir  

def distance(pt1, pt2):
    return math.sqrt( (pt1[0] - pt2[0]) * (pt1[0] - pt2[0]) + (pt1[1] - pt2[1]) * (pt1[1] - pt2[1]))

def distanceOfRicos(rico1, rico2):
    if rico1 == math.inf and rico2 == math.inf:
        return True
    if rico1 == math.inf:
        return rico2 > 10
    if rico2 == math.inf:
        return rico1 > 10
    return abs(rico1 - rico2)

def analyzeLine(edgesImg, _rho, _theta, dstImg, edgeLines):
    
    def xOfLine(_y, _rho, _theta):
         _theta_cos = math.cos(_theta)
         _theta_sin = math.sin(_theta)
         _x = (  _rho / _theta_sin - _y ) * _theta_sin / _theta_cos
         return _x
    
    def newLine(pt_start, pt_end, dstImg, ptAtBorder, line_length):
        nonlocal edgeLines
        radius = 1
        color = (0, 255, 0)
        thickness = 2
        #cv2.circle(dstImg, pt_start, radius, color, thickness)
        #cv2.circle(dstImg, pt_end, radius, (255, 0, 0), thickness)
       # cv2.line(dstImg, (int(pt_start[0]), int(pt_start[1])), (int(pt_end[0]), int(pt_end[1])), (0,0, 255), 1, cv2.LINE_AA)
        rico = (pt_start[1] - pt_end[1]) / (pt_start[0] - pt_end[0]) if not math.isclose(pt_start[0], pt_end[0], abs_tol=10**-3) else math.inf
        
        edge_line = (pt_start, pt_end, rico, ptAtBorder, line_length) if pt_start[0] < pt_end[0] else (pt_end, pt_start, rico, ptAtBorder, line_length)
        #print('Line added: '+str(edge_line))
        edgeLines.append(edge_line)
    
    height, width = edgesImg.shape[:2] 
    
    pt_start, v_dir, perp_dir = startAndDirVector(width, height, _rho, _theta)

    #show start & dir
   # pt_zero = ( int(pt_start[0]), int(pt_start[1]))
   # LENGTH = 50
   # pt_test = ( int(pt_start[0] + LENGTH * v_dir[0]), int(pt_start[1] + LENGTH * v_dir[1]))
   # cv2.line(dstImg, pt_zero, pt_test, (255,255, 0), 2, cv2.LINE_AA)

    pt_start = ( pt_start[0] + 0.5, pt_start[1] + 0.5)
    
    MIN_LINE_SIZE = 20
    MAX_LINE_GAP = 10
    line_start = None
    line_end = None
    line_gap_start = None
    prev_pt = None
    prev_pt_on_edge = False
    pt_on_edge = False
    foundLine = False
    
    l = 1
    while True:
      pt_dir = ( pt_start[0] + l * v_dir[0], pt_start[1] + l * v_dir[1])
      pt = ( int(pt_dir[0]), int(pt_dir[1]))
      pt_left = ( int(pt_dir[0] + perp_dir[0]), int(pt_dir[1] + perp_dir[1]) )
      pt_right = ( int(pt_dir[0] - perp_dir[0]), int(pt_dir[1] - perp_dir[1]) )
      if pt[0] < 1 or pt[0] >= (width - 1) or pt[1] < 1 or pt[1] >= (height - 1):
           if line_start != None:
               if line_end is None:
                   line_end = prev_pt
               line_length = distance(line_start, line_end)
               if  line_length >= MIN_LINE_SIZE:
                   newLine(line_start, line_end, dstImg, pt_start, line_length) # NEW LINE
                   foundLine = True
           break        
                              
      pt_on_edge = edgesImg[pt[1], pt[0]] > 0  or edgesImg[pt_left[1], pt_left[0]] > 0 or edgesImg[pt_right[1], pt_right[0]] > 0
           
      if pt_on_edge: 
          edgesImg[pt[1], pt[0]] = 0
          edgesImg[pt_left[1], pt_left[0]] = 0
          edgesImg[pt_right[1], pt_right[0]] = 0
          
          dstImg[pt[1], pt[0]] = 122
          dstImg[pt_left[1], pt_left[0]] = 122
          dstImg[pt_right[1], pt_right[0]] = 122
          
          radius = 1
          color = (0, 255, 0)
          thickness = 2
           # cv2.circle(dstImg, pt, radius, color, thickness)
            
      if pt_on_edge:      
            if line_start == None: # (1) new line
                line_start = pt
            else:
                if not prev_pt_on_edge :  # (4) end of gap
                    if line_gap_start is not None:
                        gap_length = distance(pt, line_end)
                        if gap_length < MAX_LINE_GAP:
                           # IGNORE GAP
                           line_gap_start = None
                           line_end = None
                        else:
                            line_length = distance(line_start, line_end)
                            if  line_length >= MIN_LINE_SIZE:
                                newLine(line_start, line_end, dstImg, pt_start, line_length) # NEW LINE
                                foundLine = True
                            line_start = None
                            line_end = None
                            line_gap_start = None
                #else: # (2) line continues
                    
      else: # not on edge
           if line_start != None and line_gap_start is None: # (3) start of gap
               line_end = prev_pt
               line_gap_start = pt
               
               
      prev_pt_on_edge = pt_on_edge    
      prev_pt = pt
      l = l + 1 
    return foundLine
   
def mergeLines(edgeLines, dstImg):
    ctr=0
    for i in range(0, len(edgeLines)):
        for j in range(i+1, len(edgeLines)):
            line1 = edgeLines[i]
            line2 = edgeLines[j]
            if line1 is None or line2 is None:
                continue
            rico1 = line1[2]
            rico2 = line2[2]
            borderPt1 = line1[3]
            borderPt2 = line2[3]
            if distanceOfRicos(rico1, rico2) < 0.25 and distance(borderPt1, borderPt2) < 10:
                ctr = ctr + 1
               # if ctr < 5: # FOR TESTING
               #     continue
                #print('Lines close to each other')
                #print('LINE1: '+str(line1))
                #print('LINE2: '+str(line2))
                pt_start1 = line1[0]
                pt_end1 = line1[1]
                #cv2.line(dstImg, (int(pt_start1[0]), int(pt_start1[1])), (int(pt_end1[0]), int(pt_end1[1])), (0,0, 255), 1, cv2.LINE_AA)
                pt_start2 = line2[0]
                pt_end2 = line2[1]
                #cv2.line(dstImg, (int(pt_start2[0]), int(pt_start2[1])), (int(pt_end2[0]), int(pt_end2[1])), (0,0, 255), 1, cv2.LINE_AA)
                if pt_start1[0] > pt_start2[0]: # swap
                    line_tmp = line1
                    line1 = line2
                    line2 = line_tmp
                    pt_start1 = line1[0]
                    pt_end1 = line1[1]
                    pt_start2 = line2[0]
                    pt_end2 = line2[1]
                MAX_LINE_GAP = 10
                if pt_end1[0] >= pt_start2[0] or distance(pt_end1, pt_start2) < MAX_LINE_GAP:
                    pt_end_new = pt_end1 if pt_end1[0] > pt_end2[0] else pt_end2
                    line_new = (pt_start1, pt_end_new, rico1, borderPt1, distance(pt_start1, pt_end_new))
                    #print('=> merge into '+str(line_new))
                    line1 = line_new
                    edgeLines[i] = line_new
                    edgeLines[j] = None
                #else:
                 #   print('=> remain separate')
                
    
WINDOW_HOUGH_LINES = "Hough Lines - Detected Lines (in red)" 
cv2.namedWindow(WINDOW_HOUGH_LINES, cv2.WINDOW_NORMAL)

edgesImg = cv2.Canny(img, canny_low_threshold, canny_low_threshold *canny_ratio, None, canny_kernel_size)    
        
# Copy edges to the images that will display the results in BGR
cdst = cv2.cvtColor(edgesImg, cv2.COLOR_GRAY2BGR)
cdstP = np.copy(cdst)
                   
edgeLines = []
    
lines = cv2.HoughLines(edgesImg, rho, theta, hough_threshold, None, 0, 0) # doc: https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
if lines is not None:
    print('Hough lines found: '+str(len(lines)))
    
    for i in range(0, len(lines)):
    #x = 3
    #for i in range(x, x+1):
        _rho = lines[i][0][0] # meaning: see https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
        _theta = lines[i][0][1]
        
        a = math.cos(_theta)
        b = math.sin(_theta)
        x0 = a * _rho
        y0 = b * _rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #cv2.line(cdst, pt1, pt2, (0,255,255), 2, cv2.LINE_AA)
        
        _theta_deg = int(_theta * 180.0 / math.pi)
        #print("Line "+str(i)+": rho="+str(_rho)+" theta="+str(_theta) + " ="+str(_theta_deg) + "deg")
        
        
        foundLine = analyzeLine(edgesImg, _rho, _theta, cdst, edgeLines)
        #print("Line "+str(i)+": "+str(foundLine))
          
    mergeLines(edgeLines, cdst)

    # show all lines
    for edgeLine in edgeLines:
        if edgeLine is not None:
            pt_start = edgeLine[0]
            pt_end = edgeLine[1]
            cv2.line(cdst, (int(pt_start[0]), int(pt_start[1])), (int(pt_end[0]), int(pt_end[1])), (0,0, 255), 1, cv2.LINE_AA)
            radius = 1
            color = (0, 255, 0)
            thickness = 2
            cv2.circle(cdst, pt_start, radius, color, thickness)
            cv2.circle(cdst, pt_end, radius, (255, 0, 0), thickness)

else:
    print('No Hough lines found')
    
cv2.imshow(WINDOW_HOUGH_LINES, cdst)

print('Press a key to finish')
while cv2.waitKey() <= 0:
    x = 1 
cv2.destroyAllWindows()
    