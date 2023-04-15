# -*- coding: utf-8 -*-
"""
Created on Saturday June 12 2021

@author: Jan Lemeire
"""

import cv2
from random import randrange


def analyzeShape(img, file_name, show=False):
    height, width = img.shape[:2] 
    img_size =  height *  width
    if show:
        print("Image of "+str(width) +" x "+str(height))
        
    
    def maxContoursAndNotAtTop(contours):
        _max = 0
        contour_max = None
        for contour in contours:
            roi = cv2.boundingRect(contour)
            if roi[0] > 20 and roi[1] > 20:
                area = cv2.contourArea(contour) 
                if _max < area:
                    _max = area
                    contour_max = contour
        return contour_max, _max
    
    
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_threshold = cv2.inRange(img_HSV, (0, 0, 80), (256, 256, 256)) # non-black
    
    img_filtered = cv2.bitwise_and(img, img, mask = mask_threshold)
    contours_tuple = cv2.findContours(mask_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # returns tuple of 2 or 3 elements
    contours = contours_tuple[0]
    contour, contour_area = maxContoursAndNotAtTop(contours)
    if contour is None:
        print('Image <'+file_name+'> SKIPPED: No Brick found which is separate from white upper band')
        return None

    # contour properties
    contour_rel_size = float(contour_area) / img_size
    roi = cv2.boundingRect(contour)
    contour_length = cv2.arcLength(contour,True)
    epsilon = 0.01*contour_length # parameter
    contour_polygon = cv2.approxPolyDP(contour,epsilon,True)

    # crop image with roi
    border_size = 10
    x0 = roi[0] - border_size
    y0 = roi[1] - border_size
    x1 = roi[0]+roi[2] + border_size
    y1 = roi[1]+roi[3] + border_size
    img_cropped = img[y0:y1, x0:x1]
        
    # canny edges
    img_gray = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2GRAY)    
    img_blur = cv2.blur(img_gray, (3,3))
    ratio = 3
    kernel_size = 5
    low_threshold = 30
    detected_edges = cv2.Canny(img_blur, low_threshold, low_threshold*ratio, kernel_size)
    mask = detected_edges != 0
    img_edges = img_cropped * (mask[:,:,None].astype(img_cropped.dtype))
    

    if show:
        print('Image <'+file_name+'>: Brick found at ('+str(roi[0]+roi[2]/2)+', '+str(roi[1]+roi[3]/2)+') with area = '+str(contour_area)+' and contour length of '+str(int(contour_length))+' (rel size = '+str(100*contour_rel_size)+'pct)' )
        
       
        cv2.namedWindow('Main contour', cv2.WINDOW_NORMAL)
        #cv2.rectangle(img_filtered, (x0, y0), (x1, y1), (255,255,255), 2)
        img_contour = img.copy()
        cv2.drawContours(img_contour, contour, -1, (0, 255, 0), 2)
        cv2.imshow('Main contour', img_contour)
        
        cv2.namedWindow('Blurred image', cv2.WINDOW_NORMAL)
        cv2.imshow('Blurred image', img_blur)
      
        cv2.namedWindow('Edges')  
        cv2.imshow('Edges', img_edges)
        
        print('Press a key to finish')
        while cv2.waitKey() <= 0:
             x = 1 
        cv2.destroyAllWindows()
        
   # return img_cropped

################ MAIN AND TEST PROGRAM ################
if __name__== "__main__":
    

    print(' ==  ==')
    
    # test with 1 file
    if True: 
        #img_file = 'D:/references/vision/lego brick images/Neal/Real/3001/1588081753.45.jpg' 1585148260.82.jpg
        img_file = 'D:/references/vision/lego brick images/Neal/without lamp/1585148260.82.jpg'
        img=cv2.imread(img_file)
        analyzeShape(img, img_file, show=True)
        
    # process images of folder
    if False:
        import os
        from tkinter import filedialog
        
        #folder = filedialog.askdirectory(title="Geef folder met images")
        folder ='D:/references/vision/lego brick images/Neal/Blender/3001'
        print('Gekozen images folder is ' + str(folder) )
        #folder_results = filedialog.askdirectory(title="Geef folder waarin nieuwe images moeten komen")
        folder_results = folder + '/cropped'
        
        inhoud = os.listdir(folder)
        for file in inhoud:
            _file = folder + '/' + file
            #print('Next ', str(file), ': ', os.path.isfile(_file))
            if os.path.isfile(_file):
                try:
                    img=cv2.imread(_file)
                    img_cropped = analyzeShape(img, file, show=False)
                    if img_cropped is not None:
                        file_to = folder_results + '/' + file[0:-4]+'_cropped.jpg'
                        cv2.imwrite(file_to, img_cropped)   
                except Exception as e:
                    print('Error with file '+ file+': '+ str(e))
                    
               
                 
       
                