# -*- coding: utf-8 -*-
"""
Created on Saturday June 12 2021

@author: Jan Lemeire
"""

MIN_BORDER_SIZE = 30
MAX_BORDER_SIZE = 60
MIN_SIZE = 0.01 # percentage of total image size
BLUR_RADIUS = 0 # if 0 no blur action will happen


import cv2
from random import randrange


def cropLegoBrick(img, file_name, show=False):
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
    
    def containsWhiteOnTop(img_HSV):
        mask_white = cv2.inRange(img_HSV, (0, 80, 160), (256, 256, 256)) 
       # img_filtered = cv2.bitwise_and(img_HSV, img_HSV, mask = mask_white)
       # cv2.namedWindow('image cr', cv2.WINDOW_NORMAL)
       # cv2.imshow('image cr', img_filtered)
        
        contours_white_tuple = cv2.findContours(mask_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for contour in contours_white_tuple[0]:
            area = cv2.contourArea(contour) 
            roi = cv2.boundingRect(contour)
            if area > 20 and roi[1] < 2:
                return True
                
        return False
    
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # mask_threshold = cv2.inRange(img_HSV, (0, 0, 80), (256, 256, 256)) # non-black via V-component
    mask_threshold = cv2.inRange(img_HSV, (0, 150, 0), (256, 256, 256)) # non-black via S-component
    
    img_filtered = img # cv2.bitwise_and(img, img, mask = mask_threshold)
    contours_tuple = cv2.findContours(mask_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # returns tuple of 2 or 3 elements
    contours = contours_tuple[0]
    contour, contour_area = maxContoursAndNotAtTop(contours)
    if contour is None:
        print('Image <'+file_name+'> SKIPPED: No Brick found which is separate from white upper band')
        return None

    contour_rel_size = float(contour_area) / img_size
    roi = cv2.boundingRect(contour)
    
    if show:
        print('Image <'+file_name+'>: Brick found at ('+str(roi[0]+roi[2]/2)+', '+str(roi[1]+roi[3]/2)+' with area = '+str(contour_area)+' (rel size = '+str(100*contour_rel_size)+'pct)' )
    
    if contour_rel_size < MIN_SIZE:
        print('Image <'+file_name+'> SKIPPED: size is too small: '+str(contour_rel_size)+'<'+str(MIN_SIZE)+'. Lego brick probably overlaps with white segment.')
        return None
    
    border_size = randrange(MIN_BORDER_SIZE, MAX_BORDER_SIZE)
    x0 = roi[0] - border_size
    y0 = roi[1] - border_size
    x1 = roi[0]+roi[2] + border_size
    y1 = roi[1]+roi[3] + border_size
    
    if BLUR_RADIUS > 0:
        img_filtered = cv2.GaussianBlur(img_filtered, (BLUR_RADIUS, BLUR_RADIUS), cv2.BORDER_DEFAULT)
        img = cv2.GaussianBlur(img, (BLUR_RADIUS, BLUR_RADIUS), cv2.BORDER_DEFAULT)
        
    img_cropped = img[y0:y1, x0:x1]
    if containsWhiteOnTop(img_cropped):
        # try again with smallest border
        border_size = MIN_BORDER_SIZE
        x0 = roi[0] - border_size
        y0 = roi[1] - border_size
        x1 = roi[0]+roi[2] + border_size
        y1 = roi[1]+roi[3] + border_size
        img_cropped = img[y0:y1, x0:x1]
        if containsWhiteOnTop(img_cropped):
            print('Image <'+file_name+'> SKIPPED: cropped image contains white at top.')
            cv2.imwrite(file_name+'.cropped.jpg', img_cropped)
            return None
    
    if show:
        print('cropped image OK')
        
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.rectangle(img_filtered, (x0, y0), (x1, y1), (255,255,255), 2)
        cv2.drawContours(img_filtered, contour, -1, (0, 255, 0), 1)
        
        cv2.imshow('image', img_filtered)
        #cv2.imshow('image', img_cropped)
    
    
        print('Print a key to finish')
        while cv2.waitKey() <= 0:
             x = 1 

        cv2.destroyAllWindows()
    return img_cropped

################ MAIN AND TEST PROGRAM ################
if __name__== "__main__":
    

    print(' == Cropping brick images with border ('+str(MIN_BORDER_SIZE)+', '+str(MAX_BORDER_SIZE)+') and blur radius = '+str(BLUR_RADIUS)+' ==')
    
    # test with 1 file
    if False: 
        #img_file = 'D:/references/vision/lego brick images/Neal/Real/3001/1588081753.45.jpg'
        img_file = 'D:/references/vision/lego brick images/Neal/Slecht_uitgeknipt/1585138705.08.jpg'
        img=cv2.imread(img_file)
        cropLegoBrick(img, img_file, show=True)
        
    # process images of folder
    if True:
        import os
        from tkinter import filedialog
        
        #folder = filedialog.askdirectory(title="Geef folder met images")
        folder ='D:/references/vision/lego brick images/Neal/Slecht_uitgeknipt/'
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
                    img_cropped = cropLegoBrick(img, file, show=False)
                    if img_cropped is not None:
                        file_to = folder_results + '/' + file[0:-4]+'_cropped.jpg'
                        cv2.imwrite(file_to, img_cropped)   
                except Exception as e:
                    print('Error with file '+ file+': '+ str(e))
                    
               
                 
       
                