"""
ThresholdingOnHSV.py

Test thresholds on the HSV colors

Created on Thu Mar 12 2020

@author: Jan Lemeire
  based on the code of Pieter
  
"""

import cv2
from Contours import showContours
from ImageUtils import Color, char2color, show_HSV_histogram 
from matplotlib import pyplot as plt
from matplotlib import colors

################ HSV masks ################
img = None
mask_threshold = None

def HSVRangesMask(img_HSV, rangesLower, rangesUpper):
    global mask_threshold
    if rangesLower[0] < rangesUpper[0]: # Hue-component
        mask_threshold = cv2.inRange(img_HSV, rangesLower, rangesUpper)
    else:
        range_upper2 = (256, rangesUpper[1], rangesUpper[2])
        mask_threshold1 = cv2.inRange(img_HSV, rangesLower, range_upper2)        
        range_lower2 = (0, rangesLower[1], rangesLower[2])
        mask_threshold2 = cv2.inRange(img_HSV, range_lower2, rangesUpper)
        mask_threshold = cv2.bitwise_or(mask_threshold1,mask_threshold2)
    return mask_threshold
   
            
def maskedImageByHSVRanges(img, rangesLower, rangesUpper, imgHSV = None):    
    global mask_threshold
    if imgHSV is None:
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_threshold = HSVRangesMask(imgHSV, rangesLower, rangesUpper)
    return cv2.bitwise_and(img,img, mask= mask_threshold)

img = None
def showThresholdingGUI(_img, rangesTuple=None):
    global img
    img = _img
    if rangesTuple is not None:
        setHSVRange(rangesTuple[0], rangesTuple[1])

 
################ GUI ################
# creation
max_value = 255
max_value_H = 180
low_H = 0
low_S = 0
low_V = 0
blur_radius = 0
min_contour_size = 2000

high_H = max_value_H
high_S = max_value
high_V = max_value


window_thresholds = 'Thresholds'
window_image = 'Original image'
window_thresholded_image = 'Thresholded image'
window_mask = 'Mask'

low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def applyHSVRanges():
    global img, low_H, low_S, low_V, high_H, high_S, high_V
    rangesLower = (low_H, low_S, low_V)
    rangesUpper = (high_H, high_S, high_V)            
    img_masked = maskedImageByHSVRanges(img, rangesLower, rangesUpper)
    cv2.imshow(window_thresholded_image, img_masked)

def setLowH(val):
    global low_H, high_H
    low_H = int(val)
   # low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_thresholds, low_H)
    applyHSVRanges()
def setHighH(val):
    global low_H, high_H
    high_H =  int(val)
   # high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_thresholds, high_H)
    applyHSVRanges()
def setLowS(val):
    global low_S, high_S
    low_S = int(val)
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_thresholds, low_S)
    applyHSVRanges()
def setHighS(val):
    global low_S, high_S
    high_S =  int(val)
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_thresholds, high_S)
    applyHSVRanges()
def setLowV(val):
    global low_V, high_V
    low_V =  int(val)
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_thresholds, low_V)
    applyHSVRanges()
def setHighV(val):
    global low_V, high_V
    high_V = int(val)
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_thresholds, high_V)
    applyHSVRanges()

def setHSVRange(ranges_lower, ranges_upper):
    global low_H, low_S, low_V, high_H, high_S, high_V
    low_H = int(ranges_lower[0])
    cv2.setTrackbarPos(low_H_name, window_thresholds, low_H)
    high_H =  int(ranges_upper[0])
    cv2.setTrackbarPos(high_H_name, window_thresholds, high_H)
    
    low_S = int(ranges_lower[1])
    cv2.setTrackbarPos(low_S_name, window_thresholds, low_S)
    high_S =  int(ranges_upper[1])
    cv2.setTrackbarPos(high_S_name, window_thresholds, high_S)
    
    low_V =  int(ranges_lower[2])
    cv2.setTrackbarPos(low_V_name, window_thresholds, low_V)    
    high_V = int(ranges_upper[2])
    cv2.setTrackbarPos(high_V_name, window_thresholds, high_V)
    applyHSVRanges()


    
def thresholdOnColor(color):
    if type(color) is Color:
        setHSVRange(color.rangeLower(), color.rangeUpper())
        
def on_blur_trackbar(val):
    global blur_radius
    if val > 0 and val % 2 == 0:
        val += 1 # blur radius must be odd!
    blur_radius = val
    cv2.setTrackbarPos("Blur radius", window_thresholds, blur_radius)

def on_contours():
    global img, low_H, low_S, low_V, high_H, high_S, high_V, min_contour_size
    rangesTuple = ( (low_H, low_S, low_V), (high_H, high_S, high_V) )
    showContours(img, img_hsv = None, color = None, rangesTuple = rangesTuple, minimalSize = min_contour_size, showFilteredImage = False)

def on_contour_size_trackbar(val):
    global min_contour_size
    min_contour_size = val
    cv2.setTrackbarPos("Minimal Contour Size", window_thresholds, min_contour_size)
    
def on_click_show_pixel(event, x, y, flags, param):
    global width, height, img, img_HSV, window_image
    #print('Clicked '+str(x)+', '+str(y)+': event '+str(event))
    if event == cv2.EVENT_LBUTTONUP:
        #print('Clicked '+str(x)+', '+str(y))
        if x < width  and y < height:
          
          intensity = img[y, x]  # omgekeerd!!
          blue, green, red  = intensity
          hsv_intensity = img_HSV[y, x]
          hue, saturation, value = hsv_intensity
          pix_x = int(x - width / 2)
          pix_y = int(height / 2 - y)
          text = "Pixel ({:d},{:d}):(B={:d},G={:d},R={:}) (H={:d},S={:d},V={:}) ".format(pix_x, pix_y, blue, green, red, hue, saturation, value)
          print(text)
          if False:
              #text = "(B={:d},G={:d},R={:}) ".format(blue, green, red)
              text = "(H={:d},S={:d},V={:}) ".format(hue, saturation, value)
              cv2.circle(img, (int(x), int(y)), int(3), (0,0,255), 2)
              cv2.putText(img,text,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
              cv2.imshow(window_image, img)

cv2.namedWindow(window_thresholded_image, cv2.WINDOW_NORMAL)
cv2.namedWindow(window_thresholds, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_thresholds, 300, 200)

cv2.createTrackbar(low_H_name, window_thresholds , low_H, max_value_H, setLowH)
cv2.createTrackbar(high_H_name, window_thresholds , high_H, max_value_H, setHighH)
cv2.createTrackbar(low_S_name, window_thresholds , low_S, max_value, setLowS)
cv2.createTrackbar(high_S_name, window_thresholds , high_S, max_value, setHighS)
cv2.createTrackbar(low_V_name, window_thresholds , low_V, max_value, setLowV)
cv2.createTrackbar(high_V_name, window_thresholds , high_V, max_value, setHighV)
#cv2.createTrackbar("Blur radius", window_thresholds , blur_radius, 40, on_blur_trackbar)
#cv2.createTrackbar("Minimal Contour Size", window_thresholds , min_contour_size, 10000, on_contour_size_trackbar)
#cv2.createButton('Find Contours', on_contours)


################ MAIN ################
if __name__== "__main__":
    import ImageUtils
    import CannyEdge
    import GradientsWithSobel
    
    print('** Test thresholds on the HSV colors **')
    print('Define ranges for the HSV-values.')
    print('Click on a point to see the pixel color.')
  #  print('Blur image by setting the radius of the blur filter.')
  #  print('Press c to find contours (minimal size set by trackbar)')
    print('Press x to print the chosen ranges on the HSV')
    print('Press character of predefined color to set thresholds for that color')
    print('Press h to see all predefined colors and their corresponding character')
    print('Press q to quit')

    import os    
#    folder = 'D:/research/robotics/grijper Arion Tommy/grijper tommy/pictures'
#    folder = 'D:/references/vision/lego brick images' #'/from phone 2'
   # folder = 'D:/references/vision/lego brick images/Neal/Blender/3001'
    folder = 'D:/references/vision/lego brick images/colors'
   # folder = 'D:/research/vision/coloredObjectIdentification'
   # folder = 'D:/references/vision/from 2D to 3D/images'
    folder_inhoud = os.listdir(folder)
    img_names = []
    img_files = []
    for f in folder_inhoud:
        img_names.append(f)
        ff = folder + '/' + f
        if os.path.isfile(ff):
            print(ff)
            img_files.append(ff)
    filePtr = 0
    img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, img_files[filePtr])
    
    
    #img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/research/robotics/grijper Arion Tommy/legobricks/image8.jpg')
    #img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/vision/lego brick images/Neal/Slecht_uitgeknipt/1585138705.08.jpg') # 'LegoBricks - medium - ROI red.jpg')
        
    # img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/vision/from 2D to 3D/images/Escher trap-ROI 1.jpg')
    
    height, width = img.shape[:2]   
    print("Image of "+str(width) +" x "+str(height))
    
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.namedWindow(window_image, cv2.WINDOW_NORMAL)
    #cv2.imshow(window_image, img)
    #cv2.setMouseCallback(window_image, on_click_show_pixel)

    cv2.imshow(window_thresholded_image, img)
    cv2.setWindowTitle(window_thresholded_image, 'Image '+img_names[filePtr])
    cv2.setMouseCallback(window_thresholded_image, on_click_show_pixel)
    
    
      
    #cv2.namedWindow(window_mask, cv2.WINDOW_NORMAL)

    while True:
        if blur_radius > 0:
            try:
                frame = cv2.GaussianBlur(img, (blur_radius, blur_radius), cv2.BORDER_DEFAULT)
                img_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            except Exception as e:
                print('Failed to blur image: '+str(e))
                frame = img
        else:
            frame = img
            
        #try:
        #    applyHSVRanges()
        #except Exception as e:
        #    print('Failed with thresholding: '+str(e))
            
        #key = cv2.waitKey(200)
        key = cv2.waitKey()
        if key > 0:
           try: 
                c = chr(key)
                #print("key = "+str(key)+" char = "+c)
                
                if c == '1':
                    on_contours()
                if c == 'q' or key==27:
                    break
                if c == 'x' or key == ord('x') or key == 112:
                    print('Lower HSV range: ['+str(low_H)+','+str(low_S)+','+str(low_V)+']')
                    print('Higher HSV range: ['+str(high_H)+','+str(high_S)+','+str(high_V)+']')
        
                if c == '2': 
                    show_HSV_histogram(img_HSV, mask=mask_threshold, name='HSV')
    
                if c == '3': 
                    CannyEdge.analyzeCannyEdge(img, img_file)
                if c == '4': 
                    GradientsWithSobel.analyzeGradients(img, img_file, comp = 'h', showOriginalImage = False)
 
               
 
                if c == '7' or c == '8':
                    new_image = False
                    while not new_image:
                        try:
                            if c == '7': # back
                                filePtr= (filePtr + len(img_files) - 1) % len(img_files)
                            if c == '8': # forward
                                filePtr= (1 + filePtr) % len(img_files)
                                
                            # print("Loading Image  "+img_files[filePtr])
                            img, img_file  = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, img_files[filePtr])
                            img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                            cv2.imshow(window_thresholded_image, img)
                            cv2.setWindowTitle(window_thresholded_image, 'Image '+img_names[filePtr])
                            print('Loaded image '+img_file)
                            new_image = True
                            mask_threshold = None
                        except:
                            print('Could not load image from file '+img_files[filePtr])
                    
                if c == '9': # reset
                    cv2.imshow(window_thresholded_image, img)
       
     
                    
                if c == 'h':
                    print('Color map: \n'+str(char2color))
                        
                color = char2color.get(c, None)
                if color is not None:
                   # print('Color set to '+str(color))
                    print('Chosen color = '+str(color)+': '+str(color.rangeLower())+' - '+str(color.rangeUpper()))
                    thresholdOnColor(color)
                   # print('Lower HSV range: ['+str(low_H)+','+str(low_S)+','+str(low_V)+']')
                   # print('Higher HSV range: ['+str(high_H)+','+str(high_S)+','+str(high_V)+']')
           except Exception as e:
               print('Error with key '+ chr(key)+': '+ str(e))
               
    cv2.destroyAllWindows()