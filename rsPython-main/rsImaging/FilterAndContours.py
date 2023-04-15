"""
FilterAndContours.py

Identify objects through thresholding on color ranges

Created on Thu Mar 12 2020

@author: Jan Lemeire & Jonathan Ntumba Kanyinda

"""
import cv2
import ImageUtils





################ Contours ################
    
def contoursOfColor(img_hsv, color):
    color_object = ImageUtils.char2color.get(c, None)
    return contoursOfHSVRange(img_hsv, color_object.rangeLower(), color_object.rangeUpper())
    
def contoursOfHSVRange(img_hsv, range_lower, range_upper):
    if range_lower[0] < range_upper[0]:
        mask = cv2.inRange(img_hsv, range_lower, range_upper)
    else:
        range_upper2 = range_upper.copy()
        range_upper2[0] = 256
        mask1 = cv2.inRange(img_hsv, range_lower, range_upper2)
        range_lower2 = range_lower.copy()
        range_lower2[0] = 0
        mask2 = cv2.inRange(img_hsv, range_lower2, range_upper)
        mask = cv2.bitwise_or(mask1, mask2)
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return contours, mask
    

        


def contoursOnHSV(img, range_lower, range_upper, minimal_size = 30, show_filtered_image = True):
    height, width = img.shape[:2]
    
    ## TO showContours $$$$$$$$$$$$
    def doWithContour(contour):
        #global height, width
        area = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour)
        #shape = detectShape(contour)
        #circ = 2 * w + 2 * h
        relSize = area * 100 / ( width * height)
        filling = int(area / w * 100 / h)
        expanse = w / h if w > h else h / w 
        epsilon = 0.01*cv2.arcLength(contour,True)
        polygon = cv2.approxPolyDP(contour,epsilon,True)
 #       print('('+str(i)+') area='+str(area)+' ['+str(x+w//2)+', '+str(y+h//2)+', '+str(w)+' x '+str(h)+'] '+' rel size = 0.'+str( relSize ) +'% filling = ' +str(filling)+'pct')
        print('('+str(i)+') rel size =  {:.2f}% filling = '.format(relSize) +str(filling)+'pct, expanse = {:.1f}x, polygon with {:d}corners'.format(expanse, len(polygon)))
            
            
       
        drawPoints(img_contour, polygon, color = (0, 255, 0))
        #cv2.polylines(img_contour, [polygon], False, (0, 255, 0), 2, lineType=cv2.LINE_AA) 
        
        #cv2.line(img_contour,(80, 201),(80, 629),(255,0,0),5)
            
        cv2.circle(img_contour, (int(x), int(y)), 1, (0,0,255), 2)
        cv2.rectangle(img_contour, (int(x), int(y)), (int(x+w), int(y+h )), (0,0,255), 3)

        text = "{:d}: {:d}corners".format(i, len(polygon))
        
        #cv2.rectangle(img_contour, (int(x), int(y)), (int(w), int(h )), (0,0,255), 2)
        cv2.putText(img_contour, text,(x+w//2,y+25), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),4,cv2.LINE_AA)
        cv2.drawContours(img_contour, contour, -1, (0, 255, 0), 1) # draws points

    
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
    contours, mask = contoursOfHSVRange(img_hsv, range_lower, range_upper)
        
    if show_filtered_image:
        cv2.namedWindow('mask', cv2.WINDOW_NORMAL) 
        cv2.imshow('mask', mask)
        img_filter = cv2.bitwise_and(img, img, mask = mask)
        cv2.namedWindow('filtered image', cv2.WINDOW_NORMAL) 
        cv2.imshow('filtered image', img_filter)
    
    print('Found '+str(len(contours[1]))+' contours')
    img_contour = img.copy()
    i=0
   # contour = maxContours(contours)
   # doWithContour(contour)
    
    for contour in contours[1]:
        area = cv2.contourArea(contour)
        if area > minimal_size:
            i += 1
            doWithContour(contour)
    if __name__!= "__main__":
        cv2.namedWindow('image with contours', cv2.WINDOW_NORMAL)
    cv2.imshow('image with contours', img_contour)  
        
    
    
def maxContours(contours):
    _max = 0
    contour_max = None
    idx=0
    for contour in contours[1]:
        area = cv2.contourArea(contour)
        if _max < area:
            _max = area
            contour_max = contour
        idx += 1
    return contour_max


################ MAIN ################
if __name__== "__main__":
    from ImageUtils import selectAndOpenImageFile, getImage, Input 
    
    #    img=cv2.imread('D:/research/robotics/grijper Arion Tommy/grijper tommy/U-vorm - 480 op 640.jpg')
    #img=cv2.imread('D:/research/robotics/grijper Arion Tommy/grijper tommy/0degrees.jpg') #image4_blauwe U - 15 op 10cm.jpg')
    #img=cv2.imread('D:/research/robotics/grijper Arion Tommy/grijper tommy/pictures/image_2020-06-23 12_27_1.jpg') #image4_blauwe U - 15 op 10cm.jpg')
#    img = getImage(Input.CURRENT_FILE, 'D:/references/robotics/camera and image processing/finishfotos/test4_1920.0x1080.0.jpg')
    #img = getImage(Input.GIVEN_FILE, 'D:/references/robotics/camera and image processing/robotfotos/image_2020-12-22_15.40_1_1920.0x1080.0.jpg')
    #img = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/references/robotics/camera and image processing/robotfotos_NOK/image_2020-12-22_19.09_6_1920x1080_original.jpg')
    img, img_file = ImageUtils.getImage(ImageUtils.Input.GIVEN_FILE, 'D:/mycode/smartdot/imaging/robotfotos/image_2021-01-05_15.13_5_1920x1080_original.jpg')
    cv2.namedWindow('image with contours', cv2.WINDOW_NORMAL)
    cv2.imshow('image with contours', img)

    #img=cv2.imread('finishfotos/test19.jpg') # test2_exposure-6.0.jpg') # test2_1920.0x1080.0.jpg') # test1_1920.0x1080.0_closeup.jpg')
    #range_lower = np.array([168,39,0])     range_upper = np.array([6,255,255])  RED
    #range_lower = np.array([0,0,59])  # grijs
    #range_upper = np.array([256,80,112])
    #range_lower = np.array([31,50,0]) # blauw
    #range_upper = np.array([168,255,255])
    while True:
        print("Choose color or 'q' to quit or 'h' for help")
        key = cv2.waitKey()
        if key > 0:
            c = chr(key)
            #print("key = "+str(key)+" char = "+c)
            
            if c == 'q' or key==27:
                break
            if c == 'h':
                print('Color map: \n'+str(ImageUtils.char2color))
                
            color = ImageUtils.char2color.get(c, None)
            if color is not None:
                print('Chosen color = '+str(color)+': '+str(color.rangeLower())+' - '+str(color.rangeUpper()))
                contoursOnHSV(img, color.rangeLower(), color.rangeUpper(), minimal_size = 50, show_filtered_image= True)
    
    # wait until a key is pressed   
   # cv2.waitKey(0)
    cv2.destroyAllWindows()
