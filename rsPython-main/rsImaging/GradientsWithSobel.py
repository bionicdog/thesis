# -*- coding: utf-8 -*-
"""
GradientsWithSobel.py

Created on 23 Feb 2022

@author: Jan Lemeire
"""

import cv2

# image
img= None
img_HSV= None
img_gray = None
img_comp = None
img_gradient = None

# parameters
blur_radius = 0
scale = 1
delta = 0
ksize=3
ddepth = cv2.CV_16S

gradient_threshold = 0.1

def setImage(image):
    global img, img_HSV, img_gray, img_comp, img_gradient
    img = image
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    
    img_comp = None    
    img_gradient = None

def chooseComponent(comp:str): # r, g, b, h, s, v, a (gray)
    global img, img_HSV, img_gray, img_comp
    if comp == 'r':
        img_comp = img[:, :, 2]
    elif comp == 'g':
        img_comp = img[:, :, 1]
    elif comp == 'b':
        img_comp = img[:, :, 0]
    elif comp == 'h':
        img_comp = img_HSV[:, :, 0]
    elif comp == 's':
        img_comp = img_HSV[:, :, 1]
    elif comp == 'v':
        img_comp = img_HSV[:, :, 2]
    elif comp == 'a':
        img_comp = img_gray
    else:
        print('Wrong component value '+str(comp)+'. Should be one of r, g, b, h, s, v, a (gray).')
    return img_comp
        
def gradientOfComponent(comp):
     global img_comp, img_gradient, blur_radius, ddepth, ksize, scale, delta
     print(' ******* component '+comp+' **************')
     if img_comp is None:
         chooseComponent(comp)
     
     img_blur = cv2.blur(img_comp, (blur_radius,blur_radius), 0) if blur_radius > 1 else img_comp
     
     grad_x = cv2.Sobel(img_blur, ddepth, 1, 0, ksize=ksize, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
     # Gradient-Y
     # grad_y = cv.Scharr(gray,ddepth,0,1)
     grad_y = cv2.Sobel(img_blur, ddepth, 0, 1, ksize=ksize, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
 
     abs_grad_x = cv2.convertScaleAbs(grad_x)
     abs_grad_y = cv2.convertScaleAbs(grad_y)
     img_gradient = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
     return img_gradient
     
################ analysis with GUI ################
def analyzeGradients(image, file_name, comp = 'h', showOriginalImage = True):
    global img, img_HSV, img_gray, img_comp, img_gradient, gradient_threshold
    
    setImage(image)
    
    print('** Test Gradients with the Sobel Filter **')
    print('Choose component: r, g, b, h, s, v, a (gray)')
    print('Press q to quit')
    

    ################ GUI ################    
    windows_component = 'ImageComponent'
    window_sobel = 'Sobeled image'
    windows_threshold_gradient = 'Threshold gradient'
    
    cv2.namedWindow(windows_component, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_sobel)
    cv2.namedWindow(windows_threshold_gradient, cv2.WINDOW_NORMAL)

    height, width = img.shape[:2] 
    img_size =  height *  width
        
   
    def showGradient():
        import ImageUtils
        cv2.imshow(window_sobel, img_gradient)
        
        hist_gradient = cv2.calcHist([img_gradient], [0], None, histSize = [256], ranges = [0, 256])
        # max, total, 10%
        tenpctval = ImageUtils.tenPctBin(hist_gradient, pct = gradient_threshold)
        print(str(tenpctval)+".")
        mask_threshold = cv2.inRange(img_gradient, (tenpctval), (256))
        
        img_masked = cv2.bitwise_and(img,img, mask= mask_threshold)
        cv2.imshow(windows_threshold_gradient, img_masked)
        cv2.setWindowTitle(windows_threshold_gradient, 'Thresholded gradient of '+comp+' at '+str(int(gradient_threshold*100))+' pct')
        
        ImageUtils.show_histogram(img_gradient, components=comp, mask = mask_threshold, name = 'Histogram of gradient')
        
        if False:
            import Contours
            # contour of gradient
            contours = cv2.findContours(img_gradient, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            minimalSize = 500
            contourObjects = Contours.contours2contourObjects(contours, color = comp, contourFilter = Contours.createSizeContourFilter(minimalSize))
            
            Contours.printContourObjects(contourObjects)
            Contours.drawContourObjects(img, contourObjects)
            cv2.imshow(window_image, img)
            
            #contourObject = Contours.maxObject(contourObjects)
            #contourObject.draw(img_comp)
            
            #cv2.imshow(windows_component, img_comp)
            
            #cv2.drawContours(img_comp, contours[0], -1, (0,0,255), thickness = 2)
        
        
        
    def on_click_show_pixel(event, x, y, flags, param):
       # global width, height, img, img_HSV, window_image
        #print('Clicked '+str(x)+', '+str(y)+': event '+str(event))
        if event == cv2.EVENT_LBUTTONUP:
            #print('Clicked '+str(x)+', '+str(y))
            if x < width  and y < height:
              
              intensity = img[y, x]  # omgekeerd!!
              blue, green, red  = intensity
              hsv_intensity = img_HSV[y, x]
              hue, saturation, value = hsv_intensity
              gray = img_gray[y, x]
              pix_x = int(x - width / 2)
              pix_y = int(height / 2 - y)
              text = "Pixel ({:d},{:d}): B={:d},G={:d},R={:}; H={:d},S={:d},V={:},gray={:} ".format(pix_x, pix_y, blue, green, red, hue, saturation, value, gray)
              print(text)
              if False:
                  #text = "(B={:d},G={:d},R={:}) ".format(blue, green, red)
                  text = "(H={:d},S={:d},V={:}) ".format(hue, saturation, value)
                  cv2.circle(img, (int(x), int(y)), int(3), (0,0,255), 2)
                  cv2.putText(img,text,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
                  cv2.imshow(window_image, img)
    if showOriginalImage:
        window_image = 'Original image'
        cv2.namedWindow(window_image, cv2.WINDOW_NORMAL)
        cv2.imshow(window_image, img)
        cv2.setWindowTitle(window_image, 'Image '+file_name)
        cv2.setMouseCallback(window_image, on_click_show_pixel)  
    
    
    img_comp = chooseComponent(comp)
    cv2.imshow(windows_component, img_comp)
    cv2.setWindowTitle(windows_component, 'Component '+comp)    
          
    gradientOfComponent(comp)
    showGradient()
    
    while True:
        key = cv2.waitKey()
        if key > 0:
           try: 
                c = chr(key)
               # print("key = "+str(key)+" char = "+c)
                
                if c == 'q' or key==27:
                    break
           #     if c == 'x' or key == ord('x') or key == 112:
           #         print('Lower HSV range: ['+str(low_H)+','+str(low_S)+','+str(low_V)+']')
           #         print('Higher HSV range: ['+str(high_H)+','+str(high_S)+','+str(high_V)+']')
        
                if c == '2': 
                    ImageUtils.show_HSV_histogram(img_HSV, name='HSV') # mask=mask_threshold, 
    
                if c == '3': 
                    import CannyEdge
                    CannyEdge.analyzeCannyEdge(img, img_file)
                    
       #         if c == '7' or c == '8':
       
                if c in 'rgbhsva':
                    comp = c
                    img_comp = chooseComponent(c)
                    cv2.setWindowTitle(windows_component, 'Component '+comp)
                    cv2.imshow(windows_component, img_comp)
                    gradientOfComponent(comp)
                    showGradient()
                if c == '+' or c == '=':
                    gradient_threshold += 0.02;
                    print('Gradient threshold set at '+str(gradient_threshold)+': ', end='')
                    showGradient()
                    
                if c == '-' or c == '_':
                    gradient_threshold -= 0.02;
                    print('Gradient threshold set at '+str(gradient_threshold)+': ', end='')
                    showGradient()
                    
                if c == '9': # reset
                    cv2.imshow(window_image, img)
                    
       
    
           except Exception as e:
               print('Error with key '+ chr(key)+': '+ str(e))
               
    if showOriginalImage:
        cv2.destroyWindow(window_image)
    cv2.destroyWindow(windows_component)
    cv2.destroyWindow(window_sobel)
    cv2.destroyWindow(windows_threshold_gradient)
    
################ MAIN ################
if __name__== "__main__":
    import ImageUtils
    
#    img_file = 'D:/references/vision/lego brick images/Neal/without lamp/1585148260.82.jpg'
   # img_file = 'D:/references/vision/lego brick images/details/IMG_6240a.jpg'
#    img_file = 'D:/research/vision/coloredObjectIdentification/image_2020-06-16 09_36_1 - brown NOK - S too low.png'
   # img_file = 'D:/research/vision/coloredObjectIdentification/IMG_6220_small_part.jpg'
    img_file = 'D:/references/vision/lego brick images/details/LegoBricks - medium - ROI red.jpg'
    #img_file = 'D:/references/vision/lego brick images/lego-bricks-1x1-3d.jpg'
    img=cv2.imread(img_file)
    
    height, width = img.shape[:2]   
    print("Image of "+str(width) +" x "+str(height)+": "+str(width * height)+" pixels.")
    
    if True:
        analyzeGradients(img, img_file)
    else:
        setImage(img)
        comp = 'v'
        chooseComponent(comp)
        gradientOfComponent(comp)
        ImageUtils.show_histogram(img_gradient, components=comp, name = 'Histogram of gradient of '+comp)
        # retain high gradient
        gradient_threshold = 50
        gradient_max =  256 # 100 #
        mask_threshold = cv2.inRange(img_gradient, gradient_threshold, gradient_max)
        
        ImageUtils.show_histogram(img_gradient, components=comp, mask = mask_threshold, name = 'Histogram of thresholded gradient on img_gradient of '+comp)
        comp='h'
        ImageUtils.show_histogram(chooseComponent(comp), components=comp, mask = mask_threshold, name = 'Histogram of thresholded gradient on img_comp of '+comp)
       # ImageUtils.show_histogram(img_comp, components=comp, mask = None, name = 'Histogram of thresholded gradient of '+comp)
    
### #### #### #### ### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ### #### #### ###