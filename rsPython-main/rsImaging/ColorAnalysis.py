
"""
ColorAnalysis.py
Analyze colors (RGB and HSV) of images through histograms and scatter plots

@author: Jan Lemeire
Created on Thu Mar 12 2020

about color spaces, see https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
"""

import cv2
from matplotlib import pyplot as plt
from matplotlib import colors
#from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import cm
import numpy as np

print('** Analyze colors of an image and image regions **')
print('Richtclick on pixel to see pixel colors.')
print('Choose points by clicking on image, press spacebar or p to create polygonic Region Of Interest (ROI).')    
print('Alternatively create rectangular ROI by dragging mouse.')
print('Press r to switch between RGB and HSV color space .')
print('Press h to show histogram of entire image or ROI.')
print('Press s to show scatter plot of entire image or ROI.')
print('Press q to quit.')

################ MAIN ################
if __name__== "__main__":
    
    from ImageUtils import selectAndOpenImageFile, getImage, Input 
    #original_img = selectAndOpenImageFile()
    #if original_img is None:
    #    exit
    # inlezen van welbepaalde file die in dezelfde folder zit
    #img = cv2.imread('20161115_162500.jpg')  # inlezen en omzetten met grijswaarden

    #original_img, img_file = getImage(Input.GIVEN_FILE, 'D:/research/robotics/grijper Arion Tommy/grijper tommy/pictures/image_2020-06-16 09_39_7.jpg')
    original_img, img_file  = getImage(Input.GIVEN_FILE, 'D:/research/robotics/grijper Arion Tommy/legobricks/image8.jpg')

height, width = original_img.shape[:2]
print("Image size {:d}x{:d} ".format(width, height))

# if bigger than RESCALE_TO we make the image smaller
RESCALE_TO = 1200
rescalefactor = int(height/RESCALE_TO)  
if rescalefactor > 1:
     img = cv2.resize(original_img,((int)(  width/rescalefactor), (int)(height/rescalefactor)), interpolation = cv2.INTER_AREA)
     height, width = img.shape[:2]
     print("Image rescaled to size {:d}x{:d} ".format(width, height))
else:
    img = original_img

img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def show_histogram(image, mask=None, name='RGB'):
    plt.figure(name)
    color = ('b', 'g', 'r')
    for i, col in enumerate(color):
        hist = cv2.calcHist([image], [i], mask, [256], [0, 256])
        plt.plot(hist, color=col, label = col)
    plt.xlim([0, 256])
   # plt.ylim([0, 100])
    plt.legend(['b', 'g', 'r'])
    plt.show()


def show_HSV_histogram(image, mask=None, name='HSV'):
    global img_hsv
    plt.figure(name)
    color = ('r', 'k', colors.CSS4_COLORS['gray'])
    
    for i, col in enumerate(color):
        hist = cv2.calcHist([img_hsv], [i], mask, [256], [0, 256])
        plt.plot(hist, color=col)
    plt.xlim([0, 256])
    plt.legend(['h', 's', 'v'])
    plt.show()

FACTOR = 8 # reduce 

def uniquePixels(image, PRECISION):
    height, width = image.shape[:2]
    pixel_colors = image.reshape((height * width, 3))
    pixel_colors = pixel_colors // PRECISION # nearby points are considered equal
    pixel_colors = np.unique(pixel_colors, axis=0) # eliminate doubles
    pixel_colors = pixel_colors * PRECISION
    return pixel_colors

def addScatter(image_rgb, axis, marker, toHSV = False):
    global img_hsv
    pixel_colors = uniquePixels(image_rgb, FACTOR)
    
    if toHSV:
        pixel_colors_img = np.uint8([pixel_colors]) #create image structure
        img_hsv = cv2.cvtColor(pixel_colors_img, cv2.COLOR_RGB2HSV)
        pixel_colors_hsv = img_hsv.reshape((np.shape(img_hsv)[0]*np.shape(img_hsv)[1], 3))
        comp1, comp2, comp3 = pixel_colors_hsv[:,0], pixel_colors_hsv[:,1], pixel_colors_hsv[:,2]
    else:
        comp1, comp2, comp3 = pixel_colors[:,0], pixel_colors[:,1], pixel_colors[:,2]

    norm = colors.Normalize(vmin=-1.,vmax=1.) 
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    axis.scatter(comp1.flatten(), comp2.flatten(), comp3.flatten(), facecolors=pixel_colors, marker=marker)


# https://realpython.com/python-opencv-color-spaces/
def show_scatter_plot(image, image2 = None, toHSV = False):
    global fig
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # colors is expecting RGB

    fig = plt.figure('RGB scatter plot' if image2 is None else 'RGB scatter plot of ROI (o) and rest of image(^)')
    axis = fig.add_subplot(1, 2, 1, projection="3d")
    if toHSV:
        axis.set_xlabel("Hue"); axis.set_ylabel("Saturation"); axis.set_zlabel("Value")
    else:
        axis.set_xlabel("Red"); axis.set_ylabel("Green"); axis.set_zlabel("Blue")
  #  axis.xlim([0, 256]) INCORRECT
  #  axis.ylim([0, 256])
  #  axis.zlim([0, 256])    
  
    addScatter(image_rgb, axis, 'o', toHSV)
    
    if not (image2 is None):
        image2_rgb = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)      
        addScatter(image2_rgb, axis, '^', toHSV)
        
    plt.show()


def inverseMask(mask_roi):
    mask_roi_inv = np.zeros(mask_roi.shape[0:2], dtype = np.uint8)
    mask255 = mask_roi * 255 # mask = 0 or 1, must be 0 or 255
    mask_roi_inv = cv2.bitwise_not(mask255, mask_roi_inv)
    return mask_roi_inv

def analyzePolygon(points):
    global img, mask_roi, mask_roi_inv
    pts = np.array(points)
    cv2.line(img, tuple(points[-1]), tuple(points[0]), (0,0,255), 1) # is not shown ????????
    #print('Connecting '+str(points[-1])+' with ' + str(points[0]))
    mask_roi = np.zeros(img.shape[0:2], dtype = np.uint8)
    cv2.fillConvexPoly(mask_roi, pts, 1)
    mask_roi_inv = inverseMask(mask_roi)
    print('ROI is created')
    


xmin=width; ymin=height; xmax=0; ymax=0
x1=0; y1=0
points = []

def on_click_show_pixel(event, x, y, flags, param):
    
    if event == cv2.EVENT_MOUSEMOVE:
        return
    global img, width, height, xmin, xmax, ymin, ymax, x1, y1
    
    if event == cv2.EVENT_LBUTTONDOWN and x < width and y < height:
        print("Clicked on [{:d}, {:d}] ".format(x, y))
        x1, y1 = x, y
        

    if event == cv2.EVENT_LBUTTONUP and x < width and y < height:
        #print("Button up on [{:d},{:d}] ".format(x, y))
        if (abs(x1-x) > 2 and abs(y1-y) > 2):
            global mask_roi, mask_roi_inv
            print('rectangular ROI')
            #crop_img = img[y1:y, x1:x] # select rectangular ROI
            # cv2.imshow("crop_img", crop_img)
            cv2.rectangle
            img_rect = cv2.rectangle(img,(x1,y1),(x,y),(0,255,0),2)
            cv2.imshow('Original image',img_rect)
            
            # mask
            mask_roi = np.zeros(img.shape[0:2], dtype = np.uint8)
            cv2.rectangle(mask_roi,(x1,y1),(x,y),1,-1) # fill
            mask_roi_inv = inverseMask(mask_roi)
            
            # ADD 2   cv2.bitwise_and
            
        else:
            points.append([x, y])
            if x < xmin:
                xmin = x
            if x > xmax:
                xmax = x
            if y < ymin:
                ymin = y
            if y > ymax:
                ymax = y
            cv2.circle(img, (int(x), int(y)), 2, (0,0,255), 2)
            if len(points) > 1:
                cv2.line(img, tuple(points[-1]), tuple(points[-2]), (0,0,255), 1)
            cv2.imshow('Original image', img)
         
    if event == cv2.EVENT_RBUTTONUP:
        blue, green, red = img[y, x]  # the inverse!!
        hue, saturation, value = img_hsv[y, x]
        text = "({:d},{:d}):(B={:d},G={:d},R={:}) (H={:d},S={:d},V={:}) ".format(x,y, blue, green, red, hue, saturation, value)
        print(text)
        text = "(B={:d},G={:d},R={:}) ".format(blue, green, red)
        #text = "(H={:d},S={:d},V={:}) ".format(hue, saturation, value)
        cv2.circle(img, (int(x), int(y)), int(3), (0,0,255), 2)
        cv2.putText(img, text,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.imshow('Original image',img)

        
cv2.namedWindow('Original image')
cv2.setMouseCallback('Original image', on_click_show_pixel)
cv2.imshow('Original image',img)

useRGB = False
mask_roi = None
mask_roi_inv = None

while True:
    key = cv2.waitKey(0)
    if key == 113 or key == 81: # 'q' for quit
        break
    elif key == 112 or key == 80 or key == 32: # 'p' or space
        if len(points) > 2:
            analyzePolygon(points)
            points = [] # reset
        else:
            print('select at least 3 points')
    elif key == 114 or key == 82: # 'r'
        useRGB = not useRGB
        if useRGB:
            print('RGB color space chosen. Press r to switch to HSV.')
        else:
            print('HSV color space chosen. Press r to switch to RGB.')
    elif key == 104 or key == 72: # 'h'
        print('Histogram(s) will be shown')
        if useRGB:
            if mask_roi is None:
                show_histogram(img, None, 'RGB of image')
            else:
                show_histogram(img, mask_roi, 'RGB of ROI')
                #show_histogram(img, mask_roi_inv, 'RGB of rest of image')
        else:
            if mask_roi is None:
                show_HSV_histogram(img, None, 'HSV of image')
            else:
                show_HSV_histogram(img, mask_roi, 'HSV of ROI')
                #show_HSV_histogram(img, mask_roi_inv, 'HSV of rest of image')

    elif key == 115 or key == 83: # 's'
        print('Scatter plot(s) will be shown')
        if not mask_roi is None:
             mask_bool = mask_roi.astype(np.bool)
             img_roi = np.zeros_like(img)
             img_roi[mask_bool] = img[mask_bool]
             mask_inv_bool = mask_roi_inv.astype(np.bool)
             img_roi_inv = np.zeros_like(img)
             img_roi_inv[mask_inv_bool] = img[mask_inv_bool]
        
        if mask_roi is None:
            show_scatter_plot(img, None, not useRGB)
        else:
            show_scatter_plot(img_roi, img_roi_inv, not useRGB)
    else:
        print('You pressed an undefined key: '+chr(key)+' ('+str(key)+')')

cv2.destroyAllWindows()