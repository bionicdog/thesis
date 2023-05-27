# -*- coding: utf-8 -*-
"""
ImageUtils.py

General utility functions for images: colors and opening image files

@author: Jan Lemeire, VUB
Created on Mon Mar 16 2020
"""
import cv2
from enum import Enum
import numpy as np
import collections

###### COLORS ######

ranges_lower = {}
ranges_upper = {}

class Color(Enum):
    BLUE = 0
    RED = 1
    GREEN = 2
    ORANGE = 3
    PINK = 9
    YELLOW = 8
    BROWN = 4
    BLACK = 5
    WHITE = 6
    GRAY = 7
    CHROMATIC = 10
    NONCHROMATIC = 11
    
    def rangeLower(self):
        return ranges_lower[self]
    def rangeUpper(self):
        return ranges_upper[self]

char2color = {}
char2color['b'] = Color.BLUE
char2color['r'] = Color.RED
char2color['g'] = Color.GREEN
char2color['o'] = Color.ORANGE
char2color['p'] = Color.PINK
char2color['y'] = Color.YELLOW
char2color['u'] = Color.BROWN
char2color['k'] = Color.BLACK
char2color['w'] = Color.WHITE
char2color['a'] = Color.GRAY
char2color['c'] = Color.CHROMATIC
char2color['n'] = Color.NONCHROMATIC

def char2Color(c):
    return char2color[c] if c in char2color.keys() else None

# chromatic
ranges_lower[Color.RED] =  np.array([168,119,0])
ranges_upper[Color.RED] = np.array([6,255,255])
ranges_lower[Color.BROWN] =  np.array([147,49,38])
ranges_upper[Color.BROWN] = np.array([6,119,199])

ranges_lower[Color.ORANGE] =  np.array([5,132,133])
ranges_upper[Color.ORANGE] = np.array([17,256,256])

#ranges_lower[Color.PINK] =  np.array([166,145,126]) 
ranges_lower[Color.PINK] =  np.array([166,79,111])
ranges_upper[Color.PINK] = np.array([3,256,256])

ranges_lower[Color.YELLOW] =  np.array([12,98,22])
ranges_upper[Color.YELLOW] = np.array([25,255,255])

ranges_lower[Color.GREEN] =  np.array([25,36,0])
ranges_upper[Color.GREEN] = np.array([44,255,255])

ranges_lower[Color.BLUE] = np.array([52,60,78]) 
ranges_upper[Color.BLUE] = np.array([132,255,255])

ranges_lower[Color.BLACK] = np.array([0,0,0]) 
ranges_upper[Color.BLACK] = np.array([148,62,75])

ranges_lower[Color.WHITE] = np.array([0,0,0]) 
ranges_upper[Color.WHITE] = np.array([256,256,90]) # low v-component

ranges_lower[Color.GRAY] = np.array([0,0,59]) 
ranges_upper[Color.GRAY] = np.array([256,80,140])

ranges_lower[Color.CHROMATIC] = np.array([0,50,0]) # S > 49   - in fact we should also limit V!!
ranges_upper[Color.CHROMATIC] = np.array([256,256,256])

ranges_lower[Color.NONCHROMATIC] = np.array([0,0,0]) 
ranges_upper[Color.NONCHROMATIC] = np.array([256,49,256])


def show_histogram(img, components:str='HSV', mask=None, name:str='HSV of image'):
    from matplotlib import pyplot as plt, colors

    print('+ + + '+name+' + + +')
    plt.figure(name)
    line_colors = ('r', 'k', colors.CSS4_COLORS['gray'])
    
    _legend = []
    _max = 0
    for i in range(0, len(components)):
        hist = cv2.calcHist([img], [i], mask, histSize = [256], ranges = [0, 256])
        _max = max(maxNonZeroBin(hist), _max)
        plt.plot(hist, color=line_colors[i])
        _legend.append(components[i])
    plt.xlim([0, _max])
    plt.legend(_legend)
    plt.show()
    
def maxNonZeroBin(hist):
    for i in range(len(hist)-1, -1, -1):
        if hist[i] > 0:
            return i
    return 0

def tenPctBin(hist, pct:float=0.1):
    total = sum(hist)[0]
    threshold = pct * total
    _sum = 0
    for i in range(len(hist)-1, -1, -1):
        _sum += hist[i][0]
        if _sum > threshold:
            return i
    return i
    
def valTenPctBin(hist, pct:float=0.1):
    idx = tenPctBin(hist, pct=pct)
    return hist[idx][0]
    
def show_HSV_histogram(imgHSV, mask=None, name='HSV'):
    from matplotlib import pyplot as plt, colors
    print(' = = = '+name+' = = =')
    plt.figure(name)
    color = ('r', 'k', colors.CSS4_COLORS['gray'])
    
    for i, col in enumerate(color):
        hist = cv2.calcHist([imgHSV], [i], mask, [256], [0, 256])
        plt.plot(hist, color=col)
    plt.xlim([0, 256])
    plt.legend(['h', 's', 'v'])
    plt.show()    
    
###### DRAWING ######
def isList(var):
    return isinstance(var, (collections.Sequence, np.ndarray))

def drawPoints(img, points, color = (0,0,255), radius = 8, connect = True):
    prev = points[-1]
   # print('type = '+str(type(prev)))
    if isList(prev) and isList(prev[0]): # opencv polygon returns each point within an array
        prev = prev[0]
        
    if type(prev[0]) is not int:
        prev = ( int(prev[0]), int(prev[1]) )

    for p in points:
        if isList(prev) and isList(prev[0]):
            p = p[0]
        if type(p[0]) is not int:
            p = ( int(p[0]), int(p[1]) )
        cv2.circle(img, p , 1, (0, 127, 255), radius)
        if connect:
            cv2.line(img, p, prev,5)
        prev = p

###### ACCESSING FUNCTIONS ######
# documentation: see https://docs.python.org/3.10/library/dialog.html
# read file through dialog
def selectAndOpenImageFile():
       
    from tkinter import filedialog
    import tkinter
    tkinter.Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    filename = filedialog.askopenfilename() # show an "Open" dialog box and return the path to the selected file
    img = cv2.imread(filename)  # inlezen 
    return img, filename

def selectExistingFile():
       
    from tkinter import filedialog
    import tkinter
    tkinter.Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    filename = filedialog.askopenfilename() # show an "Open" dialog box and return the path to the selected file
    return filename
    
def selectNewFile():
       
    from tkinter import filedialog
    import tkinter
    tkinter.Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    filename = filedialog.asksaveasfilename() # show an "Open" dialog box and return the path to the selected file
    return filename
    

class Input(Enum):
    GIVEN_FILE = 0
    CHOOSE_FILE = 1
    CURRENT_FILE = 2
    USB_CAMERA = 3
    PI_CAM = 4
    

def getImage(inputType, filename = '', cameraNbr = 0):
    if inputType == Input.CHOOSE_FILE:
        return selectAndOpenImageFile()
    elif inputType == Input.GIVEN_FILE:
        if type(filename) is not str:
            print("String expected with Input.GIVEN_FILE")
        return cv2.imread(filename), filename
    elif inputType == Input.CURRENT_FILE:
        import UserPreferences
        filename = UserPreferences.getUserPref('currentImageFile')
        if filename is None:
            return selectAndOpenImageFile()
        return cv2.imread(filename), filename

    elif inputType == Input.USB_CAMERA:
        return None, None
    elif inputType == Input.PI_CAMERA:
        return None, None
    
def resizeImage(img, scalePercentage):
    if scalePercentage == 100:
        return img
    width = int(img.shape[1] * scalePercentage / 100)
    height = int(img.shape[0] * scalePercentage / 100)
    return cv2.resize(img, (width, height)) 
################ #### ################