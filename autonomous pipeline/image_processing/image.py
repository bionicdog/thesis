import cv2
from matplotlib import pyplot as plt

def readImage(path):
    return cv2.imread(path) # cv2.cvtColor(cv2.imread(path),cv2.COLOR_BGR2RGB)

def Rgb2HsvImage(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

def showImage(img):
    plt.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    plt.show()
    return
