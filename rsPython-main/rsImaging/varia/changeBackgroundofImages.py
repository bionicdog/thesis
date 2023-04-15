# -*- coding: utf-8 -*-
"""
Created on Fri May 15 19:20:06 2020

@author: Jan Lemeire

Niet-chromatische pixels (zwart/wit/grijs) worden vervangen door een random gekozen achtergrondbeeld
"""

import os
from tkinter import filedialog
import cv2
import random



folder = filedialog.askdirectory(title="Geef folder met images");
print('Gekozen images folder is ' + str(folder) )
folder_results = filedialog.askdirectory(title="Geef folder waarin nieuwe images moeten komen");
folder_bckgr = filedialog.askdirectory(title="Geef folder met backgrounds");

    # bckgr = 'D:/research/robotics/legosorteerband/backgrounds/floor.jpg
    
def processImage(file, fileName, background):
    global folder_results
    img=cv2.imread(file)
    if type(img) != type(None):
        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #mask_threshold = cv2.inRange(img_HSV, (0, 115, 80), (256, 256, 256))
        mask_threshold = cv2.inRange(img_HSV, (0, 42, 88), (256, 256, 256))
        
        img_combi = cv2.bitwise_and(img,img, background.copy(), mask= mask_threshold)
        
        file_to = folder_results + '/' + fileName[0:-4]+'_background'+str(idx)+'.jpg'
        cv2.imwrite(file_to, img_combi)
        #file_mask = folder_results + '/' + fileName[0:-4]+'mask.jpg'
        #cv2.imwrite(file_mask, mask_threshold)
        print('Result written to ', file_to)

backgrounds=[]
inhoud = os.listdir(folder_bckgr)
for file in inhoud:
    _file = folder_bckgr + '/' + file
    #print('Next ', str(file), ': ', os.path.isfile(_file))
    if os.path.isfile(_file):
        img=cv2.imread(_file)
        if type(img) != type(None):
            backgrounds.append(img)
print('Loaded ', len(backgrounds), ' backgrounds from ', folder_bckgr)

inhoud = os.listdir(folder)
for file in inhoud:
    _file = folder + '/' + file
    #print('Next ', str(file), ': ', os.path.isfile(_file))
    if os.path.isfile(_file):
        idx = random.randint(0, len(backgrounds) - 1)
        processImage(_file, file, backgrounds[idx])
