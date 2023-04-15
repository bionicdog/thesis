# -*- coding: utf-8 -*-
"""
Created on Wed Jun 24 10:10:29 2020

@author: Jan Lemeire
"""

PREFS_FILE_NAME = 'UserPreferencesInPython.txt'

try:
    prefs_file = open(PREFS_FILE_NAME, "r") # FileNotFoundError: [Errno 2] No such file or directory: 'UserPreferencesInPython.txt'
    content = ''
    for line in prefs_file:
        content += line
    userPreferences = eval(content)
    prefs_file.close()
except:
    userPreferences = {}
    #if prefs_file is not None:
    #    prefs_file.close()
    

def setUserPref(key, value):
    global userPreferences
    userPreferences[key] = value
    saveUserPrefs()

def getUserPref(key):
    global userPreferences
    return userPreferences[key] if key in userPreferences else None

def saveUserPrefs():
    global userPreferences
    output_file = open(PREFS_FILE_NAME, "w")
    output_file.write(str(userPreferences))
    output_file.close()

def printUserPrefs():
    global userPreferences
    import os
    path = os.getcwd()
    print('User preferences on this location ('+path+'):')
    for key in userPreferences.keys():
        print(str(key)+": "+str(userPreferences[key]))
    
################ TESTING ################
#if __name__== "__main__":