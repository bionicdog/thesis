# -*- coding: utf-8 -*-
"""
General utility functions for data

@author: Jan Lemeire
Created on Fri Jan 15 11:57:33 2021

"""

def arr2str(arr):
    s = '['
    for a in arr:
        if abs(a) > 10:
            s += str(int(a)) + ' '
        elif abs(a) > 0.1:
            s += str( int(a*100)/100) + ' '
        else:   
            s += str( int(a*10000)/10000) + ' '
    s += ']'
    return s

# Sum of squared errors
def sse(v):
    score = 0
    for x in v:
        score += x * x
    return score

