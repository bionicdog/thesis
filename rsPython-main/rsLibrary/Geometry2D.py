# -*- coding: utf-8 -*-
"""
Created on Tue Aug  2 10:02:26 2022

@author: Nick
"""

from math import atan2, sqrt

"""
Geometric utilities
"""

def distance(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def angle(L1, L2):
    """angle between two lines (3 coefficients)"""
    return atan2((-L2[0]*L1[1] + L1[0]*L2[1]) , (L1[0]*L2[0] + L1[1]*L2[1]))

def line3Coef(p1, p2):
    """
    returns the coefficients of the line equation:
        ax + by + c = 0
    Advantage: vertical lines!
    """
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersect(A, B, C, D):
    """
    checks for the intersection of segment AB with CD
    https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    """
    def ccw(A,B,C):
        """
        checks for counterclockwise rotation of three points:
        """
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
    
    