from scipy.spatial import Delaunay
from math import sqrt, pow
import numpy as np

def getLineClass(classPoint0, classPoint1):
    if classPoint0 != classPoint1:
        return 2 # mixed
    else: # when 2 points are from the same class
        return classPoint0 # 0 = yellow, 1 = blue

def getCenterPoints(edges):
    centersPoints = []
    for points in edges:
        centersPoints.append([(points[0][0]+points[1][0])/2,
                              (points[0][1]+points[1][1])/2])
    return centersPoints

def getDistance(point0, point1):
    return sqrt(pow(point1[0]-point0[0], 2) + pow(point1[1]-point0[1], 2))

class Delaunay:
    def __init__(self):
        self.blue_edges = []
        self.yellow_edges = []
        self.mixed_edges = []
    
    def delaunay(self, coordinates, class_ids):
        # reset edges
        self.blue_edges = []
        self.yellow_edges = []
        self.mixed_edges = []

        # calculate delaunay_triangles
        delaunay_input = [coordinate[0] for coordinate in coordinates]
        delaunay_triangles = Delaunay(delaunay_input)

        for triangle in delaunay_triangles.simplices:
            point0 = delaunay_input[triangle[0]]
            point1 = delaunay_input[triangle[1]]
            point2 = delaunay_input[triangle[2]]

            class0 = class_ids[triangle[0]]
            class1 = class_ids[triangle[1]]
            class2 = class_ids[triangle[2]]

            # first line
            classLine0 = getLineClass(class0, class1)
            if classLine0 == 0:
                self.yellow_edges.append([point0, point1])
            elif classLine0 == 1:
                self.blue_edges.append([point0, point1])
            else:
                self.mixed_edges.append([point0, point1])
            
            # second line
            classLine1 = getLineClass(class0, class2)
            if classLine1 == 0:
                self.yellow_edges.append([point0, point2])
            elif classLine1 == 1:
                self.blue_edges.append([point0, point2])
            else:
                self.mixed_edges.append([point0, point2])
            
            # third line
            classLine2 = getLineClass(class1, class2)
            if classLine2 == 0:
                self.yellow_edges.append([point1, point2])
            elif classLine2 == 1:
                self.blue_edges.append([point1, point2])
            else:
                self.mixed_edges.append([point1, point2])
        
        return self.yellow_edges, self.blue_edges, self.mixed_edges
    
    def getPath(self):
        path = []
        path.append([0, 0]) # start from origin, is position of car

        pathPoints = getCenterPoints(self.mixed_edges)
        while len(pathPoints) > 0:
            prev_point = path[-1]
            distances = [getDistance(prev_point, point) for point in pathPoints]
            # next pathPoint is the closest point not yet part of the path
            next_point_idx = np.argmin(distances)
            path.append(pathPoints[next_point_idx])
            # remove point from pathPoints once added to the path
            pathPoints.pop(next_point_idx)
        # the list of points that make up the path is returned
        return path