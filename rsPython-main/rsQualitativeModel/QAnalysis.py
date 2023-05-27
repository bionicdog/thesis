# -*- coding: utf-8 -*-
"""
Created on May 1st 2023

@author: Jan Lemeire
"""

from sklearn import svm
import matplotlib.pyplot as plt
import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from rsLibrary.Variable import Variable, RobotVariable, DerivedVariable, VariableType, QualType, OrdinalType
from rsQualitativeModel.QHistory import QHistory
from rsQualitativeModel.QDataType import Sign

def createPredictFunction(clf1, clf2):
    def predict(xy):
        pos = clf1.predict(xy)
        neg = clf2.predict(xy)
        return np.array( [ Sign.PLUS if pos[i] == 1 else Sign.MINUS if neg[i] == 1 else Sign.ZERO for i in range(len(xy))] )
#        return np.array( [ 2 if pos[i] == 1 else 0 if neg[i] == 1 else -1 for i in range(len(xy))] )
    return predict

# https://scikit-learn.org/stable/modules/svm.html#svm-classification
def testForDet2D(x: list[float], y:list[float], q:list[Sign], show:bool=False):
    
    if False:
        plot_chars = ( '_', '+' ,'o',   '|')
        colors = ('blue', 'red', 'green')
        unique_labels = set(q)
        for j, label in enumerate(unique_labels):
            _xs = [ t for i, t in enumerate(x) if q[i] == label]
            _ys = [ t for i, t in enumerate(y) if q[i] == label]
            plt.plot(_xs, _ys, plot_chars[j], color=colors[j])
        plt.title('Data')
        plt.show()    
            
    xy = list(zip(x, y)) # list of tuples
    
    nbr_pos = len( [ qv for qv in q if qv == Sign.PLUS ] )
    
    if nbr_pos > 0:
        q_pos = [ int(qv == Sign.PLUS) for qv in q ]
        clf_pos = svm.SVC(kernel='linear', gamma='auto', C=1.0)
        clf_pos.fit(xy, q_pos)
       # showSVM(x, y, q_pos, clf_pos)
    else:
        print('No positives in set...')

    nbr_neg = len( [ qv for qv in q if qv == Sign.MINUS ] )
    nbr_zero = len( [ qv for qv in q if qv == Sign.ZERO ] )
    
    if nbr_neg > 0:
        q_neg = [ int(qv == Sign.MINUS) for qv in q ]
        clf_neg = svm.SVC(kernel='linear', gamma='auto', C=1.0)
        clf_neg.fit(xy, q_neg)
       # showSVM(x, y, q_neg, clf_neg)
    else:
        print('No negatives in set...')

    
    predict_function = createPredictFunction(clf_pos, clf_neg)
    q_predict = predict_function(xy)
    
    conflicts = [ (xy[i][0], xy[i][1] , q[i], q_predict[i]) for i in range(len(xy)) if q[i] != q_predict[i]]
    
    print('Conflicts at ', conflicts)

    if show: showDoubleSVM(x, y, q, clf_pos, clf_neg, extraPoints = conflicts)
    
    #  return xy, q_pos, clf
    
   # q_int = [ int(qv) for qv in q ]
   # lin_clf = svm.LinearSVC()
   # lin_clf.fit(xy, q_int)
   # return xy, q_int, lin_clf
   
    return len(conflicts) > 0

def showSVM(x: list[float], y:list[float], labels: list, clf):
    plot_chars = ( 'o', '+' , '_', '|')
    colors = ('blue', 'red', 'green')
    unique_labels = set(labels)
    for j, label in enumerate(unique_labels):
        _xs = [ t for i, t in enumerate(x) if labels[i] == label]
        _ys = [ t for i, t in enumerate(y) if labels[i] == label]
        plt.plot(_xs, _ys, plot_chars[j], color=colors[j])
    
    # show the support vectors
    support_vectors = clf.support_vectors_
    plt.scatter(support_vectors[:, 0], support_vectors[:, 1], color='green')
    
    # Create a meshgrid of points to evaluate the decision boundary
  #  x_min, x_max = xy[:, 0].min() - 1, xy[:, 0].max() + 1
  #  y_min, y_max = xy[:, 1].min() - 1, xy[:, 1].max() + 1
    x_min = min(x) -1; y_min = min(y) - 1
    x_max = max(x) + 1; y_max = max(y) + 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, 0.1), np.arange(y_min, y_max, 0.1))
    Z = clf.predict(np.c_[xx.ravel(), yy.ravel()])
    Z = Z.reshape(xx.shape)
    plt.contourf(xx, yy, Z, alpha=0.4, cmap='coolwarm')
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)

    plt.title('Decision boundary')
    plt.show()
    

    
    
def showDoubleSVM(x: list[float], y:list[float], labels: list, clf1, clf2, extraPoints = None):
    plot_chars = ( '+', '+' , '+', '|')
    colors = ('blue', 'yellow', 'green')
    unique_labels = set(labels)
    for j, label in enumerate(unique_labels):
        _xs = [ t for i, t in enumerate(x) if labels[i] == label]
        _ys = [ t for i, t in enumerate(y) if labels[i] == label]
        plt.plot(_xs, _ys, plot_chars[j], color=colors[j])
    
    # show the support vectors
    if False:
        support_vectors = clf1.support_vectors_
        plt.scatter(support_vectors[:, 0], support_vectors[:, 1], color='green')
 
        support_vectors = clf2.support_vectors_
        plt.scatter(support_vectors[:, 0], support_vectors[:, 1], color='black')
 
    # Create a meshgrid of points to evaluate the decision boundary
  #  x_min, x_max = xy[:, 0].min() - 1, xy[:, 0].max() + 1
  #  y_min, y_max = xy[:, 1].min() - 1, xy[:, 1].max() + 1
    x_min = min(x) -1; y_min = min(y) - 1
    x_max = max(x) + 1; y_max = max(y) + 1
    
    predict_function = createPredictFunction(clf1, clf2)
    xx, yy = np.meshgrid(np.arange(x_min, x_max, 0.1), np.arange(y_min, y_max, 0.1))
    Z = predict_function(np.c_[xx.ravel(), yy.ravel()])
    Z = np.array( [ int(x) + 1 for x in Z ] ) # convert to numbers
    Z = Z.reshape(xx.shape)
    plt.contourf(xx, yy, Z, alpha=0.4, cmap='coolwarm')
    
    if extraPoints != None:
        _xs = [x[0] for x in extraPoints]
        _ys = [x[1] for x in extraPoints]
        plt.plot(_xs, _ys, 'o', color='red')
        
    
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.title('Decision boundary')
    plt.show()
    
### #### #### #### ### #### #### #### #### #### #### #### ###      
### #### #### ####       DEMO    CODE      #### #### #### ###
### #### #### #### ### #### #### #### #### #### #### #### ### 
if __name__== "__main__":
    print('*** Code analysis.py ***')
    
    if False:
        # data from robot 1, hist.plot('x_old', 'x', 'qdx')
        x_old = [0, 1.0, 2.0, 2.0, 4.0, 4.0, 6.0, 5.0, 4.0, 2.0, 1.0, 1.0, 1.0, 2.0, 4.0]
        x = [1.0, 2.0, 2.0, 4.0, 4.0, 6.0, 5.0, 4.0, 2.0, 1.0, 1.0, 1.0, 2.0, 4.0, 6.0]
        qdx = [Sign(1),Sign(1),Sign(0),Sign(1),Sign(0),Sign(1),Sign(-1),Sign(-1),Sign(-1),Sign(-1),Sign(0),Sign(0),Sign(1),Sign(1),Sign(1)]
        #xy, q_pos, clf = 
        testForDet2D(x_old, x, qdx, show=True)
        
        
    if False: # werkt nog niet
        import pickle
        file = open('robot1History.pkl', 'rb')
        data = pickle.load(file)
        file.close()
        print(' * * *  Loaded robot data of '+file.name+' * * *')
        
        _vars = [ RobotVariable(d, VariableType.STATEVAR) for d in data]
        
        hist = QHistory(*_vars)
        for v in hist.data:
            hist[v] = data[v.name]
            
        hist.plot('x_old', 'x', 'qdx') 