# -*- coding: utf-8 -*-
"""
Created on Tue Apr 18 12:46:00 2023

@author: Jan Lemeire
"""

import matplotlib.pyplot as plt
from sklearn import svm
import numpy as np

# https://scikit-learn.org/stable/modules/svm.html#svm-classification

def plot_decision_boundary(positive_points: List[tuple], negative_points: List[tuple], kernel='linear', degree=3, gamma='auto'):
    # Combine the positive and negative points into one array
    points = np.concatenate([positive_points, negative_points])
    
    # Create an array of labels (1 for positive points, -1 for negative points)
    labels = np.concatenate([np.ones(len(positive_points)), -np.ones(len(negative_points))])
    
    # Train an SVM classifier with the specified kernel function
    clf = svm.SVC(kernel=kernel, degree=degree, gamma=gamma, C=1.0)
    clf.fit(points, labels)
    
    # Get the support vectors
    support_vectors = clf.support_vectors_
    
    # Plot the points and the support vectors
    plt.scatter(positive_points[:, 0], positive_points[:, 1], color='blue')
    plt.scatter(negative_points[:, 0], negative_points[:, 1], color='red')
    plt.scatter(support_vectors[:, 0], support_vectors[:, 1], color='green')
    
    # Create a meshgrid of points to evaluate the decision boundary
    x_min, x_max = points[:, 0].min() - 1, points[:, 0].max() + 1
    y_min, y_max = points[:, 1].min() - 1, points[:, 1].max() + 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, 0.1), np.arange(y_min, y_max, 0.1))
    Z = clf.predict(np.c_[xx.ravel(), yy.ravel()])
    Z = Z.reshape(xx.shape)
    
    # Plot the decision boundary
    plt.contourf(xx, yy, Z, alpha=0.4, cmap='coolwarm')
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.xlabel('x1')
    plt.ylabel('x2')
    plt.title('Decision boundary')
    plt.show()
    
    return clf
    
    
positive_points = [(1, 1.2), (1, 2), (1, 3), (2, 3), (1, 4), (2, 4), (3, 4)]
negative_points = [(1, 0.8), (2, 1), (3, 1), (4, 1), (3, 2), (4, 2), (4, 3)]



clf = plot_decision_boundary( np.array(positive_points), np.array(negative_points))

Zp = clf.predict(np.array(positive_points))
Zn = clf.predict(np.array(negative_points))

if False:
    # random data
    positive_points = np.random.multivariate_normal([1, 1], [[1, 0], [0, 1]], size=50)
    negative_points = np.random.multivariate_normal([-1, -1], [[1, 0], [0, 1]], size=50)
    
    # Plot the points
    
    plt.scatter(positive_points[:, 0], positive_points[:, 1], color='blue')
    plt.scatter(negative_points[:, 0], negative_points[:, 1], color='red')
    plt.show()
    
    plot_decision_boundary(positive_points, negative_points)

if False:
    # 3 classes
    class_1_points = np.random.multivariate_normal([1, 1], [[1, 0], [0, 1]], size=50)
    class_2_points = np.random.multivariate_normal([-1, -1], [[1, 0], [0, 1]], size=50)
    class_3_points = np.random.multivariate_normal([1, -1], [[1, 0], [0, 1]], size=50)
    
    # Concatenate the points and labels into a single array
    X = np.vstack([class_1_points, class_2_points, class_3_points])
    y = np.hstack([np.ones(50), np.ones(50)*2, np.ones(50)*3])
    
    # Plot the points
    colors = ['blue', 'red', 'green']
    for i in range(3):
        plt.scatter(X[y==i+1, 0], X[y==i+1, 1], color=colors[i])
    plt.show()