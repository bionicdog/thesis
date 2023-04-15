# -*- coding: utf-8 -*-
"""
Created on Thu Mar 12 22:23:52 2020

@author: Jan Lemeire

from https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

xs = [1, 2, 3, 4, 5]
ys = [5, 4, 3, 4, 5]
zs = [1, 2, 3, 4, 5]
ax.scatter(xs, ys, zs)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()