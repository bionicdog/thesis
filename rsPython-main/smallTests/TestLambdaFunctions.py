# -*- coding: utf-8 -*-
"""
Created on April 2023

@author: Jan Lemeire
"""

# see https://www.w3schools.com/python/python_lambda.asp

f = lambda a : a < 1

print(f(0))

print(f(1))

g = lambda d: d['a'] < 1

d = {}
d['a'] = 1
print('g(d)', g(d))



def createConditieForVariable( var: str, threshold:int):
    return lambda d: d[var] < threshold

h = createConditieForVariable('b', 1)

d['b'] = 0
print('h(d) met b = 0:', h(d))
d['b'] = 1
print('h(d) met b = 1:', h(d))

# type definieren
# https://stackoverflow.com/questions/33833881/is-it-possible-to-type-hint-a-lambda-function

from typing import Callable

g2: Callable[[dict], bool] = lambda d: d['a'] < 1

print('g2(d)', g2(d))