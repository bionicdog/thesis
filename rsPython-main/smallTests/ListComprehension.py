# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 2021

@author: Jan Lemeire

see also https://www.w3schools.com/python/python_lists_comprehension.asp
"""

fruits = ["apple", "banana", "cherry", "kiwi", "mango"]

newlist = [x[::-1] for x in fruits if "a" in x]  # x[::-1] reverses string

print(newlist)

print('Without filter:')

newlist2 = [x[::-1] for x in fruits]  # x[::-1] reverses string

print(newlist2)


def create_a_function(substring):
    def containsSubstring(word):
        return substring in word

    return containsSubstring

my_new_function = create_a_function("an")

print('Without created function:')

newlist = [x[::-1] for x in fruits if my_new_function(x)]  # x[::-1] reverses string

print(newlist)
