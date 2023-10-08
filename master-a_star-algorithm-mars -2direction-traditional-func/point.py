# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 20:40:16 2023
point.py
@author: Mars
"""
# point.py
import sys

class Point:
    
        def __init__(self,x,y):
            
            self.x = x
            self.y = y
            self.cost = sys.maxsize
            
