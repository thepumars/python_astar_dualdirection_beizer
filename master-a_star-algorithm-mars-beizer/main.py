# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 21:36:05 2023

@author: Mars
"""

# main.py

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Rectangle

import random_map
import a_star
def drawPic(l):
    plt.figure(figsize=(5,5))
    plt.xlim(0,50,0.001)     #x轴的刻度范围
    plt.ylim(0,50,0.001)       #y轴的刻度范围
    plt.xlabel('x轴',fontproperties="simhei")    #x轴的标题
    plt.ylabel('y轴',fontproperties="simhei")    #y轴的标题
	#绘制各个点
    for i in range(len(l)-1):
        plt.plot(l[i][0],l[i][1],'o',color='#0085c3',linewidth = 1)
    #连接各个点
    for i in range(len(l)-1):
        start = (l[i][0],l[i+1][0])
        end = (l[i][1],l[i+1][1])
        plt.plot(start,end,color='#0085c3',linewidth = 1)
    plt.show()
plt.figure(figsize=(5, 5))

map = random_map.RandomMap() 
ax = plt.gca()
ax.set_xlim([0, map.size]) 
ax.set_ylim([0, map.size])

for i in range(map.size): 
    for j in range(map.size):
        if map.IsObstacle(i,j):
            rec = Rectangle((i, j), width=1, height=1, color='gray')
            ax.add_patch(rec)
        else:
            rec = Rectangle((i, j), width=1, height=1, edgecolor='gray', facecolor='w')
            ax.add_patch(rec)

rec = Rectangle((0, 0), width = 1, height = 1, facecolor='b')
ax.add_patch(rec) 

rec = Rectangle((map.size-1, map.size-1), width = 1, height = 1, facecolor='r')
ax.add_patch(rec) 

plt.axis('equal') 
plt.axis('off')
plt.tight_layout()
#plt.show()

a_star = a_star.Astar(map)
path = a_star.RunAndSaveImage(ax, plt) 


drawPic(path)