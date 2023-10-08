# -*- coding: utf-8 -*-
"""
Created on Mon Sep 25 20:54:18 2023
a_star.py
@author: Mars
"""

# a_star.py

import sys

import time 

import numpy as np

from matplotlib.patches import Rectangle

import point 

import random_map

class Astar:
    
    def __init__(self,map):
        self.map = map
        self.open_set1 = []
        self.close_set1 = []
        self.open_set2 = []
        self.close_set2 = []
        self.head = point.Point(100,100)
        self.tail = point.Point(100,100)
        self.target_set = 0
    def BaseCost(self,p):
        x_dis = p.x
        y_dis = p.y
        
        return x_dis + y_dis + (np.sqrt(2)- 2)*min(x_dis,y_dis)
    
    def HeuristicCost(self,p):
        
        x_dis = self.map.size - 1 - p.x
        y_dis = self.map.size - 1 - p.y
        
        return x_dis + y_dis + (np.sqrt(2)- 2)*min(x_dis,y_dis)
        
        
    def TotalCost(self,p):
        return self.BaseCost(p) + self.HeuristicCost(p)
    
    def IsValidPoint(self,x,y):
        
        if x<0 or y<0:
            return False
        if x >= self.map.size or y >= self.map.size:
            return False
        
        return not self.map.IsObstacle(x,y)
    
    def IsInPointList(self,p,point_list):
         for _point in point_list:
             if _point.x == p.x and _point.y == p.y:
                 return True
         return False
         
    def IsInOpenList(self, p,openset):
        return self.IsInPointList(p, openset)

    def IsInCloseList(self, p,closeset):
        return self.IsInPointList(p, closeset)

    def IsStartPoint(self, p):
        return p.x == 0 and p.y ==0
    def IsEndPoint(self,p):
        return p.x == self.map.size - 1 and p.y == self.map.size - 1
    
            
                    
        return False
    def SaveImage(self, plt):
        millis = int(round(time.time() * 1000))
        filename = './' + str(millis) + '.png'
        plt.savefig(filename)
    
    def ProcessPoint(self, x, y, parent,mode):
        if self.target_set == False:
            if mode == 's':
                if not self.IsValidPoint(x, y):
                    return # Do nothing for invalid point
                p = point.Point(x, y)
                if self.IsInCloseList(p,self.close_set1):
                    return # Do nothing for visited point
                if self.IsInPointList(p,self.close_set2):
                    
                    for _point in self.close_set2:
                       if _point.x == p.x and _point.y == p.y:
                            self.target_set = 1
                            self.head = p
                            self.tail = _point
                if not self.IsInOpenList(p,self.open_set1):
                    p.parent = parent
                    print(p.parent.cost)
                    p.cost = self.TotalCost(p)
                    self.open_set1.append(p)
                    print('Process PointS [', p.x, ',', p.y, ']', ', cost: ', p.cost)
                else:
                    if self.TotalCost(p) < p.cost:
                        p.parent = parent
                        p.cost = self.TotalCost(p)
                        print('Process PointS [', p.x, ',', p.y, ']', ', cost: ', p.cost)
            elif mode == 'e':
                if not self.IsValidPoint(x, y):
                    return # Do nothing for invalid point
                p = point.Point(x, y)
                if self.IsInCloseList(p,self.close_set2):
                    return # Do nothing for visited point
                if self.IsInPointList(p,self.close_set1):
                   
                    for _point in self.close_set1:
                        if _point.x == p.x and _point.y == p.y:
                            self.target_set = 2
                            self.head = _point
                            self.tail = p
                            
                if not self.IsInOpenList(p,self.open_set2):
                    p.parent = parent
                    p.cost = self.TotalCost(p)
                    self.open_set2.append(p)
                    print('Process PointE [', p.x, ',', p.y, ']', ', cost: ', p.cost)
                else:
                    if self.TotalCost(p) < p.cost:
                        p.parent = parent
                        p.cost = self.TotalCost(p)
                        print('Process PointE [', p.x, ',', p.y, ']', ', cost: ', p.cost)
		

    def SelectPointInOpenList(self,openlist):
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in openlist:
            cost = self.TotalCost(p)
            if cost < min_cost:
                min_cost = cost
                selected_index = index
            index += 1
        return selected_index
    
    def BuildPath(self, p1,p2, ax, plt, start_time):
        path = []
        while True:
            path.insert(0, p1) # Insert first
            if self.IsStartPoint(p1):
                break
            else:
                p1 = p1.parent
        while True:
            path.append(p2) # Insert first
            if self.IsEndPoint(p2):
                break
            else:
                p2 = p2.parent        
        for p in path:
            rec = Rectangle((p.x, p.y), 1, 1, color='g')
            ax.add_patch(rec)
            plt.draw()
            # self.SaveImage(plt)
        end_time = time.time()
        print('===== Algorithm finish in', int(end_time-start_time), ' seconds')
        print('coordinate of the path is as follows')
        for p in path:
            print(p.x,p.y)
    def RunAndSaveImage(self,ax,plt):
        start_time = time.time()
        start_point = point.Point(0,0)
        start_point.cost = 0
        self.open_set1.append(start_point)
        end_point = point.Point(self.map.size-1,self.map.size-1)
        end_point.cost = 0
        self.open_set2.append(end_point)
        while True:
            index1 = self.SelectPointInOpenList(self.open_set1)
            index2 = self.SelectPointInOpenList(self.open_set2)
            if index1 < 0 or index2 < 0:
                print('No path found,algorithm failed!!!')
                return
            #对于正向搜索的遍历实�?
            p1 = self.open_set1[index1]
            x = p1.x
            y = p1.y
            self.ProcessPoint(x-1, y+1, p1,'s')
            self.ProcessPoint(x-1, y, p1,'s')
            self.ProcessPoint(x-1, y-1, p1,'s')
            self.ProcessPoint(x, y-1, p1,'s')
            self.ProcessPoint(x+1, y-1, p1,'s')
            self.ProcessPoint(x+1, y, p1,'s')
            self.ProcessPoint(x+1, y+1, p1,'s')
            self.ProcessPoint(x, y+1, p1,'s')
            rec1 = Rectangle((p1.x,p1.y), 1, 1, color = 'c')
            ax.add_patch(rec1)
            del self.open_set1[index1]
            self.close_set1.append(p1)
            
            
            #对于反向搜索的遍历实�?
            p2 = self.open_set2[index2]
            x = p2.x
            y = p2.y
            self.ProcessPoint(x-1, y+1, p2,'e')
            self.ProcessPoint(x-1, y, p2,'e')
            self.ProcessPoint(x-1, y-1, p2,'e')
            self.ProcessPoint(x, y-1, p2,'e')
            self.ProcessPoint(x+1, y-1, p2,'e')
            self.ProcessPoint(x+1, y, p2,'e')
            self.ProcessPoint(x+1, y+1, p2,'e')
            self.ProcessPoint(x, y+1, p2,'e')
            rec2 = Rectangle((p2.x,p2.y), 1, 1, color = 'y')
            ax.add_patch(rec2)
            del self.open_set2[index2]
            self.close_set2.append(p2)
            if self.target_set==1:
                self.BuildPath(self.head,self.tail,ax, plt, start_time)
                return
            elif self.target_set==2:
                self.BuildPath(self.head,self.tail,ax, plt, start_time)
                return
            #self.SaveImage(plt)


            
                    
           
    
            
            
            # Process all neighbors
            
            
    
    