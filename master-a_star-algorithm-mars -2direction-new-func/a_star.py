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



class Astar:
    
    def __init__(self,map):
        self.map = map
        self.open_set1 = []
        self.close_set1 = []
        self.open_set2 = []
        self.close_set2 = []
        self.head = point.Point(None,None)
        self.tail = point.Point(None,None)
        self.target_set = 0
        self.a = 2
        self.b = 0.5
    def BaseCost(self,p1,p2):
        x_dis = p2.x - p1.x
        y_dis = p2.y -p1.y
        
        return x_dis + y_dis + (np.sqrt(2)- 2)*min(x_dis,y_dis)
    
    def Gcost(self,p):

        return p.parent.gcost+self.BaseCost(p.parent,p)
        
        
    def TotalCost(self,p1,p2):
        return (1-self.b)*(self.BaseCost(p1,point.Point(self.map.size-1,self.map.size-1))/(1+self.a)+self.a*self.BaseCost(p1,p2)/(1+self.a))+self.b*self.Gcost(p1)
    def Cost_Update(self,p):
        pass
        
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
    def VisitAround(self,x,y,mode,parent,s_point):
        if mode == 's':
            if not self.IsValidPoint(x, y):
                return # Do nothing for invalid point
            p = point.Point(x, y)
           
            if self.IsInCloseList(p,self.close_set1):
                return # Do nothing for visited point
            if self.IsInPointList(p,self.close_set2):
                
                
                for _point in self.close_set2:
                   if _point.x == p.x and _point.y == p.y:
                       p = _point
                       break   
                self.target_set = 1
                self.head = parent
                self.tail = p
                return
            if not self.IsInOpenList(p,self.open_set1):
                p.parent = parent
                p.fcost = self.TotalCost(p,s_point)
                p.gcost = self.Gcost(p)
                self.open_set1.append(p)
                print('Process PointS [', p.x, ',', p.y, ']', ', cost: ', p.fcost)
            else:
                for _point in self.open_set1:
                   if _point.x == p.x and _point.y == p.y:
                        p = _point
                        break
                if self.Gcost(p) < p.gcost:
                    p.parent = parent
                    p.gcost = self.Gcost(p)
                    p.fcost = self.TotalCost(p,s_point)
        elif mode == 'e':
            if not self.IsValidPoint(x, y):
                return # Do nothing for invalid point
            p = point.Point(x, y)
            
               
            
            if self.IsInCloseList(p,self.close_set2):
                return # Do nothing for visited point
            if self.IsInPointList(p,self.close_set1):
                
                
                for _point in self.close_set1:
                   if _point.x == p.x and _point.y == p.y:
                       p = _point
                       break   
                self.target_set = 2
                self.head = p
                self.tail = parent
                return
            if not self.IsInOpenList(p,self.open_set2):
                p.parent = parent
                p.fcost = self.TotalCost(p,s_point)
                p.gcost = self.Gcost(p)
                self.open_set2.append(p)
                print('Process PointE [', p.x, ',', p.y, ']', ', cost: ', p.fcost)
            else:
                for _point in self.open_set2:
                   if _point.x == p.x and _point.y == p.y:
                        p = _point
                        break
                    
                if self.Gcost(p) < p.gcost:
                    p.parent = parent
                    p.gcost = self.Gcost(p)
                    p.fcost = self.TotalCost(p,s_point)
    def ProcessPoint(self,parent,s_point,mode):
        x = parent.x
        y = parent.y
        
        self.VisitAround(x-1,y+1,mode,parent,s_point)
        self.VisitAround(x-1,y,mode,parent,s_point)
        self.VisitAround(x-1,y-1,mode,parent,s_point)
        self.VisitAround(x,y-1,mode,parent,s_point)
        self.VisitAround(x+1,y-1,mode,parent,s_point)
        self.VisitAround(x+1,y,mode,parent,s_point)
        self.VisitAround(x+1,y+1,mode,parent,s_point)
        self.VisitAround(x,y+1,mode,parent,s_point)

        
        
  
    def SelectPointInOpenList(self,openlist):
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in openlist:
            fcost = p.fcost
            if fcost < min_cost:
                min_cost = fcost
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
                print('p1(',p1.x,',',p1.y,')s father')
                p1 = p1.parent
                print('(',p1.x,',',p1.y,')')
        while True:
            path.append(p2) # Insert first
            if self.IsEndPoint(p2):
                break
            else:
                print('p2(',p2.x,',',p2.y,')s father')
                p2 = p2.parent
                print('(',p2.x,',',p2.y,')')
                        
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
        start_point.gcost = 0
        start_point.fcost = 0
        
        self.open_set1.append(start_point)
        end_point = point.Point(self.map.size-1,self.map.size-1)
        end_point.gcost = 0
        end_point.fcost = 0
        self.open_set2.append(end_point)
        p1 = start_point
        
        while self.open_set1 and self.open_set2:
            
            index1 = self.SelectPointInOpenList(self.open_set1)
            p1 = self.open_set1[index1]
            rec1 = Rectangle((p1.x,p1.y), 1, 1, color = 'c')
            ax.add_patch(rec1)
            del self.open_set1[index1]
            self.close_set1.append(p1)
            index2 = self.SelectPointInOpenList(self.open_set2)
            p2 = self.open_set2[index2]
            rec2 = Rectangle((p2.x,p2.y), 1, 1, color = 'y')
            ax.add_patch(rec2)
            del self.open_set2[index2]
            self.close_set2.append(p2)
           
            self.ProcessPoint(p1,p2,'s')
            self.ProcessPoint(p2,p1,'e')
            
        
            #self.SaveImage(plt)
            
            
            if self.target_set:
                self.BuildPath(self.head,self.tail,ax, plt, start_time)
                return
            
            
        print('No path found,algorithm failed!!!')
        return           
            


            
                    
           
    
            
            
            # Process all neighbors
            
            
    
    