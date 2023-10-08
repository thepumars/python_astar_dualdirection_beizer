'''
(RRT) Rapidly-Exploring Random Trees with constraints
@Author: Alanby
Last edit: 2021/11/06
'''

import os
import sys
import random
import math
import numpy as np

import Utils
import Plotting
import Envs

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PathPlanning/CurveGenerator/")
import CurveGenerator.Bezier as Bezier
from CurveGenerator.B_spline import approximate_b_spline_path

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RRT:
    def __init__(self, startPoint, goalPoint, stepSize, maxIter, oddsOfTarget):
        self.startPoint = Node(startPoint)
        self.goalPoint = Node(goalPoint)
        self.stepSize = stepSize
        self.maxIter = maxIter
        self.oddsOfTarget = oddsOfTarget
        self.exitNode = []

        self.utils = Utils.Utils()
        self.plotting = Plotting.Plotting(startPoint, goalPoint)
        self.env = Envs.Env()

    def Planning(self):
        self.exitNode.append(self.startPoint)
        for i in range(self.maxIter):
            newRandNode = self.Generate_randNode()
            nearestNode = self.Find_nearest_neighbor(newRandNode)
            newRealNode = self.Calculate_newNode(newRandNode, nearestNode)
            if newRealNode and self.Is_constraint_satisfaction(newRealNode, nearestNode):
                newRealNode.parent = nearestNode
                self.exitNode.append(newRealNode)
                distance = math.hypot(newRealNode.x - self.goalPoint.x, newRealNode.y - self.goalPoint.y)
                if distance < self.stepSize and self.Is_constraint_satisfaction(newRealNode, self.goalPoint):
                    return self.Extract_path(newRealNode)
        print('Fail to find the path')
        return None

    def Generate_randNode(self):
        if random.random() > self.oddsOfTarget:
            return Node([random.uniform(self.env.x_range[0] + 0.5, self.env.x_range[1] - 0.5),
                         random.uniform(self.env.y_range[0] + 0.5, self.env.y_range[1]-0.5)])
        else:
            return self.goalPoint

    def Find_nearest_neighbor(self, newRandNode):
        return self.exitNode[np.argmin([math.hypot(node.x - newRandNode.x, node.y - newRandNode.y)  # how to write it in simple way!!!
                       for node in self.exitNode])]

    def Calculate_newNode(self, newRandNode, nearestNode):  # how to calculate the angle of two point
        angle = math.atan2(newRandNode.y - nearestNode.y, newRandNode.x - nearestNode.x)
        newRealNode = Node([nearestNode.x + self.stepSize * math.cos(angle), nearestNode.y + self.stepSize * math.sin(angle)])
        return newRealNode

    def Is_constraint_satisfaction(self, newRealNode, nearestNode):
        obstacleConstraint = self.utils.is_collision(nearestNode, newRealNode)
        KinematicConstraint = self.utils.Kinematic_check(nearestNode, newRealNode, self.goalPoint)  # assume as the robot kinematic constraint
        return not KinematicConstraint and not obstacleConstraint

    def Extract_path(self, endNode):
        path = []
        traj = []
        traj.append([self.goalPoint.x, self.goalPoint.y])
        traj.append([self.goalPoint.x, self.goalPoint.y])
        node_now = endNode
        while node_now.parent is not None:
            node_now = node_now.parent
            path.append(node_now)
            traj.append([node_now.x, node_now.y])
        traj.append([self.startPoint.x, self.startPoint.y])
        return path, traj


if __name__ == '__main__':
    startPoint = [5, 5]
    goalPoint = [45, 45]
    stepSize = 1
    maxIter = 10000
    oddsOfTarget = 0.05

    rrt = RRT(startPoint, goalPoint, stepSize, maxIter, oddsOfTarget)
    path, traj = rrt.Planning()

    if path:
        # Path-Smoothing with Bezier
        bezier = Bezier.Bezier(100)
        smoothPathBezier = bezier.calc_bezier_path_G1(traj)
        # Path-Smoothing with clamped B-spline

        traj = np.array(traj)
        rax, ray = approximate_b_spline_path(list(traj.T[0]), list(traj.T[1]), 100)
        smoothPathBspline = (np.array([rax, ray])).T
        # plotting
        rrt.plotting.animation(rrt.exitNode, path, smoothPathBezier, smoothPathBspline, 'RRT', animation=True)




