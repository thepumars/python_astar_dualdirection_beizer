'''
(GA) path planning with Genetic algorithm
@Author: Zhihai Bi
Last edit: 2021/12/16
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
from CurveGenerator.B_spline import approximate_b_spline_path
import CurveGenerator.Bezier as Bezier

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class GA:
    def __init__(self, startPoint, goalPoint, chromosome_size, population_size, cross_rate, mutate_rate, elitismNum, maxIter):
        self.startPoint = startPoint
        self.goalPoint = goalPoint
        self.chromosome_size = chromosome_size  # 染色体长度
        self.population_size = population_size  # 种群大小
        self.cross_rate = cross_rate
        self.mutate_rate = mutate_rate
        self.maxIter = maxIter
        self.elitismNum = elitismNum
        self.bestIndividual = np.zeros((2, self.chromosome_size))  # 记录最优个体
        self.selectNum = 70  # 选择保留的个数
        self.minFit = 10000  # 记录当前最小适应度值

        self.utils = Utils.Utils()
        self.plotting = Plotting.Plotting(startPoint, goalPoint)
        self.env = Envs.Env()

    def Genetic_algorithm(self):
        """
        :遗传算法主流程
        :return:成功规划则返回最优个体，否则返回失败提示
        """
        count = 0
        Flag = 0
        population = self.Popu_init()
        for i in range(self.maxIter):
            fitTable = self.Fit_function(population)
            SeqFitTable = np.sort(fitTable)
            population = self.Selection(fitTable, population)
            count = self.Is_over(fitTable, count)
            if count >= 5:
                print("success!")
                Flag = 1
                break
            population = self.Crossover(population)
            population = self.Mutation(population)

        if Flag == 1:
            return self.bestIndividual
        else:
            print("fail!")
            return False

    def Popu_init(self):
        """
        :初始化种群信息
        :return:
        """
        stepSize = (self.goalPoint[0] - self.startPoint[0]) / (self.chromosome_size - 1.0)
        population = np.zeros((self.population_size, 2, self.chromosome_size))
        for i in range(self.population_size):
            for j in range(self.chromosome_size):
                randPoint = [self.startPoint[0] + j * stepSize,
                             random.randint(self.startPoint[1], self.goalPoint[1])]  # 默认目标点在起始点右上角
                population[i, :, j] = randPoint
            population[i, 1, 0] = self.startPoint[1]
            population[i, 1, self.chromosome_size-1] = self.goalPoint[1]
        return population

    def Fit_function(self, population):  # J = distance  + obs_err
        """
        :计算种群的适应度函数，返回代价列表，用于后续选择操作
        :param population:
        :return:
        """
        fitTable = []
        for i in range(self.population_size):
            cost = 0
            for j in range(self.chromosome_size - 1):
                cost = self.Calculate_cost(population[i, :, j], population[i, :, j + 1]) + cost
            fitTable.append(cost)
        return fitTable

    def Calculate_cost(self, backPosition, prePosition):
        """
        :总代价 = 欧氏距离 + 碰到障碍物的代价
        :param backPosition:
        :param prePosition:
        :return:
        """
        DistanceCost = math.sqrt(
            pow((backPosition[0] - prePosition[0]), 2) + pow((backPosition[1] - prePosition[1]), 2))
        backPositionNode = Node(backPosition)
        prePositionNode = Node(prePosition)
        obstacleCost = self.utils.is_collision(backPositionNode, prePositionNode)
        if obstacleCost:
            return DistanceCost + 100.0  # 10 might be changed!!!
        return DistanceCost

    def Selection(self, fitTable, population):  # select 70% from original population
        """
        :选择适应度函数在前70%的个体保留下来
        :param fitTable:
        :param population:
        :return:
        """
        sorted_id = np.argsort(fitTable)
        NewPopulation = np.zeros((self.population_size, 2, self.chromosome_size))
        self.bestIndividual = population[sorted_id[0], :, :]  # 更新最优个体
        for i in range(self.selectNum):
            NewPopulation[i, :, :] = population[sorted_id[i], :, :]
        return NewPopulation

    def Crossover(self, population):
        """
        :对前60%的个体进行两两交叉，产生新个体，补充回选择后的种群中，使得种群大小不变
        :param population:
        :return: population[[[]]]
        """
        for i in range(self.population_size - self.selectNum):
            crossPosition = random.randint(1, self.chromosome_size - 2)  # 生成交叉位置
            newIndividual = np.zeros((2, self.chromosome_size))
            for j in range(self.chromosome_size):  # 开始交叉，产生新个体
                if j < crossPosition:
                    newIndividual[:, j] = population[i * 2, :, j]
                else:
                    newIndividual[:, j] = population[i * 2 + 1, :, j]
            population[i + self.selectNum, :, :] = newIndividual  # 补充回原种群中
        return population

    def Mutation(self, population):
        """
        :根据节点前后连线是否有碰撞决定是否变异
        :param population:
        :return:
        """
        for i in range(self.population_size - self.elitismNum):
            for j in range(self.chromosome_size-2):
                backNode = Node(population[self.elitismNum+i, :, j])
                nowNode = Node(population[self.elitismNum+i, :, j+1])
                for k in range(10):  # 加快变异
                    if self.utils.is_collision(backNode, nowNode):  # 有障碍物，需要变异
                        MutatePosition = random.randint(self.startPoint[1], self.goalPoint[1])
                        nowNode.y = MutatePosition
                    else:
                        break
                population[self.elitismNum+i, 1, j+1] = nowNode.y
        return population

    def Is_over(self, fitTable, count):
        """
        :判断是否成功找到路径,连续5次迭代最小代价无变化，且路径安全，则认为成功找到
        :param fitTable:
        :return:
        """
        sorted_id = np.argmin(fitTable)
        if count >= 1 and fitTable[sorted_id] == self.minFit:
            count = count + 1
        else:
            count = 0
        if count == 0 and fitTable[sorted_id] < 1.2*math.sqrt(pow((self.goalPoint[1]-self.startPoint[1]), 2) + pow((self.goalPoint[0]-self.startPoint[0]), 2)):
            count = 1
            self.minFit = fitTable[sorted_id]
        return count


if __name__ == '__main__':
    GeneticA = GA([5, 5], [45, 45], 10, 100, 0.5, 0.5, 10, 100)
    result = GeneticA.Genetic_algorithm()
    path = []
    traj = []
    traj.append([GeneticA.startPoint[0], GeneticA.startPoint[1]])
    if result is not False:
        for i in range(GeneticA.chromosome_size):
            path.append(Node(result[:, i]))
            traj.append([result[0, i], result[1, i]])
        bezier = Bezier.Bezier(100)
        smoothPathBezier = bezier.calc_bezier_path_G1(traj)
        GeneticA.plotting.animation([], path, smoothPathBezier, [], 'Genetic Algorithm', animation=True)


