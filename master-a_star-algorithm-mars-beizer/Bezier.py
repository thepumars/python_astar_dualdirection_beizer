'''
Second order Bezier curve Path-Smoothing with G1 continuity
@Author: Alanby
Last edit: 2021/11/24
'''

import numpy as np
from scipy.special import comb
from math import floor


class Bezier:
    def __init__(self, n_points):
        self.samplePoints = n_points

    def calc_bezier_path_G1(self, WayPoints):
        controlPoints = self.WayPoints_to_ControlPoints(WayPoints)
        traj = []
        imax = floor(len(controlPoints) / 2)

        for i in range(imax):
            for t in np.linspace(0, 1, self.samplePoints):  # Bezier parameter t
                traj.append(self.bezier(t, controlPoints[0 + i * 2: 3 + i * 2]))
        path = np.array(traj)

        return path

    def WayPoints_to_ControlPoints(self, WayPoints):  # generate the control points according to the way points
        WayPointsSample = []
        sampling = 1
        for i in range(int(len(WayPoints)/sampling)):  # down sampling for the way points with 4 intervals
            WayPointsSample.append(WayPoints[i*sampling])
        WayPointsSample.append(WayPoints[len(WayPoints)-1])  # add the start point to the way point
        print(WayPointsSample[0])
        ControlPoints = []
        for i in range(len(WayPointsSample)-1):
            cp = [(WayPointsSample[0+i][0] + WayPointsSample[1+i][0]) / 2, (WayPointsSample[0+i][1] + WayPointsSample[1+i][1]) / 2]
            ControlPoints.append(cp)
            if i+1 < len(WayPointsSample):
                ControlPoints.append(WayPointsSample[i+1])
        return np.array(ControlPoints)

    def Comb(self, n, i, t):  # Bernstein polynomials
        return comb(n, i) * t ** i * (1 - t) ** (n - i)

    def bezier(self, t, controlPoints):  # Bezier equation
        n = len(controlPoints) - 1
        return np.sum([self.Comb(n, i, t) * controlPoints[i] for i in range(n + 1)], axis=0)  # Bezier curve
