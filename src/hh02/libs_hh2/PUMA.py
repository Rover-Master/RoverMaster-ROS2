"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np
from .Transforms import X, Y, Z, rotate_around, get_transform


class Puma_FK:
    def __init__(self, a2, a3, d3, d4):
        # all alpha_{i-1}
        self.alpha = [0, -np.pi / 2, 0, -np.pi / 2, np.pi / 2, -np.pi / 2]
        # all a_{i-1}
        self.a = [0, 0, a2, a3, 0, 0]
        # all d_i
        self.d = [None, 0, 0, d3, d4, 0, 0]

    def get_Ti(self, i=1, theta_i=0):
        # return the transformation {i-1}T{i}

        theta = theta_i
        d = self.d[i]
        Rz = rotate_around(Z, theta)
        Tz = get_transform(Rz, np.array([0, 0, d]))

        alpha = self.alpha[i - 1]
        a = self.a[i - 1]
        Rx = rotate_around(X, alpha)
        Tx = get_transform(Rx, np.array([a, 0, 0]))

        return Tz @ Tx
