import numpy as np
from math import radians


from .util import R, Vec

def solve(J1: float, J2: float, J3: float, L1: float, L2: float, L3: float):
    r: float = 0.0
    t = Vec(0, 0)
    joints: list[tuple[float, np.ndarray]] = []
    for _r, _t in [(J1, L1), (J2, L2), (J3, L3)]:
        r += _r
        t += R(radians(r)) @ Vec(_t, 0)
        joints.append((r, t.copy()))
    return joints
