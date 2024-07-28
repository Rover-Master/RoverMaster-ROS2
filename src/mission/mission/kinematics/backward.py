import numpy as np
from .util import R, Vec, internal_angles, normalize_angle
from math import radians, degrees, sqrt, atan2


def solve(x, y, r, L1, L2, L3) -> tuple[float, float, float] | None:
    ee = Vec(x, y)
    # Solve for position of J3
    j3 = ee - R(radians(r)) @ Vec(L3, 0)
    # Check for feasibility of J3 position
    dist = sqrt(float(np.sum(j3**2)))
    if dist > abs(abs(L1) + abs(L2)) or dist < abs(abs(L1) - abs(L2)):
        return None
    # Find rotational angle for pseudo joint J1-J3
    delta_r = degrees(atan2(j3[1], j3[0]))
    # Find viable solution(s) using Heron's formula
    # a = L1, b = L2, c = dist
    alpha, beta, gamma = internal_angles(L1, L2, dist)
    # dual solutions
    solution_a = [delta_r + beta, gamma - 180.0, r - delta_r + alpha]
    solution_b = [delta_r - beta, 180.0 - gamma, r - delta_r - alpha]
    return [
        [normalize_angle(r) for r in solution_a],
        [normalize_angle(r) for r in solution_b],
    ]
