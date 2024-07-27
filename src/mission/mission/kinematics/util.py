import numpy as np
from math import sin, cos, degrees, acos


def R(theta: float):
    return np.array(
        [
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)],
        ]
    )


def Vec(x: float, y: float) -> np.ndarray:
    return np.array([[x, y]], dtype=np.float64).T


def internal_angles(a, b, c) -> tuple[float, float, float]:
    def angle(a, b, c):
        return degrees(acos((b**2 + c**2 - a**2) / (2 * b * c)))

    alpha = angle(a, b, c)
    beta = angle(b, c, a)
    gamma = angle(c, a, b)

    return alpha, beta, gamma


def normalize_angle(degrees: float) -> float:
    return ((degrees + 180) % 360) - 180
