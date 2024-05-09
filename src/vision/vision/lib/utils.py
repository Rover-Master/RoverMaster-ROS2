import numpy as np


def degrees(radians: float) -> float:
    return (radians * 180) / np.pi


def radians(degrees: float) -> float:
    return (degrees * np.pi) / 180


def normalize(vec: np.ndarray):
    return vec / np.linalg.norm(vec)
