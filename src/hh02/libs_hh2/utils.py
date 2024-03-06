"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import math
import numpy as np


def deg_to_red(theta):
    return (theta * np.pi) / 180


def red_to_deg(theta):
    return (theta * 180) / np.pi


def DisplayFrame(R, t, ax, came_scale=1, Z_came_scale=0.1):
    """
    Given rotation matrix R and translation vector t
          Draw the frame of reference
    """
    R = R.T
    t = -np.dot(R, t)
    window11 = came_scale * np.array([1, 1, Z_came_scale]).reshape(3, 1)
    window12 = came_scale * np.array([-1, 1, Z_came_scale]).reshape(3, 1)
    window21 = came_scale * np.array([-1, -1, Z_came_scale]).reshape(3, 1)
    window22 = came_scale * np.array([1, -1, Z_came_scale]).reshape(3, 1)

    windowPrime11 = np.dot(R, window11) + t.reshape(3, 1)
    windowPrime12 = np.dot(R, window12) + t.reshape(3, 1)
    windowPrime21 = np.dot(R, window21) + t.reshape(3, 1)
    windowPrime22 = np.dot(R, window22) + t.reshape(3, 1)

    ax_colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0.5, 0.5, 0.5)]
    Z_came_scale = 1
    # axis
    for i in range(3):
        Xs = np.hstack((t[0], t[0] + came_scale * R[0, i]))
        Ys = np.hstack((t[1], t[1] + came_scale * R[1, i]))
        Zs = np.hstack((t[2], t[2] + came_scale * R[2, i]))
        ax.plot(Xs, Ys, Zs, c=ax_colors[i])

    col = ax_colors[-1]
    ## cones
    # ax.plot(np.hstack((windowPrime11[0], t[0])), np.hstack((windowPrime11[1], t[1])), np.hstack((windowPrime11[2], t[2])), c=col)
    # ax.plot(np.hstack((windowPrime12[0], t[0])), np.hstack((windowPrime12[1], t[1])), np.hstack((windowPrime12[2], t[2])), c=col)
    # ax.plot(np.hstack((windowPrime22[0], t[0])), np.hstack((windowPrime22[1], t[1])), np.hstack((windowPrime22[2], t[2])), c=col)
    # ax.plot(np.hstack((windowPrime21[0], t[0])), np.hstack((windowPrime21[1], t[1])), np.hstack((windowPrime21[2], t[2])), c=col)
    ## furthest plane
    Xs = np.hstack(
        (
            windowPrime11[0],
            windowPrime12[0],
            windowPrime21[0],
            windowPrime22[0],
            windowPrime11[0],
        )
    )
    Ys = np.hstack(
        (
            windowPrime11[1],
            windowPrime12[1],
            windowPrime21[1],
            windowPrime22[1],
            windowPrime11[1],
        )
    )
    Zs = np.hstack(
        (
            windowPrime11[2],
            windowPrime12[2],
            windowPrime21[2],
            windowPrime22[2],
            windowPrime11[2],
        )
    )
    ax.plot(Xs, Ys, Zs, c=col)

    return ax


def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. Euclidean norm, along axis."""
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data
