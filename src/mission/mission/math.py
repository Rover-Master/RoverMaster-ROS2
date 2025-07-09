# ==============================================================================
# Author: Yuxuan Zhang (robotics@z-yx.cc)
# License: MIT
# ==============================================================================
from sys import float_info
EPS = float_info.epsilon

def sign(value: float) -> int:
    """Return the sign of a value"""
    if value > 0:
        return 1
    elif value < 0:
        return -1
    else:
        return 0


def clamp(min: float, max: float) -> float:
    """lambda function to clamp a value between min and max"""
    assert min <= max

    def fn(value: float) -> float:
        return max if value > max else min if value < min else value

    return fn


def ang_diff(src: float, dst: float, half_period=180.0, direction=None) -> float:
    """
    Check the minimum angular distance between two angles.
    Positive if dst is clockwise from src, negative otherwise.
    """
    period = 2 * half_period
    diff = dst % period - src % period
    if diff > half_period:
        diff -= 2 * half_period
    elif diff < -half_period:
        diff += 2 * half_period
    if direction is not None and direction != 0:
        if diff < 0 and direction > 0:
            diff += period
        elif diff > 0 and direction < 0:
            diff -= period
    return diff


def interpolate(*pt: tuple[float, float]):
    """Linear interpolation using a set of points"""

    def clean(l):
        if len(l) < 2:
            return l
        (x0, last_y), *l = l
        v = [last_y]
        for x, y in l:
            if x != x0:
                yield (x0, sum(v) / len(v))
                x0, v = x, [y]
            else:
                v.append(y)
        yield (x0, sum(v) / len(v))

    curve = list(clean(sorted(pt, key=lambda p: p[0])))
    segments = list((curve[i - 1], curve[i]) for i in range(1, len(curve)))

    def fn(x: float) -> float:
        search = segments.copy()
        while len(search):
            i = len(search) // 2
            (x0, y0), (x1, y1) = search[i]
            if x < x0:
                search = search[:i]
            elif x > x1:
                search = search[i + 1 :]
            else:
                break
        k0 = (x - x0) / (x1 - x0)
        k1 = 1 - k0
        return k1 * y0 + k0 * y1

    return fn


def near_zero(value: int | float, EPS=EPS):
    abs(value) <= EPS


def project(src: tuple[float, float], dst: tuple[float, float], clamp: bool = False):
    x1, x2 = src
    y1, y2 = dst

    assert x1 != x2, f"Invalid projection source range ({x1}, {x2})"
    scale = (y2 - y1) / (x2 - x1)

    def projection(value: float) -> float:
        return y1 + (value - x1) * scale

    if clamp:

        def clamp_projection(value: float) -> float:
            return y1 if value < x1 else y2 if value > x2 else projection(value)

        return clamp_projection
    else:
        return projection


def period_constraint(left: float, right: float):
    """Return a function that constrains a value to given period window"""
    offset = float(min(left, right))
    period = float(abs(right - left))
    assert period > 0, f"Invalid period range ({left}, {right})"

    def fn(value: float) -> float:
        value = (value - offset) % period + offset
        return value

    return fn

