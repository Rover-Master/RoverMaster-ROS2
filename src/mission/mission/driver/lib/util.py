# ==============================================================================
# Author: Yuxuan Zhang (dev@z-yx.cc)
# License: TBD (UNLICENSED)
# ==============================================================================

from time import time


def loop_for(seconds: float):
    ddl = time() + seconds
    while time() < ddl:
        yield


def bytes_repr(b: bytes):
    return " ".join(f"{byte:02X}" for byte in b)


def sign(x: float):
    """Return the sign of a number."""
    return (x > 0) - (x < 0)


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between a minimum and maximum."""
    return max(min_value, min(value, max_value))


class Range:
    def __init__(self, start: float, end: float, step: float = 1.0):
        self.value = start
        self.end = end
        self.step = step

    def __iter__(self):
        return self

    def __next__(self):
        if self.value >= self.end:
            raise StopIteration
        current = self.value
        self.value += self.step
        return current


_NO_RESULT = object()


def debounce(timeout: float):
    from time import time as now
    from functools import wraps

    def decorator(func):
        ddl = 0.0
        last_result = _NO_RESULT

        @wraps(func)
        def wrapper(*args, **kwargs):
            nonlocal ddl, last_result
            tp = now()
            if last_result is _NO_RESULT or tp >= ddl:
                ddl = tp + timeout
                last_result = func(*args, **kwargs)
                return last_result
            else:
                return last_result

        return wrapper

    return decorator

def loop_for(seconds: float):
    ddl = time() + seconds
    while time() < ddl:
        yield