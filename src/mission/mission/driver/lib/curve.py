from dataclasses import dataclass, field
from typing import Callable
from .action import Action
from .util import Range, clamp
from .units import Duration, Frequency, KHz

type Point = tuple[float, float]

type Fn = Callable[[float], float]
type Step = Callable[[float, float, float, float], float]


def checkStep(
    step: float,
    prev: float | None,
    next: float | None,
    force: bool = False,
) -> float | None:
    if force:
        return next
    if (prev is None) or (next is None):
        return next
    if abs(next - prev) >= step:
        return next


class Recorder:
    t = 0.0
    x = 0.0
    y = 0.0
    X: list[tuple[float, float]] = [(0, 0)]
    Y: list[tuple[float, float]] = [(0, 0)]

    def __call__(self, x: float | None, y: float | None, dt: float = 0.0):
        """Record a point in the curve."""
        self.t += dt
        self.X.append((self.t, self.x))
        self.Y.append((self.t, self.y))
        if x is not None:
            self.X.append((self.t, x))
            self.x = x
        if y is not None:
            self.Y.append((self.t, y))
            self.y = y


@dataclass(frozen=False)
class Curve:
    action: Action
    tStep: Duration | Frequency = KHz(50)
    vStep: float = 2.0
    recorder: Recorder = field(default_factory=Recorder)

    def __call__(
        self,
        T: Duration,
        X: Fn | None = None,
        Y: Fn | None = None,
        *,
        tStep: Duration | Frequency | None = None,
        vStep: float | None = None,
    ):
        """Generate a curve with the given functions for X and Y."""
        action = self.action
        tStep = (tStep or self.tStep).T.us
        vStep = vStep or self.vStep
        vStep = clamp(vStep, 0.0, 200.0) / 200.0
        result: list[bytes] = []

        def delay(dt: int):
            while dt > 65535:
                result.append(action.Delay(65535))
                dt -= 65535
            result.append(action.Delay(dt))

        x0 = X and X(0)
        y0 = Y and Y(0)
        self.recorder(x0, y0)
        if x0 is None:
            vx = None
        else:
            vx = action.SetVoltX(x0)
        if y0 is None:
            vy = None
        else:
            vy = action.SetVoltY(y0)
        if vx is not None:
            result.append(vx)
        if vy is not None:
            result.append(vy)

        t0 = 0
        duration = T.us
        for t in Range(tStep, duration - tStep / 2, tStep):
            t = int(round(t))
            p = t / duration
            dt = int(t - t0)
            if dt <= 0:
                continue
            x1 = checkStep(vStep, x0, X and X(p))
            y1 = checkStep(vStep, y0, Y and Y(p))
            if x1 is None:
                vx = None
            else:
                vx = action.SetVoltX(x1)
                x0 = x1
            if y1 is None:
                vy = None
            else:
                vy = action.SetVoltY(y1)
                y0 = y1
            if (vx is not None) or (vy is not None):
                delay(dt)
                t0 = t
                if vx is not None:
                    result.append(vx)
                if vy is not None:
                    result.append(vy)
                self.recorder(x1, y1, dt)
        t = int(round(duration))
        dt = int(t - t0)
        if dt > 0:
            delay(dt)
            p = 1.0
            x1 = X and X(p)
            y1 = Y and Y(p)
            if x1 is not None:
                vx = action.SetVoltX(x1)
                if vx is not None:
                    result.append(vx)
            if y1 is not None:
                vy = action.SetVoltY(y1)
                if vy is not None:
                    result.append(vy)
            self.recorder(x1, y1, dt)
        return result


class Fn:
    def __init__(self, x0: float, x1: float):
        self.x0 = x0
        self.x1 = x1

    def cvt(self, pct: float) -> float:
        """Convert percentage to a value in the range [x0, x1]."""
        raise NotImplementedError("Subclasses must implement cvt method")

    def __call__(self, y: float) -> float:
        return self.x0 + (self.x1 - self.x0) * self.cvt(y)


class Const:
    def __init__(self, x: float):
        self.x = x

    def __call__(self, _: float) -> float:
        return self.x


class Linear(Fn):
    def cvt(self, pct: float) -> float:
        return pct


class Posicast(Fn):
    def __init__(self, x0: float, x1: float, ratio: float = 0.5):
        super().__init__(x0, x1)
        self.ratio = float(ratio)

    def cvt(self, pct: float) -> float:
        if 0.01 < pct < 0.99:
            return self.ratio
        else:
            return pct


class Power(Fn):
    def __init__(self, x0: float, x1: float, power: float):
        super().__init__(x0, x1)
        k = 0.5 / (0.5**power)
        self.fn = lambda p: k * (p**power)

    def cvt(self, pct: float) -> float:
        if pct < 0.5:
            return self.fn(pct)
        elif pct > 0.5:
            return 1 - self.fn(1 - pct)
        else:
            return 0.5


class Quadratic(Power):
    def __init__(self, x0, x1):
        super().__init__(x0, x1, 2)


class Cubic(Power):
    def __init__(self, x0, x1):
        super().__init__(x0, x1, 3)


class Sine(Fn):
    def cvt(self, pct: float) -> float:
        from math import sin, pi

        return sin((pct - 0.5) * pi) / 2 + 0.5


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(2, 2, figsize=(8, 8))
    x = np.linspace(0, 1, 100)

    def plot(ax: plt.Axes, fn: Fn, label: str):
        ax.grid()
        ax.set_title(label)
        ax.plot(x, list(map(fn, x)), label=label)

    plot(ax[0][0], Linear(0, 1), "Linear")
    # plot(ax[0][1], Quadratic(0, 0.5, 1), "Quadratic")
    # plot(ax[1][0], Cubic(0, 0.5, 0.5, 1), "Cubic")
    plot(ax[1][1], Sine(0, 1), "Sine")
    plt.tight_layout()
    plt.show()
