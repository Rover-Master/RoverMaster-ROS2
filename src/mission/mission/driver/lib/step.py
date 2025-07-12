class Step:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        """Initialize the step function."""
        self.x = x
        self.y = y

    def __call__(self, x: float, y: float, p: float) -> float:
        dx, dy = x - self.x, y - self.y
        self.x, self.y = x, y
        return self.step(dx, dy, p)

    def step(self, dx: float, dy: float, p: float) -> float:
        """
        Step function to be implemented by subclasses.
        Parameters:
            dx (float, -2.0 ~ +2.0): Change in x position.
            dy (float, -2.0 ~ +2.0): Change in y position.
            p (float, 0.0 ~ 1.0): Percentage of progress.
        Returns step time in us (microseconds).
        """
        raise NotImplementedError("Subclasses must implement step method")


class Constant(Step):
    def __init__(self, interval: float = 10.0):
        super().__init__()
        self.interval = interval

    def step(self, *_):
        return self.interval


class Linear(Step):
    def __init__(self, speed: float, x: float = 0.0, y: float = 0.0):
        """
        Initialize the linear step function.
        """
        super().__init__(x, y)
        self.speed = speed

    def step(self, dx, dy):
        """Linear step function."""
        distance = (dx**2 + dy**2) ** 0.5
        return distance * self.speed


class Quadratic(Step):
    """
    Limits max acceleration
    """

    def __init__(
        self,
        acc: float = 1.0,
        x: float = 0.0,
        y: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
    ):
        super().__init__(x, y)
        self.acc = acc
        self.vx = vx
        self.vy = vy

    def step(self, dx, dy):
        """Linear step function."""
        distance = (dx**2 + dy**2) ** 0.5
        return distance * self.speed
