from . import forward, backward


class Kinematics:
    def __init__(self, **kwargs: float):
        self.params = kwargs

    def forward(self, J1, J2, J3):
        return forward.solve(J1=J1, J2=J2, J3=J3, **self.params)

    def backward(self, x, y, r):
        return backward.solve(x=x, y=y, r=r, **self.params)
