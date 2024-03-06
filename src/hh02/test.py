#!/usr/bin/python3
import numpy as np
from math import radians, degrees
from libs_hh2.Transforms import R_from_axis_angle, R_to_axis_angle
from libs_hh2.Transforms import get_transform, is_same_transform
import matplotlib.pyplot as plt
from libs_hh2.utils import unit_vector


def round(arr, d=4):
    return np.round(arr, d)


# more complex rotation
R2 = np.array(
    [
        [0.61237244, -0.35355339, 0.70710678],
        [0.78033009, 0.12682648, -0.61237244],
        [0.12682648, 0.9267767, 0.35355339],
    ]
)
t2 = np.array([3.0, 5.0, 0.5])
T2 = get_transform(R2, t2)  # pose for {R1, t1}

# do the same check
axis2, angle2 = R_to_axis_angle(R2)
print(axis2, degrees(angle2), round(axis2 - R2 @ axis2))
R2_check = R_from_axis_angle(axis2, angle2)

# Draw the original rotation on [1, 0, 0]
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.quiver(0, 0, 0, *unit_vector(axis2), color="k")
u = np.array([1, 0, 0])
v = np.array([1, 0, 0])

for c in ["c", "g", "m", "b", "y"]:

    u1 = R2 @ u
    ax.plot(*zip(u, u1), color="r")
    u = u1

    v1 = R2_check @ v
    ax.plot(*zip(v, v1), color=c)
    v = v1

ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)
fig.show()
plt.show()
