#!/usr/bin/python3
"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np
from math import radians
from matplotlib import pyplot as plt
from libs_hh2.Transforms import R_from_axis_angle, get_transform
from libs_hh2.Transforms import quaternion_from_R, R_from_quaternion
from libs_hh2.Transforms import quaternion_slerp
from libs_hh2.utils import DisplayFrame

######## Part B #######################
t0 = np.array([0.0, 0.0, 0.0])
R0 = np.identity(3)
T0 = np.identity(4)  # origin

theta_init = radians(60)
t1 = np.array([-5.0, -5.0, -5.0])
K1 = np.array([0.0, 0.0, 1.0])  # axis of rotation
R1 = R_from_axis_angle(K1, theta_init)
T1 = get_transform(R1, t1)  # frame 1
print("T1 =", T1, sep="\n")
"""
print(T1)=
[[ 0.5       -0.8660254  0.        -5.       ]
 [ 0.8660254  0.5        0.        -5.       ]
 [ 0.         0.         1.        -5.       ]
 [ 0.         0.         0.         1.       ]]
"""


theta_final = radians(45)
t2 = np.array([-15.0, -15.0, -15.0])
K2 = np.array([0.0, 1.0, 1.0])  # axis of rotation
R2 = R_from_axis_angle(K2, theta_final)
T2 = get_transform(R2, t2)  # frame 2
print("T2 =", T2, sep="\n")
"""
print(T2)=
[[  0.70710678  -0.5          0.5        -15.        ]
 [  0.5          0.85355339   0.14644661 -15.        ]
 [ -0.5          0.14644661   0.85355339 -15.        ]
 [  0.           0.           0.           1.        ]]
"""


## draw intermediate frames by interpolation

## draw the frames
fig = plt.figure(figsize=(8, 8), dpi=150)
fig.tight_layout()
# num_levels = 2
# TITLE = "Two Poses T1 and T2"
num_levels = 50
TITLE = f"Intermediate poses; num_levels={num_levels}"
plt.title(TITLE)
plt.grid(False)
plt.axis("off")
ax = fig.add_subplot(111, projection="3d")
# ax = DisplayFrame(R0, t0, ax, came_scale=1) # origin
ax = DisplayFrame(R1, t1, ax, came_scale=2)  # frame 1
ax = DisplayFrame(R2, t2, ax, came_scale=2)  # frame 2

# rotation interpolation in quaternion space
q1 = quaternion_from_R(R1)
q2 = quaternion_from_R(R2)
q_interp = quaternion_slerp(q1, q2, levels=num_levels)

# translation is easy to interpolate
x_interp = np.linspace(t1[0], t2[0], num_levels)
y_interp = np.linspace(t1[1], t2[1], num_levels)
z_interp = np.linspace(t1[2], t2[2], num_levels)

for i in range(num_levels):
    q = q_interp[i]
    R = R_from_quaternion(q)
    t = np.array([x_interp[i], y_interp[i], z_interp[i]])
    ax = DisplayFrame(R, t, ax, came_scale=1)  # frame i

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.axis("auto")
ax.set_box_aspect([1, 1, 1])
plt.savefig(TITLE + ".png")
plt.show()
