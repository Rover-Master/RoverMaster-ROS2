#!/usr/bin/python3
"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np
from math import radians, degrees
from libs_hh2.Transforms import R_from_axis_angle, R_to_axis_angle
from libs_hh2.Transforms import get_transform, is_same_transform


def round(arr, d=4):
    return np.round(arr, d)


######## Part A #######################
R1 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
t1 = np.array([-5.0, 10.0, 0.0])
T1 = get_transform(R1, t1)  # pose for {R1, t1}

# should be 90 degree rotation around z axis
axis1, angle1 = R_to_axis_angle(R1)
print(axis1, degrees(angle1), round(axis1 - R1 @ axis1))
# see if you can recover R1
R1_check = R_from_axis_angle(axis1, angle1)
print("Should be identity matrix:", round(R1_check.T @ R1), sep="\n")  # should be = R1

T1_check = get_transform(R1_check, t1)
print("T1_check =", round(T1_check), sep="\n")
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
print("axis2 =", axis2, ", angle2 =", degrees(angle2), "degrees")
R2_check = R_from_axis_angle(axis2, angle2)
print(R2_check)
print("Should all be 0:", round(axis2 - R2_check @ axis2), sep="\n")
print("Should be identity matrix:", round(R2_check.T @ R2), sep="\n")  # should be = R2

T2_check = get_transform(R2_check, t2)
print("T2_check =", round(T2_check), sep="\n")
print(is_same_transform(T2, T2_check))  # should be True
print(is_same_transform(T1, T2_check))  # should be False
