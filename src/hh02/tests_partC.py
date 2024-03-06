#!/usr/bin/python3
"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np
from math import radians
from libs_hh2.PUMA import Puma_FK

######## Part C #######################

# PUMA robot: all joint angles
theta_1 = radians(10)
theta_2 = radians(20)
theta_3 = radians(30)
theta_4 = radians(45)
theta_5 = radians(50)
theta_6 = radians(60)

a2 = 5.0
a3 = 0.5
d3 = 1.0
d4 = 10.0

puma = Puma_FK(a2, a3, d3, d4)
T1 = puma.get_Ti(i=1, theta_i=theta_1)
T2 = puma.get_Ti(i=2, theta_i=theta_2)
T3 = puma.get_Ti(i=3, theta_i=theta_3)
T4 = puma.get_Ti(i=4, theta_i=theta_4)
T5 = puma.get_Ti(i=5, theta_i=theta_5)
T6 = puma.get_Ti(i=6, theta_i=theta_6)

T46 = T5 @ T6  # 4T6 = 4T5 * 5T6
print("", "T46 =", np.round(T46, 4), sep="\n")
"""
Match your result with Eq. 3.6 (Craig's Book)
"""

# Calculate results for 0T6 and 6T0
T06 = T1 @ T2 @ T3 @ T4 @ T46  # note that T0 = I, reuse 4T6
print("", "T06 =", np.round(T06, 4), sep="\n")

T60 = np.linalg.inv(T06)
print("", "T60 =", np.round(T60, 4), sep="\n")

Point_0 = np.array([5, 5, 5, 1])  # 0_P (point at {0} frame)
# note that we made it homogeneous by putting 1 in the 4th dimension
# Calculate Point_6 (point at {6} frame)
Point_6 = np.dot(np.round(T60, 4), Point_0)
print("", "P6 =", Point_6[:3], sep="\n")  # only display the 3d point
