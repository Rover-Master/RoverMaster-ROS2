# ==============================================================================
# Author: Yuxuan Zhang (robotics@z-yx.cc)
# License: MIT
# ==============================================================================

from geometry_msgs.msg import Quaternion
from math import atan2, asin, degrees
from .math import ang_diff

def attitude_from_quaternion(q: Quaternion):
    # Convert attitude quaternion to euler angles
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = asin(2.0 * (w * y - z * x))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    # Convert to degrees
    return [ang_diff(0.0, degrees(x)) for x in [roll, pitch, yaw]]

