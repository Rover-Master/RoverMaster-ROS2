import math
import numpy as np

X, Y, Z = 0, 1, 2

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0


def normalize(vec: np.ndarray):
    return vec / np.linalg.norm(vec)


def t_from_T(matrix):
    """Return translation vector from transformation matrix."""
    return np.array(matrix, copy=False)[:3, 3].copy()


def angle2d(v, u=[1, 0]):
    """
    Return angle from u to v (both are 2D vectors),
    value range: (-pi, pi]
    """
    sin = u[X] * v[Y] - v[X] * u[Y]
    cos = u[X] * v[X] + u[Y] * v[Y]
    return math.atan2(sin, cos)


def rotate_around(AXIS, theta):
    """
    Utility function to get the rotation matrix R(theta) around axis N.
    N = X, Y or Z
    """
    s = math.sin(theta)
    c = math.cos(theta)
    if AXIS == X:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif AXIS == Y:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif AXIS == Z:
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    else:
        raise ValueError(f"Invalid axis {AXIS}")


def rotate_from_z(v):
    """
    [0,0,1]
    Returns the rotation matrix that rotats Z axis to given vector `v`
    """
    # First, check if axis's projection to XY plane has zero length
    l_xy = np.linalg.norm(v[:2])
    if l_xy == 0:
        return np.sign(v[Z]) * np.identity(3)
    # Rotation around Y, projection on XY will point to +X
    R1 = rotate_around(Y, angle2d([v[Z], l_xy]))
    # Rotation around Z, projection on XY will align with v
    R2 = rotate_around(Z, angle2d(v[:2]))
    # Return the composition of the two rotations
    return R2 @ R1


def R_from_axis_angle(axis, angle):
    """Find the rotation matrix R(K, theta)
    @param axis (K): axis of rotation and
    @param theta: rotation angle in radians
    @return R: rotation matrix (3x3 numpy array)
    """
    axis = normalize(axis[:3])
    # Convert rotation around Z to rotation around "axis"
    R = rotate_from_z(axis)
    Rz = rotate_around(Z, angle)
    return R @ Rz @ R.T


def R_to_axis_angle(matrix):
    """Convert the rotation matrix into the axis-angle notation.
    That is find (K, theta) = axis of rotation and rotation angle (in radians)
    Hint: see the Wikipedia notes (http://en.wikipedia.org/wiki/Rotation_matrix)
        @param matrix:  The rotation matrix (3x3 numpy array)
        @return axis: The 3D rotation axis (3x1 numpy array)
        @return theta: rotation angle in radians (float)
    """
    # Axes calculations
    axis = np.zeros(3, np.float64)
    axis[0] = matrix[2, 1] - matrix[1, 2]
    axis[1] = matrix[0, 2] - matrix[2, 0]
    axis[2] = matrix[1, 0] - matrix[0, 1]
    # Convert rotation around "axis" to rotation around Z
    R = rotate_from_z(axis)
    R = R.T @ matrix @ R
    # Extract the angles from known rotation matrix
    cos = R[0, 0]
    sin = R[1, 0]
    return axis, math.atan2(sin, cos)


def get_transform(R, t):
    """Return the transformation matrix T given rotation matrix R and translation t
    @param R:  rotation matrix (3x3 numpy array)
    @param t:  translation vector  (3x1 numpy array)
    @return T: transformation matrix  (4x4 numpy array)
    """
    T = np.zeros((4, 4), dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    T[3, 3] = 1
    return T


def is_same_transform(T_a, T_b):
    """Return True if two matrices perform same transformation
    Hint: use np.allclose for cleaner code
    @param T_a: transformation matrix  (4x4 numpy array)
    @param T_a: transformation matrix  (4x4 numpy array)
    @return flag: True or False (boolean)
    """
    return np.allclose(T_a, T_b, atol=_EPS)


def quaternion_slerp(quat0, quat1, levels=5):
    """Return spherical linear interpolation between two quaternions
    @param quat0: quaternion (4x1 numpy array)
    @param quat1: quaternion (4x1 numpy array)
    @param levels: number of levels for interpolation
    @return all_q: all interpolated quaternions (list of quaternions)
    """
    q0 = normalize(quat0[:4])
    q1 = normalize(quat1[:4])
    d = np.dot(q0, q1)
    a = math.acos(d)  # 0 ~ pi
    T = np.linspace(0, 1, levels)

    if abs(a) < _EPS:
        # Fall back to linear interpolation
        return [q0 * (1 - t) + q1 * t for t in T]

    def slerp(t):
        """
        Let x = q0, then y = [q1 - cos(a) * x] / sin(a)
        Hence, q(t) = cos(t * a) * x + sin(t * a) * y
                    = cos(t * a) * q0 + sin(t * a) * [q1 - cos(a) q0] / sin(a)

        ========== Sanity check ==========

        When t = 0, q(0) = q0 + 0
                         = q0

        When t = 1, q(1) = cos(a) * q0 + sin(a) * [q1 - cos(a) q0] / sin(a)
                         = (cos(a) - sin(a)) * q0 + sin(a) * q1 / sin(a)
                         = q1
        """
        cos_ta = math.cos(t * a)
        sin_ta = math.sin(t * a)
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        return cos_ta * q0 + sin_ta * (q1 - cos_a * q0) / sin_a

    return list(map(slerp, T))


def quaternion_about_axis(angle, axis):
    """Return quaternion for rotation about axis."""
    q = np.array([0.0, axis[0], axis[1], axis[2]])
    qlen = np.linalg.norm(q)
    if qlen > _EPS:
        q *= math.sin(angle / 2.0) / qlen
    q[0] = math.cos(angle / 2.0)
    return q


def R_from_quaternion(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    @param quaternion: a quaternion (4x1 numpy array)
    @return R: corresponding rotation matrix (3x3 numpy array)
    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        R = np.identity(3)
    else:
        axis = q[1:]
        cos = q[0]
        sin = np.linalg.norm(axis)
        R = R_from_axis_angle(axis, 2 * math.atan2(sin, cos))
    return R


def quaternion_from_R(matrix, isprecise=False):
    """Return quaternion from rotation matrix.
    @param matrix: rotation matrix (3x3 numpy array)
    @return q: corresponding quaternion (4x1 numpy array)
    """
    M = np.array(matrix, dtype=np.float64, copy=False)
    axis, angle = R_to_axis_angle(M)
    q = quaternion_about_axis(angle, axis)
    # -vity check
    if q[0] < 0.0:
        np.negative(q, q)
    return q


def compund_transform(*matrices):
    """Return concatenation of series of transformation matrices."""
    T = np.identity(4)
    for M in matrices:
        T = np.dot(T, M)
    return T
