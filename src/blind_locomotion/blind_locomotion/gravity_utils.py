"""
Gravity projection utilities using pure numpy (no SciPy dependency).

Replaces scipy.spatial.transform.Rotation for the single-quaternion case,
which is ~10x faster on embedded hardware.
"""

import numpy as np


def quat_wxyz_to_rotation_matrix(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert a [w, x, y, z] quaternion to a 3x3 rotation matrix.

    Args:
        q_wxyz: Quaternion in [w, x, y, z] format (as from Unitree IMU).

    Returns:
        3x3 rotation matrix (body-to-world).
    """
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])

    x2, y2, z2 = x + x, y + y, z + z
    xx = x * x2
    xy = x * y2
    xz = x * z2
    yy = y * y2
    yz = y * z2
    zz = z * z2
    wx = w * x2
    wy = w * y2
    wz = w * z2

    return np.array([
        [1.0 - (yy + zz),       xy - wz,         xz + wy],
        [      xy + wz,   1.0 - (xx + zz),        yz - wx],
        [      xz - wy,         yz + wx,   1.0 - (xx + yy)],
    ], dtype=np.float32)


def projected_gravity_inverse(q_wxyz: np.ndarray) -> np.ndarray:
    """Compute gravity projection using inverse rotation: R^T @ [0, 0, -1].

    This is the convention used by the flat locomotion policy (rl_actions).
    Equivalent to: Rotation.from_quat([x,y,z,w]).inv().as_matrix() @ [0,0,-1]

    Args:
        q_wxyz: IMU quaternion in [w, x, y, z] format.

    Returns:
        Projected gravity vector in body frame, shape (3,).
    """
    R = quat_wxyz_to_rotation_matrix(q_wxyz)
    # R^T @ g = R^T @ [0, 0, -1] = -R^T[:, 2] = -R[2, :]^T (third row, negated)
    return np.array([-R[2, 0], -R[2, 1], -R[2, 2]], dtype=np.float32)


def projected_gravity_direct(q_wxyz: np.ndarray) -> np.ndarray:
    """Compute gravity projection using direct rotation: R @ [0, 0, -1].

    This is the convention used by the reach policy (rl_reach_actions).
    Equivalent to: Rotation.from_quat([x,y,z,w]).as_matrix() @ [0,0,-1]

    Args:
        q_wxyz: IMU quaternion in [w, x, y, z] format.

    Returns:
        Projected gravity vector in body frame, shape (3,).
    """
    R = quat_wxyz_to_rotation_matrix(q_wxyz)
    # R @ [0, 0, -1] = -R[:, 2] (third column, negated)
    return np.array([-R[0, 2], -R[1, 2], -R[2, 2]], dtype=np.float32)
