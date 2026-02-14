"""Minimal forward kinematics for the Unitree Go2 quadruped."""

import numpy as np
from scipy.spatial.transform import Rotation

# Body-to-hip offsets (meters).
HIP_OFFSETS = {
    0: np.array([0.247, 0.050, 0.0]),   # FL
    1: np.array([0.247, -0.050, 0.0]),   # FR
    2: np.array([-0.247, 0.050, 0.0]),   # RL
    3: np.array([-0.247, -0.050, 0.0]),  # RR
}

THIGH_LENGTH = 0.210
CALF_LENGTH = 0.210
ABAD_OFFSET = 0.083


def fk(leg_id: int, q: np.ndarray) -> np.ndarray:
    """Compute foot position in body frame for a single leg.

    Args:
        leg_id: 0=FL, 1=FR, 2=RL, 3=RR.
        q: Joint angles [abad, hip, knee] in radians.

    Returns:
        3D foot position in body frame.
    """
    q_abad, q_hip, q_knee = q

    # Left legs: +Y abad offset; right legs: -Y.
    s = 1 if leg_id in (0, 2) else -1

    R_abad = Rotation.from_euler('x', q_abad).as_matrix()
    R_hip = Rotation.from_euler('y', q_hip).as_matrix()
    R_knee = Rotation.from_euler('y', q_knee).as_matrix()

    p_ab = np.array([0.0, s * ABAD_OFFSET, 0.0])
    p_th = np.array([0.0, 0.0, -THIGH_LENGTH])
    p_cf = np.array([0.0, 0.0, -CALF_LENGTH])

    return HIP_OFFSETS[leg_id] + R_abad @ (p_ab + R_hip @ (p_th + R_knee @ p_cf))
