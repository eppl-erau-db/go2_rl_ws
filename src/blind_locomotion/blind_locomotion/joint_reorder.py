"""
Joint ordering utilities for Unitree Go2 <-> Isaac Lab conversion.

Unitree hardware order (grouped by leg):
  [0] FR_hip, [1] FR_thigh, [2] FR_calf,
  [3] FL_hip, [4] FL_thigh, [5] FL_calf,
  [6] RR_hip, [7] RR_thigh, [8] RR_calf,
  [9] RL_hip, [10] RL_thigh, [11] RL_calf

Isaac Lab order (grouped by joint type):
  [0] FL_hip, [1] FR_hip, [2] RL_hip, [3] RR_hip,
  [4] FL_thigh, [5] FR_thigh, [6] RL_thigh, [7] RR_thigh,
  [8] FL_calf, [9] FR_calf, [10] RL_calf, [11] RR_calf
"""

import numpy as np

# Unitree motor_state indices to read in Isaac Lab order.
# Usage: isaac_q[i] = motor_state[UNITREE_TO_ISAAC[i]].q
UNITREE_TO_ISAAC: list[int] = [
    3, 0, 9, 6,       # hips:   FL, FR, RL, RR
    4, 1, 10, 7,      # thighs: FL, FR, RL, RR
    5, 2, 11, 8,      # calfs:  FL, FR, RL, RR
]

# Isaac Lab indices to write in Unitree order.
# Usage: unitree_cmd[i] = isaac_actions[ISAAC_TO_UNITREE[i]]
ISAAC_TO_UNITREE: list[int] = [
    1, 5, 9,           # FR: hip, thigh, calf
    0, 4, 8,           # FL: hip, thigh, calf
    3, 7, 11,          # RR: hip, thigh, calf
    2, 6, 10,          # RL: hip, thigh, calf
]

# Pre-computed numpy array for fancy indexing.
_ISAAC_TO_UNITREE_NP = np.array(ISAAC_TO_UNITREE, dtype=np.intp)


def read_joint_positions_isaac(motor_states) -> np.ndarray:
    """Extract 12 joint positions from LowState.motor_state in Isaac Lab order.

    Args:
        motor_states: The motor_state array from a unitree_go/LowState message.

    Returns:
        np.ndarray of shape (12,) with joint angles in Isaac Lab order.
    """
    return np.array(
        [motor_states[i].q for i in UNITREE_TO_ISAAC],
        dtype=np.float32,
    )


def read_joint_velocities_isaac(motor_states) -> np.ndarray:
    """Extract 12 joint velocities from LowState.motor_state in Isaac Lab order.

    Args:
        motor_states: The motor_state array from a unitree_go/LowState message.

    Returns:
        np.ndarray of shape (12,) with joint velocities in Isaac Lab order.
    """
    return np.array(
        [motor_states[i].dq for i in UNITREE_TO_ISAAC],
        dtype=np.float32,
    )


def actions_to_unitree_order(isaac_actions: np.ndarray) -> list[float]:
    """Reorder processed actions from Isaac Lab order to Unitree hardware order.

    Args:
        isaac_actions: np.ndarray of shape (12,) in Isaac Lab joint order.

    Returns:
        list of 12 floats in Unitree hardware order.
    """
    return isaac_actions[_ISAAC_TO_UNITREE_NP].tolist()
