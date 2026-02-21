"""
Shared ONNX model loading and validation for RL policy nodes.
"""

import os

import onnxruntime as ort
from ament_index_python.packages import get_package_share_directory


def load_onnx_policy(
    logger,
    policy_name: str,
    expected_input_dim: int,
    expected_output_dim: int,
) -> ort.InferenceSession | None:
    """Load an ONNX policy from the blind_locomotion share directory.

    Args:
        logger: ROS 2 node logger (e.g. self.get_logger()).
        policy_name: ONNX file stem (without .onnx extension).
        expected_input_dim: Expected last dimension of model input.
        expected_output_dim: Expected last dimension of model output.

    Returns:
        ort.InferenceSession on success, None on failure.
    """
    share_dir = get_package_share_directory('blind_locomotion')
    model_path = os.path.join(share_dir, 'models', f'{policy_name}.onnx')
    logger.info(f'Model path: {model_path}')

    try:
        session = ort.InferenceSession(model_path)
    except Exception as exc:
        logger.fatal(f'Failed to load ONNX model: {exc}')
        return None

    input_shape = session.get_inputs()[0].shape
    output_shape = session.get_outputs()[0].shape
    input_dim = _extract_last_dim(input_shape)
    output_dim = _extract_last_dim(output_shape)

    if input_dim != expected_input_dim:
        logger.fatal(
            f'Invalid model input dim: expected {expected_input_dim}, '
            f'got {input_shape}. Refusing to run.'
        )
        return None

    if output_dim != expected_output_dim:
        logger.fatal(
            f'Invalid model output dim: expected {expected_output_dim}, '
            f'got {output_shape}. Refusing to run.'
        )
        return None

    logger.info(
        f'Successfully loaded ONNX model: {policy_name} '
        f'(input={input_shape}, output={output_shape})'
    )
    return session


def _extract_last_dim(shape) -> int | None:
    """Extract the last dimension from an ONNX shape, handling dynamic dims."""
    if not shape:
        return None
    dim = shape[-1]
    try:
        return int(dim)
    except (TypeError, ValueError):
        return None
