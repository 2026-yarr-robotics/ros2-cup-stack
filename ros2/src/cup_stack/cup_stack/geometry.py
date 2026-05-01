"""Geometry helpers for cup stacking tasks."""

import math

import numpy as np


def clamp_z(z: float, safe_z_min: float) -> float:
    """Clamp a target z coordinate to the configured minimum safe height."""

    return max(z, safe_z_min)


def make_twist_orientation(angle_deg: float) -> dict[str, float]:
    """Return the downward-facing quaternion with a yaw twist applied."""

    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    rotation = np.array(
        [
            [-cosine, sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, -1.0],
        ]
    )
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]

    if trace > 0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / scale
        qx = (rotation[2, 1] - rotation[1, 2]) * scale
        qy = (rotation[0, 2] - rotation[2, 0]) * scale
        qz = (rotation[1, 0] - rotation[0, 1]) * scale
    elif (
        rotation[0, 0] > rotation[1, 1]
        and rotation[0, 0] > rotation[2, 2]
    ):
        scale = 2.0 * math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        )
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
        )
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2.0 * math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
        )
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale

    return {
        "x": float(qx),
        "y": float(qy),
        "z": float(qz),
        "w": float(qw),
    }
