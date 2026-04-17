"""Path quality metrics for FRET planning evaluation.

Provides pure-Python / NumPy functions that compute scalar metrics
from joint-space paths and trajectory data.  All functions are free of
ROS 2 runtime dependencies and are safe to call from unit tests and
offline analysis scripts.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import numpy.typing as npt


def path_length(path: list[npt.NDArray[np.floating[Any]]]) -> float:
    """Compute joint-space arc length of a path.

    Args:
        path: List of joint configurations, each shape (DOF,).

    Returns:
        Total arc length (sum of L2 norms of consecutive differences).
        Returns 0.0 for paths with fewer than 2 waypoints.
    """
    if len(path) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        total += float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
    return total


def path_smoothness(path: list[npt.NDArray[np.floating[Any]]]) -> float:
    """Compute total direction-change angle (radians) along a path.

    Lower values indicate smoother paths.  A perfectly straight path has
    smoothness 0.0.

    Args:
        path: List of joint configurations, each shape (DOF,).

    Returns:
        Sum of interior angles between consecutive segment directions [rad].
        Returns 0.0 for paths with fewer than 3 waypoints.
    """
    if len(path) < 3:
        return 0.0
    total = 0.0
    for i in range(1, len(path) - 1):
        d1 = np.asarray(path[i]) - np.asarray(path[i - 1])
        d2 = np.asarray(path[i + 1]) - np.asarray(path[i])
        n1, n2 = np.linalg.norm(d1), np.linalg.norm(d2)
        if n1 < 1e-12 or n2 < 1e-12:
            continue
        cos_a = float(np.clip(np.dot(d1, d2) / (n1 * n2), -1.0, 1.0))
        total += math.acos(cos_a)
    return total


def min_obstacle_clearance(
    path: list[npt.NDArray[np.floating[Any]]],
    occupancy: Any,
    kinematics: Any,
) -> float:
    """Compute minimum obstacle clearance along a path.

    For each waypoint, computes FK then queries ``occupancy.clearance()``.

    Args:
        path: List of joint configurations, each shape (DOF,).
        occupancy: Object exposing ``clearance(pts: NDArray) -> float``.
        kinematics: Object exposing ``forward_kinematics(q) -> 4x4 matrix``.

    Returns:
        Minimum clearance [m] across all waypoints.  Returns ``+inf`` for
        empty paths.
    """
    if not path:
        return math.inf
    min_c = math.inf
    for q in path:
        transform: npt.NDArray[np.floating[Any]] = (
            kinematics.forward_kinematics(np.asarray(q))
        )
        # Extract the translation column from the 4x4 homogeneous matrix.
        pt = transform[:3, 3].reshape(1, 3)
        c = float(occupancy.clearance(pt))
        if c < min_c:
            min_c = c
    return min_c


def tracking_rmse(
    reference: list[npt.NDArray[np.floating[Any]]],
    actual: list[npt.NDArray[np.floating[Any]]],
) -> float:
    """Compute RMSE between reference and actual joint trajectories.

    Args:
        reference: List of reference joint configurations, shape (DOF,) each.
        actual: List of actual joint configurations, same length as reference.

    Returns:
        Root mean square error across all joints and timesteps [rad or m].

    Raises:
        ValueError: If reference and actual have different lengths or shapes.
    """
    if len(reference) != len(actual):
        raise ValueError(
            f"reference and actual must have the same length, "
            f"got {len(reference)} vs {len(actual)}"
        )
    if len(reference) == 0:
        return 0.0
    ref_arr = np.asarray(reference)
    act_arr = np.asarray(actual)
    if ref_arr.shape != act_arr.shape:
        raise ValueError(
            f"reference and actual must have the same shape, "
            f"got {ref_arr.shape} vs {act_arr.shape}"
        )
    return float(np.sqrt(np.mean((ref_arr - act_arr) ** 2)))
