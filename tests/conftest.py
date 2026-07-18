"""Shared pytest fixtures for the FRET test suite.

Fixtures here are available to all test modules without explicit import.
"""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Generic ROS 2 mock node
# ---------------------------------------------------------------------------


@pytest.fixture()
def mock_node() -> MagicMock:
    """Return a minimal mock that satisfies the ``rclpy.node.Node`` type hint."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    return node


# ---------------------------------------------------------------------------
# Mock 3-DOF arm kinematics (generic stand-in for planner unit tests)
# ---------------------------------------------------------------------------

_L1: float = 0.325
_L2: float = 0.275
_Z_BASE: float = 0.238


@pytest.fixture()
def mock_kinematics() -> MagicMock:
    """Return a mock 3-DOF planar arm ``Kinematics`` for C-space unit tests."""
    k = MagicMock()
    k.dof = 3
    k.joint_names = ["joint_0", "joint_1", "joint_2"]
    k.joint_limits = np.array(
        [
            [-math.pi, math.pi],
            [-math.pi, math.pi],
            [0.0, 0.20],
        ]
    )

    def _fk(q: np.ndarray) -> np.ndarray:
        c1 = math.cos(float(q[0]))
        s1 = math.sin(float(q[0]))
        c12 = math.cos(float(q[0]) + float(q[1]))
        s12 = math.sin(float(q[0]) + float(q[1]))
        x = _L1 * c1 + _L2 * c12
        y = _L1 * s1 + _L2 * s12
        z = _Z_BASE - float(q[2])
        T = np.eye(4)
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    k.forward_kinematics.side_effect = _fk
    k.jacobian.return_value = np.zeros((6, 3))
    return k


# ---------------------------------------------------------------------------
# Mock ARCO KDTreeOccupancy
# ---------------------------------------------------------------------------


@pytest.fixture()
def mock_occupancy() -> MagicMock:
    """Return a mock occupancy matching the ``KDTreeOccupancy`` interface."""
    occ = MagicMock()
    occ.clearance = 0.05

    def _query_distances(pts: np.ndarray) -> np.ndarray:
        pts_2d = np.atleast_2d(pts)
        ee_z = float(pts_2d[-1, 2])
        dist = 0.10 if ee_z > 0.18 else 0.03
        return np.full(pts_2d.shape[0], dist)

    occ.query_distances.side_effect = _query_distances
    return occ


# ---------------------------------------------------------------------------
# Algorithm configuration fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def arm_planning_config() -> dict[str, object]:
    """Inline arm planning parameters for unit tests (no bundled YAML)."""
    return {
        "trajectory": {
            "control_hz": 50.0,
            "v_max": [1.5, 1.5, 0.1],
            "a_max": [2.0, 2.0, 0.2],
            "dt_min": 0.02,
            "joint_names": ["joint_0", "joint_1", "joint_2"],
        },
        "replanning": {
            "tracking_error_threshold": 0.020,
            "occupancy_change_threshold": 0.050,
            "min_replan_interval": 1.0,
            "max_replan_attempts": 3,
        },
        "trajectory_generator": {
            "max_interp_step_m": 0.004,
        },
    }


# ---------------------------------------------------------------------------
# Mock OccupancyAdapter
# ---------------------------------------------------------------------------


@pytest.fixture()
def mock_occupancy_adapter(mock_occupancy: MagicMock) -> MagicMock:
    """Return a mock ``OccupancyAdapter`` backed by ``mock_occupancy``."""
    adapter = MagicMock()
    adapter.get_occupancy.return_value = mock_occupancy
    return adapter
