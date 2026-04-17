"""Shared pytest fixtures for the FRET test suite.

Fixtures here are available to all test modules without explicit import.
"""

from __future__ import annotations

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
# Mock kinematics engine (SCARA RRP, DOF=3)
# ---------------------------------------------------------------------------


@pytest.fixture()
def mock_kinematics() -> MagicMock:
    """Return a mock ``Kinematics`` configured for the SCARA RRP (DOF=3)."""
    k = MagicMock()
    k.dof = 3
    k.joint_names = ["joint_arm_0", "joint_arm_1", "joint_extension"]
    k.joint_limits = np.array(
        [[-np.pi, np.pi], [-np.pi / 2, np.pi / 2], [0.0, 0.20]]
    )

    # FK returns a valid 4×4 identity-like transform at zeros.
    # Home EE position: x = L1 + L2 = 0.325 + 0.275 = 0.600 m
    def _fk(q: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[0, 3] = 0.60  # home EE x (L1 + L2)
        return T

    k.forward_kinematics.side_effect = _fk
    k.jacobian.return_value = np.zeros((6, 3))
    return k


# ---------------------------------------------------------------------------
# Mock ARCO KDTreeOccupancy
# ---------------------------------------------------------------------------


@pytest.fixture()
def mock_occupancy() -> MagicMock:
    """Return a mock occupancy that reports free space by default."""
    occ = MagicMock()
    # clearance: positive (free) for zeros, negative for [0,0,0.1]
    def _clearance(pts: np.ndarray) -> float:
        if np.allclose(pts, 0.0):
            return 0.05  # free
        return -0.01  # penetrating

    occ.clearance.side_effect = _clearance
    return occ


# ---------------------------------------------------------------------------
# Mock OccupancyAdapter
# ---------------------------------------------------------------------------


@pytest.fixture()
def mock_occupancy_adapter(mock_occupancy: MagicMock) -> MagicMock:
    """Return a mock ``OccupancyAdapter`` backed by ``mock_occupancy``."""
    adapter = MagicMock()
    adapter.get_occupancy.return_value = mock_occupancy
    return adapter
