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
# Mock kinematics engine (SCARA RRP, DOF=3)
# ---------------------------------------------------------------------------

# SCARA model constants (must match src/fret/control/kinematics.py)
_L1: float = 0.325
_L2: float = 0.275
_Z_BASE: float = 0.1665 + 0.0715  # J1_Z + J2_Z = 0.238


@pytest.fixture()
def mock_kinematics() -> MagicMock:
    """Return a mock ``Kinematics`` configured for the SCARA RRP (DOF=3).

    The ``forward_kinematics`` side-effect computes the actual SCARA FK so
    that ``CSpaceChecker`` receives realistic EE positions for different
    configurations.
    """
    k = MagicMock()
    k.dof = 3
    k.joint_names = ["joint_arm_0", "joint_arm_1", "joint_extension"]
    k.joint_limits = np.array(
        [
            [-132.0 * math.pi / 180.0, 132.0 * math.pi / 180.0],
            [-150.0 * math.pi / 180.0, 150.0 * math.pi / 180.0],
            [0.0, 0.20],
        ]
    )

    def _fk(q: np.ndarray) -> np.ndarray:
        """Compute SCARA FK: EE = (L1·cos q1 + L2·cos(q1+q2), …, z_base−q3)."""
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
    """Return a mock occupancy matching the ``KDTreeOccupancy`` interface.

    ``CSpaceChecker.clearance`` detects ``KDTreeOccupancy`` by looking for a
    ``query_distances`` method; this mock exposes the same API so that the
    production code path is exercised even in unit tests.

    Returns distances that yield positive clearance (free space) when the EE
    z-height is above 0.18 m, and negative clearance (obstacle) otherwise.

    SCARA reference heights:
      - q3 = 0.00 → EE z = 0.238 > 0.18 → free (home config).
      - q3 = 0.10 → EE z = 0.138 < 0.18 → penetrating (colliding config).
    """
    occ = MagicMock()
    # ``clearance`` is a float *attribute* on KDTreeOccupancy, not callable.
    occ.clearance = 0.05  # 5 cm clearance radius

    def _query_distances(pts: np.ndarray) -> np.ndarray:
        pts_2d = np.atleast_2d(pts)
        # Last row is the EE position (alpha = 1.0 in the sampler).
        ee_z = float(pts_2d[-1, 2])
        # 0.10 → clearance = 0.10 − 0.05 =  0.05 > 0  (free space)
        # 0.03 → clearance = 0.03 − 0.05 = −0.02 < 0  (inside obstacle)
        dist = 0.10 if ee_z > 0.18 else 0.03
        return np.full(pts_2d.shape[0], dist)

    occ.query_distances.side_effect = _query_distances
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
