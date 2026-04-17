"""Tests for fret.planning.CSpaceChecker.

Acceptance criteria (FR-PLN-02):
  - ``is_collision_free(q)`` returns True for a configuration with positive
    clearance from all obstacles.
  - ``is_collision_free(q)`` returns False for a configuration inside an
    obstacle.
  - ``clearance(q)`` returns a float in the same sign convention
    (positive = free, negative = penetrating).
  - Both methods raise ``ValueError`` if ``q.shape != (DOF,)``.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.planning.cspace_checker import CSpaceChecker


def test_construction(mock_kinematics: object, mock_occupancy: object) -> None:
    CSpaceChecker(kinematics=mock_kinematics, occupancy=mock_occupancy)


def test_free_space_config_returns_true(
    mock_kinematics: object, mock_occupancy: object
) -> None:
    """Home configuration (all zeros) is collision-free in an empty scene."""
    checker = CSpaceChecker(
        kinematics=mock_kinematics, occupancy=mock_occupancy
    )
    q_home = np.zeros(3)
    assert checker.is_collision_free(q_home) is True


def test_obstacle_config_returns_false(
    mock_kinematics: object, mock_occupancy: object
) -> None:
    """A configuration that puts a link inside an obstacle is not free."""
    checker = CSpaceChecker(
        kinematics=mock_kinematics, occupancy=mock_occupancy
    )
    # The mock occupancy returns negative clearance when EE z < 0.18 m,
    # which happens at q3 = 0.1 m (EE z = 0.238 − 0.1 = 0.138).
    q_colliding = np.array([0.0, 0.0, 0.1])
    assert checker.is_collision_free(q_colliding) is False


def test_clearance_positive_in_free_space(
    mock_kinematics: object, mock_occupancy: object
) -> None:
    checker = CSpaceChecker(
        kinematics=mock_kinematics, occupancy=mock_occupancy
    )
    clearance = checker.clearance(np.zeros(3))
    assert isinstance(clearance, float)
    assert clearance > 0.0


def test_clearance_negative_inside_obstacle(
    mock_kinematics: object, mock_occupancy: object
) -> None:
    checker = CSpaceChecker(
        kinematics=mock_kinematics, occupancy=mock_occupancy
    )
    clearance = checker.clearance(np.array([0.0, 0.0, 0.1]))
    assert clearance < 0.0


def test_wrong_dof_raises(
    mock_kinematics: object, mock_occupancy: object
) -> None:
    checker = CSpaceChecker(
        kinematics=mock_kinematics, occupancy=mock_occupancy
    )
    with pytest.raises(ValueError):
        checker.is_collision_free(np.zeros(5))  # wrong DOF
