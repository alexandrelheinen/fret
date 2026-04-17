"""Tests for fret.planning.TrajectoryGenerator.

Acceptance criteria (FR-PLN-06):
  - ``process(path)`` returns a non-empty JointTrajectory with ≥ 2 points.
  - The output trajectory is C² smooth (velocity continuity at waypoints).
  - Fewer than 2 input waypoints raises ``ValueError``.
  - A RuntimeError in any post-processing stage propagates without
    swallowing (mapped to POST_PROCESS_FAILED by the planner node).
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.planning.trajectory_generator import TrajectoryGenerator


def test_construction(mock_kinematics: object) -> None:
    TrajectoryGenerator(kinematics=mock_kinematics)


def test_process_returns_trajectory_with_enough_points(
    mock_kinematics: object,
) -> None:
    """Output JointTrajectory must have at least 2 points."""
    gen = TrajectoryGenerator(kinematics=mock_kinematics)
    path = [np.zeros(3), np.array([0.3, 0.3, 0.05]), np.array([0.5, 0.2, 0.1])]
    traj = gen.process(path)
    assert len(traj.points) >= 2  # type: ignore[attr-defined]


def test_single_waypoint_raises(mock_kinematics: object) -> None:
    gen = TrajectoryGenerator(kinematics=mock_kinematics)
    with pytest.raises(ValueError):
        gen.process([np.zeros(3)])


def test_empty_path_raises(mock_kinematics: object) -> None:
    gen = TrajectoryGenerator(kinematics=mock_kinematics)
    with pytest.raises(ValueError):
        gen.process([])
