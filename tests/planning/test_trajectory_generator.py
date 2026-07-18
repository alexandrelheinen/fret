"""Tests for fret.planning.TrajectoryGenerator."""

from __future__ import annotations

import numpy as np
import pytest

from fret.planning.trajectory_generator import TrajectoryGenerator

_MAX_INTERP_STEP_M = 0.004


def test_construction(
    mock_kinematics: object, arm_planning_config: dict[str, object]
) -> None:
    TrajectoryGenerator(kinematics=mock_kinematics, config=arm_planning_config)


def test_process_returns_trajectory_with_enough_points(
    mock_kinematics: object, arm_planning_config: dict[str, object]
) -> None:
    """Output JointTrajectory must have at least 2 points."""
    gen = TrajectoryGenerator(
        kinematics=mock_kinematics, config=arm_planning_config
    )
    path = [np.zeros(3), np.array([0.3, 0.3, 0.05]), np.array([0.5, 0.2, 0.1])]
    traj = gen.process(path)
    assert len(traj.points) >= 2  # type: ignore[attr-defined]


def test_single_waypoint_raises(
    mock_kinematics: object, arm_planning_config: dict[str, object]
) -> None:
    gen = TrajectoryGenerator(
        kinematics=mock_kinematics, config=arm_planning_config
    )
    with pytest.raises(ValueError):
        gen.process([np.zeros(3)])


def test_empty_path_raises(
    mock_kinematics: object, arm_planning_config: dict[str, object]
) -> None:
    gen = TrajectoryGenerator(
        kinematics=mock_kinematics, config=arm_planning_config
    )
    with pytest.raises(ValueError):
        gen.process([])


def test_interp_step_bounded(
    mock_kinematics: object, arm_planning_config: dict[str, object]
) -> None:
    """Consecutive waypoint EE distance must stay below the interp limit."""
    gen = TrajectoryGenerator(
        kinematics=mock_kinematics, config=arm_planning_config
    )
    path = [np.zeros(3), np.array([0.3272, 0.4712, 0.05])]
    traj = gen.process(path)
    points = traj.points
    assert len(points) >= 2

    max_step_m = 0.0
    for k in range(len(points) - 1):
        q0 = np.array(points[k].positions, dtype=np.float64)  # type: ignore[attr-defined]
        q1 = np.array(
            points[k + 1].positions, dtype=np.float64
        )  # type: ignore[attr-defined]
        ee0 = mock_kinematics.forward_kinematics(q0)[:3, 3]  # type: ignore[attr-defined]
        ee1 = mock_kinematics.forward_kinematics(q1)[:3, 3]  # type: ignore[attr-defined]
        max_step_m = max(max_step_m, float(np.linalg.norm(ee1 - ee0)))

    assert max_step_m <= _MAX_INTERP_STEP_M * 1.01
