"""Tests for fret.planning.TrajectoryGenerator.

Acceptance criteria (FR-PLN-06):
  - ``process(path)`` returns a non-empty JointTrajectory with ≥ 2 points.
  - The output trajectory is C² smooth (velocity continuity at waypoints).
  - Fewer than 2 input waypoints raises ``ValueError``.
  - A RuntimeError in any post-processing stage propagates without
    swallowing (mapped to POST_PROCESS_FAILED by the planner node).
  - Consecutive waypoint EE steps stay below the controller fault threshold
    (FR-CTL-04) so the Jacobian controller never faults on waypoint advance.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.planning.trajectory_generator import (
    _MAX_INTERP_STEP_M,
    TrajectoryGenerator,
)


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


def test_interp_step_bounded_for_static_reach() -> None:
    """Consecutive waypoint EE distance must stay below fault_threshold (20 mm).

    The linear fallback must produce dense enough waypoints so that the
    Jacobian controller (fault_threshold = 0.020 m) does not fault when it
    advances from waypoint k to k+1 while the robot is still at waypoint k.
    The max allowed step is ``_MAX_INTERP_STEP_M`` (8 mm, half of 20 mm).
    """
    from fret.control.kinematics import Kinematics

    kin = Kinematics("scara")
    gen = TrajectoryGenerator(kinematics=kin)

    # Static reach scenario: rest pose → goal near obstacle cluster.
    path = [np.zeros(3), np.array([0.3272, 0.4712, 0.05])]
    traj = gen.process(path)

    points = traj.points
    assert len(points) >= 2

    max_step_m = 0.0
    for k in range(len(points) - 1):
        q0 = np.array(points[k].positions, dtype=np.float64)  # type: ignore[attr-defined]
        q1 = np.array(points[k + 1].positions, dtype=np.float64)  # type: ignore[attr-defined]
        ee0 = kin.forward_kinematics(q0)[:3, 3]
        ee1 = kin.forward_kinematics(q1)[:3, 3]
        step = float(np.linalg.norm(ee1 - ee0))
        max_step_m = max(max_step_m, step)

    assert (
        max_step_m <= _MAX_INTERP_STEP_M * 1.01
    ), (  # 1 % numerical tolerance
        f"Max inter-waypoint EE step {max_step_m * 1000:.1f} mm exceeds "
        f"{_MAX_INTERP_STEP_M * 1000:.0f} mm limit; controller would fault"
    )


def test_no_fault_with_trajectory_generator_output() -> None:
    """Controller must NOT enter HALTED when tracking TrajectoryGenerator output.

    This is an end-to-end regression test for the bug where 9-waypoint output
    from the linear fallback caused an immediate controller halt because the
    inter-waypoint Cartesian step (≈40 mm) exceeded fault_threshold (20 mm).
    """
    from fret.control.controller_node import ControllerNode, _NodeState
    from fret.control.kinematics import Kinematics

    kin = Kinematics("scara")
    gen = TrajectoryGenerator(kinematics=kin)

    path = [np.zeros(3), np.array([0.3272, 0.4712, 0.05])]
    traj = gen.process(path)
    waypoints = [np.array(pt.positions, dtype=np.float64) for pt in traj.points]  # type: ignore[attr-defined]

    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(waypoints)

    q_cur = np.zeros(3, dtype=np.float64)
    dt = 1.0 / 50.0
    for _ in waypoints:
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * dt

    assert ctrl._state != _NodeState.HALTED, (
        "Controller entered HALTED state while tracking trajectory_generator "
        "output — likely an inter-waypoint step exceeded fault_threshold"
    )
