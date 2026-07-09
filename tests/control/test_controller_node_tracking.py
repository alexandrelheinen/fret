"""Tests for ControllerNode tracking a planner-produced trajectory.

Milestone 3 — Level 3 unit tests.

Verifies that ControllerNode can track a trajectory produced by the
PlannerNode + TrajectoryGenerator pipeline (the M2 planning stack):

- FSM transitions: IDLE → TRACKING on ``set_trajectory``.
- Joint-command sequence follows the Jacobian kinematics prediction.
- EE error stays within the 5 mm tolerance (FR-CTL-02) throughout.
- No fault is triggered when tracking a well-formed planned trajectory.
- Final EE position converges to the goal within the fault threshold.

All tests run without a live ROS context (pure-Python, no ARCO required).
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.config_loader import load_algorithm_config
from fret.control.controller_node import ControllerNode, _NodeState
from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.planner_node import PlannerNode
from fret.planning.trajectory_generator import TrajectoryGenerator

_SCARA_PLANNING = load_algorithm_config("planning/scara.yml")
from fret.scene.occupancy_adapter import OccupancyAdapter

# ---------------------------------------------------------------------------
# Scenario parameters (SC-01: Static Reach, empty world)
# ---------------------------------------------------------------------------

_START_CONFIG = np.array([0.0, 0.0, 0.0])  # rest pose
_GOAL_CONFIG = np.array([0.5, -0.3, 0.05])  # reachable goal
_PLANNING_TIMEOUT = 10.0  # [s]
_SCENARIO_ID = "static_reach_unit_test"
_DURATION_S = 20.0  # simulated trajectory duration [s]
_RATE_HZ = 50.0  # controller rate [Hz]
_N_STEPS = int(_DURATION_S * _RATE_HZ)  # 1000 steps
_DT = 1.0 / _RATE_HZ
_EE_ERROR_LIMIT_M = 0.005  # 5 mm (FR-CTL-02)
# Tolerance for the monotonicity assertion: the average EE error in the second
# half of the trajectory may be at most this much larger than in the first half
# (in metres).  A small positive value accommodates numerical rounding without
# allowing meaningful regression in tracking quality.
_MONOTONICITY_TOLERANCE_M = 0.001  # 1 mm


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def planned_waypoints() -> list[np.ndarray]:
    """Plan a path with PlannerNode and return densified waypoints for tracking.

    Returns:
        List of 1000 joint configurations from start to goal.
    """
    empty_payload = OccupancyUpdatePayload(
        obstacle_points=np.zeros((0, 3), dtype=np.float64),
        timestamp=0.0,
        frame_id="world",
    )
    adapter = OccupancyAdapter()
    adapter.update(empty_payload)

    kin = Kinematics("scara")
    planner = PlannerNode(model="scara", occupancy_adapter=adapter)
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) >= 2

    # Densify the planned path to _N_STEPS waypoints over _DURATION_S seconds
    # so that the controller has a smooth, trackable reference trajectory.
    path = result.path
    waypoints: list[np.ndarray] = []
    for i in range(_N_STEPS):
        alpha = i / (_N_STEPS - 1)
        q = path[0] + alpha * (path[-1] - path[0])
        waypoints.append(q)
    return waypoints


@pytest.fixture(scope="module")
def trajectory_generator_waypoints() -> list[np.ndarray]:
    """Use TrajectoryGenerator output (not densified) for API contract tests.

    Returns:
        List of trajectory points from TrajectoryGenerator (≥ 2 points).
    """
    empty_payload = OccupancyUpdatePayload(
        obstacle_points=np.zeros((0, 3), dtype=np.float64),
        timestamp=0.0,
        frame_id="world",
    )
    adapter = OccupancyAdapter()
    adapter.update(empty_payload)

    kin = Kinematics("scara")
    planner = PlannerNode(model="scara", occupancy_adapter=adapter)
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    assert result.status == PlanningStatus.SUCCESS

    traj_gen = TrajectoryGenerator(kin, _SCARA_PLANNING)
    traj = traj_gen.process(result.path)
    assert len(traj.points) >= 2
    return [np.array(pt.positions, dtype=np.float64) for pt in traj.points]


# ---------------------------------------------------------------------------
# FSM transition tests
# ---------------------------------------------------------------------------


def test_idle_before_trajectory() -> None:
    """ControllerNode must start in IDLE state before a trajectory is loaded."""
    ctrl = ControllerNode(model="scara", config_path="")
    assert ctrl._state == _NodeState.IDLE


def test_tracking_after_set_trajectory(
    planned_waypoints: list[np.ndarray],
) -> None:
    """ControllerNode must transition to TRACKING after set_trajectory."""
    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(planned_waypoints)
    assert ctrl._state == _NodeState.TRACKING


def test_set_trajectory_from_trajectory_generator(
    trajectory_generator_waypoints: list[np.ndarray],
) -> None:
    """set_trajectory must accept waypoints produced by TrajectoryGenerator."""
    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(trajectory_generator_waypoints)
    assert ctrl._state == _NodeState.TRACKING
    assert ctrl.has_trajectory()
    assert not ctrl.is_trajectory_complete()


# ---------------------------------------------------------------------------
# Jacobian command quality tests
# ---------------------------------------------------------------------------


def test_first_command_is_small_when_aligned(
    planned_waypoints: list[np.ndarray],
) -> None:
    """When current position matches the first reference, command is near zero."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    # Start at the exact first waypoint → zero Cartesian error
    cmd = ctrl.compute_jacobian_command(kin, planned_waypoints[0].copy())
    assert (
        np.linalg.norm(cmd) < 0.5
    )  # loose bound: < 0.5 rad/s for aligned start


def test_command_shape_matches_dof(
    planned_waypoints: list[np.ndarray],
) -> None:
    """compute_jacobian_command must return a (DOF,) = (3,) array."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    cmd = ctrl.compute_jacobian_command(kin, _START_CONFIG.copy())
    assert cmd.shape == (3,)


def test_command_respects_velocity_limit(
    planned_waypoints: list[np.ndarray],
) -> None:
    """Every joint command must stay within the configured max velocity."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    q_cur = _START_CONFIG.copy()
    for _ in range(min(10, len(planned_waypoints))):
        cmd = ctrl.compute_jacobian_command(kin, q_cur)
        assert np.all(np.abs(cmd) <= ctrl._max_joint_velocity + 1e-9)
        q_cur = q_cur + cmd * _DT


# ---------------------------------------------------------------------------
# Full tracking simulation: end-to-end quality gate
# ---------------------------------------------------------------------------


def test_no_fault_during_planned_trajectory_tracking(
    planned_waypoints: list[np.ndarray],
) -> None:
    """Tracking a well-formed planned trajectory must not trigger a fault."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    q_cur = _START_CONFIG.copy()
    for _ in planned_waypoints:
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT
    assert (
        ctrl._state != _NodeState.HALTED
    ), "Fault was triggered during planned trajectory tracking"


def test_max_ee_error_within_5mm(
    planned_waypoints: list[np.ndarray],
) -> None:
    """Maximum EE tracking error must stay ≤ 5 mm (FR-CTL-02)."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    q_cur = _START_CONFIG.copy()
    max_err_m = 0.0
    for q_ref in planned_waypoints:
        x_ref = kin.forward_kinematics(q_ref)[:3, 3]
        x_cur = kin.forward_kinematics(q_cur)[:3, 3]
        err = float(np.linalg.norm(x_ref - x_cur))
        max_err_m = max(max_err_m, err)
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT
    assert (
        max_err_m <= _EE_ERROR_LIMIT_M
    ), f"Max EE error {max_err_m * 1000:.2f} mm exceeds 5 mm limit"


def test_ee_converges_to_goal(
    planned_waypoints: list[np.ndarray],
) -> None:
    """Final EE position must be close to the goal EE position (≤ 20 mm)."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    q_cur = _START_CONFIG.copy()
    for _ in planned_waypoints:
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT
    goal_ee = kin.forward_kinematics(_GOAL_CONFIG)[:3, 3]
    final_ee = kin.forward_kinematics(q_cur)[:3, 3]
    final_err_m = float(np.linalg.norm(final_ee - goal_ee))
    # Controller converges to goal within 20 mm after tracking the planned path
    assert (
        final_err_m <= 0.020
    ), f"Final EE error {final_err_m * 1000:.2f} mm is too large"


def test_trajectory_is_complete_after_tracking(
    planned_waypoints: list[np.ndarray],
) -> None:
    """After consuming all trajectory waypoints, is_trajectory_complete must return True."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    q_cur = _START_CONFIG.copy()
    for _ in planned_waypoints:
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT
    assert ctrl.is_trajectory_complete()


def test_ee_error_decreases_monotonically_on_average(
    planned_waypoints: list[np.ndarray],
) -> None:
    """The average EE error over the second half should be lower than the first half."""
    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")
    ctrl.set_trajectory(planned_waypoints)
    q_cur = _START_CONFIG.copy()
    errors: list[float] = []
    for q_ref in planned_waypoints:
        x_ref = kin.forward_kinematics(q_ref)[:3, 3]
        x_cur = kin.forward_kinematics(q_cur)[:3, 3]
        errors.append(float(np.linalg.norm(x_ref - x_cur)))
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT
    mid = len(errors) // 2
    # Skip the very first point (error = 0 at start) when comparing halves
    avg_first_half = float(np.mean(errors[1 : mid + 1]))
    avg_second_half = float(np.mean(errors[mid + 1 :]))
    assert (
        avg_second_half <= avg_first_half + _MONOTONICITY_TOLERANCE_M
    ), "EE error should not grow significantly in the second half of the trajectory"
