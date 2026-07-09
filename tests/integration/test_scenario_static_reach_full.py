"""End-to-end integration test: static reach scenario (Milestone 3 full pipeline).

Milestone 3 — Level 3 integration tests.

Verifies the complete pipeline without a live ROS context:
  Planning (PlannerNode) → TrajectoryGenerator → ControllerNode tracking.

These tests serve as the pure-Python proxy for the full SITL launch test
(``ros2 launch fret sitl.py scenario:=static_reach``).  They validate
acceptance criteria 1–7 of Milestone 3 in a deterministic, ROS-free
environment.

Acceptance criteria checked:
  1. PlannerNode returns SUCCESS within the timeout.
  2. Trajectory has ≥ 2 waypoints and all waypoints are within joint limits.
  3. ControllerNode transitions from IDLE to TRACKING on trajectory receipt.
  4. Controller publishes commands (non-zero) while tracking.
  5. Max EE position error ≤ 5 mm throughout tracking (FR-CTL-02).
  6. No fault is triggered.
  7. Final EE position is within 20 mm of the goal EE position.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from fret.control.controller_node import ControllerNode, _NodeState
from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.planner_node import PlannerNode
from fret.config_loader import load_algorithm_config
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter

_SCARA_PLANNING = load_algorithm_config("planning/scara.yml")

# ---------------------------------------------------------------------------
# Scenario parameters — SC-01 Static Reach
# ---------------------------------------------------------------------------

_START_CONFIG = np.array([0.0, 0.0, 0.0])  # rest pose
_GOAL_CONFIG = np.array([0.5, -0.3, 0.05])  # pre-computed IK of goal position
_PLANNING_TIMEOUT = 10.0  # [s]  (must be < 30 s — FR-PLN-01)
_SCENARIO_ID = "static_reach"
_DURATION_S = 20.0  # simulated trajectory duration [s]
_RATE_HZ = 50.0  # controller update rate [Hz]
_N_STEPS = int(_DURATION_S * _RATE_HZ)  # 1000 steps
_DT = 1.0 / _RATE_HZ
_EE_ERROR_LIMIT_M = 0.005  # 5 mm (FR-CTL-02)
_GOAL_CONVERGENCE_M = 0.020  # 20 mm final goal tolerance


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _build_pipeline() -> tuple[PlannerNode, TrajectoryGenerator, Kinematics]:
    """Construct the planning pipeline for the static-reach scenario.

    Returns:
        Tuple of (PlannerNode, TrajectoryGenerator, Kinematics).
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
    traj_gen = TrajectoryGenerator(kin, _SCARA_PLANNING)
    return planner, traj_gen, kin


def _densify_path(path: list[np.ndarray], n_steps: int) -> list[np.ndarray]:
    """Linearly interpolate a joint-space path to n_steps waypoints.

    Args:
        path: List of joint configurations (at least 2 elements).
        n_steps: Number of output waypoints.

    Returns:
        List of n_steps joint configurations.
    """
    waypoints: list[np.ndarray] = []
    for i in range(n_steps):
        alpha = i / (n_steps - 1)
        q = path[0] + alpha * (path[-1] - path[0])
        waypoints.append(q)
    return waypoints


# ---------------------------------------------------------------------------
# Acceptance criterion 1 — Planning succeeds within timeout
# ---------------------------------------------------------------------------


def test_planner_returns_success_within_timeout() -> None:
    """PlannerNode must return SUCCESS within the planning timeout."""
    planner, _, _ = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    t0 = time.monotonic()
    result = planner.plan(req)
    elapsed = time.monotonic() - t0
    assert result.status == PlanningStatus.SUCCESS
    assert (
        elapsed <= _PLANNING_TIMEOUT
    ), f"Planning took {elapsed:.2f} s, exceeds timeout {_PLANNING_TIMEOUT} s"


# ---------------------------------------------------------------------------
# Acceptance criterion 2 — Trajectory has ≥ 2 waypoints within joint limits
# ---------------------------------------------------------------------------


def test_trajectory_has_minimum_waypoints() -> None:
    """Planned path must have ≥ 2 waypoints."""
    planner, traj_gen, _ = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    assert len(result.path) >= 2


def test_trajectory_waypoints_within_joint_limits() -> None:
    """Every planned waypoint must be within the SCARA joint limits."""
    planner, _, kin = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    limits = kin.joint_limits  # (DOF, 2)
    for q in result.path:
        for i, (lo, hi) in enumerate(limits):
            assert (
                lo - 1e-9 <= float(q[i]) <= hi + 1e-9
            ), f"Waypoint joint {i} = {q[i]:.4f} out of limits [{lo:.4f}, {hi:.4f}]"


def test_trajectory_generator_produces_points() -> None:
    """TrajectoryGenerator must produce ≥ 2 trajectory points from the plan."""
    planner, traj_gen, _ = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    traj = traj_gen.process(result.path)
    assert len(traj.points) >= 2


# ---------------------------------------------------------------------------
# Acceptance criterion 3 — Controller transitions IDLE → TRACKING
# ---------------------------------------------------------------------------


def test_controller_transitions_idle_to_tracking() -> None:
    """ControllerNode must be IDLE before trajectory receipt and TRACKING after."""
    planner, traj_gen, kin = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)

    ctrl = ControllerNode(model="scara", config_path="")
    assert ctrl._state == _NodeState.IDLE

    waypoints = _densify_path(result.path, _N_STEPS)
    ctrl.set_trajectory(waypoints)
    assert ctrl._state == _NodeState.TRACKING


# ---------------------------------------------------------------------------
# Acceptance criterion 4 — Controller publishes non-zero commands
# ---------------------------------------------------------------------------


def test_controller_publishes_nonzero_commands() -> None:
    """The controller must issue non-zero joint commands when tracking."""
    planner, _, kin = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    ctrl = ControllerNode(model="scara", config_path="")
    waypoints = _densify_path(result.path, _N_STEPS)
    ctrl.set_trajectory(waypoints)

    # With a non-zero reference trajectory, the first few commands must be
    # non-zero (the arm has to move from start to goal)
    q_cur = _START_CONFIG.copy()
    nonzero_count = 0
    for _ in range(min(100, _N_STEPS)):
        cmd = ctrl.compute_jacobian_command(kin, q_cur)
        if np.linalg.norm(cmd) > 1e-6:
            nonzero_count += 1
        q_cur = q_cur + cmd * _DT
    assert nonzero_count > 0, "Controller never issued non-zero commands"


# ---------------------------------------------------------------------------
# Acceptance criterion 5 — Max EE error ≤ 5 mm
# ---------------------------------------------------------------------------


def test_max_ee_error_within_5mm_full_run() -> None:
    """Full tracking simulation: max EE error must stay ≤ 5 mm (FR-CTL-02)."""
    planner, _, kin = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    ctrl = ControllerNode(model="scara", config_path="")
    waypoints = _densify_path(result.path, _N_STEPS)
    ctrl.set_trajectory(waypoints)

    q_cur = _START_CONFIG.copy()
    max_err_m = 0.0
    for q_ref in waypoints:
        x_ref = kin.forward_kinematics(q_ref)[:3, 3]
        x_cur = kin.forward_kinematics(q_cur)[:3, 3]
        max_err_m = max(max_err_m, float(np.linalg.norm(x_ref - x_cur)))
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT

    assert (
        max_err_m <= _EE_ERROR_LIMIT_M
    ), f"Max EE error {max_err_m * 1000:.2f} mm exceeds 5 mm limit"


# ---------------------------------------------------------------------------
# Acceptance criterion 6 — No fault triggered
# ---------------------------------------------------------------------------


def test_no_fault_throughout_run() -> None:
    """No fault must be triggered during the full tracking run."""
    planner, _, kin = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    ctrl = ControllerNode(model="scara", config_path="")
    waypoints = _densify_path(result.path, _N_STEPS)
    ctrl.set_trajectory(waypoints)

    q_cur = _START_CONFIG.copy()
    for _ in waypoints:
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT

    assert ctrl._state != _NodeState.HALTED, "Fault was triggered during run"


# ---------------------------------------------------------------------------
# Acceptance criterion 7 — EE converges to goal
# ---------------------------------------------------------------------------


def test_ee_arrives_at_goal() -> None:
    """After tracking the planned trajectory, EE must be close to the goal."""
    planner, _, kin = _build_pipeline()
    req = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )
    result = planner.plan(req)
    ctrl = ControllerNode(model="scara", config_path="")
    waypoints = _densify_path(result.path, _N_STEPS)
    ctrl.set_trajectory(waypoints)

    q_cur = _START_CONFIG.copy()
    for _ in waypoints:
        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT

    goal_ee = kin.forward_kinematics(_GOAL_CONFIG)[:3, 3]
    final_ee = kin.forward_kinematics(q_cur)[:3, 3]
    final_err_m = float(np.linalg.norm(final_ee - goal_ee))
    assert final_err_m <= _GOAL_CONVERGENCE_M, (
        f"Final EE error {final_err_m * 1000:.2f} mm "
        f"exceeds {_GOAL_CONVERGENCE_M * 1000:.0f} mm convergence limit"
    )
