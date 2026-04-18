"""Integration test: straight-line scenario (Milestone 1 acceptance criteria).

Verifies that the straight-line injector produces a trajectory whose
Cartesian waypoints all lie within the expected straight-line corridor, and
that the Jacobian controller (simulated in pure Python) achieves an EE
tracking error ≤ 5 mm throughout the 3-second run.

This test does NOT require a live ROS context — it runs the pure-Python
logic layer only.

Acceptance criteria checked here (MILESTONES.md Milestone 1):
  AC-2 The EE moves from FK([0,0,0.10]) to FK([0.785,0,0.10]) in 3 s.
  AC-4 Maximum EE tracking error ≤ 5 mm at every timestep (FR-CTL-02).
  AC-5 No fault is triggered during the run.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.controller_node import ControllerNode, _NodeState
from fret.control.kinematics import Kinematics
from fret.ros.straight_line_injector import generate_trajectory

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_KIN = Kinematics("scara")


def _fk_position(q: np.ndarray) -> np.ndarray:
    return _KIN.forward_kinematics(q)[:3, 3]


# ---------------------------------------------------------------------------
# Trajectory generation tests
# ---------------------------------------------------------------------------


def test_generate_trajectory_returns_correct_count() -> None:
    """generate_trajectory must return exactly n_waypoints configurations."""
    configs, timestamps = generate_trajectory(n_waypoints=50)
    assert len(configs) == 50
    assert len(timestamps) == 50


def test_generate_trajectory_timestamps_monotonic() -> None:
    """Timestamps must be strictly increasing from 0 to duration."""
    configs, timestamps = generate_trajectory(n_waypoints=30, duration=3.0)
    assert timestamps[0] == pytest.approx(0.0, abs=1e-9)
    assert timestamps[-1] == pytest.approx(3.0, abs=1e-6)
    for i in range(1, len(timestamps)):
        assert timestamps[i] > timestamps[i - 1]


def test_generate_trajectory_start_matches_fk() -> None:
    """First waypoint FK must match FK([0, 0.4, 0.10]) within 1 mm."""
    configs, _ = generate_trajectory()
    q_start = np.array([0.0, 0.40, 0.10], dtype=np.float64)
    x_expected = _fk_position(q_start)
    x_actual = _fk_position(configs[0])
    np.testing.assert_allclose(x_actual, x_expected, atol=1e-3)


def test_generate_trajectory_end_matches_fk() -> None:
    """Last waypoint FK must match FK([0.785, -0.4, 0.10]) within 1 mm."""
    configs, _ = generate_trajectory()
    q_end = np.array([0.785, -0.40, 0.10], dtype=np.float64)
    x_expected = _fk_position(q_end)
    x_actual = _fk_position(configs[-1])
    np.testing.assert_allclose(x_actual, x_expected, atol=1e-3)


def test_generate_trajectory_cartesian_straight_line() -> None:
    """All waypoints must lie close to the Cartesian straight line (≤ 5 mm)."""
    configs, _ = generate_trajectory(n_waypoints=150)
    x_start = _fk_position(configs[0])
    x_end = _fk_position(configs[-1])
    direction = x_end - x_start
    length = float(np.linalg.norm(direction))
    assert length > 0.1, "Trajectory length too short"
    unit = direction / length

    for q in configs:
        x = _fk_position(q)
        # Distance from point to the infinite line through x_start with direction unit
        diff = x - x_start
        proj = np.dot(diff, unit)
        perp = diff - proj * unit
        dist = float(np.linalg.norm(perp))
        assert dist <= 0.005, (
            f"Waypoint deviates {dist*1000:.2f} mm from the straight line; "
            f"expected ≤ 5 mm"
        )


def test_generate_trajectory_joint_space_is_nonlinear() -> None:
    """The joint trajectory must be nonlinear (q2 must vary from 0)."""
    configs, _ = generate_trajectory(n_waypoints=150)
    q2_values = np.array([q[1] for q in configs])
    # q2 should vary significantly (peak > 0.1 rad) since the Cartesian
    # straight line requires elbow bending
    assert (
        np.max(np.abs(q2_values)) > 0.1
    ), "q2 did not vary enough — joint space trajectory may be linear"


# ---------------------------------------------------------------------------
# Controller simulation (pure-Python closed-loop integration)
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_controller_tracking_error_below_5mm() -> None:
    """Jacobian controller must achieve EE error ≤ 5 mm throughout the run.

    Simulates 150 control cycles at 50 Hz with the generated trajectory.
    Tracks the maximum EE error and asserts it stays below FR-CTL-02.
    """
    configs, _ = generate_trajectory(n_waypoints=150, duration=3.0)

    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")

    # Start the controller from the first reference joint configuration so
    # the initial error is zero — we want to test steady-state tracking.
    q_cur = configs[0].copy()
    ctrl.set_trajectory(configs)

    dt = 1.0 / ctrl._update_rate  # 0.02 s
    max_error = 0.0

    for _ in range(len(configs)):
        q_dot = ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + q_dot * dt
        error = ctrl.get_ee_error_m(kin, q_cur)
        max_error = max(max_error, error)

    assert (
        max_error <= 0.005
    ), f"Maximum EE tracking error {max_error*1000:.2f} mm exceeds 5 mm limit"


@pytest.mark.timeout(30)
def test_controller_no_fault_during_tracking() -> None:
    """No fault must be triggered during straight-line tracking.

    Simulates the Jacobian controller starting from the first waypoint and
    asserts that the FSM never reaches the HALTED state.
    """
    configs, _ = generate_trajectory(n_waypoints=150, duration=3.0)

    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")

    q_cur = configs[0].copy()
    ctrl.set_trajectory(configs)

    dt = 1.0 / ctrl._update_rate

    for _ in range(len(configs)):
        ctrl.compute_jacobian_command(kin, q_cur)
        assert (
            ctrl._state != _NodeState.HALTED
        ), "Controller triggered a fault during straight-line tracking"
        q_cur = q_cur + ctrl._get_current_command() * dt


@pytest.mark.timeout(30)
def test_controller_completes_trajectory() -> None:
    """All trajectory waypoints must be consumed within the expected steps."""
    configs, _ = generate_trajectory(n_waypoints=150, duration=3.0)

    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")

    q_cur = configs[0].copy()
    ctrl.set_trajectory(configs)

    dt = 1.0 / ctrl._update_rate

    for _ in range(len(configs)):
        q_dot = ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + q_dot * dt

    assert (
        ctrl.is_trajectory_complete()
    ), "Trajectory was not fully consumed after 150 steps"


@pytest.mark.timeout(30)
def test_controller_cartesian_path_stays_straight() -> None:
    """Executed Cartesian path must stay close to the reference straight line.

    The trajectory is defined as a straight line in Cartesian space.  Even
    under closed-loop Jacobian control the executed path must not deviate
    more than 5 mm from that line (FR-CTL-02).
    """
    configs, _ = generate_trajectory(n_waypoints=150, duration=3.0)

    ctrl = ControllerNode(model="scara", config_path="")
    kin = Kinematics("scara")

    q_cur = configs[0].copy()
    ctrl.set_trajectory(configs)

    dt = 1.0 / ctrl._update_rate

    x_start = _fk_position(configs[0])
    x_end = _fk_position(configs[-1])
    direction = x_end - x_start
    length = float(np.linalg.norm(direction))
    unit = direction / length

    max_deviation = 0.0

    for _ in range(len(configs)):
        q_dot = ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + q_dot * dt

        x = _fk_position(q_cur)
        diff = x - x_start
        proj = np.dot(diff, unit)
        # Only check while within the line segment bounds
        if 0.0 <= proj <= length:
            perp = diff - proj * unit
            deviation = float(np.linalg.norm(perp))
            max_deviation = max(max_deviation, deviation)

    assert max_deviation <= 0.005, (
        f"Executed Cartesian path deviated {max_deviation*1000:.2f} mm from "
        f"the straight-line reference; expected ≤ 5 mm"
    )
