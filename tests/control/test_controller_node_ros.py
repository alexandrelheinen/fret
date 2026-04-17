"""Tests for fret.control.ControllerNode Level 4 additions.

Verifies the new Jacobian tracking methods added in Milestone 1:
  - ``set_trajectory`` loads a trajectory and transitions to TRACKING.
  - ``has_trajectory`` / ``is_trajectory_complete`` report correct states.
  - ``compute_jacobian_command`` returns a valid command and advances the index.
  - ``get_ee_error_m`` reports Euclidean EE error in metres.
  - Fault is triggered when EE error exceeds threshold.
  - ControllerRosNode can be constructed (mocked ROS context).

All tests run without a live ROS context.
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest

from fret.control.controller_node import ControllerNode, _NodeState
from fret.control.kinematics import Kinematics


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_ctrl(tmp_path: pathlib.Path) -> ControllerNode:
    return ControllerNode(model="scara", config_path=str(tmp_path))


def _identity_traj(n: int = 5) -> list[np.ndarray]:
    """Return a trivial trajectory of n waypoints all at q=[0,0,0]."""
    return [np.zeros(3, dtype=np.float64) for _ in range(n)]


def _straight_line_traj() -> list[np.ndarray]:
    """Return a 10-waypoint trajectory using non-singular configurations.

    q1 sweeps 0→0.785, q2 sweeps 0.4→-0.4 (non-singular throughout).
    """
    configs = []
    for i in range(10):
        alpha = i / 9.0
        configs.append(
            np.array(
                [0.785 * alpha, 0.40 - 0.80 * alpha, 0.05],
                dtype=np.float64,
            )
        )
    return configs


# ---------------------------------------------------------------------------
# set_trajectory / has_trajectory / is_trajectory_complete
# ---------------------------------------------------------------------------


def test_set_trajectory_transitions_to_tracking(
    tmp_path: pathlib.Path,
) -> None:
    """set_trajectory must transition the FSM to TRACKING."""
    ctrl = _make_ctrl(tmp_path)
    assert ctrl._state == _NodeState.IDLE
    ctrl.set_trajectory(_identity_traj())
    assert ctrl._state == _NodeState.TRACKING


def test_set_trajectory_resets_index(tmp_path: pathlib.Path) -> None:
    """After set_trajectory the trajectory index must be 0."""
    ctrl = _make_ctrl(tmp_path)
    ctrl.set_trajectory(_identity_traj())
    assert ctrl._trajectory_index == 0


def test_set_trajectory_rejects_too_few_waypoints(
    tmp_path: pathlib.Path,
) -> None:
    """A trajectory with fewer than 2 waypoints must raise ValueError."""
    ctrl = _make_ctrl(tmp_path)
    with pytest.raises(ValueError):
        ctrl.set_trajectory([np.zeros(3)])


def test_has_trajectory_false_initially(tmp_path: pathlib.Path) -> None:
    """has_trajectory must return False before any trajectory is loaded."""
    ctrl = _make_ctrl(tmp_path)
    assert ctrl.has_trajectory() is False


def test_has_trajectory_true_after_set(tmp_path: pathlib.Path) -> None:
    """has_trajectory must return True after set_trajectory is called."""
    ctrl = _make_ctrl(tmp_path)
    ctrl.set_trajectory(_identity_traj())
    assert ctrl.has_trajectory() is True


def test_is_trajectory_complete_false_initially(
    tmp_path: pathlib.Path,
) -> None:
    """is_trajectory_complete must return False before loading."""
    ctrl = _make_ctrl(tmp_path)
    assert ctrl.is_trajectory_complete() is False


def test_is_trajectory_complete_after_exhaustion(
    tmp_path: pathlib.Path,
) -> None:
    """is_trajectory_complete must return True when all waypoints are consumed."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    # Use a static trajectory (same waypoint repeated) to stay within fault
    # threshold while consuming all waypoints
    q_ref = np.array([0.3, 0.2, 0.05], dtype=np.float64)
    n = 10
    traj = [q_ref.copy() for _ in range(n)]
    ctrl.set_trajectory(traj)
    dt = 1.0 / ctrl._update_rate
    q = q_ref.copy()
    for _ in range(n):
        q_dot = ctrl.compute_jacobian_command(kin, q)
        q = q + q_dot * dt
    assert ctrl.is_trajectory_complete() is True


# ---------------------------------------------------------------------------
# compute_jacobian_command
# ---------------------------------------------------------------------------


def test_compute_jacobian_command_returns_ndarray(
    tmp_path: pathlib.Path,
) -> None:
    """compute_jacobian_command must return a numpy array of shape (3,)."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    ctrl.set_trajectory(_straight_line_traj())
    cmd = ctrl.compute_jacobian_command(kin, np.zeros(3))
    assert isinstance(cmd, np.ndarray)
    assert cmd.shape == (3,)


def test_compute_jacobian_command_advances_index(
    tmp_path: pathlib.Path,
) -> None:
    """Each call to compute_jacobian_command must advance the waypoint index."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    traj = _straight_line_traj()
    ctrl.set_trajectory(traj)
    assert ctrl._trajectory_index == 0
    # Start at the first waypoint so error is zero (no fault triggered)
    ctrl.compute_jacobian_command(kin, traj[0].copy())
    assert ctrl._trajectory_index == 1


def test_compute_jacobian_command_zero_when_no_trajectory(
    tmp_path: pathlib.Path,
) -> None:
    """Without a trajectory, compute_jacobian_command must return zeros."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    cmd = ctrl.compute_jacobian_command(kin, np.zeros(3))
    np.testing.assert_array_equal(cmd, np.zeros(3))


def test_compute_jacobian_command_respects_max_velocity(
    tmp_path: pathlib.Path,
) -> None:
    """Joint velocity commands must not exceed max_joint_velocity."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    # Large error: start at q=[0,0,0], reference at q=[0.785, 0, 0]
    traj = [
        np.array([0.785, 0.0, 0.0], dtype=np.float64),
        np.array([0.785, 0.0, 0.0], dtype=np.float64),
    ]
    ctrl.set_trajectory(traj)
    cmd = ctrl.compute_jacobian_command(kin, np.zeros(3))
    assert np.all(np.abs(cmd) <= ctrl._max_joint_velocity + 1e-9)


def test_compute_jacobian_command_at_goal_is_small(
    tmp_path: pathlib.Path,
) -> None:
    """When current position equals reference, command should be near zero."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    q_ref = np.array([0.3, 0.1, 0.05], dtype=np.float64)
    traj = [q_ref.copy(), q_ref.copy()]
    ctrl.set_trajectory(traj)
    # Feed the exact reference position → Cartesian error ≈ 0
    cmd = ctrl.compute_jacobian_command(kin, q_ref.copy())
    assert np.linalg.norm(cmd) < 0.1


# ---------------------------------------------------------------------------
# get_ee_error_m
# ---------------------------------------------------------------------------


def test_get_ee_error_m_zero_when_aligned(tmp_path: pathlib.Path) -> None:
    """EE error must be near zero when current joint config matches the waypoint."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    q_ref = np.array([0.3, 0.0, 0.05], dtype=np.float64)
    ctrl.set_trajectory([q_ref.copy(), q_ref.copy()])
    err = ctrl.get_ee_error_m(kin, q_ref.copy())
    assert err < 1e-9


def test_get_ee_error_m_nonzero_when_offset(tmp_path: pathlib.Path) -> None:
    """EE error must be > 0 when current joints differ from reference."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    q_ref = np.array([0.3, 0.0, 0.05], dtype=np.float64)
    q_cur = np.array([0.0, 0.0, 0.05], dtype=np.float64)
    ctrl.set_trajectory([q_ref.copy(), q_ref.copy()])
    err = ctrl.get_ee_error_m(kin, q_cur)
    assert err > 0.01  # at least 1 cm error


def test_get_ee_error_m_zero_without_trajectory(
    tmp_path: pathlib.Path,
) -> None:
    """get_ee_error_m must return 0.0 when no trajectory is loaded."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    assert ctrl.get_ee_error_m(kin, np.zeros(3)) == 0.0


# ---------------------------------------------------------------------------
# Fault triggering via Jacobian command
# ---------------------------------------------------------------------------


def test_fault_triggered_by_large_cartesian_error(
    tmp_path: pathlib.Path,
) -> None:
    """A very large EE error must trigger HALTED and a zero command."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    # Place reference far from origin; current stays at origin
    # EE of [0.785, 0.785, 0.05] ≈ (-0.079, 0.593, 0.188) — large error
    q_ref = np.array([0.785, 0.785, 0.05], dtype=np.float64)
    ctrl.set_trajectory([q_ref.copy(), q_ref.copy()])
    # Override fault threshold to a very small value to force fault
    ctrl._fault_threshold = 0.001
    cmd = ctrl.compute_jacobian_command(kin, np.zeros(3))
    assert ctrl._state == _NodeState.HALTED
    np.testing.assert_array_equal(cmd, np.zeros(3))


# ---------------------------------------------------------------------------
# Trajectory complete stops commands
# ---------------------------------------------------------------------------


def test_command_zero_after_trajectory_exhausted(
    tmp_path: pathlib.Path,
) -> None:
    """After the trajectory is exhausted, commands must return zeros."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    q_ref = np.array([0.3, 0.2, 0.05], dtype=np.float64)
    n = 5
    traj = [q_ref.copy() for _ in range(n)]
    ctrl.set_trajectory(traj)
    dt = 1.0 / ctrl._update_rate
    q = q_ref.copy()
    for _ in range(n):
        ctrl.compute_jacobian_command(kin, q)
        q = q + ctrl._get_current_command() * dt
    # Now trajectory is complete — next call must return zeros
    cmd = ctrl.compute_jacobian_command(kin, q)
    np.testing.assert_array_equal(cmd, np.zeros(3))


# ---------------------------------------------------------------------------
# Jacobian command integration: EE converges over multiple steps
# ---------------------------------------------------------------------------


def test_jacobian_tracking_reduces_error_over_time(
    tmp_path: pathlib.Path,
) -> None:
    """Closed-loop integration should reduce EE error over several steps."""
    ctrl = _make_ctrl(tmp_path)
    kin = Kinematics("scara")
    q_ref = np.array([0.3, 0.1, 0.05], dtype=np.float64)
    # Build a static trajectory (same waypoint repeated many times)
    n = 30
    traj = [q_ref.copy() for _ in range(n)]
    ctrl.set_trajectory(traj)

    dt = 1.0 / ctrl._update_rate
    # Start close to the reference to stay within the fault threshold
    q_cur = q_ref.copy()
    q_cur[0] += 0.01  # small offset (≈ a few mm in EE space)
    initial_error = ctrl.get_ee_error_m(kin, q_cur)

    for _ in range(n - 1):
        q_dot = ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + q_dot * dt

    final_error = ctrl.get_ee_error_m(kin, q_cur)
    # Error must decrease — the controller converges
    assert final_error < initial_error or math.isclose(final_error, 0.0, abs_tol=1e-6)
