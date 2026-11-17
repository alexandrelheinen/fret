"""Tests for VelocityToPositionIntegrator (FR-CTL-07)."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.velocity_to_position import VelocityToPositionIntegrator

_LIMITS = np.array([[-1.0, 1.0], [-2.0, 2.0]], dtype=np.float64)


def _integrator(**overrides: float) -> VelocityToPositionIntegrator:
    kwargs: dict[str, object] = dict(
        joint_limits=_LIMITS,
        max_joint_velocity=1.0,
        command_timeout_s=0.2,
        max_dt_s=0.06,
    )
    kwargs.update(overrides)
    return VelocityToPositionIntegrator(**kwargs)  # type: ignore[arg-type]


def test_starts_from_measured_position() -> None:
    integ = _integrator()
    q0 = np.array([0.3, -0.5])
    integ.start(q0)
    q = integ.step(now_s=0.0)
    np.testing.assert_array_equal(q, q0)


def test_integrates_velocity_over_actual_elapsed_time() -> None:
    integ = _integrator()
    integ.start(np.array([0.0, 0.0]))
    integ.step(now_s=0.0)
    integ.command_velocity(np.array([0.5, -0.5]), now_s=0.0)
    q = integ.step(now_s=0.02)
    np.testing.assert_allclose(q, [0.01, -0.01])


def test_clamps_velocity_to_profile_max() -> None:
    integ = _integrator(
        max_joint_velocity=0.5, command_timeout_s=10.0, max_dt_s=10.0
    )
    integ.start(np.array([0.0, 0.0]))
    integ.step(now_s=0.0)
    integ.command_velocity(np.array([10.0, -10.0]), now_s=0.0)
    q = integ.step(now_s=1.0)
    # Velocity clamped to +/-0.5 rad/s, integrated over 1.0 s = +/-0.5 rad,
    # well short of saturating joint_limits.
    np.testing.assert_allclose(q, [0.5, -0.5])


def test_clamps_result_to_joint_limits() -> None:
    integ = _integrator(max_joint_velocity=100.0, command_timeout_s=10.0)
    integ.start(np.array([0.0, 0.0]))
    integ.step(now_s=0.0)
    integ.command_velocity(np.array([100.0, 100.0]), now_s=0.0)
    q = integ.step(now_s=1.0)
    np.testing.assert_array_equal(q, [1.0, 2.0])


def test_holds_position_after_command_timeout() -> None:
    integ = _integrator(command_timeout_s=0.1, max_dt_s=10.0)
    integ.start(np.array([0.0, 0.0]))
    integ.step(now_s=0.0)
    integ.command_velocity(np.array([0.5, 0.5]), now_s=0.0)
    q1 = integ.step(now_s=0.05)
    assert not np.allclose(q1, [0.0, 0.0])
    # No further commands; past the timeout the integrator must hold.
    q2 = integ.step(now_s=1.0)
    np.testing.assert_array_equal(q2, q1)


def test_rejects_wrong_length_command_and_counts_it() -> None:
    integ = _integrator()
    integ.start(np.array([0.0, 0.0]))
    integ.step(now_s=0.0)
    integ.command_velocity(np.array([0.5, 0.5]), now_s=0.0)
    assert integ.rejected_command_count == 0

    integ.command_velocity(np.array([0.1, 0.1, 0.1]), now_s=0.01)
    assert integ.rejected_command_count == 1

    # Rejected command must not replace the last good one.
    q = integ.step(now_s=0.02)
    np.testing.assert_allclose(q, [0.01, 0.01])


def test_dt_spike_is_capped_so_one_late_tick_cannot_jump() -> None:
    integ = _integrator(
        max_joint_velocity=10.0, max_dt_s=0.06, command_timeout_s=10.0
    )
    integ.start(np.array([0.0, 0.0]))
    integ.step(now_s=0.0)
    integ.command_velocity(np.array([10.0, 0.0]), now_s=0.0)
    # Scheduler hiccup: a huge gap before the next tick.
    q = integ.step(now_s=5.0)
    # dt capped at max_dt_s=0.06s * 10 rad/s = 0.6 rad, not 50 rad.
    np.testing.assert_allclose(q[0], 0.6)


def test_step_before_start_raises() -> None:
    integ = _integrator()
    with pytest.raises(RuntimeError):
        integ.step(now_s=0.0)
