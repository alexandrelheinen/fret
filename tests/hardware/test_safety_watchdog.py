"""FR-HW-41: 100 ms command watchdog holds measured pose."""

from __future__ import annotations

import numpy as np

from fret.hardware.safety import SafetyConfig, SafetySupervisor


def _supervisor(*, watchdog_s: float = 0.1) -> SafetySupervisor:
    limits = np.array([[-2.0, 2.0]] * 6, dtype=np.float64)
    return SafetySupervisor(
        limits,
        config=SafetyConfig(
            watchdog_s=watchdog_s,
            dt_s=0.02,
            max_joint_velocity=10.0,
            limit_margin_rad=0.0,
        ),
    )


def test_watchdog_holds_measured_after_timeout_fr_hw_41() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6, dtype=np.float64)
    goals = np.full(6, 0.15, dtype=np.float64)  # within one step
    s.soft_start_goals(measured)
    out_ok = s.filter_goals(goals, measured, 0.05)
    assert np.allclose(out_ok, goals)
    assert s.diagnostics.watchdog is False

    out_hold = s.filter_goals(goals, measured, 0.1000001)
    assert np.allclose(out_hold, measured)
    assert s.diagnostics.watchdog is True
    assert "watchdog" in s.diagnostics.reasons


def test_watchdog_boundary_exact_is_deterministic_fr_hw_41() -> None:
    s = _supervisor(watchdog_s=0.1)
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6, dtype=np.float64)
    goals = np.full(6, 0.15, dtype=np.float64)
    s.soft_start_goals(measured)
    # (now - last) > watchdog holds; equality still accepts the command.
    out_eq = s.filter_goals(goals, measured, 0.1)
    assert s.diagnostics.watchdog is False
    assert np.allclose(out_eq, goals)
    out_over = s.filter_goals(goals, measured, 0.1000001)
    assert np.allclose(out_over, measured)
    assert s.diagnostics.watchdog is True


def test_new_command_resumes_motion_fr_hw_41() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6, dtype=np.float64)
    goals = np.full(6, 0.15, dtype=np.float64)
    s.soft_start_goals(measured)
    s.filter_goals(goals, measured, 0.2)
    assert s.diagnostics.watchdog is True
    s.note_command(0.25)
    s.renew_lease(0.25)
    out = s.filter_goals(goals, measured, 0.26)
    assert s.diagnostics.watchdog is False
    assert np.allclose(out, goals)
