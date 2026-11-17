"""FR-HW-42: bridge-side limit clamp and per-cycle step limiting."""

from __future__ import annotations

import numpy as np

from fret.hardware.safety import SafetyConfig, SafetySupervisor


def _supervisor() -> SafetySupervisor:
    limits = np.array([[-1.0, 1.0]] * 6, dtype=np.float64)
    return SafetySupervisor(
        limits,
        config=SafetyConfig(
            watchdog_s=10.0,
            enable_lease_s=10.0,
            limit_margin_rad=0.03,
            max_joint_velocity=1.0,
            dt_s=0.02,
        ),
    )


def test_goals_clamped_to_effective_limits_fr_hw_42() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6, dtype=np.float64)
    goals = np.full(6, 5.0, dtype=np.float64)
    out = s.filter_goals(goals, measured, 0.0)
    assert float(np.max(out)) <= 1.0 - 0.03 + 1e-9
    assert s.diagnostics.clamp_events >= 1
    assert (
        "limit_clamp" in s.diagnostics.reasons
        or "step_clamp" in s.diagnostics.reasons
    )


def test_per_cycle_step_limited_by_max_velocity_dt_fr_hw_42() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6, dtype=np.float64)
    # Soft-start seed so last_written is measured.
    s.soft_start_goals(measured)
    goals = np.full(6, 0.5, dtype=np.float64)  # 0.5 > 1.0*0.02
    out = s.filter_goals(goals, measured, 0.0)
    max_step = 1.0 * 0.02
    assert float(np.max(np.abs(out - measured))) <= max_step + 1e-9
    assert s.diagnostics.clamp_events >= 1


def test_limit_sequences_stay_inside_bounds_fr_hw_42() -> None:
    """Deterministic property-style sweep (no hypothesis dependency)."""
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    measured = np.zeros(6, dtype=np.float64)
    s.soft_start_goals(measured)
    rng = np.random.default_rng(0)
    t = 0.0
    for _ in range(200):
        s.note_command(t)
        s.renew_lease(t)
        goals = rng.uniform(-3.0, 3.0, size=6)
        out = s.filter_goals(goals, measured, t)
        assert np.all(out >= -1.0 - 1e-9)
        assert np.all(out <= 1.0 + 1e-9)
        measured = out
        t += 0.02
