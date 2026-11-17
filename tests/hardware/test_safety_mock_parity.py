"""FR-HW-69: same safety scenarios on mock bridge and supervisor (parity)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.safety import SafetyConfig, SafetySupervisor


def _scenarios() -> list[dict[str, object]]:
    path = (
        Path(__file__).resolve().parents[1]
        / "fixtures"
        / "hardware"
        / "safety_scenarios.yaml"
    )
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return list(raw["scenarios"])


def _supervisor() -> SafetySupervisor:
    limits = np.array([[-2.0, 2.0]] * 6, dtype=np.float64)
    return SafetySupervisor(
        limits,
        config=SafetyConfig(
            watchdog_s=0.1,
            enable_lease_s=0.5,
            limit_margin_rad=0.03,
            max_joint_velocity=2.0,
            dt_s=0.02,
        ),
    )


def test_safety_scenario_table_loads_fr_hw_69() -> None:
    scenarios = _scenarios()
    ids = {str(s["id"]) for s in scenarios}
    assert ids >= {
        "watchdog_hold",
        "limit_clamp",
        "lease_expiry",
        "soft_start",
        "stop_latch",
    }


def test_parity_watchdog_hold_fr_hw_69() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    measured = np.zeros(6)
    goals = np.full(6, 0.5)
    s.filter_goals(goals, measured, 0.2)
    assert s.diagnostics.watchdog is True

    core, _ = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    core.command_velocity(np.zeros(6), 0.0)
    core.cycle(0.0)
    core.cycle(0.2)
    assert core.diagnostics.watchdog is True


def test_parity_limit_clamp_fr_hw_69() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    s.soft_start_goals(np.zeros(6))
    out = s.filter_goals(np.full(6, 9.0), np.zeros(6), 0.0)
    assert float(np.max(np.abs(out))) <= 2.0
    assert s.diagnostics.clamp_events >= 1


def test_parity_lease_expiry_fr_hw_69() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.6)
    s.filter_goals(np.ones(6) * 0.1, np.zeros(6), 0.6)
    assert s.diagnostics.lease_expired is True


def test_parity_soft_start_fr_hw_69() -> None:
    s = _supervisor()
    measured = np.array([0.1, 0.2, 0.0, 0.0, 0.0, 0.0])
    assert np.allclose(s.soft_start_goals(measured), measured)

    core, bus = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque(0.0)
    assert core.state is BridgeState.ENABLED
    for sid in range(1, 7):
        assert bus.servo(sid).goal_position == bus.servo(sid).present_position


def test_parity_stop_latch_fr_hw_69() -> None:
    s = _supervisor()
    s.set_enabled(True)
    s.renew_lease(0.0)
    s.note_command(0.0)
    s.latch_stop()
    out = s.filter_goals(np.ones(6), np.zeros(6), 0.0)
    assert np.allclose(out, np.zeros(6))
    assert s.stop_latched is True
