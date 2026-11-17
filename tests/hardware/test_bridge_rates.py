"""FR-HW-21: joint-state cadence from SYNC READ (simulated clock)."""

from __future__ import annotations

import pytest

from fret.hardware.bridge_core import make_bridge_core_for_mock


def test_fifty_hz_cycle_publishes_states_fr_hw_21() -> None:
    clock = {"t": 0.0}

    def now() -> float:
        return clock["t"]

    core, _bus = make_bridge_core_for_mock()
    core._clock = now
    core.connect()
    stamps: list[float] = []
    for i in range(50):
        clock["t"] = i * 0.02
        core.cycle(clock["t"])
        stamps.append(clock["t"])
        assert core.positions.shape == (6,)
    assert len(stamps) == 50
    assert stamps[-1] - stamps[0] == pytest.approx(0.98)
