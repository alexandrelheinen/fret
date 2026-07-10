"""Physics SITL integration tests for Dubins race (SC-v12 / T12-06)."""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from fret.scenario.dubins_race_runner import DubinsRaceRunner

_SCENARIO_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "scenarios"
    / "dubins_race.yml"
)

_PLANNER_RNG_SEED = 11


@pytest.fixture(autouse=True)
def _deterministic_planner_rng(monkeypatch: pytest.MonkeyPatch) -> None:
    """ARC planners use unseeded ``default_rng()``; fix CI flakiness."""
    original_default_rng = np.random.default_rng

    def _seeded_default_rng(seed: int | None = None) -> np.random.Generator:
        return original_default_rng(
            _PLANNER_RNG_SEED if seed is None else seed
        )

    monkeypatch.setattr(np.random, "default_rng", _seeded_default_rng)


def _mujoco_available() -> bool:
    from fret.ros.mujoco_bridge import make_dubins_race_bridge_core

    return make_dubins_race_bridge_core().has_mujoco_runtime


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_dubins_physics_run_produces_contact_log() -> None:
    """V12-5/7: physics_mode Dubins run writes contact artifacts."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True, contact_log_enabled=True)
    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
    assert result.contact_log_path is not None
    assert result.contact_log_path.is_file()
    assert result.physics_metrics_path is not None
    assert result.physics_metrics_path.is_file()
    assert result.penetration_violations == 0


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_dubins_physics_planners_succeed() -> None:
    """V12-3: both agents plan successfully under physics_mode."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True)
    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
