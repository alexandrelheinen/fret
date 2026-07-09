"""Physics SITL integration tests for Dubins race (SC-v12 / T12-06)."""

from __future__ import annotations

import pathlib

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
