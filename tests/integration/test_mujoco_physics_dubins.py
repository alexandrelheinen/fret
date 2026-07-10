"""Physics SITL integration tests for Dubins race (SC-v12 / T12-06)."""

from __future__ import annotations

import pathlib
import shutil

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

_PHYSICS_ARTIFACT_DIR = pathlib.Path("/tmp/fret_physics/dubins_race")


@pytest.fixture(autouse=True)
def _clean_dubins_physics_artifacts() -> None:
    """Remove stale contact logs so penetration metrics are per-run."""
    if _PHYSICS_ARTIFACT_DIR.exists():
        shutil.rmtree(_PHYSICS_ARTIFACT_DIR)


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


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_dubins_physics_race_duration_within_v114_limit() -> None:
    """V114-01: physics race duration ≤ 180 s (CI runner headroom; ~90 s on dev VM)."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True)
    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
    assert result.race_duration_s <= 180.0
