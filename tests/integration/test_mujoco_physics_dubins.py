"""Physics SITL integration tests for Dubins race (SC-v12 / T12-06)."""

from __future__ import annotations

import pathlib
import shutil
from collections.abc import Iterator

import numpy as np
import pytest

from fret.scenario.dubins_race_runner import DubinsRaceRunner, DubinsRaceRunResult

_SCENARIO_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "scenarios"
    / "dubins_race.yml"
)

_PHYSICS_ARTIFACT_DIR = pathlib.Path("/tmp/fret_physics/dubins_race")
_PLANNER_RNG_SEED = 11


@pytest.fixture(scope="module", autouse=True)
def _deterministic_planner_rng() -> Iterator[None]:
    """ARC planners use unseeded ``default_rng()``; fix CI flakiness."""
    original_default_rng = np.random.default_rng

    def _seeded_default_rng(seed: int | None = None) -> np.random.Generator:
        return original_default_rng(
            _PLANNER_RNG_SEED if seed is None else seed
        )

    np.random.default_rng = _seeded_default_rng  # type: ignore[method-assign]
    yield
    np.random.default_rng = original_default_rng  # type: ignore[method-assign]


@pytest.fixture(scope="module", autouse=True)
def _clean_dubins_physics_artifacts() -> None:
    """Remove stale contact logs once per module before the shared physics run."""
    if _PHYSICS_ARTIFACT_DIR.exists():
        shutil.rmtree(_PHYSICS_ARTIFACT_DIR)


def _mujoco_available() -> bool:
    from fret.ros.mujoco_bridge import make_dubins_race_bridge_core

    return make_dubins_race_bridge_core().has_mujoco_runtime


@pytest.fixture(scope="module")
def dubins_physics_contact_run() -> DubinsRaceRunResult:
    """One physics+contact-log run shared by V12 and V114 Dubins gates."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    return runner.run(physics_mode=True, contact_log_enabled=True)


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_dubins_physics_run_produces_contact_log(
    dubins_physics_contact_run: DubinsRaceRunResult,
) -> None:
    """V12-5/7: physics_mode Dubins run writes contact artifacts."""
    result = dubins_physics_contact_run
    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
    assert result.both_reached_goal is True
    assert result.contact_log_path is not None
    assert result.contact_log_path.is_file()
    assert result.physics_metrics_path is not None
    assert result.physics_metrics_path.is_file()
    assert result.penetration_violations == 0


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_dubins_physics_planners_succeed() -> None:
    """V12-3: both agents plan and reach goal under physics_mode."""
    runner = DubinsRaceRunner(scenario_path=_SCENARIO_PATH)
    result = runner.run(physics_mode=True)
    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
    assert result.both_reached_goal is True


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_dubins_physics_race_duration_within_v114_limit(
    dubins_physics_contact_run: DubinsRaceRunResult,
) -> None:
    """V114-01: physics race duration ≤ 180 s (CI runner headroom; ~90 s on dev VM)."""
    result = dubins_physics_contact_run
    assert result.rrt_plan.path_found is True
    assert result.sst_plan.path_found is True
    assert result.race_duration_s <= 180.0
