"""Scenario hook: Dubins race writes PlotJuggler telemetry when enabled."""

from __future__ import annotations

import csv
from collections.abc import Iterator
from pathlib import Path

import pytest

from fret.scenario.dubins_race_runner import DubinsRaceRunner
from fret.scenario.planner_rng import (
    SHOWCASE_PLANNER_RNG_SEED,
    deterministic_planner_rng,
)


@pytest.fixture(scope="module", autouse=True)
def _deterministic_planner_rng() -> Iterator[None]:
    with deterministic_planner_rng(SHOWCASE_PLANNER_RNG_SEED):
        yield


def test_dubins_race_writes_telemetry_csv(tmp_path: Path) -> None:
    result = DubinsRaceRunner().run(
        telemetry_enabled=True,
        telemetry_output_dir=tmp_path,
        telemetry_csv_basename="dubins_race_overview",
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
    )
    assert result.telemetry_csv_path is not None
    assert result.telemetry_csv_path.is_file()
    assert result.telemetry_manifest_path is not None
    assert result.telemetry_manifest_path.is_file()
    with open(result.telemetry_csv_path, encoding="utf-8", newline="") as fh:
        rows = list(csv.reader(fh))
    header = rows[0]
    assert header[0] == "t"
    assert "tb3_sst.position_enu.x" in header
    assert "tb3_rrt.cmd_velocity_ctrl.val" in header
    assert len(rows) > 10
