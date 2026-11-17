"""SC-v20b — SO-101 known-pose pick-and-place (friction grasp mode)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.pick_place_fsm import PickPlaceState
from fret.control.so101_pick_place_sim import run_so101_pick_place
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/so101_pick_place.yml")


def test_so101_pick_place_mjcf_resolves() -> None:
    path = mjcf_path("so101", "so101_pick_place")
    assert path.name == "so101_pick_place.xml"
    assert path.is_file()
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu == 6


def test_so101_pick_place_physics_done_ball_in_bin() -> None:
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    cone_r = float(params.get("place_cone_radius_m", 0.06))
    state, ball = run_so101_pick_place(
        duration_s=55.0,
        scenario_path=_SCENARIO,
        use_vision=False,
    )
    assert state != PickPlaceState.FAULT
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    assert float(np.linalg.norm(ball[:2] - place_xy)) < cone_r
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.10
    assert float(ball[2]) < float(params.get("place_cone_height_m", 0.10))
