"""SC-v13c desk-clutter: wall forces planned detour + physical place."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.pick_place_clutter_sim import (
    plan_transfer_path,
    run_pick_place_clutter,
    wall_from_scenario,
)
from fret.control.pick_place_fsm import PickPlaceState
from fret.control.pick_place_sim import waypoints_from_scenario
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/omx_desk_clutter.yml")


def test_omx_desk_clutter_mjcf_has_wall_and_cone() -> None:
    path = mjcf_path("open_manipulator_x", "omx_desk_clutter")
    model = mujoco.MjModel.from_xml_path(str(path))
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "transfer_wall")
        >= 0
    )
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "place_bucket")
        >= 0
    )
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box") >= 0
    assert model.nu >= 7


def test_transfer_plan_is_detour_not_straight_line() -> None:
    wp = waypoints_from_scenario(_SCENARIO)
    path, straight_collides = plan_transfer_path(wp.pick_hover, wp.place_hover)
    assert straight_collides, "wall should block the pick→place chord"
    assert len(path) > 2, "detour must be denser than a 2-point line"
    # Retract signature: some waypoint with smaller EE XY radius than ends.
    from fret.control.kinematics import Kinematics

    kin = Kinematics("open_manipulator_x")
    radii = [
        float(np.linalg.norm(kin.forward_kinematics(q)[:2, 3])) for q in path
    ]
    end_r = 0.5 * (radii[0] + radii[-1])
    assert (
        min(radii) < 0.85 * end_r
    ), f"expected retract detour, min_r={min(radii):.3f} end_r={end_r:.3f}"


def test_omx_desk_clutter_physics_places_ball_around_wall() -> None:
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    wall = wall_from_scenario(_SCENARIO)

    result = run_pick_place_clutter(duration_s=45.0)
    assert result.straight_line_collides
    assert result.state != PickPlaceState.FAULT, "clutter FSM faulted"
    assert result.state == PickPlaceState.DONE, f"ended in {result.state.name}"
    ball = result.box_pos
    assert float(np.linalg.norm(ball[:2] - place_xy)) < 0.08
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.15
    assert float(ball[2]) < 0.10, "ball should settle inside the place cone"
    assert not (
        wall.x_min <= ball[0] <= wall.x_max
        and wall.y_min <= ball[1] <= wall.y_max
        and wall.z_min <= ball[2] <= wall.z_max
    )
