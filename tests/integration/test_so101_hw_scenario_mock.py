"""FR-HW-67: SC-v23a reaches DONE against mock bus + synthetic vision."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from fret.control.pick_place_fsm import (
    MotionKind,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
)
from fret.control.pick_place_sim import waypoints_from_scenario
from fret.hardware.bridge_core import make_bridge_core_for_mock
from fret.hardware.robot_io import HardwareRobotIO

_SCENARIO = Path("src/fret/config/scenarios/so101_pick_place_hw.yml")


def _load_params() -> dict[object, object]:
    raw = yaml.safe_load(_SCENARIO.read_text(encoding="utf-8"))
    return dict(raw["/**"]["ros__parameters"])


def test_sc_v23a_scenario_file_declares_speed_and_mock_port() -> None:
    params = _load_params()
    assert params["scenario_id"] == "so101_pick_place_hw"
    assert float(params["speed_scale"]) == pytest.approx(0.3)
    assert params["port"] == "mock:so101"
    assert params["use_vision"] is True


def test_sc_v23a_fsm_reaches_done_with_mock_io_fr_hw_67() -> None:
    """Drive the SO-101 FSM happy path; IO is the hardware adapter on mock."""
    params = _load_params()
    wp = waypoints_from_scenario(_SCENARIO)
    core, _bus = make_bridge_core_for_mock()
    core.connect()
    io = HardwareRobotIO(core)
    _ = io.read_joint_state()  # smoke the adapter

    fsm = PickPlaceFSM(
        wp,
        joint_tol_rad=0.05,
        grasp_hold_s=0.2,
        release_hold_s=0.2,
        auto_start_on_ball=True,
    )
    pick_xy = params["pick_xy"]
    assert isinstance(pick_xy, list)

    def step(q: np.ndarray, obj_z: float, *, ball: bool = True):
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.array([float(pick_xy[0]), float(pick_xy[1]), obj_z]),
            ee_pos=np.array([float(pick_xy[0]), float(pick_xy[1]), 0.16]),
            ball_detected=ball,
        )
        return fsm.tick(obs, 0.05)

    cmd = step(wp.idle, 0.020)
    assert fsm.state == PickPlaceState.APPROACH_PICK
    assert cmd.motion is MotionKind.PLAN_TO_GOAL

    step(wp.pick_hover, 0.020)
    assert fsm.state == PickPlaceState.DESCEND_PICK
    step(wp.pick_grasp, 0.020)
    assert fsm.state == PickPlaceState.GRASP
    for _ in range(5):
        step(wp.pick_grasp, 0.020)
    assert fsm.state == PickPlaceState.LIFT

    lift_q = wp.lift_hover if wp.lift_hover is not None else wp.pick_hover
    step(lift_q, 0.08)
    assert fsm.state == PickPlaceState.MOVE_PLACE
    step(wp.place_hover, 0.18)
    assert fsm.state == PickPlaceState.DESCEND_PLACE
    step(wp.place_grasp, 0.11)
    assert fsm.state == PickPlaceState.RELEASE
    for _ in range(5):
        step(wp.place_grasp, 0.11)
    assert fsm.state == PickPlaceState.RETREAT
    step(wp.place_hover, 0.11)
    step(wp.idle, 0.11)
    assert fsm.state == PickPlaceState.DONE
    assert fsm.cycles_completed == 1
