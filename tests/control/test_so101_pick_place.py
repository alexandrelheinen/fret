"""SC-v20b — SO-101 known-pose pick-and-place (friction grasp mode)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.pick_place_fsm import PickPlaceState
from fret.control.so101_pick_place_sim import simulate_so101_pick_place
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/so101_pick_place.yml")


def test_so101_pick_place_mjcf_resolves() -> None:
    path = mjcf_path("so101", "so101_pick_place")
    assert path.name == "so101_pick_place.xml"
    assert path.is_file()
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu == 6


def _min_bin_clearance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_arm: np.ndarray,
) -> float:
    """Minimum MuJoCo distance [m] from arm collision geoms to the bin."""
    joints = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    )
    for index, name in enumerate(joints):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = float(q_arm[index])
    mujoco.mj_forward(model, data)
    bin_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        ).startswith("place_bin")
    ]
    arm_ids = []
    for geom_id in range(model.ngeom):
        if geom_id in bin_ids:
            continue
        body = int(model.geom_bodyid[geom_id])
        body_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        )
        if body_name in {"world", "pick_box", "cv_ball_ghost"}:
            continue
        if int(model.geom_contype[geom_id]) == 0:
            continue
        arm_ids.append(geom_id)
    fromto = np.zeros(6, dtype=np.float64)
    worst = 1.0
    for arm in arm_ids:
        for wall in bin_ids:
            distance = float(
                mujoco.mj_geomDistance(model, data, arm, wall, 0.2, fromto)
            )
            worst = min(worst, distance)
    return worst


def test_so101_pick_place_physics_done_ball_in_bin() -> None:
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    state, samples = simulate_so101_pick_place(
        duration_s=55.0,
        scenario_path=_SCENARIO,
        use_vision=False,
        record_every_steps=25,
    )
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    ball = samples[-1].box_qpos[:3]
    # Dropped near the centre of the opening, not merely somewhere in the cone.
    assert float(np.linalg.norm(ball[:2] - place_xy)) < 0.03
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.10
    assert float(ball[2]) < float(params.get("place_cone_height_m", 0.10))
    model = mujoco.MjModel.from_xml_path(
        str(mjcf_path("so101", "so101_pick_place"))
    )
    data = mujoco.MjData(model)
    transfer = [
        sample
        for sample in samples
        if sample.state
        in {
            PickPlaceState.MOVE_PLACE,
            PickPlaceState.DESCEND_PLACE,
            PickPlaceState.RELEASE,
        }
    ]
    assert transfer
    worst = min(
        _min_bin_clearance(model, data, sample.q_arm) for sample in transfer
    )
    assert worst >= -1.0e-3, f"arm entered the bin by {-worst * 1000:.1f} mm"
