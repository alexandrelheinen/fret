"""SC-v20d — SO-101 pick-and-place through a dual Γ."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.kinematics_so101 import (
    HeldPointKinematics,
    grasp_offset_in_gripper,
)
from fret.control.pick_place_fsm import PickPlaceState
from fret.control.so101_pick_place_sim import (
    held_point_in_transfer_corridor,
    place_obstacles_from_params,
    release_segment_from_params,
    simulate_so101_pick_place,
)
from fret.planning.segment_planner import plan_to_segment
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/so101_wall_maze.yml")
_WALLS = (
    "transfer_wall_stem",
    "transfer_wall_cap",
    "transfer_wall_stem_p",
    "transfer_wall_cap_p",
)


def test_so101_wall_maze_mjcf_has_gamma() -> None:
    path = mjcf_path("so101", "so101_wall_maze")
    assert path.name == "so101_wall_maze.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    for name in _WALLS:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert geom >= 0, name
        assert int(model.geom_contype[geom]) == 5
        assert int(model.geom_conaffinity[geom]) == 5


def _clears(configuration: np.ndarray, boxes: list, margin: float) -> bool:
    from fret.control.so101_pick_place_sim import _arm_clears_place

    return _arm_clears_place(configuration, boxes, margin=margin)


def test_gamma_blocks_the_direct_release() -> None:
    params = load_scenario_parameters(_SCENARIO)
    bare = dict(params)
    bare.pop("walls")
    lift = np.asarray(params["lift_hover_configuration"], dtype=np.float64)
    pick = np.asarray(params["pick_grasp_configuration"], dtype=np.float64)
    offset = grasp_offset_in_gripper(pick, np.array([0.20, -0.12, 0.02]))
    kinematics = HeldPointKinematics(offset)
    margin = float(params["place_arm_margin_m"])
    segment = release_segment_from_params(params)
    direct = plan_to_segment(
        lift,
        segment,
        kinematics=kinematics,
        is_free=lambda q: _clears(
            q, place_obstacles_from_params(bare), margin
        ),
        seed=int(params["planner_rng_seed"]),
        timeout_s=4.0,
        goal_tolerance_m=float(params["place_segment_tolerance_m"]),
    )
    walls = place_obstacles_from_params(params)
    blocked = False
    for fraction in np.linspace(0.0, 1.0, 21):
        configuration = (1.0 - fraction) * direct[0] + fraction * direct[-1]
        if not _clears(configuration, walls, margin):
            blocked = True
            break
    assert blocked, "the dual Γ should cut the straight release"

    def transfer_free(configuration: np.ndarray) -> bool:
        return _clears(configuration, walls, margin) and (
            held_point_in_transfer_corridor(configuration, kinematics, params)
        )

    path = plan_to_segment(
        lift,
        segment,
        kinematics=kinematics,
        is_free=transfer_free,
        seed=int(params["planner_rng_seed"]),
        timeout_s=8.0,
        goal_tolerance_m=float(params["place_segment_tolerance_m"]),
    )
    assert len(path) > len(direct)
    steps = [
        float(np.linalg.norm(path[i] - path[i - 1]))
        for i in range(1, len(path))
    ]
    # The old plan's last edge was a 2 rad joint leap. Every step stays
    # inside the planner's joint step.
    assert max(steps) <= 0.35 + 1e-6
    held = np.array(
        [kinematics.forward_kinematics(q)[:3, 3] for q in path],
        dtype=np.float64,
    )
    # Climb over the cap while staying between the pick and the bin.
    # A retreat to y ≈ −0.30 is the motion that slews the outstretched arm.
    assert float(held[:, 1].min()) >= float(held[0, 1]) - 0.07
    assert float(held[:, 2].max()) >= 0.24
    assert float(held[-1, 2]) > float(params["place_cone_height_m"]) + 0.05
    for configuration in path:
        assert transfer_free(configuration)


def _min_clearance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_arm: np.ndarray,
    *,
    prefixes: tuple[str, ...],
) -> float:
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
    obstacle_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        ).startswith(prefixes)
    ]
    arm_ids = []
    for geom_id in range(model.ngeom):
        if geom_id in obstacle_ids:
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
        for wall in obstacle_ids:
            distance = float(
                mujoco.mj_geomDistance(model, data, arm, wall, 0.2, fromto)
            )
            worst = min(worst, distance)
    return worst


def test_so101_wall_maze_physics_drops_in_the_bin() -> None:
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    state, samples = simulate_so101_pick_place(
        duration_s=120.0,
        scenario_path=_SCENARIO,
        use_vision=False,
        record_every_steps=25,
    )
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    ball = samples[-1].box_qpos[:3]
    assert float(np.linalg.norm(ball[:2] - place_xy)) < 0.03
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.10
    assert float(ball[2]) < float(params["place_cone_height_m"])
    model = mujoco.MjModel.from_xml_path(
        str(mjcf_path("so101", "so101_wall_maze"))
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
        _min_clearance(
            model,
            data,
            sample.q_arm,
            prefixes=("place_bin", "transfer_wall"),
        )
        for sample in transfer
    )
    assert worst >= -1.0e-3, f"arm entered a wall by {-worst * 1000:.1f} mm"
