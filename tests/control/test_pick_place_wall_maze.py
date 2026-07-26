"""SC-v13d dual Γ-wall maze: back out under roofs → climb → front place."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.cspace_mpc_occupancy import (
    mujoco_wall_occupied_predicate,
    sample_colliding_configurations,
)
from fret.control.kinematics import Kinematics
from fret.control.pick_place_clutter_sim import (
    arm_contacts_transfer_wall,
    plan_transfer_path,
    run_pick_place_clutter,
    walls_from_scenario,
)
from fret.control.pick_place_fsm import PickPlaceState
from fret.control.pick_place_sim import waypoints_from_scenario
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/omx_wall_maze.yml")


def test_omx_wall_maze_mjcf_has_gamma_walls() -> None:
    path = mjcf_path("open_manipulator_x", "omx_wall_maze")
    model = mujoco.MjModel.from_xml_path(str(path))
    for name in (
        "transfer_wall_stem",
        "transfer_wall_cap",
        "transfer_wall_stem_p",
        "transfer_wall_cap_p",
        "place_cone",
    ):
        assert (
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
        ), name
    stem = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "transfer_wall_stem"
    )
    # Arm (contype/affinity 1) must collide with the Γ walls.
    assert int(model.geom_contype[stem]) == 1
    assert int(model.geom_conaffinity[stem]) == 1
    walls = walls_from_scenario(_SCENARIO)
    assert len(walls) == 4
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_y = float(params["pick_xy"][1])
    # Front place cone on +X (y ≈ 0).
    assert abs(float(place_xy[1])) < 1e-6
    # Dual roofs: one toward pick (−Y), one mirrored (+Y).
    roofs = [w for w in walls if w.z_min >= 0.2]
    assert len(roofs) == 2
    assert any(r.y_min < pick_y + 0.05 and r.y_max < 0.0 for r in roofs)
    assert any(r.y_min > 0.0 and r.y_max > 0.05 for r in roofs)


def test_wall_maze_transfer_plan_backs_out_and_climbs() -> None:
    wp = waypoints_from_scenario(_SCENARIO)
    params = load_scenario_parameters(_SCENARIO)
    peak_req = float(params["min_transfer_peak_ee_z_m"])
    path = None
    straight_collides = False
    last_err: Exception | None = None
    for seed_offset in range(0, 85, 17):
        try:
            path, straight_collides = plan_transfer_path(
                wp.pick_hover,
                wp.place_hover,
                scenario_path=_SCENARIO,
                seed_offset=seed_offset,
            )
            break
        except RuntimeError as exc:
            last_err = exc
    if path is None:
        raise AssertionError(f"no mesh-clear Γ path ({last_err})")
    assert straight_collides, "Γ wall should block the pick→place chord"
    assert len(path) > 2

    from fret.control.kinematics import Kinematics

    kin = Kinematics("open_manipulator_x")
    ee = np.array(
        [kin.forward_kinematics(q)[:3, 3] for q in path], dtype=np.float64
    )
    radii = np.linalg.norm(ee[:, :2], axis=1)
    end_r = 0.5 * (radii[0] + radii[-1])
    assert min(radii) < 0.90 * end_r, "expected back-out under the pick roof"
    assert float(np.max(ee[:, 2])) >= peak_req, "expected climb over Γ roof"


def test_omx_wall_maze_physics_places_ball() -> None:
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    walls = walls_from_scenario(_SCENARIO)

    result = run_pick_place_clutter(
        duration_s=55.0, scenario_path=_SCENARIO, max_attempts=6
    )
    assert result.straight_line_collides
    assert result.state != PickPlaceState.FAULT, "maze FSM faulted"
    assert result.state == PickPlaceState.DONE, f"ended in {result.state.name}"
    ball = result.box_pos
    assert float(np.linalg.norm(ball[:2] - place_xy)) < 0.08
    assert float(np.linalg.norm(ball[:2] - pick_xy)) > 0.15
    assert float(ball[2]) < 0.10, "ball should settle inside the place cone"
    for wall in walls:
        assert not (
            wall.x_min <= ball[0] <= wall.x_max
            and wall.y_min <= ball[1] <= wall.y_max
            and wall.z_min <= ball[2] <= wall.z_max
        )


@pytest.mark.parametrize(
    "scenario_name",
    (
        "omx_wall_maze.yml",
        "omx_wall_maze_rrt.yml",
        "omx_wall_maze_sst.yml",
    ),
)
def test_omx_wall_maze_mpc_transfer_avoids_wall_contacts(
    scenario_name: str,
) -> None:
    """Joint MPC C-space barriers + contact FAULT must keep walls clear."""
    scenario_path = Path("src/fret/config/scenarios") / scenario_name
    result = run_pick_place_clutter(
        duration_s=55.0, scenario_path=scenario_path, max_attempts=6
    )
    assert result.state == PickPlaceState.DONE, f"ended in {result.state.name}"
    assert not result.faulted_on_wall_contact
    assert (
        result.wall_contact_steps == 0
    ), f"{scenario_name}: unexpected wall contacts: {result.wall_contact_steps}"
    assert len(result.samples) >= 20

    path = mjcf_path("open_manipulator_x", "omx_wall_maze")
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    names = ("Joint1", "Joint2", "Joint3", "Joint4")
    qadrs = [
        int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            ]
        )
        for n in names
    ]
    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    qid = int(model.jnt_qposadr[box_jid])
    contact_frames = 0
    stride = max(1, len(result.samples) // 200)
    checked = 0
    for sample in result.samples[::stride]:
        for i, adr in enumerate(qadrs):
            data.qpos[adr] = float(sample.q_arm[i])
        data.qpos[qid : qid + 3] = [0.5, 0.5, 0.5]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        hit = False
        for ci in range(data.ncon):
            c = data.contact[ci]
            g1 = (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
                or ""
            )
            g2 = (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
                or ""
            )
            if g1.startswith("transfer_wall") or g2.startswith(
                "transfer_wall"
            ):
                hit = True
                break
        checked += 1
        if hit:
            contact_frames += 1
    rate = contact_frames / max(checked, 1)
    # v1.4.0 release clips were ~20% wall-contact; barriers should cut that.
    assert rate < 0.08, f"wall-contact rate {rate:.1%} too high"


def test_arm_contacts_transfer_wall_detects_collision_pose() -> None:
    """Contact predicate must fire on a MuJoCo wall-colliding joint pose."""
    kin = Kinematics("open_manipulator_x")
    xml = mjcf_path("open_manipulator_x", "omx_wall_maze")
    pred = mujoco_wall_occupied_predicate(str(xml))
    hits = sample_colliding_configurations(
        pred,
        kin.joint_limits,
        n_samples=6000,
        rng=np.random.default_rng(11),
    )
    assert hits.shape[0] >= 5
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    names = ("Joint1", "Joint2", "Joint3", "Joint4")
    for i, n in enumerate(names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        data.qpos[model.jnt_qposadr[jid]] = float(hits[0][i])
    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    data.qpos[int(model.jnt_qposadr[box_jid]) : int(model.jnt_qposadr[box_jid]) + 3] = [
        0.5,
        0.5,
        0.5,
    ]
    mujoco.mj_forward(model, data)
    assert arm_contacts_transfer_wall(mujoco, model, data)


def test_fsm_fault_helper_latches_fault() -> None:
    from fret.control.pick_place_fsm import PickPlaceFSM, PickPlaceWaypoints

    z = np.zeros(4, dtype=np.float64)
    wp = PickPlaceWaypoints(
        idle=z,
        pick_hover=z,
        pick_grasp=z,
        place_hover=z,
        place_grasp=z,
    )
    fsm = PickPlaceFSM(wp, dof=4)
    fsm.start()
    fsm.fault()
    assert fsm.state == PickPlaceState.FAULT
