"""Unit + physics smoke for SC-v13b pick-and-place FSM."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from fret.control.pick_place_fsm import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    OMX_GRIPPER,
    MotionKind,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
)
from fret.control.pick_place_motion import (
    StraightJointPlanner,
    plan_joint_segment,
)
from fret.control.pick_place_sim import run_pick_place, waypoints_from_scenario
from fret.sitl_config import load_scenario_parameters, mjcf_path

mujoco = pytest.importorskip("mujoco")

_SCENARIO = Path("src/fret/config/scenarios/omx_pick_place.yml")


def test_fsm_happy_path_transitions() -> None:
    wp = waypoints_from_scenario()
    fsm = PickPlaceFSM(
        wp,
        gripper=OMX_GRIPPER,
        joint_tol_rad=0.05,
        grasp_hold_s=0.2,
        release_hold_s=0.2,
    )
    fsm.start()
    assert fsm.state == PickPlaceState.APPROACH_PICK

    def step(q: np.ndarray, obj_z: float, dt: float = 0.05):
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.array([0.273, -0.160, obj_z]),
            ee_pos=np.array([0.273, -0.160, 0.16]),
        )
        return fsm.tick(obs, dt)

    cmd = step(wp.idle, 0.020)
    assert cmd.state == PickPlaceState.APPROACH_PICK
    assert cmd.gripper == pytest.approx(GRIPPER_OPEN)
    assert cmd.motion is MotionKind.PLAN_TO_GOAL
    assert cmd.needs_plan is True
    assert cmd.plan_goal is not None
    assert np.allclose(cmd.plan_goal, wp.pick_hover)

    cmd = step(wp.idle, 0.020)
    assert cmd.needs_plan is False  # edge trigger only on entry

    step(wp.pick_hover, 0.020)
    assert fsm.state == PickPlaceState.DESCEND_PICK

    step(wp.pick_grasp, 0.020)
    assert fsm.state == PickPlaceState.GRASP
    cmd = step(wp.pick_grasp, 0.020)
    assert cmd.gripper == pytest.approx(GRIPPER_CLOSED)
    assert cmd.motion is MotionKind.HOLD

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
        cmd = step(wp.place_grasp, 0.11)
    assert fsm.state == PickPlaceState.RETREAT
    assert cmd.gripper == pytest.approx(GRIPPER_OPEN)

    step(wp.place_hover, 0.11)
    step(wp.idle, 0.11)
    assert fsm.state == PickPlaceState.DONE
    assert fsm.cycles_completed == 1


def test_fsm_starts_on_ball_detected() -> None:
    wp = waypoints_from_scenario()
    fsm = PickPlaceFSM(wp, joint_tol_rad=0.05, auto_start_on_ball=True)
    obs = PickPlaceObservation(
        q=wp.idle,
        object_pos=np.array([0.22, -0.20, 0.11]),
        ee_pos=np.array([0.2, 0.0, 0.2]),
        ball_detected=True,
    )
    cmd = fsm.tick(obs, 0.05)
    assert fsm.state == PickPlaceState.APPROACH_PICK
    assert cmd.needs_plan is True
    assert cmd.motion is MotionKind.PLAN_TO_GOAL


def test_waypoints_with_pick_and_straight_plan() -> None:
    wp = waypoints_from_scenario()
    refreshed = wp.with_pick(
        pick_hover=wp.pick_hover + 0.01,
        pick_grasp=wp.pick_grasp + 0.01,
    )
    assert not np.allclose(refreshed.pick_hover, wp.pick_hover)
    assert np.allclose(refreshed.place_hover, wp.place_hover)
    path = plan_joint_segment(
        wp.idle, refreshed.pick_hover, planner=StraightJointPlanner(n_points=5)
    )
    assert len(path) == 5
    assert np.allclose(path[0], wp.idle)
    assert np.allclose(path[-1], refreshed.pick_hover)


def test_fsm_faults_on_drop_during_transfer() -> None:
    wp = waypoints_from_scenario()
    fsm = PickPlaceFSM(wp, joint_tol_rad=0.05, lift_height_m=0.15)
    fsm.start()
    fsm._enter(PickPlaceState.MOVE_PLACE)  # noqa: SLF001
    obs = PickPlaceObservation(
        q=wp.pick_hover,
        object_pos=np.array([0.273, 0.0, 0.04]),
        ee_pos=np.array([0.273, 0.0, 0.18]),
    )
    fsm.tick(obs, 0.05)
    assert fsm.state == PickPlaceState.FAULT


def test_omx_pick_place_mjcf_loads() -> None:
    path = mjcf_path("open_manipulator_x", "omx_pick_place")
    assert path.name == "omx_pick_place.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nu >= 7
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_box") >= 0
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "place_bin_bottom")
        >= 0
    )
    # Bin walls collide with proximal arm (bit 1); funnel catcher is ball-only
    # (bit 2). Distal pads stay on bit 4 so top-down drops are still free.
    wall = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "place_bin_wall_px"
    )
    assert wall >= 0
    assert int(model.geom_contype[wall]) == 1
    assert int(model.geom_conaffinity[wall]) == 1
    funnel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "funnel_w0")
    assert funnel >= 0
    assert int(model.geom_contype[funnel]) == 2
    ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pick_box_geom")
    assert ball >= 0
    assert int(model.geom_type[ball]) == int(mujoco.mjtGeom.mjGEOM_SPHERE)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pad_left") >= 0
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "grip_left")
        >= 0
    )


def test_omy_place_bin_blocks_proximal_arm() -> None:
    """Elbow-down dunk into the bin must contact place_bin walls (not clip)."""
    from fret.mjcf.omy import ensure_omy_pick_place_mjcf

    model = mujoco.MjModel.from_xml_path(str(ensure_omy_pick_place_mjcf()))
    data = mujoco.MjData(model)
    names = [model.geom(i).name for i in range(model.ngeom)]
    joints = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6")
    # Historical elbow-down place_grasp that penetrates wall_nx when walls
    # collide (contype=1). Must produce a place_bin contact.
    dunk = [0.4298, 0.3654, 1.8952, -0.2575, -0.9997, -0.0128]
    for name, val in zip(joints, dunk, strict=True):
        adr = int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
        )
        data.qpos[adr] = float(val)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    for _ in range(8):
        for name, val in zip(joints, dunk, strict=True):
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            data.ctrl[aid] = float(val)
        mujoco.mj_step(model, data)
    bin_hits = []
    for k in range(int(data.ncon)):
        c = data.contact[k]
        g1, g2 = names[c.geom1], names[c.geom2]
        if (g1 and g1.startswith("place_bin")) or (
            g2 and g2.startswith("place_bin")
        ):
            bin_hits.append((g1, g2))
    assert bin_hits, "expected proximal arm ↔ place_bin contact on dunk pose"


def test_omx_pick_place_physics_moves_ball_into_place_cone() -> None:
    params = load_scenario_parameters(_SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64)
    state, ball = run_pick_place(duration_s=25.0)
    assert state != PickPlaceState.FAULT, "pick-place FSM faulted"
    assert state == PickPlaceState.DONE, f"ended in {state.name}"
    assert (
        float(np.linalg.norm(ball[:2] - place_xy)) < 0.08
    ), f"ball XY {ball[:2]} not near place {place_xy}"
    assert (
        float(np.linalg.norm(ball[:2] - pick_xy)) > 0.15
    ), "ball should leave the pick spot"
    # Settled in the place receptacle (not still near rim height ~0.11).
    # Ø40 mm ball rests a bit higher than the old Ø25 mm threshold.
    assert float(ball[2]) < 0.10, "ball should settle inside the place bin"
