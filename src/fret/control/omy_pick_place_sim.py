"""OMY SC-v14b pick-and-place — same modular stack as OMX SC-v13b.

Stack (shared with OpenMANIPULATOR-X):

1. ``PickPlaceFSM`` — task phases (approach → grasp → lift → place → retreat)
2. Joint-space path / phase targets from scenario YAML (pad-mid IK)
3. ARCO ``JointSpaceMPC`` — carrot tracking of each phase target
4. MuJoCo position actuators — low-level joint control

OMY-specific: fang gripper + Ø86 mm ball cannot rely on MuJoCo adhesion alone
for a reliable lift, so during ``LIFT`` / ``MOVE_PLACE`` / ``DESCEND_PLACE`` the
ball is carried at the pad midpoint while the arm still follows the MPC
command (kinematic follow). No parallel FSM / ``force_state`` timers.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.joint_mpc import build_joint_mpc
from fret.control.pick_place_common import PickPlaceSample, adhesion_command
from fret.control.pick_place_fsm import (
    OMY_GRIPPER_CLOSED,
    OMY_GRIPPER_OPEN,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
    PickPlaceWaypoints,
)
from fret.sitl_config import load_scenario_parameters, mjcf_path

_ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6")
_MODEL = "omy"
_CTRL_PERIOD_S = 0.02
_SCENARIO = Path("src/fret/config/scenarios/omy_pick_place.yml")
_CARRY_STATES = frozenset(
    {
        PickPlaceState.LIFT,
        PickPlaceState.MOVE_PLACE,
        PickPlaceState.DESCEND_PLACE,
    }
)
_SNAP_STATES = frozenset(
    {
        PickPlaceState.DESCEND_PICK,
        PickPlaceState.GRASP,
        PickPlaceState.LIFT,
        PickPlaceState.DESCEND_PLACE,
        PickPlaceState.RETREAT,
    }
)


def waypoints_from_scenario(
    scenario_path: str | Path | None = None,
) -> PickPlaceWaypoints:
    """Load joint waypoints from an OMY pick-place scenario YAML."""
    path = Path(scenario_path or _SCENARIO)
    p = load_scenario_parameters(path)
    retreat = p.get("retreat_configuration", p["idle_configuration"])
    lift = p.get("lift_hover_configuration")
    return PickPlaceWaypoints(
        idle=np.asarray(p["idle_configuration"], dtype=np.float64),
        pick_hover=np.asarray(p["pick_hover_configuration"], dtype=np.float64),
        pick_grasp=np.asarray(p["pick_grasp_configuration"], dtype=np.float64),
        place_hover=np.asarray(
            p["place_hover_configuration"], dtype=np.float64
        ),
        place_grasp=np.asarray(
            p["place_grasp_configuration"], dtype=np.float64
        ),
        lift_hover=(
            np.asarray(lift, dtype=np.float64) if lift is not None else None
        ),
        retreat=np.asarray(retreat, dtype=np.float64),
    )


def _arm_q(mj: Any, model: Any, data: Any) -> npt.NDArray[np.float64]:
    return np.array(
        [
            data.qpos[
                model.jnt_qposadr[
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
                ]
            ]
            for name in _ARM_JOINTS
        ],
        dtype=np.float64,
    )


def _actuator_id(mj: Any, model: Any, name: str) -> int:
    aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"Missing MuJoCo actuator {name!r}")
    return int(aid)


def _drive_arm_to(
    mj: Any,
    model: Any,
    data: Any,
    q_des: npt.NDArray[np.float64],
    act_arm: list[int],
) -> None:
    """Set arm ``qpos``/``ctrl`` together so 6-DOF targets stay reachable."""
    q_des = np.asarray(q_des, dtype=np.float64)
    for i, name in enumerate(_ARM_JOINTS):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[jid])] = float(q_des[i])
        data.qvel[int(model.jnt_dofadr[jid])] = 0.0
        data.ctrl[act_arm[i]] = float(q_des[i])
    mj.mj_forward(model, data)


def _attach_ball_to_pads(
    mj: Any,
    model: Any,
    data: Any,
    *,
    box_qadr: int,
    box_jid: int,
    pad_right_id: int,
    pad_left_id: int,
) -> None:
    """Latch the free ball at the fingertip pad midpoint."""
    mid = 0.5 * (
        np.asarray(data.geom_xpos[pad_right_id], dtype=np.float64)
        + np.asarray(data.geom_xpos[pad_left_id], dtype=np.float64)
    )
    data.qpos[box_qadr : box_qadr + 3] = mid
    dof_adr = int(model.jnt_dofadr[box_jid])
    data.qvel[dof_adr : dof_adr + 6] = 0.0
    mj.mj_forward(model, data)


def simulate_omy_pick_place(
    *,
    duration_s: float = 55.0,
    joint_tol_rad: float = 0.18,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
) -> tuple[PickPlaceState, list[PickPlaceSample]]:
    """Run one OMY pick-place cycle (FSM → MPC → joints + pad-mid carry)."""
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for OMY pick-place sim") from exc

    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    scenario_id = str(params.get("scenario_id", "omy_pick_place"))
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path(_MODEL, scenario_id)
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)

    ee_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link6")
    box_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])
    pad_right_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right")
    pad_left_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left")
    if pad_right_id < 0 or pad_left_id < 0:
        raise ValueError("OMY pad geoms missing; regenerate MJCF with pads")

    act_arm = [_actuator_id(mj, model, n) for n in _ARM_JOINTS]
    act_grip = _actuator_id(mj, model, "Gripper")
    act_al = _actuator_id(mj, model, "grip_left")
    act_ar = _actuator_id(mj, model, "grip_right")

    lift_z = float(params.get("lift_height_m", 0.20))
    phase_timeout = float(params.get("phase_timeout_s", 75.0))
    fsm = PickPlaceFSM(
        wp,
        dof=6,
        gripper_open=OMY_GRIPPER_OPEN,
        gripper_closed=OMY_GRIPPER_CLOSED,
        joint_tol_rad=float(joint_tol_rad),
        grasp_hold_s=1.4,
        release_hold_s=0.8,
        lift_height_m=lift_z,
        phase_timeout_s=phase_timeout,
        drop_fault_enabled=False,
    )
    fsm.start()

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = OMY_GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(500):
        mj.mj_step(model, data)

    mpc = build_joint_mpc(6)
    mpc.reset(_arm_q(mj, model, data))
    q_cmd = _arm_q(mj, model, data).copy()
    last_target = wp.idle.copy()
    carrying = False
    ctrl_accum = 0.0

    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))

    for step_i in range(max_steps):
        q = _arm_q(mj, model, data)
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.asarray(data.xpos[box_id], dtype=np.float64).copy(),
            ee_pos=np.asarray(data.xpos[ee_id], dtype=np.float64).copy(),
        )
        cmd = fsm.tick(obs, dt)

        if cmd.state in _CARRY_STATES:
            carrying = True
        if cmd.state in {
            PickPlaceState.RELEASE,
            PickPlaceState.RETREAT,
            PickPlaceState.DONE,
            PickPlaceState.FAULT,
        }:
            carrying = False

        if float(np.linalg.norm(cmd.q_des - last_target)) > 1e-9:
            mpc.reset(q)
            last_target = cmd.q_des.copy()

        ctrl_accum += dt
        if ctrl_accum >= _CTRL_PERIOD_S:
            ctrl_accum -= _CTRL_PERIOD_S
            if cmd.state in _SNAP_STATES:
                q_cmd = np.asarray(cmd.q_des, dtype=np.float64).copy()
                mpc.q = q_cmd.copy()
                mpc.vel = np.zeros_like(q_cmd)
            else:
                q_cmd = np.asarray(
                    mpc.step(cmd.q_des, _CTRL_PERIOD_S), dtype=np.float64
                )
                if float(np.linalg.norm(q_cmd - cmd.q_des)) <= joint_tol_rad:
                    q_cmd = cmd.q_des.copy()
                    mpc.q = q_cmd.copy()
                    mpc.vel = np.zeros_like(q_cmd)

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
        # Stiff follow on grasp/carry/retreat so pad-mid IK targets are met.
        if carrying or cmd.state in _SNAP_STATES:
            _drive_arm_to(mj, model, data, q_cmd, act_arm)

        data.ctrl[act_grip] = float(cmd.gripper)
        adhere = adhesion_command(
            cmd.state,
            fsm.hold_t,
            gripper=float(cmd.gripper),
            gripper_closed=OMY_GRIPPER_CLOSED - 0.05,
        )
        data.ctrl[act_al] = adhere
        data.ctrl[act_ar] = adhere
        mj.mj_step(model, data)

        if carrying:
            _attach_ball_to_pads(
                mj,
                model,
                data,
                box_qadr=box_qadr,
                box_jid=box_jid,
                pad_right_id=pad_right_id,
                pad_left_id=pad_left_id,
            )

        if step_i % record_every_steps == 0:
            samples.append(
                PickPlaceSample(
                    q_arm=_arm_q(mj, model, data),
                    gripper=float(data.ctrl[act_grip]),
                    box_qpos=np.asarray(
                        data.qpos[box_qadr : box_qadr + 7], dtype=np.float64
                    ).copy(),
                    state=fsm.state,
                )
            )

        if fsm.state in {PickPlaceState.DONE, PickPlaceState.FAULT}:
            break

    if fsm.state == PickPlaceState.DONE:
        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(wp.idle[i])
        data.ctrl[act_grip] = OMY_GRIPPER_OPEN
        data.ctrl[act_al] = 0.0
        data.ctrl[act_ar] = 0.0
        hold_steps = max(record_every_steps, int(round(0.6 / dt)))
        for step_i in range(hold_steps):
            mj.mj_step(model, data)
            if step_i % record_every_steps == 0:
                samples.append(
                    PickPlaceSample(
                        q_arm=_arm_q(mj, model, data),
                        gripper=float(data.ctrl[act_grip]),
                        box_qpos=np.asarray(
                            data.qpos[box_qadr : box_qadr + 7],
                            dtype=np.float64,
                        ).copy(),
                        state=PickPlaceState.DONE,
                    )
                )

    return fsm.state, samples


def run_omy_pick_place(
    *,
    duration_s: float = 55.0,
    joint_tol_rad: float = 0.18,
    scenario_path: str | Path | None = None,
) -> tuple[PickPlaceState, npt.NDArray[np.float64]]:
    """Execute one OMY pick-place cycle; return final state and ball position."""
    state, samples = simulate_omy_pick_place(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        scenario_path=scenario_path,
    )
    if not samples:
        raise RuntimeError("OMY pick-place produced no samples")
    box_pos = samples[-1].box_qpos[:3].copy()
    return state, box_pos
