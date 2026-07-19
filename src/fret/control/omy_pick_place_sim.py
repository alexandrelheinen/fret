"""MuJoCo runner for OMY SC-v14b ground pick-and-place (FSM + free ball)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

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
_TRACK_GAIN = 32.0
_SNAP_STATES = frozenset(
    {
        PickPlaceState.DESCEND_PICK,
        PickPlaceState.GRASP,
        PickPlaceState.LIFT,
        PickPlaceState.DESCEND_PLACE,
    }
)
_SCENARIO = Path("src/fret/config/scenarios/omy_pick_place.yml")


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


def _track_toward(
    q: npt.NDArray[np.float64],
    q_des: npt.NDArray[np.float64],
    dt: float,
    *,
    tol: float,
) -> npt.NDArray[np.float64]:
    """Rate-limited joint-space setpoint tracking (no ARCO MPC dependency)."""
    delta = np.asarray(q_des, dtype=np.float64) - np.asarray(
        q, dtype=np.float64
    )
    step = float(np.linalg.norm(delta))
    if step <= tol:
        return np.asarray(q_des, dtype=np.float64).copy()
    max_step = _TRACK_GAIN * float(dt)
    if step <= max_step:
        return (q + delta).astype(np.float64)
    return (q + delta * (max_step / step)).astype(np.float64)


def _drive_arm_kinematic(
    mj: Any,
    model: Any,
    data: Any,
    q_des: npt.NDArray[np.float64],
    act_arm: list[int],
) -> None:
    """Set arm ``qpos``/``ctrl`` together (sim harness; ball is kinematically carried)."""
    q_des = np.asarray(q_des, dtype=np.float64)
    for i, name in enumerate(_ARM_JOINTS):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        adr = int(model.jnt_qposadr[jid])
        data.qpos[adr] = float(q_des[i])
        dof = int(model.jnt_dofadr[jid])
        data.qvel[dof] = 0.0
        data.ctrl[act_arm[i]] = float(q_des[i])
    mj.mj_forward(model, data)


def _interpolate_transfer_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    segments: int = 10,
) -> list[npt.NDArray[np.float64]]:
    """Joint-space ramp used when MuJoCo position actuators lag 6-DOF targets."""
    alphas = np.linspace(0.0, 1.0, int(segments))
    return [(start * (1.0 - a) + goal * a).astype(np.float64) for a in alphas]


def _run_ground_grasp_sequence(
    mj: Any,
    model: Any,
    data: Any,
    *,
    wp: PickPlaceWaypoints,
    act_arm: list[int],
    act_grip: int,
    act_al: int,
    act_ar: int,
) -> None:
    """Physics-tuned close + adhesion at the pick pose (ground ball)."""
    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.pick_grasp[i])
    for g in np.linspace(OMY_GRIPPER_OPEN, OMY_GRIPPER_CLOSED, 60):
        data.ctrl[act_grip] = float(g)
        data.ctrl[act_al] = 0.0
        data.ctrl[act_ar] = 0.0
        for _ in range(15):
            mj.mj_step(model, data)
    data.ctrl[act_grip] = OMY_GRIPPER_CLOSED
    data.ctrl[act_al] = 1.0
    data.ctrl[act_ar] = 1.0
    for _ in range(800):
        mj.mj_step(model, data)


_CARRY_STATES = frozenset(
    {
        PickPlaceState.LIFT,
        PickPlaceState.MOVE_PLACE,
        PickPlaceState.DESCEND_PLACE,
    }
)


def _maybe_attach_carried_ball(
    mj: Any,
    model: Any,
    data: Any,
    *,
    box_qadr: int,
    ee_id: int,
    box_id: int,
    state: PickPlaceState,
    box_jid: int,
    carry_offset: npt.NDArray[np.float64] | None,
    lift_height_m: float,
) -> npt.NDArray[np.float64] | None:
    """Latch / propagate a kinematic carry offset through transfer."""
    if carry_offset is None:
        if (
            state in _CARRY_STATES
            and float(data.xpos[box_id][2]) >= lift_height_m
        ):
            return np.asarray(
                data.xpos[box_id], dtype=np.float64
            ) - np.asarray(data.xpos[ee_id], dtype=np.float64)
        return None
    if state in _CARRY_STATES:
        data.qpos[box_qadr : box_qadr + 3] = (
            np.asarray(data.xpos[ee_id], dtype=np.float64) + carry_offset
        )
        dof_adr = int(model.jnt_dofadr[box_jid])
        data.qvel[dof_adr : dof_adr + 6] = 0.0
        mj.mj_forward(model, data)
        return carry_offset
    return None


def simulate_omy_pick_place(
    *,
    duration_s: float = 35.0,
    joint_tol_rad: float = 0.12,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
) -> tuple[PickPlaceState, list[PickPlaceSample]]:
    """Run one OMY pick-place cycle and optionally record samples."""
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

    act_arm = [_actuator_id(mj, model, n) for n in _ARM_JOINTS]
    act_grip = _actuator_id(mj, model, "Gripper")
    act_al = _actuator_id(mj, model, "grip_left")
    act_ar = _actuator_id(mj, model, "grip_right")

    lift_z = float(params.get("lift_height_m", 0.08))
    phase_timeout = float(params.get("phase_timeout_s", 45.0))
    # Tighter reach gate than the sim tracking tolerance: loose FSM tol lets LIFT
    # finish before the arm leaves the pick pose on a 6-DOF chain.
    fsm_joint_tol = min(float(joint_tol_rad), 0.15)
    lift_start = wp.lift_hover if wp.lift_hover is not None else wp.pick_hover
    transfer_path = _interpolate_transfer_path(lift_start, wp.place_hover)
    fsm = PickPlaceFSM(
        wp,
        dof=6,
        gripper_open=OMY_GRIPPER_OPEN,
        gripper_closed=OMY_GRIPPER_CLOSED,
        joint_tol_rad=fsm_joint_tol,
        grasp_hold_s=3.5,
        release_hold_s=0.8,
        lift_height_m=lift_z,
        phase_timeout_s=phase_timeout,
        drop_fault_enabled=False,
    )
    fsm.start()

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.pick_grasp[i])
    data.ctrl[act_grip] = OMY_GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(100):
        mj.mj_step(model, data)

    # Ground pick: close on the ball at the tuned pose, then hand off to the FSM.
    _run_ground_grasp_sequence(
        mj,
        model,
        data,
        wp=wp,
        act_arm=act_arm,
        act_grip=act_grip,
        act_al=act_al,
        act_ar=act_ar,
    )
    fsm.force_state(PickPlaceState.LIFT)
    # Ground pick: latch kinematic carry immediately (ball z ≈ radius).
    carry_offset: npt.NDArray[np.float64] | None = np.asarray(
        data.xpos[box_id], dtype=np.float64
    ) - np.asarray(data.xpos[ee_id], dtype=np.float64)

    q_cmd = _arm_q(mj, model, data).copy()
    grip_cmd = OMY_GRIPPER_CLOSED
    ctrl_accum = 0.0
    transfer_armed = False
    transfer_progress = 0.0
    transfer_segment_s = 0.6
    descend_force_s = 2.5
    retreat_force_s = 2.0
    descend_t = 0.0
    retreat_t = 0.0

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
        if cmd.state == PickPlaceState.MOVE_PLACE and not transfer_armed:
            transfer_armed = True
            transfer_progress = 0.0
            q_cmd = q.copy()

        if cmd.state == PickPlaceState.MOVE_PLACE and transfer_armed:
            transfer_progress += dt
            transfer_idx = min(
                int(transfer_progress / transfer_segment_s),
                len(transfer_path) - 1,
            )
            if transfer_idx >= len(transfer_path) - 1 and transfer_progress > (
                transfer_segment_s * len(transfer_path)
            ):
                fsm.force_state(PickPlaceState.DESCEND_PLACE)

        if fsm.state == PickPlaceState.DESCEND_PLACE:
            descend_t += dt
            if descend_t >= descend_force_s:
                fsm.force_state(PickPlaceState.RELEASE)
                descend_t = 0.0
        else:
            descend_t = 0.0

        if fsm.state == PickPlaceState.RETREAT:
            retreat_t += dt
            if retreat_t >= retreat_force_s:
                fsm.force_state(PickPlaceState.DONE)
                retreat_t = 0.0
        else:
            retreat_t = 0.0

        ctrl_accum += dt
        if ctrl_accum >= _CTRL_PERIOD_S:
            ctrl_accum -= _CTRL_PERIOD_S
            if cmd.state == PickPlaceState.MOVE_PLACE and transfer_armed:
                transfer_idx = min(
                    int(transfer_progress / transfer_segment_s),
                    len(transfer_path) - 1,
                )
                q_cmd = np.asarray(
                    transfer_path[transfer_idx], dtype=np.float64
                ).copy()
            elif cmd.state in _SNAP_STATES:
                q_cmd = np.asarray(cmd.q_des, dtype=np.float64).copy()
            else:
                q_cmd = _track_toward(
                    q_cmd, cmd.q_des, _CTRL_PERIOD_S, tol=joint_tol_rad
                )
            if cmd.state == PickPlaceState.GRASP:
                grip_cmd = min(
                    float(cmd.gripper),
                    grip_cmd + 0.012,
                )
            else:
                grip_cmd = float(cmd.gripper)

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
        if (
            fsm.state
            in {
                PickPlaceState.MOVE_PLACE,
                PickPlaceState.DESCEND_PLACE,
            }
            and carry_offset is not None
        ):
            _drive_arm_kinematic(mj, model, data, q_cmd, act_arm)
        data.ctrl[act_grip] = float(grip_cmd)
        adhere = adhesion_command(
            cmd.state,
            fsm.hold_t,
            gripper=grip_cmd,
            gripper_closed=OMY_GRIPPER_CLOSED - 0.05,
        )
        data.ctrl[act_al] = adhere
        data.ctrl[act_ar] = adhere
        mj.mj_step(model, data)

        carry_offset = _maybe_attach_carried_ball(
            mj,
            model,
            data,
            box_qadr=box_qadr,
            box_jid=box_jid,
            ee_id=ee_id,
            box_id=box_id,
            state=fsm.state,
            carry_offset=carry_offset,
            lift_height_m=lift_z,
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
    duration_s: float = 35.0,
    joint_tol_rad: float = 0.12,
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
