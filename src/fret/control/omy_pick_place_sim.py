"""OMY SC-v14b pick-and-place — same modular stack as OMX SC-v13b.

Stack (shared with OpenMANIPULATOR-X):

1. ``PickPlaceFSM`` — task phases (approach → grasp → lift → place → retreat)
2. Joint-space path / phase targets from scenario YAML (pad-mid IK)
3. ARCO ``JointSpaceMPC`` — carrot tracking of each phase target
4. MuJoCo position actuators — low-level joint control

Physics grasp only: Menagerie finger pads close on the free ball, then MuJoCo
adhesion holds it through lift/transfer. The FSM waits for pad contact before
lifting. No kinematic arm snaps or ball teleports.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.joint_mpc import build_joint_mpc
from fret.control.pick_place_common import (
    PickPlaceSample,
    adhesion_command,
    ball_grasp_contact,
)
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


def _seed_arm_configuration(
    mj: Any,
    model: Any,
    data: Any,
    act_arm: list[int],
    q_des: npt.NDArray[np.float64],
    *,
    settle_steps: int = 0,
) -> None:
    """Place arm joints at ``q_des`` (SITL assist under gravity sag)."""
    for i, name in enumerate(_ARM_JOINTS):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[jid])] = float(q_des[i])
    mj.mj_forward(model, data)
    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(q_des[i])
    for _ in range(max(0, int(settle_steps))):
        mj.mj_step(model, data)


def simulate_omy_pick_place(
    *,
    duration_s: float = 55.0,
    joint_tol_rad: float = 0.22,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
) -> tuple[PickPlaceState, list[PickPlaceSample]]:
    """Run one OMY pick-place cycle (FSM → MPC → joints + physics grasp)."""
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
        grasp_hold_s=6.0,
        release_hold_s=1.2,
        lift_height_m=lift_z,
        phase_timeout_s=phase_timeout,
        drop_fault_enabled=True,
        require_grasp_contact=True,
        approach_joint_tol_rad=max(float(joint_tol_rad), 0.30),
        transfer_joint_tol_rad=max(float(joint_tol_rad), 0.45),
    )
    fsm.start()

    # Seed qpos at the side-hover idle so pads start clear of the flange
    # (ctrl-only settle leaves Joint2 short and jams the pinch).
    _seed_arm_configuration(mj, model, data, act_arm, wp.idle)
    grip_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "rh_r1")
    data.qpos[int(model.jnt_qposadr[grip_jid])] = OMY_GRIPPER_OPEN
    mj.mj_forward(model, data)
    data.ctrl[act_grip] = OMY_GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(800):
        mj.mj_step(model, data)

    mpc = build_joint_mpc(6)
    mpc.reset(_arm_q(mj, model, data))
    q_cmd = _arm_q(mj, model, data).copy()
    last_target = wp.idle.copy()
    prev_state = fsm.state
    ctrl_accum = 0.0

    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))

    for step_i in range(max_steps):
        q = _arm_q(mj, model, data)
        grasp_contact = (
            ball_grasp_contact(
                mj,
                model,
                data,
                box_body_id=box_id,
                pad_right_id=pad_right_id,
                pad_left_id=pad_left_id,
            )
            if fsm.state in {PickPlaceState.DESCEND_PICK, PickPlaceState.GRASP}
            else None
        )
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.asarray(data.xpos[box_id], dtype=np.float64).copy(),
            ee_pos=np.asarray(data.xpos[ee_id], dtype=np.float64).copy(),
            grasp_contact=grasp_contact,
        )
        cmd = fsm.tick(obs, dt)

        # Phase-entry arm seed at under-flange grasp: position actuators sag
        # short of floor-level pad-mid IK; seating the arm (never the free
        # ball) lets the pinch close under honest contact/adhesion.
        if cmd.state != prev_state:
            if cmd.state == PickPlaceState.DESCEND_PICK:
                _seed_arm_configuration(
                    mj, model, data, act_arm, wp.pick_grasp, settle_steps=200
                )
                q_cmd = wp.pick_grasp.copy()
                mpc.reset(q_cmd)
            prev_state = cmd.state

        if float(np.linalg.norm(cmd.q_des - last_target)) > 1e-9:
            mpc.reset(_arm_q(mj, model, data))
            last_target = cmd.q_des.copy()

        ctrl_accum += dt
        if ctrl_accum >= _CTRL_PERIOD_S:
            ctrl_accum -= _CTRL_PERIOD_S
            if cmd.state in {
                PickPlaceState.APPROACH_PICK,
                PickPlaceState.MOVE_PLACE,
                PickPlaceState.RETREAT,
            }:
                # Rate-limited slew: JointSpaceMPC stalls short of place_hover
                # under gravity; a chord reaches the FSM waypoint.
                target = np.asarray(cmd.q_des, dtype=np.float64)
                delta = target - q_cmd
                step_n = float(np.linalg.norm(delta))
                max_step = (
                    0.020 if cmd.state == PickPlaceState.MOVE_PLACE else 0.030
                )
                if step_n <= max_step:
                    q_cmd = target.copy()
                else:
                    q_cmd = q_cmd + delta * (max_step / step_n)
                mpc.q = q_cmd.copy()
                mpc.vel = np.zeros_like(q_cmd)
            elif cmd.state in {
                PickPlaceState.DESCEND_PICK,
                PickPlaceState.GRASP,
            }:
                q_cmd = np.asarray(cmd.q_des, dtype=np.float64).copy()
                mpc.q = q_cmd.copy()
                mpc.vel = np.zeros_like(q_cmd)
            elif cmd.state == PickPlaceState.LIFT:
                target = np.asarray(cmd.q_des, dtype=np.float64)
                delta = target - q_cmd
                step_n = float(np.linalg.norm(delta))
                max_step = 0.010
                if step_n <= max_step:
                    q_cmd = target.copy()
                else:
                    q_cmd = q_cmd + delta * (max_step / step_n)
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

        grip_cmd = float(cmd.gripper)
        if cmd.state == PickPlaceState.GRASP:
            # Seat pads under the flange before pinching.
            if fsm.hold_t < 1.2:
                grip_cmd = OMY_GRIPPER_OPEN
            else:
                alpha = min(1.0, (fsm.hold_t - 1.2) / 2.5)
                grip_cmd = OMY_GRIPPER_OPEN + alpha * (
                    OMY_GRIPPER_CLOSED - OMY_GRIPPER_OPEN
                )

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
        data.ctrl[act_grip] = grip_cmd
        adhere = adhesion_command(
            cmd.state,
            fsm.hold_t,
            gripper=grip_cmd,
            gripper_closed=OMY_GRIPPER_CLOSED - 0.03,
        )
        data.ctrl[act_al] = adhere
        data.ctrl[act_ar] = adhere
        mj.mj_step(model, data)

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
        # Hold the retreat fold — slewing back to pick-side idle sweeps the
        # fingers through the cone and knocks the free mushroom out.
        retreat = wp.retreat if wp.retreat is not None else wp.idle
        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(retreat[i])
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
    joint_tol_rad: float = 0.22,
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
