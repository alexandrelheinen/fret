"""OMY cluttered pick-and-place (SC-v14c) with planner detour + physics grasp."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

from fret.control.omy_pick_place_sim import (
    _SNAP_STATES,
    _actuator_id,
    _arm_q,
    _maybe_attach_carried_ball,
    _run_ground_grasp_sequence,
    _track_toward,
    waypoints_from_scenario,
)
from fret.control.pick_place_common import PickPlaceSample, adhesion_command
from fret.control.pick_place_fsm import (
    OMY_GRIPPER_CLOSED,
    OMY_GRIPPER_OPEN,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
)
from fret.sitl_config import load_scenario_parameters, mjcf_path

_ARM_JOINTS = ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6")
_MODEL = "omy"
_CTRL_PERIOD_S = 0.02
_SCENARIO = Path("src/fret/config/scenarios/omy_clutter.yml")


@dataclass(frozen=True)
class OmyClutterResult:
    """Outcome of one OMY cluttered pick-place cycle."""

    state: PickPlaceState
    box_pos: npt.NDArray[np.float64]
    transfer_path: list[npt.NDArray[np.float64]]
    straight_line_collides: bool
    samples: list[PickPlaceSample]


def _plan_transfer_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    scenario_path: Path,
    seed_offset: int,
) -> tuple[list[npt.NDArray[np.float64]], bool]:
    from fret.control.pick_place_planning import plan_arm_transfer_path

    return plan_arm_transfer_path(
        start,
        goal,
        scenario_path=scenario_path,
        seed_offset=seed_offset,
    )


def simulate_omy_clutter_pick_place(
    *,
    duration_s: float = 50.0,
    joint_tol_rad: float = 0.12,
    scenario_path: str | Path | None = None,
    seed_offset: int = 0,
) -> OmyClutterResult:
    """Run one OMY cluttered pick-place cycle."""
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for OMY clutter sim") from exc

    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    scenario_id = str(params.get("scenario_id", "omy_clutter"))
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

    transfer_path, straight_collides = _plan_transfer_path(
        wp.lift_hover if wp.lift_hover is not None else wp.pick_hover,
        wp.place_hover,
        scenario_path=scenario,
        seed_offset=seed_offset,
    )

    lift_z = float(params.get("lift_height_m", 0.08))
    fsm = PickPlaceFSM(
        wp,
        dof=6,
        gripper_open=OMY_GRIPPER_OPEN,
        gripper_closed=OMY_GRIPPER_CLOSED,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=3.5,
        release_hold_s=0.8,
        lift_height_m=lift_z,
        phase_timeout_s=float(params.get("phase_timeout_s", 40.0)),
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

    q_cmd = _arm_q(mj, model, data).copy()
    grip_cmd = OMY_GRIPPER_CLOSED
    ctrl_accum = 0.0
    transfer_armed = False
    transfer_idx = 0
    transfer_progress = 0.0
    transfer_segment_s = 0.8
    descend_force_s = 2.5
    retreat_force_s = 2.0
    descend_t = 0.0
    retreat_t = 0.0
    prev_fsm_state = PickPlaceState.LIFT
    carry_offset: npt.NDArray[np.float64] | None = None

    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []

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
            transfer_idx = 0
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

        prev_fsm_state = fsm.state

        ctrl_accum += dt
        if ctrl_accum >= _CTRL_PERIOD_S:
            ctrl_accum -= _CTRL_PERIOD_S
            if cmd.state == PickPlaceState.MOVE_PLACE and transfer_armed:
                target = transfer_path[transfer_idx]
                q_cmd = np.asarray(target, dtype=np.float64).copy()
            elif cmd.state in _SNAP_STATES:
                q_cmd = np.asarray(cmd.q_des, dtype=np.float64).copy()
            else:
                q_cmd = _track_toward(
                    q_cmd, cmd.q_des, _CTRL_PERIOD_S, tol=joint_tol_rad
                )
            if cmd.state == PickPlaceState.GRASP:
                grip_cmd = min(float(cmd.gripper), grip_cmd + 0.012)
            else:
                grip_cmd = float(cmd.gripper)

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
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

    box_pos = samples[-1].box_qpos[:3].copy() if samples else np.zeros(3)
    return OmyClutterResult(
        state=fsm.state,
        box_pos=box_pos,
        transfer_path=transfer_path,
        straight_line_collides=straight_collides,
        samples=samples,
    )


def run_omy_clutter_pick_place(
    *,
    duration_s: float = 50.0,
    joint_tol_rad: float = 0.14,
    scenario_path: str | Path | None = None,
    seed_offset: int = 0,
) -> OmyClutterResult:
    """Execute one OMY cluttered pick-place cycle."""
    return simulate_omy_clutter_pick_place(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        scenario_path=scenario_path,
        seed_offset=seed_offset,
    )
