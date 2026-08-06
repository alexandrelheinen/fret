"""OMY SC-v14b pick-and-place — same modular stack as OMX SC-v13b.

Stack (shared with OpenMANIPULATOR-X):

1. ``PickPlaceFSM`` — task phases (approach → grasp → lift → place → retreat)
2. Joint-space path / phase targets from scenario YAML (pad-mid IK)
3. Planned ``APPROACH_PICK`` / ``MOVE_PLACE`` around the place-bin shell
   (``contype=5``), tracked with ARCO ``JointSpaceMPC`` carrot NMPC
4. MuJoCo position actuators — low-level joint control

Physics grasp only: Menagerie finger pads close on the free ball, then MuJoCo
adhesion (modest assist) holds it through lift/transfer. Distal collision bits
are remapped for floor pinch (see ``fret.mjcf.omy``). No kinematic arm snaps,
MPC-bypass setpoints, or ball teleports.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.joint_mpc import JointPathMPCTracker, build_joint_mpc
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
from fret.control.pick_place_planning import plan_arm_transfer_path
from fret.sitl_config import load_scenario_parameters, mjcf_path
from fret.telemetry.scenario_hooks import (
    arm_sample_values,
    close_telemetry,
    open_scenario_telemetry,
    setup_arm_telemetry,
)

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


def simulate_omy_pick_place(
    *,
    duration_s: float = 45.0,
    joint_tol_rad: float = 0.16,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
    use_vision: bool = True,
    vision_config: str | Path | None = None,
) -> tuple[PickPlaceState, list[PickPlaceSample]]:
    """Run one OMY pick-place cycle (FSM → MPC → joints + physics grasp).

    When ``use_vision`` is True (default), pick-side joints come from gate-camera
    ``BallObservation`` + pad-mid IK. Place / dispenser stays on scenario YAML.
    """
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for OMY pick-place sim") from exc

    from fret.control.pick_place_vision import apply_vision_pick_goals

    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    if "use_vision" in params:
        use_vision = bool(params["use_vision"])
    if vision_config is None and params.get("vision_config"):
        vision_config = str(params["vision_config"])
    scenario_id = str(params.get("scenario_id", "omy_pick_place"))
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path(_MODEL, scenario_id)
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    tele = open_scenario_telemetry(
        scenario_id,
        enabled=telemetry_enabled,
        output_dir=telemetry_output_dir,
        csv_basename=telemetry_csv_basename or f"{scenario_id}_overview",
        dt_nominal_s=float(model.opt.timestep),
    )
    joint_components: list[str] = []
    if tele is not None:
        joint_components = setup_arm_telemetry(tele, "omy", _ARM_JOINTS)

    ee_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link6")
    box_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])
    pad_right = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right")
    pad_left = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left")

    act_arm = [_actuator_id(mj, model, n) for n in _ARM_JOINTS]
    act_grip = _actuator_id(mj, model, "Gripper")
    act_al = _actuator_id(mj, model, "grip_left")
    act_ar = _actuator_id(mj, model, "grip_right")

    # Sense before idle fold — folded OMY occludes a gate camera (~18 mm XY).
    ball_detected = False
    if use_vision:
        mj.mj_forward(model, data)
        wp, _ball_obs = apply_vision_pick_goals(
            wp,
            robot="omy",
            model=model,
            data=data,
            vision_config=vision_config,
        )
        ball_detected = True

    # OMX pattern: settle under ctrl only — no qpos teleports.
    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = OMY_GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(400):
        mj.mj_step(model, data)

    lift_z = float(params.get("lift_height_m", 0.19))
    phase_timeout = float(params.get("phase_timeout_s", 45.0))
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
        drop_fault_enabled=True,
        require_grasp_contact=True,
        transfer_joint_tol_rad=max(float(joint_tol_rad), 0.20),
        auto_start_on_ball=use_vision,
    )
    if not use_vision:
        fsm.start()

    # Place-bin shell is contype=5 (proximal + distal). A joint chord through
    # idle→pick_hover and lift→place both clip place_bin_wall_nx — plan both.
    q_settled = _arm_q(mj, model, data)
    approach_path, _approach_collides = plan_arm_transfer_path(
        q_settled,
        np.asarray(wp.pick_hover, dtype=np.float64),
        scenario_path=scenario,
        seed_offset=1,
        prefer_detour=True,
    )
    approach_tracker = JointPathMPCTracker(
        approach_path,
        build_joint_mpc(6),
        race_speed=1.2,
        max_carrot_lag=0.40,
        goal_tol=max(0.12, float(joint_tol_rad)),
    )
    lift = wp.lift_hover if wp.lift_hover is not None else wp.pick_hover
    transfer_path, _straight_collides = plan_arm_transfer_path(
        np.asarray(lift, dtype=np.float64),
        np.asarray(wp.place_hover, dtype=np.float64),
        scenario_path=scenario,
    )
    transfer_tracker = JointPathMPCTracker(
        transfer_path,
        build_joint_mpc(6),
        race_speed=1.2,
        max_carrot_lag=0.40,
        goal_tol=max(0.12, float(joint_tol_rad)),
    )

    phase_mpc = build_joint_mpc(6)
    phase_mpc.reset(q_settled)
    q_cmd = q_settled.copy()
    last_target = wp.idle.copy()
    ctrl_accum = 0.0
    approach_armed = False
    transfer_armed = False

    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))

    for step_i in range(max_steps):
        q = _arm_q(mj, model, data)
        grasp_ok = ball_grasp_contact(
            mj,
            model,
            data,
            box_body_id=box_id,
            pad_right_id=pad_right,
            pad_left_id=pad_left,
            allow_pad_mid_fallback=False,
        )
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.asarray(data.xpos[box_id], dtype=np.float64).copy(),
            ee_pos=np.asarray(data.xpos[ee_id], dtype=np.float64).copy(),
            grasp_contact=grasp_ok,
            ball_detected=ball_detected
            and fsm.state in {PickPlaceState.IDLE, PickPlaceState.DONE},
        )
        cmd = fsm.tick(obs, dt)

        if cmd.state == PickPlaceState.APPROACH_PICK:
            if not approach_armed:
                approach_tracker.reset(q)
                approach_armed = True
            transfer_armed = False
            ctrl_accum += dt
            if ctrl_accum >= _CTRL_PERIOD_S:
                ctrl_accum -= _CTRL_PERIOD_S
                if approach_tracker.complete:
                    q_cmd = wp.pick_hover.copy()
                else:
                    q_cmd = approach_tracker.step(_CTRL_PERIOD_S)
        elif cmd.state == PickPlaceState.MOVE_PLACE:
            approach_armed = False
            if not transfer_armed:
                transfer_tracker.reset(q)
                transfer_armed = True
            ctrl_accum += dt
            if ctrl_accum >= _CTRL_PERIOD_S:
                ctrl_accum -= _CTRL_PERIOD_S
                if transfer_tracker.complete:
                    q_cmd = wp.place_hover.copy()
                else:
                    q_cmd = transfer_tracker.step(_CTRL_PERIOD_S)
        else:
            approach_armed = False
            transfer_armed = False
            if float(np.linalg.norm(cmd.q_des - last_target)) > 1e-9:
                phase_mpc.reset(q)
                last_target = cmd.q_des.copy()
            ctrl_accum += dt
            if ctrl_accum >= _CTRL_PERIOD_S:
                ctrl_accum -= _CTRL_PERIOD_S
                q_cmd = np.asarray(
                    phase_mpc.step(cmd.q_des, _CTRL_PERIOD_S),
                    dtype=np.float64,
                )
                if float(np.linalg.norm(q_cmd - cmd.q_des)) <= joint_tol_rad:
                    q_cmd = cmd.q_des.copy()
                    phase_mpc.q = q_cmd.copy()
                    phase_mpc.vel = np.zeros_like(q_cmd)

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
        data.ctrl[act_grip] = float(cmd.gripper)
        adhere = adhesion_command(
            cmd.state,
            fsm.hold_t,
            gripper=float(cmd.gripper),
            gripper_closed=OMY_GRIPPER_CLOSED,
            gripper_open=OMY_GRIPPER_OPEN,
        )
        data.ctrl[act_al] = adhere
        data.ctrl[act_ar] = adhere
        mj.mj_step(model, data)

        if tele is not None:
            q_now = _arm_q(mj, model, data)
            tele.record(
                step_i * dt,
                arm_sample_values(
                    "omy",
                    joint_components,
                    q_now,
                    np.asarray(data.xpos[ee_id], dtype=np.float64),
                ),
                tick=step_i,
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

    # Hold terminal idle (same as OMX) so clips end on the home pose.
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

    close_telemetry(tele)
    return fsm.state, samples


def run_omy_pick_place(
    *,
    duration_s: float = 45.0,
    joint_tol_rad: float = 0.12,
    scenario_path: str | Path | None = None,
    use_vision: bool = True,
) -> tuple[PickPlaceState, npt.NDArray[np.float64]]:
    """Execute one OMY pick-place cycle; return final state and ball position."""
    state, samples = simulate_omy_pick_place(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        scenario_path=scenario_path,
        use_vision=use_vision,
    )
    if not samples:
        raise RuntimeError("OMY pick-place produced no samples")
    box_pos = samples[-1].box_qpos[:3].copy()
    return state, box_pos
