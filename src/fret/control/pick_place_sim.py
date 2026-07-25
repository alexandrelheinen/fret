"""MuJoCo runner for SC-v13b pick-and-place (FSM + free ball).

Full physics grasp: Menagerie finger pads (injected) close on the free ball,
then MuJoCo adhesion holds it through lift/transfer. Adhesion is delayed until
the jaw has settled so the sphere is not ejected on close. No kinematic attach.

Arm motion toward each FSM joint target is tracked with ARCO
:class:`~arco.control.mpc.JointSpaceMPC` (CasADi carrot NMPC).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.joint_mpc import build_omx_joint_mpc
from fret.control.pick_place_common import PickPlaceSample, adhesion_command
from fret.control.pick_place_fsm import (
    GRIPPER_OPEN,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
    PickPlaceWaypoints,
)
from fret.sitl_config import load_scenario_parameters, mjcf_path
from fret.telemetry.scenario_hooks import (
    arm_sample_values,
    close_telemetry,
    open_scenario_telemetry,
    setup_arm_telemetry,
)

_CTRL_PERIOD_S = 0.02


def waypoints_from_scenario(
    scenario_path: str | Path | None = None,
) -> PickPlaceWaypoints:
    """Load joint waypoints from a pick-place scenario YAML.

    Supports optional ``lift_hover_configuration`` (OMY / shared FSM) and
    ``retreat_configuration``.
    """
    path = Path(
        scenario_path or "src/fret/config/scenarios/omx_pick_place.yml"
    )
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
            for name in ("Joint1", "Joint2", "Joint3", "Joint4")
        ],
        dtype=np.float64,
    )


def _actuator_id(mj: Any, model: Any, name: str) -> int:
    aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"Missing MuJoCo actuator {name!r}")
    return int(aid)


def simulate_pick_place(
    *,
    duration_s: float = 25.0,
    joint_tol_rad: float = 0.12,
    record_every_steps: int = 1,
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
    scenario_path: str | Path | None = None,
    use_vision: bool = True,
    vision_config: str | Path | None = None,
) -> tuple[PickPlaceState, list[PickPlaceSample]]:
    """Run one SC-v13b / SC-v16a cycle and optionally record trajectory samples.

    When ``use_vision`` is True (default), pick-side joints come from gate-camera
    ``BallObservation`` + IK. Place / dispenser joints stay on scenario YAML.
    ``pick_xy`` in YAML is an oracle for tests only — not a grasp command source.
    """
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for pick-place sim") from exc

    from fret.control.pick_place_vision import apply_vision_pick_goals

    scenario = Path(
        scenario_path or "src/fret/config/scenarios/omx_pick_place.yml"
    )
    params = load_scenario_parameters(scenario)
    if "use_vision" in params:
        use_vision = bool(params["use_vision"])
    if vision_config is None and params.get("vision_config"):
        vision_config = str(params["vision_config"])
    scenario_id = str(params.get("scenario_id", "omx_pick_place"))
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path("open_manipulator_x", scenario_id)
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    joint_names = ("Joint1", "Joint2", "Joint3", "Joint4")
    tele = open_scenario_telemetry(
        scenario_id,
        enabled=telemetry_enabled,
        output_dir=telemetry_output_dir,
        csv_basename=telemetry_csv_basename or f"{scenario_id}_overview",
        dt_nominal_s=float(model.opt.timestep),
    )
    joint_components: list[str] = []
    if tele is not None:
        joint_components = setup_arm_telemetry(tele, "omx", joint_names)

    ee_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link5")
    box_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])

    act_arm = [
        _actuator_id(mj, model, n)
        for n in ("Joint1", "Joint2", "Joint3", "Joint4")
    ]
    act_grip = _actuator_id(mj, model, "Gripper")
    act_al = _actuator_id(mj, model, "grip_left")
    act_ar = _actuator_id(mj, model, "grip_right")

    # Sense the ball before folding to idle — idle posture can occlude a gate cam.
    ball_detected = False
    if use_vision:
        mj.mj_forward(model, data)
        wp, _ball_obs = apply_vision_pick_goals(
            wp,
            robot="omx",
            model=model,
            data=data,
            vision_config=vision_config,
        )
        ball_detected = True

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(400):
        mj.mj_step(model, data)

    fsm = PickPlaceFSM(
        wp,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=1.4,
        release_hold_s=0.8,
        lift_height_m=0.06,
        phase_timeout_s=20.0,
        auto_start_on_ball=use_vision,
    )
    if not use_vision:
        fsm.start()

    mpc = build_omx_joint_mpc()
    mpc.reset(_arm_q(mj, model, data))
    q_cmd = _arm_q(mj, model, data).copy()
    ctrl_accum = 0.0
    last_target = wp.idle.copy()

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
            ball_detected=ball_detected
            and fsm.state in {PickPlaceState.IDLE, PickPlaceState.DONE},
        )
        cmd = fsm.tick(obs, dt)

        # New FSM target: re-seed the MPC from the measured pose so each
        # phase starts from truth, then integrate open-loop (ARCO style).
        if float(np.linalg.norm(cmd.q_des - last_target)) > 1e-9:
            mpc.reset(q)
            last_target = cmd.q_des.copy()

        ctrl_accum += dt
        if ctrl_accum >= _CTRL_PERIOD_S:
            ctrl_accum -= _CTRL_PERIOD_S
            q_cmd = np.asarray(
                mpc.step(cmd.q_des, _CTRL_PERIOD_S), dtype=np.float64
            )
            # Snap when inside tolerance so the FSM can advance under
            # stiff position actuators (matches prior setpoint behavior).
            if float(np.linalg.norm(q_cmd - cmd.q_des)) <= joint_tol_rad:
                q_cmd = cmd.q_des.copy()
                mpc.q = q_cmd.copy()
                mpc.vel = np.zeros_like(q_cmd)

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
        data.ctrl[act_grip] = float(cmd.gripper)
        adhere = adhesion_command(cmd.state, fsm.hold_t)
        data.ctrl[act_al] = adhere
        data.ctrl[act_ar] = adhere
        mj.mj_step(model, data)

        if tele is not None:
            q_now = _arm_q(mj, model, data)
            tele.record(
                step_i * dt,
                arm_sample_values(
                    "omx",
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

    # Hold the terminal pose so showcase clips end on idle (DONE), not on a
    # sparsely sampled RETREAT frame between record ticks.
    if fsm.state == PickPlaceState.DONE:
        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(wp.idle[i])
        data.ctrl[act_grip] = GRIPPER_OPEN
        data.ctrl[act_al] = 0.0
        data.ctrl[act_ar] = 0.0
        hold_steps = max(record_every_steps, int(round(0.6 / dt)))
        for step_i in range(hold_steps):
            mj.mj_step(model, data)
            if tele is not None:
                q_now = _arm_q(mj, model, data)
                tele.record(
                    (max_steps + step_i) * dt,
                    arm_sample_values(
                        "omx",
                        joint_components,
                        q_now,
                        np.asarray(data.xpos[ee_id], dtype=np.float64),
                    ),
                    tick=max_steps + step_i,
                )
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


def run_pick_place(
    *,
    duration_s: float = 25.0,
    joint_tol_rad: float = 0.12,
    scenario_path: str | Path | None = None,
    use_vision: bool = True,
) -> tuple[PickPlaceState, npt.NDArray[np.float64]]:
    """Execute one SC-v13b / SC-v16a cycle; return final FSM state and box."""
    state, samples = simulate_pick_place(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        record_every_steps=1,
        scenario_path=scenario_path,
        use_vision=use_vision,
    )
    if not samples:
        raise RuntimeError("pick-place produced no samples")
    box_pos = samples[-1].box_qpos[:3].copy()
    return state, box_pos
