"""MuJoCo runner for SO-101 SC-v20b/c pick-and-place (friction grasp mode).

Finger pads close on the free ball under physics. Hold through lift uses a
**contact-gated equality weld** (no MuJoCo adhesion actuator) — see
``docs/decisions.md``. Drop mid-transfer latches FSM ``FAULT`` (FR-SIM-18).

Arm motion toward each FSM joint target is tracked with ARCO
:class:`~arco.control.mpc.JointSpaceMPC` (carrot-on-path MPC). The
place transfer is a path from the lift pose to a vertical goal segment
above the basket centre, not a single joint target on the bin wall.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.joint_mpc import JointPathMPCTracker, build_joint_mpc
from fret.control.kinematics_so101 import HeldPointKinematics, link_positions
from fret.control.pick_place_common import PickPlaceSample, ball_grasp_contact
from fret.control.pick_place_fsm import (
    SO101_GRIPPER,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
    PickPlaceWaypoints,
)
from fret.control.pick_place_planning import (
    place_shell_from_params,
    wall_boxes_from_params,
)
from fret.planning.box_obstacle import BoxObstacle, points_clear_of_boxes
from fret.planning.goal_segment import GoalSegment, sample_polyline
from fret.planning.segment_planner import plan_to_segment
from fret.sitl_config import load_scenario_parameters, mjcf_path
from fret.telemetry.scenario_hooks import (
    arm_sample_values,
    close_telemetry,
    open_scenario_telemetry,
    setup_arm_telemetry,
)

_ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
_MODEL = "so101"
_CTRL_PERIOD_S = 0.02
_SCENARIO = Path("src/fret/config/scenarios/so101_pick_place.yml")
_PLACE_TRANSFER_TOL_RAD = 0.03
_PLACE_SETTLE_SPEED_RAD_S = 0.08


def release_segment_from_params(params: dict[str, Any]) -> GoalSegment:
    """Vertical drop segment above the basket centre.

    The segment starts ``place_release_clearance_m`` above the rim and runs
    upward for ``place_release_segment_m``. Every point of it is a valid
    release pose: the planner picks the lowest one the arm can reach.
    """
    centre = np.asarray(params["place_xy"], dtype=np.float64).reshape(-1)[:2]
    rim = float(params["place_cone_height_m"])
    clearance = float(params["place_release_clearance_m"])
    length = float(params["place_release_segment_m"])
    return GoalSegment.vertical(
        centre, rim + clearance, rim + clearance + length
    )


def place_obstacles_from_params(params: dict[str, Any]) -> list[BoxObstacle]:
    """Bin walls plus the hollow interior, up to the rim.

    The interior box stops the hand from dipping into the basket on the way
    to a release pose that sits above the rim. The release segment itself
    starts above this volume.
    """
    boxes = place_shell_from_params(params)
    centre = np.asarray(params["place_xy"], dtype=np.float64).reshape(-1)
    radius = float(params["place_cone_radius_m"])
    rim = float(params["place_cone_height_m"])
    boxes.append(
        BoxObstacle(
            x_min=float(centre[0]) - radius,
            x_max=float(centre[0]) + radius,
            y_min=float(centre[1]) - radius,
            y_max=float(centre[1]) + radius,
            z_min=0.0,
            z_max=rim,
        )
    )
    # Dual Γ (or any other transfer walls) share this clearance test.
    boxes.extend(wall_boxes_from_params(params))
    return boxes


def _arm_clears_place(
    configuration: npt.NDArray[np.float64],
    boxes: list[BoxObstacle],
    *,
    margin: float,
) -> bool:
    samples = sample_polyline(link_positions(configuration), per_edge=4)
    return points_clear_of_boxes(samples, boxes, margin=margin)


def waypoints_from_scenario(
    scenario_path: str | Path | None = None,
) -> PickPlaceWaypoints:
    """Load joint waypoints from an SO-101 pick-place scenario YAML."""
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


def _measured_grasp_offset(
    data: Any, gripper_body: int, ball_body: int
) -> npt.NDArray[np.float64]:
    """Ball offset in the gripper body frame, from the live MuJoCo pose."""
    rotation = np.asarray(data.xmat[gripper_body], dtype=np.float64).reshape(
        3, 3
    )
    gripper = np.asarray(data.xpos[gripper_body], dtype=np.float64)
    ball = np.asarray(data.xpos[ball_body], dtype=np.float64)
    offset: npt.NDArray[np.float64] = np.asarray(
        rotation.T @ (ball - gripper), dtype=np.float64
    )
    return offset


def _arm_speed(mj: Any, model: Any, data: Any) -> float:
    """L2 joint speed of the arm [rad/s]."""
    speed = np.array(
        [
            data.qvel[
                model.jnt_dofadr[
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
                ]
            ]
            for name in _ARM_JOINTS
        ],
        dtype=np.float64,
    )
    return float(np.linalg.norm(speed))


def _actuator_id(mj: Any, model: Any, name: str) -> int:
    aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"Missing MuJoCo actuator {name!r}")
    return int(aid)


def _capture_weld_pose(model: Any, data: Any, weld_id: int) -> None:
    """Write the current body1←body2 relative pose into ``eq_data``.

    Activating a weld without this yankes the free ball to the XML default
    (identity) offset. Capture once on the rising edge of engagement.
    """
    from scipy.spatial.transform import Rotation

    body1 = int(model.eq_obj1id[weld_id])
    body2 = int(model.eq_obj2id[weld_id])
    rot1 = np.asarray(data.xmat[body1], dtype=np.float64).reshape(3, 3)
    rot2 = np.asarray(data.xmat[body2], dtype=np.float64).reshape(3, 3)
    pos1 = np.asarray(data.xpos[body1], dtype=np.float64)
    pos2 = np.asarray(data.xpos[body2], dtype=np.float64)
    rot_rel = rot1.T @ rot2
    pos_rel = rot1.T @ (pos2 - pos1)
    quat_xyzw = Rotation.from_matrix(rot_rel).as_quat()
    model.eq_data[weld_id][3:6] = pos_rel
    model.eq_data[weld_id][6:10] = np.array(
        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        dtype=np.float64,
    )
    model.eq_data[weld_id][10] = 1.0


def simulate_so101_pick_place(
    *,
    duration_s: float = 35.0,
    joint_tol_rad: float = 0.14,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
    use_vision: bool = False,
    vision_config: str | Path | None = None,
    remove_ball_after_lift: bool = False,
) -> tuple[PickPlaceState, list[PickPlaceSample]]:
    """Run one SO-101 pick-place cycle (FSM → MPC → joints + friction grasp).

    Args:
        remove_ball_after_lift: Test hook — teleport the ball away once the
            FSM enters ``MOVE_PLACE`` so slip / drop detection must FAULT
            (FR-SIM-18 edge case).
    """
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "mujoco is required for SO-101 pick-place sim"
        ) from exc

    from fret.control.pick_place_vision import apply_vision_pick_goals

    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    if "use_vision" in params:
        use_vision = bool(params["use_vision"])
    if vision_config is None and params.get("vision_config"):
        vision_config = str(params["vision_config"])
    scenario_id = str(params.get("scenario_id", "so101_pick_place"))
    lift_height_m = float(params.get("lift_height_m", 0.08))
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path(_MODEL, scenario_id)
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)

    # Honesty: friction mode must not ship MuJoCo adhesion actuators.
    for act_i in range(int(model.nu)):
        if int(model.actuator_trntype[act_i]) == int(mj.mjtTrn.mjTRN_BODY):
            # Adhesion uses body transmission; position/motor use joint.
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, act_i) or ""
            raise RuntimeError(
                f"FR-SIM-18: unexpected adhesion-like actuator {name!r}"
            )
    weld_id = mj.mj_name2id(
        model, mj.mjtObj.mjOBJ_EQUALITY, "friction_grasp_weld"
    )
    if weld_id < 0:
        raise RuntimeError("SO-101 friction_grasp_weld equality missing")

    tele = open_scenario_telemetry(
        scenario_id,
        enabled=telemetry_enabled,
        output_dir=telemetry_output_dir,
        csv_basename=telemetry_csv_basename or f"{scenario_id}_overview",
        dt_nominal_s=float(model.opt.timestep),
    )
    joint_components: list[str] = []
    if tele is not None:
        joint_components = setup_arm_telemetry(tele, "so101", _ARM_JOINTS)

    ee_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "gripperframe")
    if ee_id < 0:
        ee_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "gripper")
    box_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])
    pad_right = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right")
    pad_left = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left")
    if pad_right < 0 or pad_left < 0:
        raise RuntimeError("SO-101 friction pads missing from MJCF")

    act_arm = [_actuator_id(mj, model, n) for n in _ARM_JOINTS]
    act_grip = _actuator_id(mj, model, "gripper")
    grip_joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "gripper")
    grip_qadr = int(model.jnt_qposadr[grip_joint])
    grip_span = abs(SO101_GRIPPER.open - SO101_GRIPPER.closed)
    ball_geom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pick_box_geom")
    ball_contype = int(model.geom_contype[ball_geom])
    ball_conaffinity = int(model.geom_conaffinity[ball_geom])

    ball_detected = False
    if use_vision:
        mj.mj_forward(model, data)
        wp, _ball_obs = apply_vision_pick_goals(
            wp,
            robot="so101",
            model=model,
            data=data,
            vision_config=vision_config,
        )
        ball_detected = True

    place_boxes = place_obstacles_from_params(params)
    place_margin = float(params.get("place_arm_margin_m", 0.025))
    place_segment = release_segment_from_params(params)
    lift_q = np.asarray(wp.lift_hover, dtype=np.float64)
    gripper_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "gripper")

    def _place_free(configuration: npt.NDArray[np.float64]) -> bool:
        return _arm_clears_place(
            configuration, place_boxes, margin=place_margin
        )

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = SO101_GRIPPER.open
    for _ in range(400):
        mj.mj_step(model, data)

    fsm = PickPlaceFSM(
        wp,
        gripper=SO101_GRIPPER,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=1.6,
        release_hold_s=0.8,
        lift_height_m=lift_height_m,
        phase_timeout_s=float(params.get("phase_timeout_s", 35.0)),
        transfer_joint_tol_rad=_PLACE_TRANSFER_TOL_RAD,
        transfer_settle_speed_rad_s=_PLACE_SETTLE_SPEED_RAD_S,
        drop_fault_enabled=True,
        require_grasp_contact=True,
        auto_start_on_ball=use_vision,
    )
    if not use_vision:
        fsm.start()

    mpc = build_joint_mpc(5)
    mpc.reset(_arm_q(mj, model, data))
    q_cmd = _arm_q(mj, model, data).copy()
    ctrl_accum = 0.0
    last_target = wp.idle.copy()
    removed = False
    weld_engaged = False
    transfer_armed = False
    transfer_tracker: JointPathMPCTracker | None = None
    retreat_tracker: JointPathMPCTracker | None = None
    place_path: list[npt.NDArray[np.float64]] | None = None
    home_along_transfer = bool(params.get("walls"))
    release_ball_z: float | None = None

    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))

    def _ee_pos() -> npt.NDArray[np.float64]:
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "gripperframe") >= 0:
            return np.asarray(
                data.site_xpos[
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "gripperframe")
                ],
                dtype=np.float64,
            ).copy()
        return np.asarray(data.xpos[ee_id], dtype=np.float64).copy()

    for step_i in range(max_steps):
        q = _arm_q(mj, model, data)
        grasp_ok = ball_grasp_contact(
            mj,
            model,
            data,
            box_body_id=box_id,
            pad_right_id=pad_right,
            pad_left_id=pad_left,
            max_pad_mid_dist_m=0.040,
        )
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.asarray(data.xpos[box_id], dtype=np.float64).copy(),
            ee_pos=_ee_pos(),
            ball_detected=ball_detected
            and fsm.state in {PickPlaceState.IDLE, PickPlaceState.DONE},
            grasp_contact=grasp_ok,
            q_speed_rad_s=_arm_speed(mj, model, data),
        )
        cmd = fsm.tick(obs, dt)

        # Contact-gated hold: engage weld only while pads contact and the
        # jaw is near closed (and still in a carry phase).
        jaw_closed = abs(
            float(cmd.gripper) - SO101_GRIPPER.closed
        ) <= 0.08 * abs(SO101_GRIPPER.open - SO101_GRIPPER.closed)
        carry = cmd.state in {
            PickPlaceState.GRASP,
            PickPlaceState.LIFT,
            PickPlaceState.MOVE_PLACE,
            PickPlaceState.DESCEND_PLACE,
        }
        # Engage on rising contact+close; once latched, hold through carry.
        # On RELEASE the weld stays up and ball contacts are masked until the
        # jaw has actually opened, so the finger leaves before the ball falls.
        jaw_q = float(data.qpos[grip_qadr])
        jaw_clear = abs(jaw_q - SO101_GRIPPER.open) <= 0.15 * grip_span
        if cmd.state == PickPlaceState.RELEASE and release_ball_z is None:
            release_ball_z = float(data.xpos[box_id][2])
        # Hold the weld while the jaw opens, with ball contacts masked so
        # the finger cannot flick it. After the jaw is clear, keep contacts
        # masked for a short free-fall so the ball separates before they
        # come back (needed to land in the basket).
        opening = cmd.state == PickPlaceState.RELEASE and not jaw_clear
        separating = (
            cmd.state == PickPlaceState.RELEASE
            and jaw_clear
            and release_ball_z is not None
            and float(data.xpos[box_id][2]) > release_ball_z - 0.035
        )
        if not weld_engaged and grasp_ok and jaw_closed and carry:
            _capture_weld_pose(model, data, weld_id)
            weld_engaged = True
            # The weld just froze the ball in the gripper. Plan the release
            # to the vertical segment above the basket centre for that
            # grasped point, not for the fingertip site.
            if transfer_tracker is None:
                place_path = plan_to_segment(
                    lift_q,
                    place_segment,
                    kinematics=HeldPointKinematics(
                        _measured_grasp_offset(data, gripper_body, box_id)
                    ),
                    is_free=_place_free,
                    seed=int(params.get("planner_rng_seed", 7)),
                    timeout_s=min(
                        8.0, float(params.get("planning_timeout", 8.0))
                    ),
                    goal_tolerance_m=float(
                        params.get("place_segment_tolerance_m", 0.005)
                    ),
                )
                fsm.replace_place(place_path[-1])
                transfer_tracker = JointPathMPCTracker(
                    place_path,
                    build_joint_mpc(5),
                    race_speed=0.6,
                    max_carrot_lag=0.35,
                    goal_tol=_PLACE_TRANSFER_TOL_RAD,
                )
        if weld_engaged and not opening and (not jaw_closed or not carry):
            weld_engaged = False
        data.eq_active[weld_id] = weld_engaged
        if opening or separating:
            model.geom_contype[ball_geom] = 0
            model.geom_conaffinity[ball_geom] = 0
        else:
            model.geom_contype[ball_geom] = ball_contype
            model.geom_conaffinity[ball_geom] = ball_conaffinity

        if (
            remove_ball_after_lift
            and not removed
            and fsm.state == PickPlaceState.MOVE_PLACE
        ):
            # Slip / remove mid-transfer: ball leaves the pads.
            data.eq_active[weld_id] = False
            weld_engaged = False
            data.qpos[box_qadr : box_qadr + 3] = np.array(
                [0.5, 0.5, 0.02], dtype=np.float64
            )
            data.qvel[box_qadr : box_qadr + 6] = 0.0
            mj.mj_forward(model, data)
            removed = True

        if cmd.state == PickPlaceState.MOVE_PLACE:
            if transfer_tracker is None:
                raise RuntimeError(
                    "SO-101 place segment was not planned before transfer"
                )
            if not transfer_armed:
                transfer_tracker.reset(q)
                transfer_armed = True
            ctrl_accum += dt
            if ctrl_accum >= _CTRL_PERIOD_S:
                ctrl_accum -= _CTRL_PERIOD_S
                q_cmd = np.asarray(
                    transfer_tracker.step(_CTRL_PERIOD_S), dtype=np.float64
                )
        elif (
            cmd.state == PickPlaceState.RETREAT
            and home_along_transfer
            and place_path is not None
            and len(place_path) >= 2
            and float(np.linalg.norm(cmd.q_des - np.asarray(wp.retreat)))
            < 1e-8
        ):
            # Home along the free transfer, reversed. A straight joint slew
            # from the release pose back to the pick cuts through the Γ.
            if retreat_tracker is None:
                retreat_tracker = JointPathMPCTracker(
                    list(reversed(place_path)),
                    build_joint_mpc(5),
                    race_speed=0.6,
                    max_carrot_lag=0.35,
                    goal_tol=_PLACE_TRANSFER_TOL_RAD,
                )
                retreat_tracker.reset(q)
            ctrl_accum += dt
            if ctrl_accum >= _CTRL_PERIOD_S:
                ctrl_accum -= _CTRL_PERIOD_S
                q_cmd = np.asarray(
                    retreat_tracker.step(_CTRL_PERIOD_S), dtype=np.float64
                )
        else:
            transfer_armed = False
            if float(np.linalg.norm(cmd.q_des - last_target)) > 1e-9:
                mpc.reset(q)
                last_target = cmd.q_des.copy()

            ctrl_accum += dt
            if ctrl_accum >= _CTRL_PERIOD_S:
                ctrl_accum -= _CTRL_PERIOD_S
                q_cmd = np.asarray(
                    mpc.step(cmd.q_des, _CTRL_PERIOD_S), dtype=np.float64
                )
                if float(np.linalg.norm(q_cmd - cmd.q_des)) <= joint_tol_rad:
                    q_cmd = cmd.q_des.copy()
                    mpc.reset(q_cmd.copy())

        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(q_cmd[i])
        data.ctrl[act_grip] = float(cmd.gripper)
        mj.mj_step(model, data)

        if tele is not None:
            q_now = _arm_q(mj, model, data)
            tele.record(
                step_i * dt,
                arm_sample_values(
                    "so101",
                    joint_components,
                    q_now,
                    _ee_pos(),
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

    if fsm.state == PickPlaceState.DONE:
        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(wp.idle[i])
        data.ctrl[act_grip] = SO101_GRIPPER.open
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


def run_so101_pick_place(
    *,
    duration_s: float = 35.0,
    joint_tol_rad: float = 0.14,
    scenario_path: str | Path | None = None,
    use_vision: bool = False,
    remove_ball_after_lift: bool = False,
) -> tuple[PickPlaceState, npt.NDArray[np.float64]]:
    """Execute one SC-v20b/c cycle; return final FSM state and box position."""
    state, samples = simulate_so101_pick_place(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        record_every_steps=1,
        scenario_path=scenario_path,
        use_vision=use_vision,
        remove_ball_after_lift=remove_ball_after_lift,
    )
    if not samples:
        raise RuntimeError("SO-101 pick-place produced no samples")
    box_pos = samples[-1].box_qpos[:3].copy()
    return state, box_pos
