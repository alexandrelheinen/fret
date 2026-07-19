"""SC-v13c/d pick-and-place with desk wall / Γ maze: planner + MPC + physics.

Grasp/release phases reuse the SC-v13b FSM and MuJoCo adhesion grasp. The
``MOVE_PLACE`` phase plans ``pick_hover → place_hover`` with ARCO RRT* against
an inflated wall occupancy cloud, densifies via ``TrajectoryGenerator``, then
tracks the path with ARCO :class:`~arco.control.mpc.JointSpaceMPC` (carrot
NMPC — replaces proportional ``ControllerNode`` joint-space tracking).

SC-v13c uses a single mid-cell slab; SC-v13d uses a Γ (inverted-L) stem+cap.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.config_loader import load_algorithm_config, planning_config_for_model
from fret.control.joint_mpc import JointPathMPCTracker, build_omx_joint_mpc
from fret.control.kinematics import Kinematics
from fret.control.pick_place_fsm import (
    GRIPPER_OPEN,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
)
from fret.control.pick_place_sim import (
    PickPlaceSample,
    adhesion_command,
    waypoints_from_scenario,
)
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.box_obstacle import BoxObstacle
from fret.planning.planner_node import PlannerNode
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter
from fret.sitl_config import load_scenario_parameters, mjcf_path

_SCENARIO = Path("src/fret/config/scenarios/omx_desk_clutter.yml")
_CTRL_PERIOD_S = 0.02


@dataclass(frozen=True)
class ClutterPickPlaceResult:
    """Outcome of one SC-v13c/d clutter / maze cycle."""

    state: PickPlaceState
    box_pos: npt.NDArray[np.float64]
    transfer_path: list[npt.NDArray[np.float64]]
    straight_line_collides: bool
    samples: list[PickPlaceSample]


def _scenario_id(params: dict[str, Any]) -> str:
    """Return the MJCF / planning scenario stem from YAML params."""
    return str(params.get("scenario_id", "omx_desk_clutter"))


def walls_from_scenario(
    scenario_path: str | Path | None = None,
    *,
    inflate: bool = False,
) -> list[BoxObstacle]:
    """Build wall boxes from scenario YAML (single slab or ``walls`` list)."""
    p = load_scenario_parameters(scenario_path or _SCENARIO)
    pad = float(p.get("wall_inflate_m", 0.0)) if inflate else 0.0
    raw = p.get("walls")
    if isinstance(raw, list) and raw:
        boxes: list[BoxObstacle] = []
        for entry in raw:
            if not isinstance(entry, dict):
                raise ValueError("walls entries must be mappings")
            if "y_min" in entry and "y_max" in entry:
                y_min = float(entry["y_min"]) - pad
                y_max = float(entry["y_max"]) + pad
            else:
                y_half = float(entry["y_half"])
                y_min = -y_half - pad
                y_max = y_half + pad
            boxes.append(
                BoxObstacle(
                    x_min=float(entry["x_min"]) - pad,
                    y_min=y_min,
                    z_min=float(entry.get("z_min", 0.0)),
                    x_max=float(entry["x_max"]) + pad,
                    y_max=y_max,
                    z_max=float(entry["z_max"]),
                )
            )
        return boxes
    return [
        BoxObstacle(
            x_min=float(p["wall_x_min"]) - pad,
            y_min=-float(p["wall_y_half"]) - pad,
            z_min=0.0,
            x_max=float(p["wall_x_max"]) + pad,
            y_max=float(p["wall_y_half"]) + pad,
            z_max=float(p["wall_z_max"]),
        )
    ]


def wall_from_scenario(
    scenario_path: str | Path | None = None,
    *,
    inflate: bool = False,
) -> BoxObstacle:
    """Build the primary wall box from scenario YAML (first slab)."""
    return walls_from_scenario(scenario_path, inflate=inflate)[0]


def _sample_walls(
    walls: list[BoxObstacle],
    density: float,
    rng: np.random.Generator,
) -> npt.NDArray[np.float64]:
    """Stack surface samples from every wall into one occupancy cloud."""
    clouds = [w.sample_surface(density, rng=rng) for w in walls]
    return np.vstack(clouds).astype(np.float64)


def _path_metrics(
    kin: Kinematics, path: list[npt.NDArray[np.float64]]
) -> tuple[float, float, float]:
    """Return ``(min_xy_radius, min_ee_z, max_ee_z)`` along ``path``."""
    ee = np.array(
        [kin.forward_kinematics(q)[:3, 3] for q in path], dtype=np.float64
    )
    radii = np.linalg.norm(ee[:, :2], axis=1)
    return (
        float(np.min(radii)),
        float(np.min(ee[:, 2])),
        float(np.max(ee[:, 2])),
    )


def _is_wall_geom(name: str | None) -> bool:
    """True for transfer-wall geoms (``transfer_wall`` or ``transfer_wall_*``)."""
    return name is not None and name.startswith("transfer_wall")


def _path_clear_of_wall_meshes(
    dense: list[npt.NDArray[np.float64]],
    *,
    scenario_id: str,
) -> bool:
    """Return True if teleported waypoints do not touch transfer-wall meshes.

    The C-space occupancy cloud is a coarse proxy; Menagerie arm meshes can
    still graze the Γ roof on an otherwise "free" path.
    """
    try:
        import mujoco as mj
    except ImportError:  # pragma: no cover
        return True

    model = mj.MjModel.from_xml_path(
        str(mjcf_path("open_manipulator_x", scenario_id))
    )
    data = mj.MjData(model)
    names = ("Joint1", "Joint2", "Joint3", "Joint4")
    # Hide the free ball so it cannot generate spurious contacts.
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    qid = int(model.jnt_qposadr[box_jid])
    data.qpos[qid : qid + 3] = [0.5, 0.5, 0.5]
    stride = max(1, len(dense) // 40)
    for q in dense[::stride]:
        for i, n in enumerate(names):
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)
            data.qpos[model.jnt_qposadr[jid]] = float(q[i])
        data.qvel[:] = 0.0
        mj.mj_forward(model, data)
        for ci in range(data.ncon):
            c = data.contact[ci]
            g1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom1)
            g2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom2)
            if _is_wall_geom(g1) or _is_wall_geom(g2):
                return False
    return True


def _dry_run_transfer(
    dense: list[npt.NDArray[np.float64]],
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    joint_tol_rad: float,
    scenario_id: str,
) -> bool:
    """Return True if joint-space MPC tracking reaches ``goal`` without wall jams."""
    try:
        import mujoco as mj
    except ImportError:  # pragma: no cover
        return True

    model = mj.MjModel.from_xml_path(
        str(mjcf_path("open_manipulator_x", scenario_id))
    )
    data = mj.MjData(model)
    names = ("Joint1", "Joint2", "Joint3", "Joint4")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    qid = int(model.jnt_qposadr[box_jid])
    data.qpos[qid : qid + 3] = [0.5, 0.5, 0.5]
    data.qvel[:] = 0.0

    def settle(cmd: npt.NDArray[np.float64], steps: int) -> None:
        for _ in range(steps):
            for i, n in enumerate(names):
                data.ctrl[model.actuator(n).id] = float(cmd[i])
            data.ctrl[model.actuator("Gripper").id] = -0.01
            data.ctrl[model.actuator("grip_left").id] = 0.0
            data.ctrl[model.actuator("grip_right").id] = 0.0
            mj.mj_step(model, data)

    idle = np.array([0.0, -1.05, 0.7, 0.7], dtype=np.float64)
    settle(idle, 300)
    settle(start, 700)

    tracker = JointPathMPCTracker(
        dense,
        build_omx_joint_mpc(),
        race_speed=1.2,
        max_carrot_lag=0.40,
        goal_tol=max(0.12, float(joint_tol_rad)),
    )
    tracker.reset(np.asarray(start, dtype=np.float64))
    dt = float(model.opt.timestep)
    period = _CTRL_PERIOD_S
    accum = 0.0
    hits = 0
    q_cmd = np.asarray(start, dtype=np.float64).copy()
    settle_after_complete = 0
    for _ in range(25000):
        accum += dt
        if accum >= period:
            accum -= period
            if tracker.complete:
                q_cmd = np.asarray(goal, dtype=np.float64).copy()
            else:
                q_cmd = tracker.step(period)
                if tracker.complete:
                    q_cmd = np.asarray(goal, dtype=np.float64).copy()
        for i, n in enumerate(names):
            data.ctrl[model.actuator(n).id] = float(q_cmd[i])
        data.ctrl[model.actuator("Gripper").id] = -0.01
        mj.mj_step(model, data)
        for ci in range(data.ncon):
            c = data.contact[ci]
            g1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom1)
            g2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom2)
            if _is_wall_geom(g1) or _is_wall_geom(g2):
                hits += 1
        if tracker.complete:
            settle_after_complete += 1
            q = np.array(
                [
                    data.qpos[
                        model.jnt_qposadr[
                            mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)
                        ]
                    ]
                    for n in names
                ],
                dtype=np.float64,
            )
            if (
                float(np.linalg.norm(q - goal)) < 0.15
                or settle_after_complete > 2000
            ):
                break

    q = np.array(
        [
            data.qpos[
                model.jnt_qposadr[
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)
                ]
            ]
            for n in names
        ],
        dtype=np.float64,
    )
    return float(np.linalg.norm(q - goal)) < 0.15 and hits < 200


def plan_transfer_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    scenario_path: str | Path | None = None,
    validate_mujoco: bool = True,
    seed_offset: int = 0,
) -> tuple[list[npt.NDArray[np.float64]], bool]:
    """Plan a dense retract path around the wall(s); report if the chord collides.

    Tries several RNG seeds, keeps only paths that retract and stay high
    enough to carry the box, and optionally dry-runs tracking in MuJoCo.
    """
    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    scenario_id = _scenario_id(params)
    physical = walls_from_scenario(scenario, inflate=False)
    inflated = walls_from_scenario(scenario, inflate=True)
    seed0 = int(params.get("planner_rng_seed", 7)) + int(seed_offset)
    density = float(params.get("occupancy_density", 1000.0))
    timeout = float(params.get("planning_timeout", 25.0))
    algo = str(params.get("planner_algorithm", "rrt_star"))
    max_r = float(params.get("max_transfer_radius_m", 0.12))
    min_z = float(params.get("min_transfer_ee_z_m", 0.145))
    peak_z = float(params.get("min_transfer_peak_ee_z_m", 0.0))

    kin = Kinematics("open_manipulator_x")
    cfg = load_algorithm_config(
        planning_config_for_model("open_manipulator_x")
    )
    from fret.planning.cspace_checker import make_cspace_checker

    # Straight-line check against the physical wall occupancy.
    rng_phys = np.random.default_rng(seed0)
    phys_pts = _sample_walls(physical, density, rng_phys)
    phys_adapter = OccupancyAdapter()
    phys_adapter.update(
        OccupancyUpdatePayload(
            obstacle_points=phys_pts, timestamp=0.0, frame_id="world"
        )
    )
    phys_checker = make_cspace_checker(kin, phys_adapter.get_occupancy())
    alphas = np.linspace(0.0, 1.0, 40)
    straight_collides = not all(
        phys_checker.is_collision_free((1.0 - a) * start + a * goal)
        for a in alphas
    )

    last_err: str | None = None
    # Γ maze needs more RNG draws: roof overhang + mesh-clear reject many.
    attempt_budget = 60 if peak_z > 0.0 else 24
    for attempt in range(attempt_budget):
        seed = seed0 + attempt
        rng = np.random.default_rng(seed)
        pts = _sample_walls(inflated, density, rng)
        adapter = OccupancyAdapter()
        adapter.update(
            OccupancyUpdatePayload(
                obstacle_points=pts, timestamp=0.0, frame_id="world"
            )
        )
        planner = PlannerNode(
            model="open_manipulator_x",
            occupancy_adapter=adapter,
            planner_algorithm=algo,  # type: ignore[arg-type]
            scenario=scenario_id,
        )
        result = planner.plan(
            PlanningRequest(
                start_configuration=np.asarray(start, dtype=np.float64),
                goal_configuration=np.asarray(goal, dtype=np.float64),
                planning_timeout=timeout,
                scenario_id=scenario_id,
            )
        )
        if result.status != PlanningStatus.SUCCESS or len(result.path) < 2:
            last_err = (
                f"status={result.status.name} code={result.error_code.name}"
            )
            continue

        traj = TrajectoryGenerator(kin, cfg).process(result.path)
        dense = [
            np.asarray(pt.positions, dtype=np.float64) for pt in traj.points
        ]
        min_r, path_min_z, path_peak_z = _path_metrics(kin, dense)
        if min_r > max_r or path_min_z < min_z:
            last_err = f"geometry min_r={min_r:.3f} min_z={path_min_z:.3f}"
            continue
        if peak_z > 0.0 and path_peak_z < peak_z:
            last_err = (
                f"geometry peak_z={path_peak_z:.3f} < required {peak_z:.3f}"
            )
            continue
        if validate_mujoco and peak_z > 0.0:
            if not _path_clear_of_wall_meshes(dense, scenario_id=scenario_id):
                last_err = "mujoco wall-mesh contact on path"
                continue
        elif validate_mujoco and not _dry_run_transfer(
            dense,
            start,
            goal,
            joint_tol_rad=0.12,
            scenario_id=scenario_id,
        ):
            last_err = "mujoco dry-run rejected"
            continue
        return dense, straight_collides

    raise RuntimeError(
        f"{scenario_id} transfer plan failed after retries ({last_err})"
    )


def _actuator_id(mj: Any, model: Any, name: str) -> int:
    aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"Missing MuJoCo actuator {name!r}")
    return int(aid)


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


def simulate_pick_place_clutter(
    *,
    duration_s: float = 45.0,
    joint_tol_rad: float = 0.12,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
    seed_offset: int = 0,
) -> ClutterPickPlaceResult:
    """Run one SC-v13c/d cycle (physics grasp + planned MPC transfer)."""
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for clutter pick-place") from exc

    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    scenario_id = _scenario_id(params)
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path("open_manipulator_x", scenario_id)
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)

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

    transfer_path, straight_collides = plan_transfer_path(
        wp.pick_hover,
        wp.place_hover,
        scenario_path=scenario,
        seed_offset=seed_offset,
    )

    phase_mpc = build_omx_joint_mpc()
    transfer_tracker = JointPathMPCTracker(
        transfer_path,
        build_omx_joint_mpc(),
        race_speed=1.2,
        max_carrot_lag=0.40,
        goal_tol=max(0.12, float(joint_tol_rad)),
    )

    # Roof overhang leaves little vertical room on the pick side; accept a
    # slightly lower lift confirmation than the open-cell SC-v13b/c demos.
    lift_z = float(params.get("lift_height_m", 0.125))
    phase_timeout = float(params.get("phase_timeout_s", 30.0))
    if scenario_id == "omx_wall_maze":
        phase_timeout = max(phase_timeout, 60.0)
    fsm = PickPlaceFSM(
        wp,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=1.4,
        release_hold_s=0.8,
        lift_height_m=lift_z,
        phase_timeout_s=phase_timeout,
    )
    fsm.start()

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(400):
        mj.mj_step(model, data)

    phase_mpc.reset(_arm_q(mj, model, data))
    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))
    transfer_armed = False
    ctrl_accum = 0.0
    # Start the command at the settled pose (not pick_hover) so maze
    # rate-limited slews actually traverse idle → hover under the roof.
    q_cmd = _arm_q(mj, model, data)
    last_phase_target = wp.idle.copy()

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
            transfer_tracker.reset(q)
            q_cmd = q.copy()

        if (
            cmd.state != PickPlaceState.MOVE_PLACE
            and float(np.linalg.norm(cmd.q_des - last_phase_target)) > 1e-9
        ):
            phase_mpc.reset(q)
            last_phase_target = cmd.q_des.copy()

        ctrl_accum += dt
        if ctrl_accum >= _CTRL_PERIOD_S:
            ctrl_accum -= _CTRL_PERIOD_S
            if cmd.state == PickPlaceState.MOVE_PLACE and transfer_armed:
                if transfer_tracker.complete:
                    q_cmd = wp.place_hover.copy()
                else:
                    q_cmd = transfer_tracker.step(_CTRL_PERIOD_S)
            elif scenario_id == "omx_wall_maze":
                # Rate-limited joint slew (not JointSpaceMPC): MPC arcs dive
                # into the pick-side roof, while an instantaneous jump from
                # idle also jams the telhadinho. A slow chord stays clear.
                target = np.asarray(cmd.q_des, dtype=np.float64)
                delta = target - q_cmd
                step_n = float(np.linalg.norm(delta))
                max_step = 0.035  # rad per control tick (~1.75 rad/s)
                if step_n <= max_step:
                    q_cmd = target.copy()
                else:
                    q_cmd = q_cmd + delta * (max_step / step_n)
            else:
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
        adhere = adhesion_command(cmd.state, fsm.hold_t)
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

    # Always record the terminal frame (record_every can skip the DONE tick).
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

    # Hold the retreat fold so showcase clips end at home, not mid-RETREAT.
    if fsm.state == PickPlaceState.DONE:
        hold_q = wp.retreat if wp.retreat is not None else wp.idle
        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(hold_q[i])
        data.ctrl[act_grip] = GRIPPER_OPEN
        data.ctrl[act_al] = 0.0
        data.ctrl[act_ar] = 0.0
        hold_steps = max(record_every_steps, int(round(1.5 / dt)))
        for step_i in range(hold_steps):
            mj.mj_step(model, data)
            if step_i % record_every_steps == 0 or step_i + 1 == hold_steps:
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

    box_pos = np.asarray(data.xpos[box_id], dtype=np.float64).copy()
    return ClutterPickPlaceResult(
        state=fsm.state,
        box_pos=box_pos,
        transfer_path=transfer_path,
        straight_line_collides=straight_collides,
        samples=samples,
    )


def run_pick_place_clutter(
    *,
    duration_s: float = 45.0,
    joint_tol_rad: float = 0.12,
    scenario_path: str | Path | None = None,
    max_attempts: int = 4,
    record_every_steps: int = 1,
) -> ClutterPickPlaceResult:
    """Execute one SC-v13c/d cycle; retry on place-miss / fault (planner RNG)."""
    params = load_scenario_parameters(scenario_path or _SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    last: ClutterPickPlaceResult | None = None
    last_err: Exception | None = None
    for attempt in range(max(1, int(max_attempts))):
        try:
            last = simulate_pick_place_clutter(
                duration_s=duration_s,
                joint_tol_rad=joint_tol_rad,
                record_every_steps=record_every_steps,
                scenario_path=scenario_path,
                seed_offset=attempt * 17,
            )
        except RuntimeError as exc:
            last_err = exc
            continue
        if last.state != PickPlaceState.DONE:
            continue
        if float(np.linalg.norm(last.box_pos[:2] - place_xy)) < 0.08:
            return last
    if last is None and last_err is not None:
        raise last_err
    assert last is not None
    return last
