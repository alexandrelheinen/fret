"""SC-v13c pick-and-place with mid-cell wall: planner + controller + physics.

Grasp/release phases reuse the SC-v13b FSM and MuJoCo adhesion grasp. The
``MOVE_PLACE`` phase plans ``pick_hover → place_hover`` with ARCO RRT* against
an inflated wall occupancy cloud, densifies via ``TrajectoryGenerator``, then
tracks the path with ``ControllerNode.compute_joint_command`` (joint-space —
Cartesian Jacobian chords cut through the wall).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.config_loader import load_algorithm_config, planning_config_for_model
from fret.control.controller_node import ControllerNode
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
_CONTROLLER_CFG = "src/fret/config/controllers/open_manipulator_x.yml"


@dataclass(frozen=True)
class ClutterPickPlaceResult:
    """Outcome of one SC-v13c cycle."""

    state: PickPlaceState
    box_pos: npt.NDArray[np.float64]
    transfer_path: list[npt.NDArray[np.float64]]
    straight_line_collides: bool
    samples: list[PickPlaceSample]


def wall_from_scenario(
    scenario_path: str | Path | None = None,
    *,
    inflate: bool = False,
) -> BoxObstacle:
    """Build the mid-cell wall box from scenario YAML."""
    p = load_scenario_parameters(scenario_path or _SCENARIO)
    pad = float(p.get("wall_inflate_m", 0.0)) if inflate else 0.0
    return BoxObstacle(
        x_min=float(p["wall_x_min"]) - pad,
        y_min=-float(p["wall_y_half"]) - pad,
        z_min=0.0,
        x_max=float(p["wall_x_max"]) + pad,
        y_max=float(p["wall_y_half"]) + pad,
        z_max=float(p["wall_z_max"]),
    )


def _path_metrics(
    kin: Kinematics, path: list[npt.NDArray[np.float64]]
) -> tuple[float, float]:
    ee = np.array(
        [kin.forward_kinematics(q)[:3, 3] for q in path], dtype=np.float64
    )
    radii = np.linalg.norm(ee[:, :2], axis=1)
    return float(np.min(radii)), float(np.min(ee[:, 2]))


def _dry_run_transfer(
    dense: list[npt.NDArray[np.float64]],
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    joint_tol_rad: float,
) -> bool:
    """Return True if joint-space tracking reaches ``goal`` without wall jams."""
    try:
        import mujoco as mj
    except ImportError:  # pragma: no cover
        return True

    model = mj.MjModel.from_xml_path(
        str(mjcf_path("open_manipulator_x", "omx_desk_clutter"))
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

    controller = ControllerNode("open_manipulator_x", _CONTROLLER_CFG)
    controller.set_trajectory(dense)
    dt = float(model.opt.timestep)
    period = 0.02
    accum = 0.0
    hits = 0
    q_cmd = np.asarray(start, dtype=np.float64).copy()
    for _ in range(20000):
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
        accum += dt
        if accum >= period:
            accum -= period
            controller.compute_joint_command(
                q, joint_tol_rad=joint_tol_rad, kp=12.0
            )
            waypoint = controller.current_waypoint()
            if waypoint is not None:
                q_cmd = waypoint
            elif controller.is_trajectory_complete():
                q_cmd = np.asarray(goal, dtype=np.float64)
        for i, n in enumerate(names):
            data.ctrl[model.actuator(n).id] = float(q_cmd[i])
        data.ctrl[model.actuator("Gripper").id] = -0.01
        mj.mj_step(model, data)
        for ci in range(data.ncon):
            c = data.contact[ci]
            g1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom1)
            g2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom2)
            if "transfer_wall" in {g1, g2}:
                hits += 1
        if controller.is_trajectory_complete():
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
    """Plan a dense retract path around the wall; report if the chord collides.

    Tries several RNG seeds, keeps only paths that retract and stay high
    enough to carry the box, and optionally dry-runs tracking in MuJoCo.
    """
    params = load_scenario_parameters(scenario_path or _SCENARIO)
    physical = wall_from_scenario(scenario_path, inflate=False)
    inflated = wall_from_scenario(scenario_path, inflate=True)
    seed0 = int(params.get("planner_rng_seed", 7)) + int(seed_offset)
    density = float(params.get("occupancy_density", 1000.0))
    timeout = float(params.get("planning_timeout", 25.0))
    algo = str(params.get("planner_algorithm", "rrt_star"))
    max_r = float(params.get("max_transfer_radius_m", 0.12))
    min_z = float(params.get("min_transfer_ee_z_m", 0.145))

    kin = Kinematics("open_manipulator_x")
    cfg = load_algorithm_config(
        planning_config_for_model("open_manipulator_x")
    )
    from fret.planning.cspace_checker import make_cspace_checker

    # Straight-line check against the physical wall occupancy.
    rng_phys = np.random.default_rng(seed0)
    phys_pts = physical.sample_surface(density, rng=rng_phys)
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
    for attempt in range(24):
        seed = seed0 + attempt
        rng = np.random.default_rng(seed)
        pts = inflated.sample_surface(density, rng=rng)
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
            scenario="omx_desk_clutter",
        )
        result = planner.plan(
            PlanningRequest(
                start_configuration=np.asarray(start, dtype=np.float64),
                goal_configuration=np.asarray(goal, dtype=np.float64),
                planning_timeout=timeout,
                scenario_id="omx_desk_clutter",
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
        min_r, path_min_z = _path_metrics(kin, dense)
        if min_r > max_r or path_min_z < min_z:
            last_err = f"geometry min_r={min_r:.3f} min_z={path_min_z:.3f}"
            continue
        if validate_mujoco and not _dry_run_transfer(
            dense, start, goal, joint_tol_rad=0.12
        ):
            last_err = "mujoco dry-run rejected"
            continue
        return dense, straight_collides

    raise RuntimeError(
        f"SC-v13c transfer plan failed after retries ({last_err})"
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
    """Run one SC-v13c cycle (physics grasp + planned transfer)."""
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for clutter pick-place") from exc

    scenario = Path(scenario_path or _SCENARIO)
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path("open_manipulator_x", "omx_desk_clutter")
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
    controller = ControllerNode("open_manipulator_x", _CONTROLLER_CFG)
    controller.set_trajectory(transfer_path)

    fsm = PickPlaceFSM(
        wp,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=1.4,
        release_hold_s=0.8,
        lift_height_m=0.14,
        phase_timeout_s=30.0,
    )
    fsm.start()

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = GRIPPER_OPEN
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(400):
        mj.mj_step(model, data)

    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))
    transfer_armed = False
    ctrl_period = 1.0 / 50.0
    ctrl_accum = 0.0
    q_cmd = wp.pick_hover.copy()

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
            controller.set_trajectory(transfer_path)
            q_cmd = q.copy()

        if cmd.state == PickPlaceState.MOVE_PLACE and transfer_armed:
            ctrl_accum += dt
            if ctrl_accum >= ctrl_period:
                ctrl_accum -= ctrl_period
                controller.compute_joint_command(
                    q, joint_tol_rad=joint_tol_rad, kp=12.0
                )
                waypoint = controller.current_waypoint()
                if waypoint is not None:
                    q_cmd = waypoint
            if controller.is_trajectory_complete():
                q_cmd = wp.place_hover
            for i, aid in enumerate(act_arm):
                data.ctrl[aid] = float(q_cmd[i])
        else:
            for i, aid in enumerate(act_arm):
                data.ctrl[aid] = float(cmd.q_des[i])

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
) -> ClutterPickPlaceResult:
    """Execute one SC-v13c cycle; retry on place-miss / fault (planner RNG)."""
    params = load_scenario_parameters(scenario_path or _SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    last: ClutterPickPlaceResult | None = None
    for attempt in range(max(1, int(max_attempts))):
        last = simulate_pick_place_clutter(
            duration_s=duration_s,
            joint_tol_rad=joint_tol_rad,
            record_every_steps=1,
            scenario_path=scenario_path,
            seed_offset=attempt * 17,
        )
        if last.state != PickPlaceState.DONE:
            continue
        if float(np.linalg.norm(last.box_pos[:2] - place_xy)) < 0.08:
            return last
    assert last is not None
    return last
