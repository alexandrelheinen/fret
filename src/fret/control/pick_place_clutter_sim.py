"""SC-v13c/d pick-and-place with desk wall / Γ maze: planner + MPC + physics.

Grasp/release phases reuse the SC-v13b FSM and MuJoCo adhesion grasp. The
``MOVE_PLACE`` phase plans ``pick_hover → place_hover`` with ARCO RRT* against
an inflated wall occupancy cloud, densifies via ``TrajectoryGenerator``, then
tracks the path with ARCO :class:`~arco.control.mpc.JointSpaceMPC` (carrot
NMPC — replaces proportional ``ControllerNode`` joint-space tracking).

SC-v13c uses a single mid-cell slab; SC-v13d uses a Γ (inverted-L) stem+cap.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.config_loader import load_algorithm_config, planning_config_for_model
from fret.control.cspace_mpc_occupancy import (
    build_wall_cspace_barrier_occupancy,
)
from fret.control.joint_mpc import JointPathMPCTracker, build_joint_mpc
from fret.control.kinematics import Kinematics

try:
    from arco.control.mpc import JointSpaceMPCConfig
except ImportError:  # pragma: no cover
    JointSpaceMPCConfig = None
from fret.control.pick_place_common import PickPlaceSample, adhesion_command
from fret.control.pick_place_fsm import (
    GRIPPER_OPEN,
    PickPlaceFSM,
    PickPlaceObservation,
    PickPlaceState,
)
from fret.control.pick_place_sim import waypoints_from_scenario
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
from fret.telemetry.scenario_hooks import (
    arm_sample_values,
    close_telemetry,
    open_scenario_telemetry,
    setup_arm_telemetry,
)

_SCENARIO = Path("src/fret/config/scenarios/omx_desk_clutter.yml")
_CTRL_PERIOD_S = 0.02
_OMY_MODELS = frozenset({"omy", "six_dof", "open_manipulator_y"})


def _robot_model(params: dict[str, Any]) -> str:
    return str(params.get("model", "open_manipulator_x"))


def _arm_joint_names(model: str) -> tuple[str, ...]:
    if model in _OMY_MODELS:
        return (
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
        )
    return ("Joint1", "Joint2", "Joint3", "Joint4")


def _ee_body_name(model: str) -> str:
    return "link6" if model in _OMY_MODELS else "link5"


def _joint_mpc_config(params: dict[str, Any] | None = None) -> Any | None:
    """Optional JointSpaceMPCConfig with scenario obstacle-weight overrides."""
    if JointSpaceMPCConfig is None:  # pragma: no cover
        return None
    cfg = JointSpaceMPCConfig.create_from_config()
    if not params:
        return cfg
    w_obs = params.get("mpc_weight_obstacle")
    if w_obs is not None:
        cfg = replace(cfg, weight_obstacle=float(w_obs))
    return cfg


def _joint_mpc_for_model(
    model: str,
    *,
    occupancy: Any | None = None,
    params: dict[str, Any] | None = None,
) -> Any:
    """Build joint-space MPC, optionally with C-space obstacle barriers."""
    return build_joint_mpc(
        6 if model in _OMY_MODELS else 4,
        occupancy=occupancy,
        mpc_cfg=_joint_mpc_config(params),
    )


def _require_mpc_occupancy(mpc: Any, *, context: str) -> None:
    """Fail loud if a wall-scenario MPC was built without barriers."""
    occ = getattr(mpc, "_occ", None)
    if occ is None:
        raise RuntimeError(
            f"{context}: JointSpaceMPC has occupancy=None; "
            "C-space obstacle barriers are required for wall scenarios"
        )


def _barrier_occupancy_for_scenario(
    params: dict[str, Any],
    *,
    robot_model: str,
    scenario_id: str,
    seed: int,
) -> Any | None:
    """Build C-space KDTree barriers from MuJoCo wall contacts when present.

    Returns ``None`` when the scenario MJCF has no ``transfer_wall*`` geoms
    (open pick-place cells) so MPC keeps the occupancy-free path.
    """
    try:
        import mujoco as mj
    except ImportError:  # pragma: no cover
        return None
    xml = mjcf_path(robot_model, scenario_id)
    model = mj.MjModel.from_xml_path(str(xml))
    has_wall = any(
        (mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, i) or "").startswith(
            "transfer_wall"
        )
        for i in range(model.ngeom)
    )
    if not has_wall:
        return None
    kin = Kinematics(robot_model)
    names = _arm_joint_names(robot_model)
    clearance = float(params.get("mpc_cspace_clearance_rad", 0.28))
    n_samples = int(params.get("mpc_cspace_samples", 14000))
    return build_wall_cspace_barrier_occupancy(
        mjcf_path=str(xml),
        joint_limits=kin.joint_limits,
        joint_names=names,
        clearance=clearance,
        n_samples=n_samples,
        rng=np.random.default_rng(int(seed) + 91),
    )


@dataclass(frozen=True)
class ClutterPickPlaceResult:
    """Outcome of one SC-v13c/d clutter / maze cycle."""

    state: PickPlaceState
    box_pos: npt.NDArray[np.float64]
    transfer_path: list[npt.NDArray[np.float64]]
    straight_line_collides: bool
    samples: list[PickPlaceSample]
    telemetry_csv_path: Path | None = None
    telemetry_manifest_path: Path | None = None
    wall_contact_steps: int = 0
    faulted_on_wall_contact: bool = False


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


def arm_contacts_transfer_wall(mj: Any, model: Any, data: Any) -> bool:
    """True when any MuJoCo contact involves a ``transfer_wall*`` geom."""
    for ci in range(data.ncon):
        c = data.contact[ci]
        g1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom1)
        g2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom2)
        if _is_wall_geom(g1) or _is_wall_geom(g2):
            return True
    return False


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
    robot_model: str = "open_manipulator_x",
    occupancy: Any | None = None,
    params: dict[str, Any] | None = None,
) -> bool:
    """Return True if joint-space MPC tracking reaches ``goal`` without wall jams."""
    try:
        import mujoco as mj
    except ImportError:  # pragma: no cover
        return True

    model = mj.MjModel.from_xml_path(str(mjcf_path(robot_model, scenario_id)))
    data = mj.MjData(model)
    names = _arm_joint_names(robot_model)
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    qid = int(model.jnt_qposadr[box_jid])
    data.qpos[qid : qid + 3] = [0.5, 0.5, 0.5]
    data.qvel[:] = 0.0

    from fret.control.pick_place_fsm import (
        GRIPPER_CLOSED,
        OMY_GRIPPER_CLOSED,
    )

    is_omy = robot_model in _OMY_MODELS
    grip_settle = OMY_GRIPPER_CLOSED if is_omy else GRIPPER_CLOSED

    def settle(cmd: npt.NDArray[np.float64], steps: int) -> None:
        for _ in range(steps):
            for i, n in enumerate(names):
                data.ctrl[model.actuator(n).id] = float(cmd[i])
            data.ctrl[model.actuator("Gripper").id] = float(grip_settle)
            data.ctrl[model.actuator("grip_left").id] = 0.0
            data.ctrl[model.actuator("grip_right").id] = 0.0
            mj.mj_step(model, data)

    idle = (
        np.array([0.0, -0.8, 1.2, 0.0, 0.5, 0.0], dtype=np.float64)
        if is_omy
        else np.array([0.0, -1.05, 0.7, 0.7], dtype=np.float64)
    )
    settle(idle, 300)
    settle(start, 700)

    mpc = _joint_mpc_for_model(robot_model, occupancy=occupancy, params=params)
    if occupancy is not None:
        _require_mpc_occupancy(mpc, context="_dry_run_transfer")
    tracker = JointPathMPCTracker(
        dense,
        mpc,
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
        data.ctrl[model.actuator("Gripper").id] = float(grip_settle)
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
    # Soft contacts while brushing inflated padding are tolerated; sustained
    # wall jams (MPC cutting the chord through a slab) are not.
    return float(np.linalg.norm(q - goal)) < 0.15 and hits < 40


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
    robot_model = _robot_model(params)
    physical = walls_from_scenario(scenario, inflate=False)
    inflated = walls_from_scenario(scenario, inflate=True)
    seed0 = int(params.get("planner_rng_seed", 7)) + int(seed_offset)
    density = float(params.get("occupancy_density", 1000.0))
    timeout = float(params.get("planning_timeout", 25.0))
    algo = str(params.get("planner_algorithm", "rrt_star"))
    max_r = float(params.get("max_transfer_radius_m", 0.12))
    min_z = float(params.get("min_transfer_ee_z_m", 0.145))
    peak_z = float(params.get("min_transfer_peak_ee_z_m", 0.0))

    kin = Kinematics(robot_model)
    cfg = load_algorithm_config(planning_config_for_model(robot_model))
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

    mpc_occ = _barrier_occupancy_for_scenario(
        params,
        robot_model=robot_model,
        scenario_id=scenario_id,
        seed=seed0,
    )

    from fret.scenario.planner_rng import deterministic_planner_rng

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
            model=robot_model,
            occupancy_adapter=adapter,
            planner_algorithm=algo,  # type: ignore[arg-type]
            scenario=scenario_id,
        )
        # ARCO RRT*/SST call unseeded default_rng(); pin so planner variants
        # (and release showcase seeds) are reproducible.
        with deterministic_planner_rng(seed):
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
            robot_model=robot_model,
            occupancy=mpc_occ,
            params=params,
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


def _arm_q(
    mj: Any,
    model: Any,
    data: Any,
    names: tuple[str, ...] | None = None,
) -> npt.NDArray[np.float64]:
    joint_names = names or ("Joint1", "Joint2", "Joint3", "Joint4")
    return np.array(
        [
            data.qpos[
                model.jnt_qposadr[
                    mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
                ]
            ]
            for name in joint_names
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
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
) -> ClutterPickPlaceResult:
    """Run one SC-v13c/d cycle (physics grasp + planned MPC transfer)."""
    try:
        import mujoco as mj
    except ImportError as exc:  # pragma: no cover
        raise ImportError("mujoco is required for clutter pick-place") from exc

    scenario = Path(scenario_path or _SCENARIO)
    params = load_scenario_parameters(scenario)
    scenario_id = _scenario_id(params)
    robot_model = _robot_model(params)
    arm_names = _arm_joint_names(robot_model)
    agent_name = "omy" if robot_model in _OMY_MODELS else "omx"
    wp = waypoints_from_scenario(scenario)
    xml = mjcf_path(robot_model, scenario_id)
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
        joint_components = setup_arm_telemetry(tele, agent_name, arm_names)

    ee_id = mj.mj_name2id(
        model, mj.mjtObj.mjOBJ_BODY, _ee_body_name(robot_model)
    )
    box_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "pick_box")
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    box_qadr = int(model.jnt_qposadr[box_jid])

    act_arm = [_actuator_id(mj, model, n) for n in arm_names]
    act_grip = _actuator_id(mj, model, "Gripper")
    act_al = _actuator_id(mj, model, "grip_left")
    act_ar = _actuator_id(mj, model, "grip_right")
    is_omy = robot_model in _OMY_MODELS

    # OMY transfers from the post-grasp lift pose (pad-mid carry above floor ball).
    transfer_start = (
        wp.lift_hover
        if is_omy and wp.lift_hover is not None
        else wp.pick_hover
    )
    if is_omy:
        # Shared arm planner (detour YAML + RRT*/SST) — same module OMX clutter
        # uses for wall occupancy; prefer_transfer_detour avoids 6-DOF timeouts.
        from fret.control.pick_place_planning import plan_arm_transfer_path

        transfer_path, straight_collides = plan_arm_transfer_path(
            transfer_start,
            wp.place_hover,
            scenario_path=scenario,
            seed_offset=seed_offset,
        )
    else:
        transfer_path, straight_collides = plan_transfer_path(
            transfer_start,
            wp.place_hover,
            scenario_path=scenario,
            seed_offset=seed_offset,
        )

    mpc_seed = int(params.get("planner_rng_seed", 7)) + int(seed_offset)
    mpc_occ = _barrier_occupancy_for_scenario(
        params,
        robot_model=robot_model,
        scenario_id=scenario_id,
        seed=mpc_seed,
    )
    if mpc_occ is None and bool(params.get("fault_on_wall_contact", False)):
        raise RuntimeError(
            f"{scenario_id}: fault_on_wall_contact requires a C-space "
            "MPC occupancy map (missing transfer_wall* geoms?)"
        )
    phase_mpc = _joint_mpc_for_model(
        robot_model, occupancy=mpc_occ, params=params
    )
    transfer_mpc = _joint_mpc_for_model(
        robot_model, occupancy=mpc_occ, params=params
    )
    if mpc_occ is not None:
        _require_mpc_occupancy(phase_mpc, context=f"{scenario_id}.phase_mpc")
        _require_mpc_occupancy(
            transfer_mpc, context=f"{scenario_id}.transfer_mpc"
        )
    transfer_tracker = JointPathMPCTracker(
        transfer_path,
        transfer_mpc,
        race_speed=1.2,
        max_carrot_lag=0.40,
        goal_tol=max(0.12, float(joint_tol_rad)),
    )

    # Roof overhang leaves little vertical room on the pick side; accept a
    # slightly lower lift confirmation than the open-cell SC-v13b/c demos.
    from fret.control.pick_place_fsm import (
        GRIPPER_CLOSED,
        GRIPPER_OPEN,
        OMY_GRIPPER_CLOSED,
        OMY_GRIPPER_OPEN,
    )

    gripper_open = (
        OMY_GRIPPER_OPEN if robot_model in _OMY_MODELS else GRIPPER_OPEN
    )
    gripper_closed = (
        OMY_GRIPPER_CLOSED if robot_model in _OMY_MODELS else GRIPPER_CLOSED
    )
    fsm_dof = 6 if robot_model in _OMY_MODELS else 4
    lift_z = float(params.get("lift_height_m", 0.125))
    phase_timeout = float(params.get("phase_timeout_s", 30.0))
    if scenario_id == "omx_wall_maze":
        phase_timeout = max(phase_timeout, 60.0)
    fsm = PickPlaceFSM(
        wp,
        dof=fsm_dof,
        gripper_open=gripper_open,
        gripper_closed=gripper_closed,
        joint_tol_rad=joint_tol_rad,
        grasp_hold_s=1.4,
        release_hold_s=0.8,
        lift_height_m=lift_z,
        phase_timeout_s=phase_timeout,
        drop_fault_enabled=True,
        # Pad↔ball contact required before leaving GRASP (OM-X + OMY).
        require_grasp_contact=True,
        transfer_joint_tol_rad=(
            max(float(joint_tol_rad), 0.20) if is_omy else None
        ),
    )
    fsm.start()
    pad_right = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right")
    pad_left = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left")
    from fret.control.pick_place_common import ball_grasp_contact

    for i, aid in enumerate(act_arm):
        data.ctrl[aid] = float(wp.idle[i])
    data.ctrl[act_grip] = gripper_open
    data.ctrl[act_al] = 0.0
    data.ctrl[act_ar] = 0.0
    for _ in range(400):
        mj.mj_step(model, data)

    phase_mpc.reset(_arm_q(mj, model, data, arm_names))
    dt = float(model.opt.timestep)
    max_steps = int(duration_s / dt)
    samples: list[PickPlaceSample] = []
    record_every_steps = max(1, int(record_every_steps))
    transfer_armed = False
    ctrl_accum = 0.0
    # Start the command at the settled pose (not pick_hover) so maze
    # rate-limited slews actually traverse idle → hover under the roof.
    q_cmd = _arm_q(mj, model, data, arm_names)
    last_phase_target = wp.idle.copy()
    fault_on_wall = bool(params.get("fault_on_wall_contact", False))
    # Debounce single-tick numerical flicker (~10 ms at 2 ms timestep).
    wall_hold_needed = max(1, int(params.get("wall_contact_fault_steps", 5)))
    wall_streak = 0
    wall_contact_steps = 0
    faulted_on_wall = False

    for step_i in range(max_steps):
        q = _arm_q(mj, model, data, arm_names)
        grasp_ok = None
        if pad_right >= 0 and pad_left >= 0:
            grasp_ok = ball_grasp_contact(
                mj,
                model,
                data,
                box_body_id=box_id,
                pad_right_id=pad_right,
                pad_left_id=pad_left,
                # OM-X soft pads: allow tight pad-mid fallback; OMY stays
                # contact-only for showcase honesty.
                allow_pad_mid_fallback=not is_omy,
                max_pad_mid_dist_m=0.025 if not is_omy else 0.035,
            )
        obs = PickPlaceObservation(
            q=q,
            object_pos=np.asarray(data.xpos[box_id], dtype=np.float64).copy(),
            ee_pos=np.asarray(data.xpos[ee_id], dtype=np.float64).copy(),
            grasp_contact=grasp_ok,
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
            else:
                # JointSpaceMPC soft C-space barriers (when ``mpc_occ`` is
                # set) keep carrot arcs off the Γ roofs; open cells leave
                # occupancy=None and behave as before.
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
            gripper_closed=float(gripper_closed),
            gripper_open=float(gripper_open),
        )
        data.ctrl[act_al] = adhere
        data.ctrl[act_ar] = adhere
        mj.mj_step(model, data)

        if arm_contacts_transfer_wall(mj, model, data):
            wall_contact_steps += 1
            wall_streak += 1
            if (
                fault_on_wall
                and not faulted_on_wall
                and wall_streak >= wall_hold_needed
                and fsm.state
                not in {PickPlaceState.DONE, PickPlaceState.FAULT}
            ):
                fsm.fault()
                faulted_on_wall = True
        else:
            wall_streak = 0

        if tele is not None:
            q_now = _arm_q(mj, model, data, arm_names)
            tele.record(
                step_i * dt,
                arm_sample_values(
                    agent_name,
                    joint_components,
                    q_now,
                    np.asarray(data.xpos[ee_id], dtype=np.float64),
                ),
                tick=step_i,
            )

        if step_i % record_every_steps == 0:
            samples.append(
                PickPlaceSample(
                    q_arm=_arm_q(mj, model, data, arm_names),
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
            q_arm=_arm_q(mj, model, data, arm_names),
            gripper=float(data.ctrl[act_grip]),
            box_qpos=np.asarray(
                data.qpos[box_qadr : box_qadr + 7], dtype=np.float64
            ).copy(),
            state=fsm.state,
        )
    )

    # Hold idle so showcase clips end at home (same contract as OMX / SC-v14b).
    if fsm.state == PickPlaceState.DONE:
        hold_q = wp.idle
        for i, aid in enumerate(act_arm):
            data.ctrl[aid] = float(hold_q[i])
        data.ctrl[act_grip] = gripper_open
        data.ctrl[act_al] = 0.0
        data.ctrl[act_ar] = 0.0
        hold_steps = max(record_every_steps, int(round(1.5 / dt)))
        for step_i in range(hold_steps):
            mj.mj_step(model, data)
            if tele is not None:
                q_now = _arm_q(mj, model, data, arm_names)
                tele.record(
                    (max_steps + step_i) * dt,
                    arm_sample_values(
                        agent_name,
                        joint_components,
                        q_now,
                        np.asarray(data.xpos[ee_id], dtype=np.float64),
                    ),
                    tick=max_steps + step_i,
                )
            if step_i % record_every_steps == 0 or step_i + 1 == hold_steps:
                samples.append(
                    PickPlaceSample(
                        q_arm=_arm_q(mj, model, data, arm_names),
                        gripper=float(data.ctrl[act_grip]),
                        box_qpos=np.asarray(
                            data.qpos[box_qadr : box_qadr + 7],
                            dtype=np.float64,
                        ).copy(),
                        state=PickPlaceState.DONE,
                    )
                )

    box_pos = np.asarray(data.xpos[box_id], dtype=np.float64).copy()
    tele_csv, tele_manifest = close_telemetry(tele)
    return ClutterPickPlaceResult(
        state=fsm.state,
        box_pos=box_pos,
        transfer_path=transfer_path,
        straight_line_collides=straight_collides,
        samples=samples,
        telemetry_csv_path=tele_csv,
        telemetry_manifest_path=tele_manifest,
        wall_contact_steps=wall_contact_steps,
        faulted_on_wall_contact=faulted_on_wall,
    )


def run_pick_place_clutter(
    *,
    duration_s: float = 45.0,
    joint_tol_rad: float = 0.12,
    scenario_path: str | Path | None = None,
    max_attempts: int = 4,
    record_every_steps: int = 1,
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
) -> ClutterPickPlaceResult:
    """Execute one SC-v13c/d cycle; retry on place-miss / fault (planner RNG)."""
    params = load_scenario_parameters(scenario_path or _SCENARIO)
    place_xy = np.asarray(params["place_xy"], dtype=np.float64)
    # Cone mouth is the place volume (OMY r=0.14); OM-X r=0.05 stays at 0.08.
    place_tol = max(0.08, float(params.get("place_cone_radius_m", 0.05)))
    last: ClutterPickPlaceResult | None = None
    last_err: Exception | None = None
    for attempt in range(max(1, int(max_attempts))):
        try:
            last = simulate_pick_place_clutter(
                duration_s=duration_s,
                joint_tol_rad=joint_tol_rad,
                record_every_steps=record_every_steps,
                scenario_path=scenario_path,
                # Dense offsets so pinned YAML seeds remain reachable.
                seed_offset=attempt,
                telemetry_enabled=telemetry_enabled,
                telemetry_output_dir=telemetry_output_dir,
                telemetry_csv_basename=telemetry_csv_basename,
            )
        except RuntimeError as exc:
            last_err = exc
            continue
        if last.state != PickPlaceState.DONE:
            continue
        if int(last.wall_contact_steps) > 0:
            continue
        if float(np.linalg.norm(last.box_pos[:2] - place_xy)) < place_tol:
            return last
    if last is None and last_err is not None:
        raise last_err
    assert last is not None
    if (
        last.state != PickPlaceState.DONE
        or int(last.wall_contact_steps) > 0
        or float(np.linalg.norm(last.box_pos[:2] - place_xy)) >= place_tol
    ):
        raise RuntimeError(
            f"{_scenario_id(params)} clutter cycle failed after "
            f"{max_attempts} attempts (last={last.state.name}, "
            f"wall_contact_steps={last.wall_contact_steps})"
        )
    return last
