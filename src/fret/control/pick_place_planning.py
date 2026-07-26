"""Wall occupancy helpers for cluttered pick-and-place (no MPC dependency)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.config_loader import load_algorithm_config, planning_config_for_model
from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.box_obstacle import BoxObstacle
from fret.planning.planner_node import PlannerNode
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter
from fret.sitl_config import load_scenario_parameters


def walls_from_scenario(
    scenario_path: str | Path,
    *,
    inflate: bool = False,
) -> list[BoxObstacle]:
    """Build wall boxes from scenario YAML (single slab or ``walls`` list)."""
    p = load_scenario_parameters(scenario_path)
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


def _sample_walls(
    walls: list[BoxObstacle],
    density: float,
    rng: np.random.Generator,
) -> npt.NDArray[np.float64]:
    clouds = [w.sample_surface(density, rng=rng) for w in walls]
    return np.vstack(clouds).astype(np.float64)


def _path_metrics(
    kin: Kinematics, path: list[npt.NDArray[np.float64]]
) -> tuple[float, float, float]:
    ee = np.array(
        [kin.forward_kinematics(q)[:3, 3] for q in path], dtype=np.float64
    )
    radii = np.linalg.norm(ee[:, :2], axis=1)
    return (
        float(np.min(radii)),
        float(np.min(ee[:, 2])),
        float(np.max(ee[:, 2])),
    )


def _edge_free(
    checker: Any,
    a: npt.NDArray[np.float64],
    b: npt.NDArray[np.float64],
    *,
    samples: int = 24,
) -> bool:
    """True when the joint-space chord ``a→b`` is collision-free."""
    for t in np.linspace(0.0, 1.0, int(samples)):
        if not checker.is_collision_free((1.0 - t) * a + t * b):
            return False
    return True


def _omy_rrt_star_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    checker: Any,
    seed: int,
) -> list[npt.NDArray[np.float64]] | None:
    """Run ARCO RRT* with 6-DOF-friendly parameters; return coarse path or None."""
    try:
        from arco.planning.continuous.rrt import RRTPlanner
    except ImportError:  # pragma: no cover
        try:
            from arco.planning import RRTPlanner
        except ImportError:
            return None
    from fret.planning.planner_node import _CSpaceOccupancy
    from fret.planning.planner_rng import deterministic_planner_rng

    kin = Kinematics("omy")
    bounds = [(float(lo), float(hi)) for lo, hi in kin.joint_limits]
    # ARCO RRT* has no cooperative timeout; keep sample count modest.
    planner = RRTPlanner(
        occupancy=_CSpaceOccupancy(checker),
        bounds=bounds,
        max_sample_count=2500,
        step_size=0.50,
        goal_tolerance=0.20,
        collision_check_count=5,
        goal_bias=0.35,
    )
    with deterministic_planner_rng(int(seed)):
        path = planner.plan(
            np.asarray(start, dtype=np.float64),
            np.asarray(goal, dtype=np.float64),
        )
    if path is None or len(path) < 2:
        return None
    return [np.asarray(q, dtype=np.float64) for q in path]


def _omy_sst_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    checker: Any,
    seed: int,
) -> list[npt.NDArray[np.float64]] | None:
    """Run ARCO SST with 6-DOF-friendly parameters; return coarse path or None."""
    try:
        from arco.planning.continuous.sst import SSTPlanner
    except ImportError:  # pragma: no cover
        try:
            from arco.planning import SSTPlanner
        except ImportError:
            return None
    from fret.planning.planner_node import _CSpaceOccupancy
    from fret.planning.planner_rng import deterministic_planner_rng

    kin = Kinematics("omy")
    bounds = [(float(lo), float(hi)) for lo, hi in kin.joint_limits]
    planner = SSTPlanner(
        occupancy=_CSpaceOccupancy(checker),
        bounds=bounds,
        max_sample_count=2500,
        step_size=0.50,
        goal_tolerance=0.20,
        witness_radius=0.35,
        collision_check_count=5,
        goal_bias=0.35,
    )
    with deterministic_planner_rng(int(seed)):
        path = planner.plan(
            np.asarray(start, dtype=np.float64),
            np.asarray(goal, dtype=np.float64),
        )
    if path is None or len(path) < 2:
        return None
    return [np.asarray(q, dtype=np.float64) for q in path]


def _omy_corridor_via_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    checker: Any,
) -> list[npt.NDArray[np.float64]] | None:
    """Sampling-based via planner: pad-mid IK corridor around the mid-cell wall."""
    try:
        import mujoco as mj
    except ImportError:  # pragma: no cover
        return None
    from fret.control.omy_pad_mid_ik import (
        OMY_ARM_JOINTS,
        OMY_GRIPPER_PINCH,
        pad_mid_ik,
    )
    from fret.mjcf.omy import ensure_omy_clutter_mjcf

    xml = ensure_omy_clutter_mjcf()
    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    pad_right = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_right"))
    pad_left = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "pad_left"))
    grip_adr = int(
        model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "rh_r1")]
    )
    limits = np.array(
        [
            model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]
            for n in OMY_ARM_JOINTS
        ],
        dtype=np.float64,
    )
    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    vias: list[npt.NDArray[np.float64]] = []
    for x in np.linspace(0.18, 0.34, 6):
        for y in np.linspace(-0.15, 0.22, 8):
            for z in np.linspace(0.22, 0.38, 5):
                q, err = pad_mid_ik(
                    model,
                    data,
                    mj,
                    target=np.array([x, y, z], dtype=np.float64),
                    grip_val=OMY_GRIPPER_PINCH,
                    seed=start,
                    limits=limits,
                    pad_right=pad_right,
                    pad_left=pad_left,
                    grip_adr=grip_adr,
                )
                if err > 0.015:
                    continue
                if not checker.is_collision_free(q):
                    continue
                vias.append(q)
    # Prefer a two-via path (clearer detour), then a single via.
    for v1 in vias:
        if not _edge_free(checker, start, v1):
            continue
        for v2 in vias:
            if float(np.linalg.norm(v1 - v2)) < 0.2:
                continue
            if not _edge_free(checker, v1, v2):
                continue
            if not _edge_free(checker, v2, goal):
                continue
            return [start.copy(), v1.copy(), v2.copy(), goal.copy()]
    for v1 in vias:
        if _edge_free(checker, start, v1) and _edge_free(checker, v1, goal):
            return [start.copy(), v1.copy(), goal.copy()]
    return None


def plan_arm_transfer_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    scenario_path: str | Path,
    seed_offset: int = 0,
) -> tuple[list[npt.NDArray[np.float64]], bool]:
    """Plan a dense collision-free transfer path from scenario wall occupancy."""
    scenario = Path(scenario_path)
    params = load_scenario_parameters(scenario)
    scenario_id = str(params.get("scenario_id", scenario.stem))
    robot_model = str(params.get("model", "open_manipulator_x"))
    physical = walls_from_scenario(scenario, inflate=False)
    inflated = walls_from_scenario(scenario, inflate=True)
    seed0 = int(params.get("planner_rng_seed", 7)) + int(seed_offset)
    density = float(params.get("occupancy_density", 1000.0))
    timeout = float(params.get("planning_timeout", 25.0))
    algo = str(params.get("planner_algorithm", "rrt_star"))
    max_r = float(params.get("max_transfer_radius_m", 0.12))
    min_z = float(params.get("min_transfer_ee_z_m", 0.145))

    kin = Kinematics(robot_model)
    cfg = load_algorithm_config(planning_config_for_model(robot_model))
    from fret.planning.cspace_checker import make_cspace_checker

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

    from fret.planning.planner_rng import deterministic_planner_rng

    last_err: str | None = None
    detour_mid = params.get("transfer_detour_configuration")
    if detour_mid is not None and bool(
        params.get("prefer_transfer_detour", False)
    ):
        mid = np.asarray(detour_mid, dtype=np.float64)
        detour = [start.copy(), mid, goal.copy()]
        if all(
            phys_checker.is_collision_free(detour[i])
            for i in range(len(detour))
        ) and all(
            phys_checker.is_collision_free(0.5 * (detour[i] + detour[i + 1]))
            for i in range(len(detour) - 1)
        ):
            return detour, straight_collides

    max_attempts = int(
        params.get("max_planner_attempts", 6 if robot_model == "omy" else 24)
    )
    for attempt in range(max_attempts):
        seed = seed0 + attempt
        rng = np.random.default_rng(seed)
        pts = _sample_walls(inflated, density, rng)
        adapter = OccupancyAdapter()
        adapter.update(
            OccupancyUpdatePayload(
                obstacle_points=pts, timestamp=0.0, frame_id="world"
            )
        )
        coarse: list[npt.NDArray[np.float64]] | None = None
        if robot_model == "omy":
            # 6-DOF: ARCO RRT* or SST, then IK corridor vias as fallback.
            omy_checker = make_cspace_checker(kin, adapter.get_occupancy())
            if algo == "rrt_star":
                coarse = _omy_rrt_star_path(
                    start, goal, checker=omy_checker, seed=seed
                )
            elif algo == "sst":
                coarse = _omy_sst_path(
                    start, goal, checker=omy_checker, seed=seed
                )
            if coarse is None:
                coarse = _omy_corridor_via_path(
                    start, goal, checker=omy_checker
                )
            if coarse is None:
                last_err = "omy_transfer_no_path"
                continue
        else:
            planner = PlannerNode(
                model=robot_model,
                occupancy_adapter=adapter,
                planner_algorithm=algo,  # type: ignore[arg-type]
                scenario=scenario_id,
            )
            with deterministic_planner_rng(seed):
                result = planner.plan(
                    PlanningRequest(
                        start_configuration=np.asarray(
                            start, dtype=np.float64
                        ),
                        goal_configuration=np.asarray(goal, dtype=np.float64),
                        planning_timeout=timeout,
                        scenario_id=scenario_id,
                    )
                )
            if result.status != PlanningStatus.SUCCESS or len(result.path) < 2:
                last_err = (
                    f"status={result.status.name} "
                    f"code={result.error_code.name}"
                )
                continue
            coarse = [np.asarray(q, dtype=np.float64) for q in result.path]

        traj = TrajectoryGenerator(kin, cfg).process(coarse)
        dense = [
            np.asarray(pt.positions, dtype=np.float64) for pt in traj.points
        ]
        min_r, path_min_z, _path_peak_z = _path_metrics(kin, dense)
        if min_r > max_r or path_min_z < min_z:
            last_err = f"geometry min_r={min_r:.3f} min_z={path_min_z:.3f}"
            continue
        return dense, straight_collides

    if detour_mid is not None:
        mid = np.asarray(detour_mid, dtype=np.float64)
        detour = [start.copy(), mid, goal.copy()]
        if all(
            phys_checker.is_collision_free(detour[i])
            for i in range(len(detour))
        ) and all(
            phys_checker.is_collision_free(0.5 * (detour[i] + detour[i + 1]))
            for i in range(len(detour) - 1)
        ):
            return detour, straight_collides

    fallback = _fallback_detour_path(
        start, goal, kin=kin, checker=phys_checker
    )
    if fallback is not None and len(fallback) >= 2:
        return fallback, straight_collides

    raise RuntimeError(
        f"{scenario_id} transfer plan failed after retries ({last_err})"
    )


def _fallback_detour_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    kin: Kinematics,
    checker: object,
) -> list[npt.NDArray[np.float64]] | None:
    """Simple joint-space detour when RRT* exhausts retries."""
    _ = kin
    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    base_mid = start + 0.5 * (goal - start)
    mids: list[npt.NDArray[np.float64]] = []
    for j1_scale in (0.0, 0.15, -0.15):
        for j2_delta in (-0.35, -0.55, 0.25):
            for j4_delta in (0.2, 0.35, -0.15):
                mid = base_mid.copy()
                mid[0] += j1_scale
                mid[1] += j2_delta
                mid[2] += 0.12
                mid[3] += j4_delta
                mids.append(mid)
    candidates: list[list[npt.NDArray[np.float64]]] = [[start, goal]]
    candidates.extend([[start, mid, goal] for mid in mids])
    for path in candidates:
        if all(
            checker.is_collision_free(path[i])  # type: ignore[attr-defined]
            for i in range(len(path))
        ) and all(
            checker.is_collision_free(0.5 * (path[i] + path[i + 1]))  # type: ignore[attr-defined]
            for i in range(len(path) - 1)
        ):
            return [p.copy() for p in path]
    return None
