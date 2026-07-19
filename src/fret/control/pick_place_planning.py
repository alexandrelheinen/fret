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
    for attempt in range(24):
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
        min_r, path_min_z, _path_peak_z = _path_metrics(kin, dense)
        if min_r > max_r or path_min_z < min_z:
            last_err = f"geometry min_r={min_r:.3f} min_z={path_min_z:.3f}"
            continue
        return dense, straight_collides

    detour = _fallback_detour_path(start, goal, kin=kin, checker=phys_checker)
    if detour is not None and len(detour) >= 2:
        return detour, straight_collides

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
    mid = start + 0.5 * (goal - start)
    mid[1] -= 0.25
    mid[3] += 0.15
    candidates = [
        [start, goal],
        [start, mid, goal],
    ]
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
