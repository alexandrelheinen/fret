"""Dubins dual-robot race E2E orchestrator (SC-v11 / v1.1).

Runs the ARCO vehicle race pipeline in pure Python:

  Column YAML → KDTreeOccupancy → RRT* + SST planners
  → TrajectoryPruner → Pure Pursuit + DubinsVehicle × 2
  → optional MuJoCo state sync

Validates releases.md acceptance criteria V11-1 – V11-3 in CI without ROS.
"""

from __future__ import annotations

import math
import pathlib
import time
from dataclasses import dataclass
from typing import Any, Literal, cast

import numpy as np
import numpy.typing as npt
import yaml
from arco.control.tracking import TrackingLoop
from arco.mapping import KDTreeOccupancy
from arco.planning.continuous import RRTPlanner, SSTPlanner, TrajectoryPruner
from arco.simulator.sim.tracking import VehicleConfig, build_vehicle_sim

from fret.planning.dubins_obstacles import (
    build_race_occupancy,
    default_obstacle_file,
    load_dubins_race_world,
)
from fret.sitl_config import load_scenario_parameters

_make_dubins_race_bridge_core: Any
try:
    from fret.ros.mujoco_bridge import (
        make_dubins_race_bridge_core as _make_dubins_race_bridge_core,
    )
except ImportError:  # pragma: no cover
    _make_dubins_race_bridge_core = cast(Any, None)

_DEFAULT_SCENARIO = (
    pathlib.Path(__file__).resolve().parents[1]
    / "config"
    / "scenarios"
    / "dubins_race.yml"
)
_DEFAULT_CONTROLLER = (
    pathlib.Path(__file__).resolve().parents[1]
    / "config"
    / "controllers"
    / "dubins.yml"
)

AgentName = Literal["rrt_star", "sst"]


@dataclass(frozen=True)
class AgentPlanResult:
    """Planning outcome for one race agent."""

    agent: AgentName
    path_found: bool
    path: list[npt.NDArray[np.float64]]
    path_length_m: float
    planner_time_s: float
    node_count: int


@dataclass(frozen=True)
class DubinsRaceRunResult:
    """Outcome of a Dubins race E2E run."""

    rrt_plan: AgentPlanResult
    sst_plan: AgentPlanResult
    rrt_time_to_goal_s: float | None
    sst_time_to_goal_s: float | None
    race_duration_s: float
    winner: AgentName | None
    both_reached_goal: bool
    max_cross_track_error_m: float


def _polyline_length(path: list[npt.NDArray[np.float64]]) -> float:
    if len(path) < 2:
        return 0.0
    return float(
        sum(
            float(np.linalg.norm(path[i + 1][:2] - path[i][:2]))
            for i in range(len(path) - 1)
        )
    )


def _load_controller_params(path: pathlib.Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        return {}
    for section in data.values():
        if isinstance(section, dict):
            params = section.get("ros__parameters")
            if isinstance(params, dict):
                return dict(params)
    return {}


def _vehicle_config(ctrl: dict[str, Any]) -> VehicleConfig:
    return VehicleConfig(
        max_speed=float(ctrl.get("max_speed", 4.5)),
        min_speed=float(ctrl.get("min_speed", 0.0)),
        cruise_speed=float(ctrl.get("cruise_speed", 2.8)),
        lookahead_distance=float(ctrl.get("lookahead_distance", 2.5)),
        goal_radius=float(ctrl.get("goal_radius", 0.75)),
        max_turn_rate=math.radians(float(ctrl.get("max_turn_rate_deg", 90.0))),
        max_acceleration=float(ctrl.get("max_acceleration", 4.9)),
        max_turn_rate_dot=math.radians(
            float(ctrl.get("max_turn_rate_dot_deg", 360.0))
        ),
        curvature_gain=float(ctrl.get("curvature_gain", 0.5)),
        repulsion_gain=float(ctrl.get("repulsion_gain", 1.2)),
    )


def _spawn_positions(
    world_start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    lateral_offset: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return RRT* and SST spawn ``(x, y)`` with lateral separation."""
    dx = float(goal[0] - world_start[0])
    dy = float(goal[1] - world_start[1])
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        nx, ny = 0.0, 1.0
    else:
        nx, ny = -dy / norm, dx / norm
    half = 0.5 * lateral_offset
    rrt = (
        float(world_start[0] - half * nx),
        float(world_start[1] - half * ny),
    )
    sst = (
        float(world_start[0] + half * nx),
        float(world_start[1] + half * ny),
    )
    return rrt, sst


def _plan_path(
    planner_kind: AgentName,
    occupancy: KDTreeOccupancy,
    bounds: list[tuple[float, float]],
    start_xy: tuple[float, float],
    goal_xy: npt.NDArray[np.float64],
    planner_cfg: dict[str, Any],
) -> AgentPlanResult:
    start = np.array(start_xy, dtype=np.float64)
    goal = np.asarray(goal_xy[:2], dtype=np.float64)
    t0 = time.monotonic()

    common = dict(
        occupancy=occupancy,
        bounds=bounds,
        step_size=float(planner_cfg.get("step_size", 0.35)),
        goal_tolerance=float(planner_cfg.get("goal_tolerance", 0.55)),
        collision_check_count=int(
            planner_cfg.get("collision_check_count", 10)
        ),
        goal_bias=float(planner_cfg.get("goal_bias", 0.12)),
        early_stop=True,
    )

    if planner_kind == "rrt_star":
        planner = RRTPlanner(
            max_sample_count=int(
                planner_cfg.get("rrt_max_sample_count", 4000)
            ),
            **common,
        )
        nodes, _parent, path = planner.get_tree(start.copy(), goal.copy())
    else:
        planner = SSTPlanner(
            max_sample_count=int(
                planner_cfg.get("sst_max_sample_count", 4000)
            ),
            witness_radius=float(planner_cfg.get("witness_radius", 0.22)),
            **common,
        )
        nodes, _parent, path = planner.get_tree(start.copy(), goal.copy())

    elapsed = time.monotonic() - t0
    if path is None:
        return AgentPlanResult(
            agent=planner_kind,
            path_found=False,
            path=[],
            path_length_m=0.0,
            planner_time_s=elapsed,
            node_count=len(nodes),
        )

    dense = [np.asarray(p, dtype=np.float64) for p in path]
    if bool(planner_cfg.get("enable_pruning", True)):
        step = float(common["step_size"])
        pruner = TrajectoryPruner(
            occupancy,
            step_size=np.array([step, step], dtype=np.float64),
            collision_check_count=common["collision_check_count"],
        )
        dense = [np.asarray(p, dtype=np.float64) for p in pruner.prune(dense)]

    return AgentPlanResult(
        agent=planner_kind,
        path_found=True,
        path=dense,
        path_length_m=_polyline_length(dense),
        planner_time_s=elapsed,
        node_count=len(nodes),
    )


def _path_to_tuples(
    path: list[npt.NDArray[np.float64]],
) -> list[tuple[float, float]]:
    return [(float(p[0]), float(p[1])) for p in path]


def _distance_to_goal(
    pose: tuple[float, float, float], goal: npt.NDArray[np.float64]
) -> float:
    return float(
        math.hypot(pose[0] - float(goal[0]), pose[1] - float(goal[1]))
    )


class DubinsRaceRunner:
    """Pure-Python E2E orchestrator for the Dubins warehouse race.

    Args:
        scenario_path: Path to ``dubins_race.yml``.
        obstacle_path: Optional obstacle YAML override.
        controller_config_path: Path to ``dubins.yml`` controller parameters.
        sync_mujoco: When True, mirror vehicle poses into MuJoCo if available.
    """

    def __init__(
        self,
        scenario_path: str | pathlib.Path | None = None,
        obstacle_path: str | pathlib.Path | None = None,
        controller_config_path: str | pathlib.Path | None = None,
        *,
        sync_mujoco: bool = False,
    ) -> None:
        self._scenario_path = pathlib.Path(
            scenario_path if scenario_path is not None else _DEFAULT_SCENARIO
        )
        self._obstacle_path = (
            pathlib.Path(obstacle_path) if obstacle_path is not None else None
        )
        self._controller_config_path = pathlib.Path(
            controller_config_path
            if controller_config_path is not None
            else _DEFAULT_CONTROLLER
        )
        self._sync_mujoco = sync_mujoco

    @property
    def scenario_path(self) -> pathlib.Path:
        """Resolved scenario YAML path."""
        return self._scenario_path

    def load_parameters(self) -> dict[str, Any]:
        """Load flat scenario parameters from YAML."""
        return load_scenario_parameters(self._scenario_path)

    def run(self) -> DubinsRaceRunResult:
        """Execute dual planning, simultaneous tracking, and race metrics."""
        params = self.load_parameters()
        ctrl = _load_controller_params(self._controller_config_path)
        vehicle_cfg = _vehicle_config(ctrl)
        dt = float(params.get("simulation_dt", 0.05))
        race_timeout = float(params.get("race_timeout", 90.0))
        max_steps = int(race_timeout / dt)

        world = load_dubins_race_world(
            self._obstacle_path or default_obstacle_file()
        )
        occupancy = build_race_occupancy(world)
        bounds = [
            tuple(b)
            for b in world.planner.get("bounds", world.workspace_bounds[:2])
        ]

        rrt_spawn, sst_spawn = _spawn_positions(
            world.start_xy,
            world.goal_xy,
            world.agent_lateral_offset,
        )

        rrt_plan = _plan_path(
            "rrt_star",
            occupancy,
            bounds,
            rrt_spawn,
            world.goal_xy,
            world.planner,
        )
        sst_plan = _plan_path(
            "sst",
            occupancy,
            bounds,
            sst_spawn,
            world.goal_xy,
            world.planner,
        )

        empty_result = DubinsRaceRunResult(
            rrt_plan=rrt_plan,
            sst_plan=sst_plan,
            rrt_time_to_goal_s=None,
            sst_time_to_goal_s=None,
            race_duration_s=0.0,
            winner=None,
            both_reached_goal=False,
            max_cross_track_error_m=0.0,
        )
        if not rrt_plan.path_found or not sst_plan.path_found:
            return empty_result

        rrt_path = _path_to_tuples(rrt_plan.path)
        sst_path = _path_to_tuples(sst_plan.path)

        rrt_vehicle, rrt_loop = build_vehicle_sim(
            rrt_path, vehicle_cfg, occupancy=occupancy
        )
        sst_vehicle, sst_loop = build_vehicle_sim(
            sst_path, vehicle_cfg, occupancy=occupancy
        )

        bridge = None
        if self._sync_mujoco and _make_dubins_race_bridge_core is not None:
            bridge = _make_dubins_race_bridge_core(
                initial_rrt=np.array(
                    [rrt_vehicle.x, rrt_vehicle.y, rrt_vehicle.heading],
                    dtype=np.float64,
                ),
                initial_sst=np.array(
                    [sst_vehicle.x, sst_vehicle.y, sst_vehicle.heading],
                    dtype=np.float64,
                ),
            )

        rrt_finish: float | None = None
        sst_finish: float | None = None
        max_cte = 0.0
        t0 = time.monotonic()

        for step_idx in range(max_steps):
            sim_t = step_idx * dt

            if rrt_finish is None:
                metrics = rrt_loop.step(rrt_path, dt)
                max_cte = max(
                    max_cte, abs(float(metrics["cross_track_error"]))
                )
                if (
                    _distance_to_goal(rrt_vehicle.pose, world.goal_xy)
                    <= vehicle_cfg.goal_radius
                ):
                    rrt_finish = sim_t

            if sst_finish is None:
                metrics = sst_loop.step(sst_path, dt)
                max_cte = max(
                    max_cte, abs(float(metrics["cross_track_error"]))
                )
                if (
                    _distance_to_goal(sst_vehicle.pose, world.goal_xy)
                    <= vehicle_cfg.goal_radius
                ):
                    sst_finish = sim_t

            if bridge is not None:
                bridge.set_rrt_pose(rrt_vehicle.pose)
                bridge.set_sst_pose(sst_vehicle.pose)

            if rrt_finish is not None and sst_finish is not None:
                break

        race_duration = time.monotonic() - t0
        both = rrt_finish is not None and sst_finish is not None
        winner: AgentName | None = None
        if both and rrt_finish is not None and sst_finish is not None:
            if rrt_finish < sst_finish:
                winner = "rrt_star"
            elif sst_finish < rrt_finish:
                winner = "sst"

        return DubinsRaceRunResult(
            rrt_plan=rrt_plan,
            sst_plan=sst_plan,
            rrt_time_to_goal_s=rrt_finish,
            sst_time_to_goal_s=sst_finish,
            race_duration_s=race_duration,
            winner=winner,
            both_reached_goal=both,
            max_cross_track_error_m=max_cte,
        )
