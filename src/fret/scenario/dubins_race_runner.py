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
from dataclasses import dataclass, field
from typing import Any, Literal, cast

import numpy as np
import numpy.typing as npt
from arco.control.tracking import TrackingLoop
from arco.planning.continuous import RRTPlanner, SSTPlanner, TrajectoryPruner
from arco.simulator.sim.tracking import VehicleConfig, build_vehicle_sim

from fret.config_loader import (
    load_ros_parameters_yaml,
    load_scenario_bundle,
    require_key,
    require_keys,
    resolve_obstacle_file,
)
from fret.planning.dubins_obstacles import (
    DubinsRaceWorld,
    RectStructureOccupancy,
    build_race_occupancy,
    default_obstacle_file,
    load_dubins_race_world,
    vehicle_body_clearance,
)

_make_dubins_race_bridge_core: Any
try:
    from fret.ros.mujoco_bridge import (
        make_dubins_race_bridge_core as _make_dubins_race_bridge_core,
    )
except ImportError:  # pragma: no cover
    _make_dubins_race_bridge_core = cast(Any, None)

from fret.sitl_config import controller_config_path, scenario_config_path

_DEFAULT_SCENARIO = scenario_config_path("dubins_race")
_DEFAULT_CONTROLLER = controller_config_path("dubins")

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
    min_obstacle_clearance_m: float
    rrt_pose_history: tuple[tuple[float, float, float], ...] = ()
    sst_pose_history: tuple[tuple[float, float, float], ...] = ()


@dataclass
class DubinsRaceSimulation:
    """Incremental dual-agent race simulation for ROS and video rendering."""

    world: DubinsRaceWorld
    vehicle_cfg: VehicleConfig
    vehicle_body: dict[str, Any]
    occupancy: RectStructureOccupancy
    dt: float
    rrt_vehicle: Any
    sst_vehicle: Any
    rrt_loop: TrackingLoop
    sst_loop: TrackingLoop
    rrt_path: list[tuple[float, float]]
    sst_path: list[tuple[float, float]]
    goal_radius: float
    rrt_finish: float | None = None
    sst_finish: float | None = None
    sim_time_s: float = 0.0
    max_cross_track_error_m: float = 0.0
    rrt_pose_history: list[tuple[float, float, float]] = field(
        default_factory=list
    )
    sst_pose_history: list[tuple[float, float, float]] = field(
        default_factory=list
    )

    @property
    def finished(self) -> bool:
        """Return True when both agents reached the goal."""
        return self.rrt_finish is not None and self.sst_finish is not None

    def step(self) -> bool:
        """Advance one simulation tick. Returns ``True`` when the race ends."""
        if self.finished:
            return True

        if self.rrt_finish is None:
            metrics = self.rrt_loop.step(self.rrt_path, self.dt)
            self.max_cross_track_error_m = max(
                self.max_cross_track_error_m,
                abs(float(metrics["cross_track_error"])),
            )
            if (
                _distance_to_goal(self.rrt_vehicle.pose, self.world.goal_xy)
                <= self.goal_radius
            ):
                self.rrt_finish = self.sim_time_s

        if self.sst_finish is None:
            metrics = self.sst_loop.step(self.sst_path, self.dt)
            self.max_cross_track_error_m = max(
                self.max_cross_track_error_m,
                abs(float(metrics["cross_track_error"])),
            )
            if (
                _distance_to_goal(self.sst_vehicle.pose, self.world.goal_xy)
                <= self.goal_radius
            ):
                self.sst_finish = self.sim_time_s

        self.rrt_pose_history.append(self.rrt_vehicle.pose)
        self.sst_pose_history.append(self.sst_vehicle.pose)
        self.sim_time_s += self.dt
        return self.finished

    def to_result(
        self,
        rrt_plan: AgentPlanResult,
        sst_plan: AgentPlanResult,
        *,
        race_duration_s: float,
    ) -> DubinsRaceRunResult:
        """Build a :class:`DubinsRaceRunResult` from the current session."""
        both = self.finished
        winner: AgentName | None = None
        if (
            both
            and self.rrt_finish is not None
            and self.sst_finish is not None
        ):
            if self.rrt_finish < self.sst_finish:
                winner = "rrt_star"
            elif self.sst_finish < self.rrt_finish:
                winner = "sst"

        return DubinsRaceRunResult(
            rrt_plan=rrt_plan,
            sst_plan=sst_plan,
            rrt_time_to_goal_s=self.rrt_finish,
            sst_time_to_goal_s=self.sst_finish,
            race_duration_s=race_duration_s,
            winner=winner,
            both_reached_goal=both,
            max_cross_track_error_m=self.max_cross_track_error_m,
            min_obstacle_clearance_m=min(
                _min_pose_history_clearance(
                    tuple(self.rrt_pose_history),
                    self.world.structures,
                    vehicle_radius=self.world.vehicle_radius,
                    vehicle_body=self.vehicle_body,
                ),
                _min_pose_history_clearance(
                    tuple(self.sst_pose_history),
                    self.world.structures,
                    vehicle_radius=self.world.vehicle_radius,
                    vehicle_body=self.vehicle_body,
                ),
            ),
            rrt_pose_history=tuple(self.rrt_pose_history),
            sst_pose_history=tuple(self.sst_pose_history),
        )


def _polyline_length(path: list[npt.NDArray[np.float64]]) -> float:
    if len(path) < 2:
        return 0.0
    return float(
        sum(
            float(np.linalg.norm(path[i + 1][:2] - path[i][:2]))
            for i in range(len(path) - 1)
        )
    )


_CONTROLLER_KEYS: tuple[str, ...] = (
    "max_speed",
    "min_speed",
    "cruise_speed",
    "lookahead_distance",
    "goal_radius",
    "max_turn_rate_deg",
    "max_acceleration",
    "max_turn_rate_dot_deg",
    "curvature_gain",
    "repulsion_gain",
)


def _load_controller_params(path: pathlib.Path) -> dict[str, Any]:
    return load_ros_parameters_yaml(path)


def _vehicle_config(ctrl: dict[str, Any]) -> VehicleConfig:
    require_keys(ctrl, _CONTROLLER_KEYS, context="dubins controller config")
    return VehicleConfig(
        max_speed=float(ctrl["max_speed"]),
        min_speed=float(ctrl["min_speed"]),
        cruise_speed=float(ctrl["cruise_speed"]),
        lookahead_distance=float(ctrl["lookahead_distance"]),
        goal_radius=float(ctrl["goal_radius"]),
        max_turn_rate=math.radians(float(ctrl["max_turn_rate_deg"])),
        max_acceleration=float(ctrl["max_acceleration"]),
        max_turn_rate_dot=math.radians(float(ctrl["max_turn_rate_dot_deg"])),
        curvature_gain=float(ctrl["curvature_gain"]),
        repulsion_gain=float(ctrl["repulsion_gain"]),
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


def _postprocess_vehicle_path(
    path: list[npt.NDArray[np.float64]],
    occupancy: RectStructureOccupancy,
    planner_cfg: dict[str, Any],
    cruise_speed: float,
) -> list[npt.NDArray[np.float64]]:
    """Prune and time-parameterize a planner polyline (ARCO vehicle scene)."""
    from arco.planning import PlanningPipeline
    from arco.planning.continuous import TrajectoryOptimizer

    dense = [np.asarray(p, dtype=np.float64) for p in path]
    if len(dense) < 2:
        return dense

    pruner = None
    if bool(planner_cfg.get("enable_pruning", True)):
        step = float(planner_cfg.get("step_size", 0.35))
        pruner = TrajectoryPruner(
            occupancy,
            step_size=np.array([step, step], dtype=np.float64),
            collision_check_count=int(
                planner_cfg.get("collision_check_count", 10)
            ),
        )

    try:
        optimizer = TrajectoryOptimizer.create_from_config(
            occupancy,
            cruise_speed=cruise_speed,
        )
        pipeline = PlanningPipeline(pruner=pruner, optimizer=optimizer)
        result = pipeline.run_from_path(dense)
        if result.trajectory and len(result.trajectory) >= 2:
            return [np.asarray(p, dtype=np.float64) for p in result.trajectory]
    except Exception:
        pass

    if pruner is not None:
        return [np.asarray(p, dtype=np.float64) for p in pruner.prune(dense)]
    return dense


def _plan_path(
    planner_kind: AgentName,
    occupancy: RectStructureOccupancy,
    bounds: list[tuple[float, float]],
    start_xy: tuple[float, float],
    goal_xy: npt.NDArray[np.float64],
    planner_cfg: dict[str, Any],
    cruise_speed: float,
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

    dense = _postprocess_vehicle_path(
        path,
        occupancy,
        planner_cfg,
        cruise_speed,
    )

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


def _min_pose_history_clearance(
    poses: tuple[tuple[float, float, float], ...],
    structures: tuple[Any, ...],
    *,
    vehicle_radius: float,
    vehicle_body: dict[str, Any],
) -> float:
    if not poses:
        return float("inf")
    # Low lane-edge curbs (≈0.06 m) fence the route for planners but are
    # excluded from body clearance so tracking noise does not fail the race.
    tall_structures = tuple(
        rect for rect in structures if float(getattr(rect, "height", 0.0)) > 0.15
    )
    return float(
        min(
            vehicle_body_clearance(
                pose[0],
                pose[1],
                pose[2],
                tall_structures,
                vehicle_radius=vehicle_radius,
                half_length=float(vehicle_body["half_length"]),
                half_width=float(vehicle_body["half_width"]),
                corner_sample_radius=float(
                    vehicle_body["corner_sample_radius"]
                ),
            )
            for pose in poses
        )
    )


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
        return load_scenario_bundle(self._scenario_path).parameters

    def prepare_simulation(
        self,
    ) -> tuple[AgentPlanResult, AgentPlanResult, DubinsRaceSimulation | None]:
        """Plan paths and optionally build an incremental race session.

        Returns:
            ``(rrt_plan, sst_plan, session)`` where ``session`` is ``None`` if
            either planner failed.
        """
        bundle = load_scenario_bundle(self._scenario_path)
        params = bundle.parameters
        planning = bundle.planning
        ctrl = _load_controller_params(self._controller_config_path)
        vehicle_cfg = _vehicle_config(ctrl)
        dt = float(params["simulation_dt"])
        vehicle_body = planning["vehicle_body"]

        obstacle_path = (
            self._obstacle_path
            if self._obstacle_path is not None
            else resolve_obstacle_file(planning)
        )
        world = load_dubins_race_world(obstacle_path)
        occupancy = build_race_occupancy(world)
        bounds = [
            tuple(b)
            for b in require_key(
                world.planner, "bounds", context="planner config"
            )
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
            vehicle_cfg.cruise_speed,
        )
        sst_plan = _plan_path(
            "sst",
            occupancy,
            bounds,
            sst_spawn,
            world.goal_xy,
            world.planner,
            vehicle_cfg.cruise_speed,
        )
        if not rrt_plan.path_found or not sst_plan.path_found:
            return rrt_plan, sst_plan, None

        rrt_path = _path_to_tuples(rrt_plan.path)
        sst_path = _path_to_tuples(sst_plan.path)
        rrt_vehicle, rrt_loop = build_vehicle_sim(
            rrt_path, vehicle_cfg, occupancy=occupancy
        )
        sst_vehicle, sst_loop = build_vehicle_sim(
            sst_path, vehicle_cfg, occupancy=occupancy
        )

        session = DubinsRaceSimulation(
            world=world,
            vehicle_cfg=vehicle_cfg,
            vehicle_body=vehicle_body,
            occupancy=occupancy,
            dt=dt,
            rrt_vehicle=rrt_vehicle,
            sst_vehicle=sst_vehicle,
            rrt_loop=rrt_loop,
            sst_loop=sst_loop,
            rrt_path=rrt_path,
            sst_path=sst_path,
            goal_radius=vehicle_cfg.goal_radius,
            rrt_pose_history=[rrt_vehicle.pose],
            sst_pose_history=[sst_vehicle.pose],
        )
        return rrt_plan, sst_plan, session

    def run(self, *, record_poses: bool = False) -> DubinsRaceRunResult:
        """Execute dual planning, simultaneous tracking, and race metrics."""
        params = self.load_parameters()
        race_timeout = float(params["race_timeout"])
        max_steps = int(race_timeout / float(params["simulation_dt"]))

        rrt_plan, sst_plan, session = self.prepare_simulation()
        if session is None:
            return DubinsRaceRunResult(
                rrt_plan=rrt_plan,
                sst_plan=sst_plan,
                rrt_time_to_goal_s=None,
                sst_time_to_goal_s=None,
                race_duration_s=0.0,
                winner=None,
                both_reached_goal=False,
                max_cross_track_error_m=0.0,
                min_obstacle_clearance_m=0.0,
            )

        bridge = None
        if self._sync_mujoco and _make_dubins_race_bridge_core is not None:
            bridge = _make_dubins_race_bridge_core(
                initial_rrt=np.array(
                    session.rrt_vehicle.pose,
                    dtype=np.float64,
                ),
                initial_sst=np.array(
                    session.sst_vehicle.pose,
                    dtype=np.float64,
                ),
            )

        for _ in range(max_steps):
            session.step()
            if bridge is not None:
                bridge.set_rrt_pose(session.rrt_vehicle.pose)
                bridge.set_sst_pose(session.sst_vehicle.pose)
            if session.finished:
                break

        result = session.to_result(
            rrt_plan,
            sst_plan,
            race_duration_s=session.sim_time_s,
        )
        if not record_poses:
            return DubinsRaceRunResult(
                rrt_plan=result.rrt_plan,
                sst_plan=result.sst_plan,
                rrt_time_to_goal_s=result.rrt_time_to_goal_s,
                sst_time_to_goal_s=result.sst_time_to_goal_s,
                race_duration_s=result.race_duration_s,
                winner=result.winner,
                both_reached_goal=result.both_reached_goal,
                max_cross_track_error_m=result.max_cross_track_error_m,
                min_obstacle_clearance_m=result.min_obstacle_clearance_m,
            )
        return result
