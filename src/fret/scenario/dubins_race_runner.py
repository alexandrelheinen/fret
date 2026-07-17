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
from contextlib import nullcontext
from dataclasses import dataclass, field, replace
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

from fret.scenario.planner_rng import deterministic_planner_rng
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
    dummy_pose_history: tuple[tuple[float, float, float], ...] = ()
    dummy_collided: bool = False
    contact_log_path: pathlib.Path | None = None
    contact_event_count: int = 0
    max_contact_force_n: float = 0.0
    penetration_violations: int = 0
    physics_metrics_path: pathlib.Path | None = None
    column_contacts_logged: bool = False


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
    rrt_goal_dwell: int = 0
    sst_goal_dwell: int = 0
    sim_time_s: float = 0.0
    max_cross_track_error_m: float = 0.0
    rrt_pose_history: list[tuple[float, float, float]] = field(
        default_factory=list
    )
    sst_pose_history: list[tuple[float, float, float]] = field(
        default_factory=list
    )
    dummy_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    dummy_heading_rad: float = 0.0
    dummy_stopped: bool = False
    dummy_pose_history: list[tuple[float, float, float]] = field(
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
        self._step_dummy_kinematic()
        self.sim_time_s += self.dt
        return self.finished

    def _step_dummy_kinematic(self) -> None:
        """Advance the straight-line dummy and record its pose."""
        self.dummy_pose, self.dummy_stopped = _step_dummy_straight_line(
            self.dummy_pose,
            heading_rad=self.dummy_heading_rad,
            stopped=self.dummy_stopped,
            goal=self.world.goal_xy,
            speed=self.vehicle_cfg.cruise_speed,
            dt=self.dt,
            occupancy=self.occupancy,
        )
        self.dummy_pose_history.append(self.dummy_pose)

    def step_physics(self, bridge: Any) -> bool:
        """Advance one physics SITL tick via MuJoCo actuators (v1.2).

        Steps each agent's kinematic Pure Pursuit reference, then drives
        velocity actuators with softened P-tracking toward that reference.
        Forward-only projection, heading-error speed gating, and goal dwell
        keep differential-drive agents from reversing into workspace bounds.

        Args:
            bridge: :class:`~fret.ros.mujoco_bridge.DubinsRaceBridgeCore`
                configured with ``physics_mode=True``.

        Returns:
            ``True`` when both agents reached the goal.
        """
        if self.finished:
            return True

        rrt_cmd, rrt_metrics = _agent_physics_velocity_command(
            self.rrt_loop,
            self.rrt_vehicle,
            self.rrt_path,
            dt=self.dt,
            agent_finished=self.rrt_finish is not None,
            max_speed=self.vehicle_cfg.max_speed,
            max_turn_rate=self.vehicle_cfg.max_turn_rate,
            occupancy=self.occupancy,
        )
        sst_cmd, sst_metrics = _agent_physics_velocity_command(
            self.sst_loop,
            self.sst_vehicle,
            self.sst_path,
            dt=self.dt,
            agent_finished=self.sst_finish is not None,
            max_speed=self.vehicle_cfg.max_speed,
            max_turn_rate=self.vehicle_cfg.max_turn_rate,
            occupancy=self.occupancy,
        )
        dummy_cmd, self.dummy_stopped = _dummy_physics_velocity_command(
            self.dummy_pose,
            heading_rad=self.dummy_heading_rad,
            stopped=self.dummy_stopped,
            speed=self.vehicle_cfg.cruise_speed,
            occupancy=self.occupancy,
        )
        commands = np.array([*rrt_cmd, *sst_cmd, *dummy_cmd], dtype=np.float64)
        bridge.step_physics(commands)
        _sync_vehicle_pose(self.rrt_vehicle, bridge.get_rrt_pose())
        _sync_vehicle_pose(self.sst_vehicle, bridge.get_sst_pose())
        dummy_qpos = bridge.get_dummy_pose()
        self.dummy_pose = (
            float(dummy_qpos[0]),
            float(dummy_qpos[1]),
            float(dummy_qpos[2]),
        )

        self.max_cross_track_error_m = max(
            self.max_cross_track_error_m,
            abs(float(rrt_metrics["cross_track_error"])),
            abs(float(sst_metrics["cross_track_error"])),
        )

        if self.rrt_finish is None:
            self.rrt_goal_dwell = _physics_goal_dwell_update(
                self.rrt_vehicle.pose,
                self.world.goal_xy,
                goal_radius=self.goal_radius,
                dwell=self.rrt_goal_dwell,
            )
            if self.rrt_goal_dwell >= _PHYSICS_GOAL_DWELL_TICKS:
                self.rrt_finish = self.sim_time_s

        if self.sst_finish is None:
            self.sst_goal_dwell = _physics_goal_dwell_update(
                self.sst_vehicle.pose,
                self.world.goal_xy,
                goal_radius=self.goal_radius,
                dwell=self.sst_goal_dwell,
            )
            if self.sst_goal_dwell >= _PHYSICS_GOAL_DWELL_TICKS:
                self.sst_finish = self.sim_time_s

        self.rrt_pose_history.append(self.rrt_vehicle.pose)
        self.sst_pose_history.append(self.sst_vehicle.pose)
        self.dummy_pose_history.append(self.dummy_pose)
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
            dummy_pose_history=tuple(self.dummy_pose_history),
            dummy_collided=self.dummy_stopped,
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


def _straight_line_heading(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
) -> float:
    """Return the heading [rad] from ``start`` toward ``goal``."""
    return math.atan2(
        float(goal[1] - start[1]),
        float(goal[0] - start[0]),
    )


def _initial_dummy_pose(world: DubinsRaceWorld) -> tuple[float, float, float]:
    """Return the straight-line dummy spawn at the arena start."""
    heading = _straight_line_heading(world.start_xy, world.goal_xy)
    return (
        float(world.start_xy[0]),
        float(world.start_xy[1]),
        heading,
    )


def _step_dummy_straight_line(
    pose: tuple[float, float, float],
    *,
    heading_rad: float,
    stopped: bool,
    goal: npt.NDArray[np.float64],
    speed: float,
    dt: float,
    occupancy: RectStructureOccupancy,
) -> tuple[tuple[float, float, float], bool]:
    """Advance the dummy along a straight corridor; stop on collision."""
    if stopped:
        return pose, True

    x, y, _ = pose
    centre = np.array([x, y], dtype=np.float64)
    if occupancy.is_occupied(centre):
        return (x, y, heading_rad), True

    dist_to_goal = math.hypot(float(goal[0]) - x, float(goal[1]) - y)
    step = float(speed) * float(dt)
    if step >= dist_to_goal:
        new_x = float(goal[0])
        new_y = float(goal[1])
    else:
        new_x = x + step * math.cos(heading_rad)
        new_y = y + step * math.sin(heading_rad)

    if occupancy.is_occupied(np.array([new_x, new_y], dtype=np.float64)):
        return (x, y, heading_rad), True

    return (new_x, new_y, heading_rad), False


def _dummy_physics_velocity_command(
    pose: tuple[float, float, float],
    *,
    heading_rad: float,
    stopped: bool,
    speed: float,
    occupancy: RectStructureOccupancy,
) -> tuple[tuple[float, float, float], bool]:
    """Drive the dummy forward along the start→goal diagonal until blocked."""
    if stopped:
        return (0.0, 0.0, 0.0), True

    x, y, yaw = pose
    if occupancy.is_occupied(np.array([x, y], dtype=np.float64)):
        return (0.0, 0.0, 0.0), True

    probe = np.array(
        [
            x + 0.35 * math.cos(heading_rad),
            y + 0.35 * math.sin(heading_rad),
        ],
        dtype=np.float64,
    )
    if occupancy.is_occupied(probe):
        return (0.0, 0.0, 0.0), True

    yaw_err = _wrap_heading(heading_rad - yaw)
    yaw_rate = max(-1.2, min(1.2, 3.5 * yaw_err))
    body_vx = float(speed) * max(0.0, math.cos(yaw_err))
    vx = body_vx * math.cos(yaw)
    vy = body_vx * math.sin(yaw)
    return (vx, vy, yaw_rate), False


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
    return float(
        min(
            vehicle_body_clearance(
                pose[0],
                pose[1],
                pose[2],
                structures,
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


_PHYSICS_KP_POSITION: float = 6.8
_PHYSICS_KP_YAW: float = 4.9
_PHYSICS_MAX_POSITION_LAG_M: float = 0.82
_PHYSICS_MAX_YAW_LAG_RAD: float = 0.65
_PHYSICS_REFERENCE_BLEND: float = 0.73
_PHYSICS_OBSTACLE_SPEED_FLOOR: float = 0.38
_PHYSICS_NEAR_OBSTACLE_INFLUENCE_SCALE: float = 3.0
_PHYSICS_PLANNING_CLEARANCE_BUMP_M: float = 0.15
_PHYSICS_GOAL_DWELL_TICKS: int = 6
_PHYSICS_FORWARD_SPEED_SCALE: float = 1.0


def _wrap_heading(angle: float) -> float:
    """Wrap a heading angle to ``[-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _physics_goal_dwell_update(
    pose: tuple[float, float, float],
    goal: npt.NDArray[np.float64],
    *,
    goal_radius: float,
    dwell: int,
) -> int:
    """Increment dwell while inside goal; reset when the agent leaves."""
    if _distance_to_goal(pose, goal) <= goal_radius:
        return dwell + 1
    return 0


def _physics_forward_blocked(
    pose: tuple[float, float, float],
    *,
    occupancy: RectStructureOccupancy,
    probe_m: float = 0.40,
) -> bool:
    """Return True when the body centre cannot advance into occupied space."""
    x, y, theta = pose
    centre = np.array([x, y], dtype=np.float64)
    if occupancy.is_occupied(centre):
        return True
    ahead = np.array(
        [
            x + probe_m * math.cos(theta),
            y + probe_m * math.sin(theta),
        ],
        dtype=np.float64,
    )
    return bool(occupancy.is_occupied(ahead))


def _agent_physics_velocity_command(
    loop: TrackingLoop,
    vehicle: Any,
    path: list[tuple[float, float]],
    *,
    dt: float,
    agent_finished: bool,
    max_speed: float,
    max_turn_rate: float,
    occupancy: RectStructureOccupancy,
    kp_position: float = _PHYSICS_KP_POSITION,
    kp_yaw: float = _PHYSICS_KP_YAW,
    max_position_lag_m: float = _PHYSICS_MAX_POSITION_LAG_M,
) -> tuple[tuple[float, float, float], dict[str, Any]]:
    """Return closed-loop physics velocity toward a kinematic PP reference."""
    if agent_finished:
        return (0.0, 0.0, 0.0), {
            "cross_track_error": 0.0,
            "heading_error": 0.0,
            "pose": vehicle.pose,
            "speed": 0.0,
            "turn_rate": 0.0,
            "curvature": 0.0,
            "repulsion_turn_rate": 0.0,
        }

    sim_x, sim_y, sim_theta = vehicle.pose
    vehicle.x = float(sim_x)
    vehicle.y = float(sim_y)
    vehicle.heading = float(sim_theta)

    metrics = loop.step(path, dt)
    ref_x, ref_y, ref_theta = vehicle.pose

    vehicle.x = float(sim_x)
    vehicle.y = float(sim_y)
    vehicle.heading = float(sim_theta)

    ref_x = sim_x + _PHYSICS_REFERENCE_BLEND * (ref_x - sim_x)
    ref_y = sim_y + _PHYSICS_REFERENCE_BLEND * (ref_y - sim_y)
    ref_theta = sim_theta + _PHYSICS_REFERENCE_BLEND * _wrap_heading(
        ref_theta - sim_theta
    )

    dx = ref_x - sim_x
    dy = ref_y - sim_y
    lag = math.hypot(dx, dy)
    if lag > max_position_lag_m:
        scale = max_position_lag_m / lag
        ref_x = sim_x + dx * scale
        ref_y = sim_y + dy * scale

    yaw_err = _wrap_heading(ref_theta - sim_theta)
    if abs(yaw_err) > _PHYSICS_MAX_YAW_LAG_RAD:
        ref_theta = sim_theta + math.copysign(
            _PHYSICS_MAX_YAW_LAG_RAD,
            yaw_err,
        )
        yaw_err = _wrap_heading(ref_theta - sim_theta)

    vx = kp_position * (ref_x - sim_x)
    vy = kp_position * (ref_y - sim_y)
    omega = kp_yaw * yaw_err

    repulsion_turn = loop._repulsion_turn_rate(sim_x, sim_y, sim_theta)
    omega += repulsion_turn

    dist, _closest = occupancy.nearest_obstacle(
        np.array([sim_x, sim_y], dtype=np.float64)
    )
    clearance = float(occupancy.clearance)
    influence_radius = _PHYSICS_NEAR_OBSTACLE_INFLUENCE_SCALE * clearance

    forward = vx * math.cos(sim_theta) + vy * math.sin(sim_theta)
    if _physics_forward_blocked(
        (sim_x, sim_y, sim_theta),
        occupancy=occupancy,
    ):
        forward = min(forward, 0.0)
    forward = max(0.0, forward) * _PHYSICS_FORWARD_SPEED_SCALE
    vx = forward * math.cos(sim_theta)
    vy = forward * math.sin(sim_theta)
    speed = forward
    if speed > max_speed:
        scale = max_speed / speed
        vx *= scale
        vy *= scale
    omega = max(-max_turn_rate, min(max_turn_rate, omega))

    workspace_margin = 2.0
    for coord in (sim_x, sim_y):
        if coord < workspace_margin:
            scale = max(0.0, coord / workspace_margin)
            vx *= scale
            vy *= scale

    if float(dist) < influence_radius:
        proximity_scale = max(
            _PHYSICS_OBSTACLE_SPEED_FLOOR,
            float(dist) / influence_radius,
        )
        vx *= proximity_scale
        vy *= proximity_scale

    return (
        (vx, vy, omega),
        {
            **metrics,
            "repulsion_turn_rate": repulsion_turn,
        },
    )


def _agent_world_velocity_command(
    loop: TrackingLoop,
    path: list[tuple[float, float]],
    *,
    agent_finished: bool,
) -> tuple[tuple[float, float, float], dict[str, Any]]:
    """Return world ``(v_x, v_y, omega)`` without integrating the vehicle."""
    if agent_finished:
        return (0.0, 0.0, 0.0), {
            "cross_track_error": 0.0,
            "heading_error": 0.0,
            "pose": loop.vehicle.pose,
            "speed": 0.0,
            "turn_rate": 0.0,
            "curvature": 0.0,
            "repulsion_turn_rate": 0.0,
        }

    pose = loop.vehicle.pose
    speed_ref = loop.cruise_speed / (
        1.0 + loop.curvature_gain * abs(loop.controller.curvature)
    )
    speed_cmd, turn_rate_cmd = loop.controller.track(pose, path, speed_ref)
    x, y, theta = pose
    repulsion = loop._repulsion_turn_rate(x, y, theta)
    turn_rate_cmd += repulsion
    vx = speed_cmd * math.cos(theta)
    vy = speed_cmd * math.sin(theta)
    metrics: dict[str, Any] = {
        "cross_track_error": loop.controller.cross_track_error,
        "heading_error": loop.controller.heading_error,
        "pose": pose,
        "speed": speed_cmd,
        "turn_rate": turn_rate_cmd,
        "curvature": loop.controller.curvature,
        "repulsion_turn_rate": repulsion,
    }
    return (vx, vy, turn_rate_cmd), metrics


def _sync_vehicle_pose(vehicle: Any, pose: npt.NDArray[np.float64]) -> None:
    """Copy simulated ``(x, y, heading)`` into a Dubins vehicle model."""
    vehicle.x = float(pose[0])
    vehicle.y = float(pose[1])
    vehicle.heading = float(pose[2])


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
        *,
        physics_mode: bool = False,
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
        if physics_mode:
            world = replace(
                world,
                clearance_margin=world.clearance_margin
                + _PHYSICS_PLANNING_CLEARANCE_BUMP_M,
            )
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

        dummy_pose = _initial_dummy_pose(world)
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
            dummy_pose=dummy_pose,
            dummy_heading_rad=dummy_pose[2],
            dummy_pose_history=[dummy_pose],
        )
        return rrt_plan, sst_plan, session

    def run(
        self,
        *,
        record_poses: bool = False,
        physics_mode: bool = False,
        contact_log_enabled: bool = False,
        planner_rng_seed: int | None = None,
    ) -> DubinsRaceRunResult:
        """Execute dual planning, simultaneous tracking, and race metrics."""
        params = self.load_parameters()
        race_timeout = float(params["race_timeout"])
        max_steps = int(race_timeout / float(params["simulation_dt"]))

        rng_ctx = (
            deterministic_planner_rng(planner_rng_seed)
            if planner_rng_seed is not None
            else nullcontext()
        )
        with rng_ctx:
            rrt_plan, sst_plan, session = self.prepare_simulation(
                physics_mode=physics_mode,
            )
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
        use_mujoco = self._sync_mujoco or physics_mode or contact_log_enabled
        if use_mujoco and _make_dubins_race_bridge_core is not None:
            physics_config = None
            bridge_cfg: dict[str, Any] | None = None
            if physics_mode or contact_log_enabled:
                from fret.ros.mujoco_bridge import (
                    _load_merged_bridge_config,
                    _resolve_config_path,
                    contact_log_config_from_bridge_yaml,
                    physics_config_from_bridge_yaml,
                )

                bridge_cfg = _load_merged_bridge_config(
                    _resolve_config_path(None)
                )
            if physics_mode:
                cfg_physics = dict(bridge_cfg or {})
                cfg_physics["physics_mode"] = True
                physics_config = physics_config_from_bridge_yaml(
                    cfg_physics,
                    "dubins",
                    physics_mode=True,
                )
            bridge = _make_dubins_race_bridge_core(
                initial_rrt=np.array(
                    session.rrt_vehicle.pose,
                    dtype=np.float64,
                ),
                initial_sst=np.array(
                    session.sst_vehicle.pose,
                    dtype=np.float64,
                ),
                initial_dummy=np.array(
                    session.dummy_pose,
                    dtype=np.float64,
                ),
                physics_config=physics_config,
            )
            if contact_log_enabled and bridge_cfg is not None:
                scenario_id = str(params.get("scenario_id", "dubins_race"))
                bridge.configure_contact_logging(
                    contact_log_config_from_bridge_yaml(
                        bridge_cfg,
                        scenario_id,
                        physics_mode=True,
                        enabled=True,
                    )
                )

        for _ in range(max_steps):
            if physics_mode and bridge is not None:
                session.step_physics(bridge)
            else:
                session.step()
                if bridge is not None:
                    bridge.set_rrt_pose(session.rrt_vehicle.pose)
                    bridge.set_sst_pose(session.sst_vehicle.pose)
                    bridge.set_dummy_pose(session.dummy_pose)
            if session.finished:
                break

        result = session.to_result(
            rrt_plan,
            sst_plan,
            race_duration_s=session.sim_time_s,
        )
        contact_log_path = None
        contact_event_count = 0
        max_contact_force_n = 0.0
        penetration_violations = 0
        physics_metrics_path = None
        column_contacts_logged = False
        if bridge is not None and bridge.contact_logger is not None:
            logger = bridge.contact_logger
            contact_log_path = logger.log_path
            contact_event_count = logger.metrics.contact_event_count
            max_contact_force_n = logger.metrics.max_contact_force_n
            penetration_violations = logger.metrics.penetration_violations
            physics_metrics_path = bridge.finalize_physics_metrics(
                max_tracking_error_m=result.max_cross_track_error_m,
            )
            column_contacts_logged = logger.column_contacts_logged()
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
                dummy_collided=result.dummy_collided,
                contact_log_path=contact_log_path,
                contact_event_count=contact_event_count,
                max_contact_force_n=max_contact_force_n,
                penetration_violations=penetration_violations,
                physics_metrics_path=physics_metrics_path,
                column_contacts_logged=column_contacts_logged,
            )
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
            rrt_pose_history=result.rrt_pose_history,
            sst_pose_history=result.sst_pose_history,
            dummy_pose_history=result.dummy_pose_history,
            dummy_collided=result.dummy_collided,
            contact_log_path=contact_log_path,
            contact_event_count=contact_event_count,
            max_contact_force_n=max_contact_force_n,
            penetration_violations=penetration_violations,
            physics_metrics_path=physics_metrics_path,
            column_contacts_logged=column_contacts_logged,
        )
