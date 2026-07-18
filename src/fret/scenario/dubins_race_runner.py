"""Dubins dual-robot race E2E orchestrator (SC-v11 / v1.1).

Runs the ARCO vehicle race pipeline in pure Python:

  Column YAML → KDTreeOccupancy → RRT* + SST planners
  → TrajectoryPruner → path-following MPC (RRT*/SST) + DubinsVehicle
  → grey dummy keeps a Pure Pursuit foil (no obstacle avoidance)
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
from arco.control.mpc import MPCTrackingLoop, PathFollowingMPCConfig
from arco.control.tracking import TrackingLoop
from arco.planning.continuous import RRTPlanner, SSTPlanner, TrajectoryPruner
from arco.simulator.sim.tracking import (
    VehicleConfig,
    build_vehicle_mpc_sim,
    build_vehicle_sim,
)

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
    rrt_collided: bool = False
    sst_collided: bool = False
    rrt_collision_force_n: float = 0.0
    sst_collision_force_n: float = 0.0
    contact_log_path: pathlib.Path | None = None
    contact_event_count: int = 0
    max_contact_force_n: float = 0.0
    penetration_violations: int = 0
    physics_metrics_path: pathlib.Path | None = None
    column_contacts_logged: bool = False


# Collision monitor: stop an agent's control loop after a real
# impact instead of pre-emptively blocking motion near obstacles.
# Primary signal is an actual MuJoCo contact force; the acceleration-spike
# threshold is a fallback for bounces that leave no contact at the tick
# boundary. TB3 burger mass ~1 kg, so 1 N is a light bump, not sensor
# noise; 3 m/s^2 is 6x the nominal max_acceleration (0.5 m/s^2) in
# dubins.yml, i.e. well beyond any commanded speed change.
_COLLISION_FORCE_THRESHOLD_N: float = 1.0
_COLLISION_DECEL_THRESHOLD_M_S2: float = 3.0


@dataclass
class _CollisionMonitor:
    """Sticky post-hoc collision flag for one physics-mode agent.

    Primary signal: a real MuJoCo contact force between the agent's
    collision geom and an obstacle geom (always computed in
    :meth:`~fret.ros.mujoco_bridge.DubinsRaceBridgeCore.step_physics`,
    independent of JSONL contact logging). Fallback signal: a sudden
    *loss* of realized speed between ticks too large to be explained by
    any commanded deceleration — catches a fast bounce that leaves no
    contact at the tick boundary where forces are sampled. Deliberately
    ignores speed *increases* (e.g. accelerating from a stop toward
    cruise_speed is normal, not a crash). Once latched, stays latched —
    the control loop stops for the rest of the run instead of retrying
    the same crash.
    """

    collided: bool = False
    peak_force_n: float = 0.0
    last_speed_m_s: float = 0.0

    def update(
        self,
        *,
        prev_pose: tuple[float, float, float],
        new_pose: tuple[float, float, float],
        contact_force_n: float,
        dt: float,
    ) -> bool:
        """Fold in one physics tick; return the (possibly newly set) flag."""
        self.peak_force_n = max(self.peak_force_n, contact_force_n)
        if self.collided:
            return True

        speed_now = math.hypot(
            new_pose[0] - prev_pose[0], new_pose[1] - prev_pose[1]
        ) / max(dt, 1e-9)
        decel_m_s2 = (self.last_speed_m_s - speed_now) / max(dt, 1e-9)
        self.last_speed_m_s = speed_now

        if (
            contact_force_n > _COLLISION_FORCE_THRESHOLD_N
            or decel_m_s2 > _COLLISION_DECEL_THRESHOLD_M_S2
        ):
            self.collided = True
        return self.collided


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
    dummy_vehicle: Any
    rrt_loop: MPCTrackingLoop
    sst_loop: MPCTrackingLoop
    dummy_loop: TrackingLoop
    rrt_path: list[tuple[float, float]]
    sst_path: list[tuple[float, float]]
    dummy_path: list[tuple[float, float]]
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
    dummy_stopped: bool = False
    dummy_pose_history: list[tuple[float, float, float]] = field(
        default_factory=list
    )
    rrt_collision: _CollisionMonitor = field(default_factory=_CollisionMonitor)
    sst_collision: _CollisionMonitor = field(default_factory=_CollisionMonitor)

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
        """Advance the dummy with path tracking on the straight start→goal line."""
        if self.dummy_stopped:
            self.dummy_pose_history.append(self.dummy_pose)
            return
        cmd, stopped = _dummy_path_tracker_command(
            self.dummy_pose,
            path=self.dummy_path,
            stopped=self.dummy_stopped,
            speed=self.vehicle_cfg.cruise_speed,
            max_turn_rate=self.vehicle_cfg.max_turn_rate,
            lookahead_distance=self.vehicle_cfg.lookahead_distance,
            occupancy=self.occupancy,
        )
        self.dummy_stopped = stopped
        if stopped:
            self.dummy_pose_history.append(self.dummy_pose)
            return
        vx, vy, omega = cmd
        x, y, yaw = self.dummy_pose
        yaw = _wrap_heading(yaw + float(omega) * self.dt)
        x = x + float(vx) * self.dt
        y = y + float(vy) * self.dt
        self.dummy_pose = (x, y, yaw)
        self.dummy_vehicle.x = x
        self.dummy_vehicle.y = y
        self.dummy_vehicle.heading = yaw
        self.dummy_pose_history.append(self.dummy_pose)

    def step_physics(self, bridge: Any) -> bool:
        """Advance one physics SITL tick via MuJoCo actuators (v1.2).

        Steps each planner agent with ARCO path-following MPC (tracking +
        obstacle barriers inside the optimizer), then drives velocity
        actuators with the MPC body twist. The grey dummy keeps its
        Pure Pursuit foil with no repulsion. Goal dwell keeps
        differential-drive agents from reversing into workspace bounds.
        There is no pre-emptive occupancy-based motion block — a post-hoc
        :class:`_CollisionMonitor` per agent stops the control loop only
        after a real impact is detected, so a planning/tracking bug that
        drives an agent into an obstacle surfaces as an actual collision
        rather than being silently masked.

        Args:
            bridge: :class:`~fret.ros.mujoco_bridge.DubinsRaceBridgeCore`
                configured with ``physics_mode=True``.

        Returns:
            ``True`` when both agents reached the goal.
        """
        if self.finished:
            return True

        rrt_prev_pose = self.rrt_vehicle.pose
        sst_prev_pose = self.sst_vehicle.pose

        rrt_cmd, rrt_metrics = _agent_physics_velocity_command(
            self.rrt_loop,
            self.rrt_vehicle,
            self.rrt_path,
            dt=self.dt,
            agent_finished=(self.rrt_finish is not None)
            or self.rrt_collision.collided,
            max_speed=self.vehicle_cfg.max_speed,
            max_turn_rate=self.vehicle_cfg.max_turn_rate,
        )
        sst_cmd, sst_metrics = _agent_physics_velocity_command(
            self.sst_loop,
            self.sst_vehicle,
            self.sst_path,
            dt=self.dt,
            agent_finished=(self.sst_finish is not None)
            or self.sst_collision.collided,
            max_speed=self.vehicle_cfg.max_speed,
            max_turn_rate=self.vehicle_cfg.max_turn_rate,
        )
        if self.dummy_stopped:
            dummy_cmd = (0.0, 0.0, 0.0)
        else:
            dummy_cmd, self.dummy_stopped = _dummy_path_tracker_command(
                self.dummy_pose,
                path=self.dummy_path,
                stopped=self.dummy_stopped,
                speed=self.vehicle_cfg.cruise_speed,
                max_turn_rate=self.vehicle_cfg.max_turn_rate,
                lookahead_distance=self.vehicle_cfg.lookahead_distance,
                occupancy=self.occupancy,
            )
        # Concurrent race: blue / green / grey command + step in one physics tick.
        commands = np.array([*rrt_cmd, *sst_cmd, *dummy_cmd], dtype=np.float64)
        bridge.step_physics(commands)
        rrt_pose = bridge.get_rrt_pose()
        sst_pose = bridge.get_sst_pose()
        _sync_vehicle_pose(self.rrt_vehicle, rrt_pose)
        _sync_vehicle_pose(self.sst_vehicle, sst_pose)
        dummy_qpos = bridge.get_dummy_pose()
        _sync_vehicle_pose(self.dummy_vehicle, dummy_qpos)
        self.dummy_pose = (
            float(dummy_qpos[0]),
            float(dummy_qpos[1]),
            float(dummy_qpos[2]),
        )
        if (not self.dummy_stopped) and self.occupancy.is_occupied(
            np.array(
                [self.dummy_pose[0], self.dummy_pose[1]],
                dtype=np.float64,
            )
        ):
            self.dummy_stopped = True

        rrt_force_n, sst_force_n, _dummy_force_n = (
            bridge.get_collision_forces_n()
        )
        # Skip agents that already reached the goal: their command is
        # forced to zero this tick, and the resulting braking deceleration
        # is not a collision.
        if self.rrt_finish is None:
            self.rrt_collision.update(
                prev_pose=rrt_prev_pose,
                new_pose=(
                    float(rrt_pose[0]),
                    float(rrt_pose[1]),
                    float(rrt_pose[2]),
                ),
                contact_force_n=rrt_force_n,
                dt=self.dt,
            )
        if self.sst_finish is None:
            self.sst_collision.update(
                prev_pose=sst_prev_pose,
                new_pose=(
                    float(sst_pose[0]),
                    float(sst_pose[1]),
                    float(sst_pose[2]),
                ),
                contact_force_n=sst_force_n,
                dt=self.dt,
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
            rrt_collided=self.rrt_collision.collided,
            sst_collided=self.sst_collision.collided,
            rrt_collision_force_n=self.rrt_collision.peak_force_n,
            sst_collision_force_n=self.sst_collision.peak_force_n,
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


def _mpc_config(ctrl: dict[str, Any]) -> PathFollowingMPCConfig:
    """Load path-following MPC weights; cruise comes from vehicle config."""
    base = PathFollowingMPCConfig.create_from_config(
        cruise_speed=float(ctrl["cruise_speed"])
    )
    mpc = ctrl.get("mpc")
    if not isinstance(mpc, dict):
        return base
    return PathFollowingMPCConfig(
        horizon_step_count=int(
            mpc.get("horizon_step_count", base.horizon_step_count)
        ),
        dt=float(mpc.get("dt", base.dt)),
        cruise_speed=float(ctrl["cruise_speed"]),
        weight_contour=float(mpc.get("weight_contour", base.weight_contour)),
        weight_heading=float(mpc.get("weight_heading", base.weight_heading)),
        weight_progress=float(
            mpc.get("weight_progress", base.weight_progress)
        ),
        weight_control=float(mpc.get("weight_control", base.weight_control)),
        weight_obstacle=float(
            mpc.get("weight_obstacle", base.weight_obstacle)
        ),
        obstacle_barrier_power=float(
            mpc.get("obstacle_barrier_power", base.obstacle_barrier_power)
        ),
        weight_terminal=float(
            mpc.get("weight_terminal", base.weight_terminal)
        ),
        weight_slack=float(mpc.get("weight_slack", base.weight_slack)),
        max_solver_iter_count=int(
            mpc.get("max_solver_iter_count", base.max_solver_iter_count)
        ),
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


def _straight_line_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    spacing_m: float = 0.25,
) -> list[tuple[float, float]]:
    """Return dense SE(2) XY waypoints on the naive start→goal segment."""
    dx = float(goal[0] - start[0])
    dy = float(goal[1] - start[1])
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return [(float(start[0]), float(start[1]))]
    n = max(2, int(math.ceil(length / max(spacing_m, 1e-3))) + 1)
    return [
        (
            float(start[0] + (i / (n - 1)) * dx),
            float(start[1] + (i / (n - 1)) * dy),
        )
        for i in range(n)
    ]


def _dummy_path_tracker_command(
    pose: tuple[float, float, float],
    *,
    path: list[tuple[float, float]],
    stopped: bool,
    speed: float,
    max_turn_rate: float,
    lookahead_distance: float,
    occupancy: RectStructureOccupancy,
) -> tuple[tuple[float, float, float], bool]:
    """Pure-pursuit on the straight path; no obstacle repulsion (foil).

    The dummy must keep tracking the naive start→goal line until the
    inflated occupancy reports contact — repulsion would steer it off the
    diagonal and look like reverse driving.
    """
    if stopped or not path:
        return (0.0, 0.0, 0.0), True

    x, y, yaw = pose
    # Bail out if physics flung the body outside the lab (contact explosion).
    if not (0.0 <= x <= 10.0 and 0.0 <= y <= 10.0):
        return (0.0, 0.0, 0.0), True
    if occupancy.is_occupied(np.array([x, y], dtype=np.float64)):
        return (0.0, 0.0, 0.0), True

    # Nose probe along the commanded heading — stop before scraping a wall.
    probe = np.array(
        [x + 0.30 * math.cos(yaw), y + 0.30 * math.sin(yaw)],
        dtype=np.float64,
    )
    if occupancy.is_occupied(probe):
        return (0.0, 0.0, 0.0), True

    look_x, look_y = _path_lookahead_point(
        path,
        (x, y),
        lookahead_m=max(float(lookahead_distance), 0.35),
    )
    bearing = math.atan2(look_y - y, look_x - x)
    yaw_err = _wrap_heading(bearing - yaw)
    heading_gate = 0.15 + 0.85 * max(0.0, math.cos(yaw_err))
    forward = float(speed) * heading_gate
    omega = max(
        -float(max_turn_rate),
        min(float(max_turn_rate), 2.5 * yaw_err),
    )
    vx = forward * math.cos(yaw)
    vy = forward * math.sin(yaw)
    return (vx, vy, omega), False


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


_PHYSICS_PLANNING_CLEARANCE_BUMP_M: float = 0.05
_PHYSICS_GOAL_DWELL_TICKS: int = 6
_PHYSICS_WORKSPACE_MAX_M: float = 10.0
_PHYSICS_WORKSPACE_MARGIN_M: float = 0.45


def _wrap_heading(angle: float) -> float:
    """Wrap a heading angle to ``[-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _path_lookahead_point(
    path: list[tuple[float, float]],
    pose_xy: tuple[float, float],
    *,
    lookahead_m: float,
) -> tuple[float, float]:
    """Return a point ``lookahead_m`` ahead of the closest path projection.

    Projects onto polyline segments (not vertices only) so the lookahead cannot
    collapse onto the robot when it catches a coarse waypoint.
    """
    if not path:
        return pose_xy
    if len(path) == 1:
        return (float(path[0][0]), float(path[0][1]))

    px, py = pose_xy
    best_dist = float("inf")
    best_seg = 0
    best_t = 0.0
    for i in range(len(path) - 1):
        x0, y0 = float(path[i][0]), float(path[i][1])
        x1, y1 = float(path[i + 1][0]), float(path[i + 1][1])
        dx, dy = x1 - x0, y1 - y0
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-12:
            t = 0.0
            proj_x, proj_y = x0, y0
        else:
            t = max(
                0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_len_sq)
            )
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
        dist = math.hypot(proj_x - px, proj_y - py)
        if dist < best_dist:
            best_dist = dist
            best_seg = i
            best_t = t

    # Distance already consumed along the chosen segment.
    x0, y0 = float(path[best_seg][0]), float(path[best_seg][1])
    x1, y1 = float(path[best_seg + 1][0]), float(path[best_seg + 1][1])
    seg_len = math.hypot(x1 - x0, y1 - y0)
    remaining = float(lookahead_m) + best_t * seg_len
    for i in range(best_seg, len(path) - 1):
        x0, y0 = float(path[i][0]), float(path[i][1])
        x1, y1 = float(path[i + 1][0]), float(path[i + 1][1])
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 1e-9:
            continue
        if remaining <= seg:
            t = remaining / seg
            return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        remaining -= seg
    return (float(path[-1][0]), float(path[-1][1]))


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


def _agent_physics_velocity_command(
    loop: MPCTrackingLoop,
    vehicle: Any,
    path: list[tuple[float, float]],
    *,
    dt: float,
    agent_finished: bool,
    max_speed: float,
    max_turn_rate: float,
) -> tuple[tuple[float, float, float], dict[str, Any]]:
    """Return MPC body twist for true differential-drive physics.

    ``agent_finished`` covers both "reached the goal" and "the collision
    monitor stopped this agent after a real impact" — either way the
    control loop must stop commanding motion. Obstacle avoidance lives
    inside the ARCO path-following MPC (no APF blend).
    """
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

    # MPCTrackingLoop.step integrates a kinematic preview; restore the
    # MuJoCo pose afterward and apply only the first optimal command.
    metrics = loop.step(path, dt)
    vehicle.x = float(sim_x)
    vehicle.y = float(sim_y)
    vehicle.heading = float(sim_theta)

    forward = float(metrics.get("mpc_speed_cmd", metrics.get("speed", 0.0)))
    omega = float(
        metrics.get("mpc_turn_rate_cmd", metrics.get("turn_rate", 0.0))
    )
    forward = max(0.0, min(max_speed, forward))
    omega = max(-max_turn_rate, min(max_turn_rate, omega))

    vx = forward * math.cos(sim_theta)
    vy = forward * math.sin(sim_theta)

    margin = _PHYSICS_WORKSPACE_MARGIN_M
    workspace_max = _PHYSICS_WORKSPACE_MAX_M
    if sim_x < margin:
        scale = max(0.0, sim_x / margin)
        vx *= scale
        vy *= scale
    elif sim_x > workspace_max - margin:
        outward = max(0.0, vx)
        scale = max(0.0, (workspace_max - sim_x) / margin)
        vx = vx - outward * (1.0 - scale)
    if sim_y < margin:
        scale = max(0.0, sim_y / margin)
        vx *= scale
        vy *= scale
    elif sim_y > workspace_max - margin:
        outward = max(0.0, vy)
        scale = max(0.0, (workspace_max - sim_y) / margin)
        vy = vy - outward * (1.0 - scale)

    return (
        (vx, vy, omega),
        {
            **metrics,
            "heading_error": float(metrics.get("heading_error", 0.0)),
        },
    )


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
        mpc_cfg = _mpc_config(ctrl)
        rrt_vehicle, rrt_loop = build_vehicle_mpc_sim(
            rrt_path, vehicle_cfg, mpc_cfg, occupancy=occupancy
        )
        sst_vehicle, sst_loop = build_vehicle_mpc_sim(
            sst_path, vehicle_cfg, mpc_cfg, occupancy=occupancy
        )

        dummy_pose = _initial_dummy_pose(world)
        dummy_path = _straight_line_path(world.start_xy, world.goal_xy)
        # Grey foil: Pure Pursuit only — no MPC / no APF repulsion.
        dummy_vehicle, dummy_loop = build_vehicle_sim(
            dummy_path, vehicle_cfg, occupancy=occupancy
        )
        dummy_vehicle.x = float(dummy_pose[0])
        dummy_vehicle.y = float(dummy_pose[1])
        dummy_vehicle.heading = float(dummy_pose[2])
        session = DubinsRaceSimulation(
            world=world,
            vehicle_cfg=vehicle_cfg,
            vehicle_body=vehicle_body,
            occupancy=occupancy,
            dt=dt,
            rrt_vehicle=rrt_vehicle,
            sst_vehicle=sst_vehicle,
            dummy_vehicle=dummy_vehicle,
            rrt_loop=rrt_loop,
            sst_loop=sst_loop,
            dummy_loop=dummy_loop,
            rrt_path=rrt_path,
            sst_path=sst_path,
            dummy_path=dummy_path,
            goal_radius=vehicle_cfg.goal_radius,
            rrt_pose_history=[rrt_vehicle.pose],
            sst_pose_history=[sst_vehicle.pose],
            dummy_pose=dummy_pose,
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
                rrt_collided=result.rrt_collided,
                sst_collided=result.sst_collided,
                rrt_collision_force_n=result.rrt_collision_force_n,
                sst_collision_force_n=result.sst_collision_force_n,
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
            rrt_collided=result.rrt_collided,
            sst_collided=result.sst_collided,
            rrt_collision_force_n=result.rrt_collision_force_n,
            sst_collision_force_n=result.sst_collision_force_n,
            contact_log_path=contact_log_path,
            contact_event_count=contact_event_count,
            max_contact_force_n=max_contact_force_n,
            penetration_violations=penetration_violations,
            physics_metrics_path=physics_metrics_path,
            column_contacts_logged=column_contacts_logged,
        )
