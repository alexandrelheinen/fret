"""PPP warehouse pick-and-place E2E orchestrator (SC-v10 / v1.0).

Runs the full product pipeline in pure Python:

  Occupancy → PlannerNode → TrajectoryGenerator → PPPControllerNode
  → MuJoCoBridgeCore, with ``MagneticGraspFSM`` alongside simulation.

Validates releases.md acceptance criteria V10-2 – V10-5 in CI without ROS.
"""

from __future__ import annotations

import pathlib
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.config_loader import (
    load_scenario_bundle,
    resolve_obstacle_file,
)
from fret.control.controller_ppp import PPPControllerNode
from fret.control.grasp_magnet import (
    GraspConfig,
    GraspState,
    MagneticGraspFSM,
    parse_grasp_config,
)
from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningResult,
    PlanningStatus,
)
from fret.planning.cspace_checker import make_cspace_checker
from fret.planning.planner_node import PlannerNode
from fret.planning.ppp_obstacles import (
    BoxObstacleOccupancy,
    boxes_to_point_cloud,
    load_ppp_warehouse_preview_obstacles,
    load_preview_workspace_bounds,
)
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.ros.mujoco_bridge import make_mujoco_bridge_core
from fret.scene.occupancy_adapter import OccupancyAdapter
from fret.sitl_config import controller_config_path, scenario_config_path

_DEFAULT_SCENARIO = scenario_config_path("ppp_warehouse")
_DEFAULT_CONTROLLER = controller_config_path("ppp")


@dataclass(frozen=True)
class PPPWarehouseRunResult:
    """Outcome of a PPP warehouse E2E run."""

    planning_status: PlanningStatus
    planning_duration_s: float
    max_tracking_error_m: float
    grasp_captured: bool
    grasp_released: bool
    path_collision_free: bool
    controller_faulted: bool
    n_path_waypoints: int


def _parse_grasp_config(grasp: dict[str, Any]) -> GraspConfig:
    """Build ``GraspConfig`` from scenario ``grasp`` parameters."""
    return parse_grasp_config(grasp)


def _build_occupancy_adapter(
    obstacle_path: pathlib.Path,
    *,
    samples_per_edge: int,
    contact_radius: float,
) -> tuple[OccupancyAdapter, BoxObstacleOccupancy]:
    """Load preview obstacles into point-cloud and box occupancy models."""
    boxes = load_ppp_warehouse_preview_obstacles(obstacle_path)
    cloud = boxes_to_point_cloud(boxes, samples_per_edge=samples_per_edge)
    adapter = OccupancyAdapter()
    adapter.update(
        OccupancyUpdatePayload(
            obstacle_points=cloud,
            timestamp=0.0,
            frame_id="world",
        )
    )
    return adapter, BoxObstacleOccupancy(boxes, contact_radius=contact_radius)


def compute_ppp_cruise_z(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    max_obstacle_z: float,
    cargo_half_z: float,
    cargo_ee_offset_z: float,
    cruise_cfg: dict[str, Any],
) -> float:
    """Return EE cruise height for welded-cargo transit around floor clutter."""
    cargo_drop = float(cargo_half_z + abs(cargo_ee_offset_z))
    nominal_cruise = float(
        max(start[2], goal[2], float(cruise_cfg["default_z"]))
    )
    flyover_z = (
        max_obstacle_z + cargo_drop + float(cruise_cfg["flyover_clearance_m"])
    )
    if nominal_cruise >= flyover_z:
        return max(
            float(cruise_cfg["nominal_min_z"]),
            max_obstacle_z + float(cruise_cfg["obstacle_clearance_m"]),
        )
    return nominal_cruise


def stitch_ppp_operational_path(
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    transit_path: list[npt.NDArray[np.float64]],
    *,
    pick_ee_z: float,
    cruise_z: float,
) -> list[npt.NDArray[np.float64]]:
    """Insert pick/place vertical segments around a welded-cargo transit plan."""
    if len(transit_path) < 2:
        raise ValueError("Transit path must contain at least two waypoints")
    waypoints: list[npt.NDArray[np.float64]] = [
        np.asarray(start, dtype=np.float64).copy(),
        np.array([start[0], start[1], pick_ee_z]),
        np.array([start[0], start[1], cruise_z]),
    ]
    waypoints.extend(
        np.asarray(q, dtype=np.float64) for q in transit_path[1:-1]
    )
    waypoints.extend(
        [
            np.array([goal[0], goal[1], cruise_z]),
            np.array([goal[0], goal[1], pick_ee_z]),
            np.asarray(goal, dtype=np.float64).copy(),
        ]
    )
    return waypoints


def _plan_ppp_transit_path(
    planner: PlannerNode,
    start: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    *,
    cruise_z: float,
    timeout: float,
    scenario_id: str,
) -> PlanningResult:
    """Plan horizontal transit with welded cargo at ``cruise_z``."""
    transit_start = start.copy()
    transit_goal = goal.copy()
    transit_start[2] = float(cruise_z)
    transit_goal[2] = float(cruise_z)
    return planner.plan(
        PlanningRequest(
            start_configuration=transit_start,
            goal_configuration=transit_goal,
            planning_timeout=timeout,
            scenario_id=scenario_id,
        )
    )


def _trajectory_waypoints(
    traj_gen: TrajectoryGenerator,
    path: list[npt.NDArray[np.float64]],
    *,
    max_waypoints: int | None = None,
) -> list[npt.NDArray[np.float64]]:
    """Convert a planner path to dense joint waypoints for the controller."""
    traj = traj_gen.process(path)
    dense = [np.asarray(pt.positions, dtype=np.float64) for pt in traj.points]
    if max_waypoints is None or len(dense) <= max_waypoints:
        return dense
    indices = np.linspace(0, len(dense) - 1, max_waypoints, dtype=int)
    return [dense[int(i)] for i in indices]


def _path_is_collision_free(
    kin: Kinematics,
    occupancy: BoxObstacleOccupancy,
    path: list[npt.NDArray[np.float64]],
    *,
    include_cargo: bool,
    grasp_config: GraspConfig,
    collision_backend: str = "analytic",
    scenario: str = "ppp_warehouse",
    contact_radius: float | None = None,
    workspace_bounds: (
        tuple[
            tuple[float, float],
            tuple[float, float],
            tuple[float, float],
        ]
        | None
    ) = None,
) -> bool:
    """Return True when every waypoint is collision-free."""
    checker = make_cspace_checker(
        kin,
        occupancy,
        include_cargo=include_cargo,
        grasp_config=grasp_config,
        contact_radius=contact_radius,
        collision_backend=collision_backend,  # type: ignore[arg-type]
        scenario=scenario,
        workspace_bounds=workspace_bounds,
    )
    return all(checker.is_collision_free(q) for q in path)


def _track_carrot_path(
    waypoints: list[npt.NDArray[np.float64]],
    bridge: Any,
    *,
    kp: float,
    max_joint_velocity: npt.NDArray[np.float64],
    max_joint_acc: float,
    race_speed: float,
    max_carrot_lag: float,
    dt: float,
    fault_threshold_m: float,
    goal_tolerance: float,
    on_step: Any | None = None,
) -> float:
    """Track dense waypoints with arc-length carrot following (FR-CTL-02)."""
    from fret.control.path_tracking import (
        _subsample_path,
        densify_polyline,
        simulate_joint_carrot_tracking,
    )

    dense = densify_polyline(
        [np.asarray(q, dtype=np.float64) for q in waypoints],
        max_step=0.08,
    )
    if len(dense) > 600:
        dense = _subsample_path(dense, 600)
    max_acc = np.full(3, max_joint_acc, dtype=np.float64)

    def _sync_bridge(q: npt.NDArray[np.float64]) -> None:
        bridge.set_positions(q)
        if on_step is not None:
            on_step(q)

    _, max_err_m = simulate_joint_carrot_tracking(
        dense,
        start=bridge.get_positions(),
        race_speed=race_speed,
        max_joint_velocity=max_joint_velocity,
        max_joint_acc=max_acc,
        proportional_gain=kp,
        max_carrot_lag=max_carrot_lag,
        dt=dt,
        goal=dense[-1],
        goal_tolerance=goal_tolerance,
        on_step=_sync_bridge,
    )
    return max_err_m


class PPPWarehouseRunner:
    """Pure-Python E2E orchestrator for the PPP warehouse scenario.

    Args:
        scenario_path: Path to ``ppp_warehouse.yml``.
        obstacle_path: Optional preview obstacle YAML override.
        controller_config_path: Path to ``ppp.yml`` controller parameters.
    """

    def __init__(
        self,
        scenario_path: str | pathlib.Path | None = None,
        obstacle_path: str | pathlib.Path | None = None,
        controller_config_path: str | pathlib.Path | None = None,
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

    @property
    def scenario_path(self) -> pathlib.Path:
        """Resolved scenario YAML path."""
        return self._scenario_path

    def load_parameters(self) -> dict[str, Any]:
        """Load flat scenario parameters from YAML."""
        return load_scenario_bundle(self._scenario_path).parameters

    def run(self) -> PPPWarehouseRunResult:
        """Execute planning, tracking, grasp, and collision validation."""
        bundle = load_scenario_bundle(self._scenario_path)
        params = bundle.parameters
        planning = bundle.planning
        if bundle.grasp is None:
            raise ValueError("ppp_warehouse scenario requires grasp_config")
        start = np.asarray(params["start_configuration"], dtype=np.float64)
        goal = np.asarray(params["goal_configuration"], dtype=np.float64)
        timeout = float(params["planning_timeout"])
        grasp_cfg = _parse_grasp_config(bundle.grasp)
        plan_include_cargo = bool(params["plan_include_cargo"])

        obstacle_path = (
            self._obstacle_path
            if self._obstacle_path is not None
            else resolve_obstacle_file(planning)
        )
        contact_radius = float(planning["contact_radius"])
        samples_per_edge = int(planning["samples_per_edge"])
        cargo_ee_offset_z = float(planning["cargo_ee_offset_z"])
        cruise_cfg = planning["cruise"]
        tracking_cfg = planning["tracking"]
        ee_error_limit_m = float(planning["ee_error_limit_m"])

        occ_adapter, box_occ = _build_occupancy_adapter(
            obstacle_path,
            samples_per_edge=samples_per_edge,
            contact_radius=contact_radius,
        )
        preview_bounds = load_preview_workspace_bounds(obstacle_path)
        boxes = load_ppp_warehouse_preview_obstacles(obstacle_path)
        kin = Kinematics("ppp")
        collision_backend = str(params["collision_backend"])
        planner_algorithm = str(params["planner_algorithm"])
        scenario_id = str(params["scenario_id"])
        planner = PlannerNode(
            model="ppp",
            occupancy_adapter=occ_adapter,
            occupancy=box_occ,
            collision_backend=collision_backend,  # type: ignore[arg-type]
            planner_algorithm=planner_algorithm,  # type: ignore[arg-type]
            include_cargo=plan_include_cargo,
            grasp_config=grasp_cfg,
            scenario=scenario_id,
            workspace_bounds=preview_bounds,
            planning_config=planning,
        )
        traj_gen = TrajectoryGenerator(kin, planning)

        t0 = time.monotonic()
        max_obs_z = max((box.z_max for box in boxes), default=1.2)
        cruise_z = compute_ppp_cruise_z(
            start,
            goal,
            max_obstacle_z=max_obs_z,
            cargo_half_z=float(grasp_cfg.box_half_extent[2]),
            cargo_ee_offset_z=cargo_ee_offset_z,
            cruise_cfg=cruise_cfg,
        )
        pick_ee_z = float(grasp_cfg.box_half_extent[2]) - cargo_ee_offset_z
        transit_result = _plan_ppp_transit_path(
            planner,
            start,
            goal,
            cruise_z=cruise_z,
            timeout=timeout,
            scenario_id=scenario_id,
        )
        planning_duration = time.monotonic() - t0

        if transit_result.status != PlanningStatus.SUCCESS:
            return PPPWarehouseRunResult(
                planning_status=transit_result.status,
                planning_duration_s=planning_duration,
                max_tracking_error_m=0.0,
                grasp_captured=False,
                grasp_released=False,
                path_collision_free=False,
                controller_faulted=False,
                n_path_waypoints=len(transit_result.path),
            )

        operational_path = stitch_ppp_operational_path(
            start,
            goal,
            transit_result.path,
            pick_ee_z=pick_ee_z,
            cruise_z=cruise_z,
        )

        path_collision_free = _path_is_collision_free(
            kin,
            box_occ,
            transit_result.path,
            include_cargo=plan_include_cargo,
            grasp_config=grasp_cfg,
            collision_backend=collision_backend,
            scenario=scenario_id,
            contact_radius=contact_radius,
            workspace_bounds=preview_bounds,
        )

        waypoints = _trajectory_waypoints(traj_gen, operational_path)
        ctrl = PPPControllerNode(str(self._controller_config_path))

        bridge = make_mujoco_bridge_core(
            "ppp",
            "ppp_warehouse",
            initial_positions=start.copy(),
        )

        grasp = MagneticGraspFSM(grasp_cfg)
        box_anchor = np.array(
            [
                start[0],
                start[1],
                float(grasp_cfg.box_half_extent[2]),
            ],
            dtype=np.float64,
        )
        grasp.begin_transport()

        max_err_m = 0.0
        grasp_captured = False
        grasp_released = False
        controller_faulted = False

        def _grasp_tick(q: npt.NDArray[np.float64]) -> None:
            nonlocal grasp_captured, grasp_released
            ee = kin.forward_kinematics(q)[:3, 3]
            state = grasp.update(ee, box_anchor, goal)
            if state == GraspState.TRANSPORT:
                grasp_captured = True
            if grasp_captured and not grasp.is_welded:
                grasp_released = True

        rate_hz = ctrl.update_rate
        dt = 1.0 / rate_hz
        settle_steps = int(2.0 * rate_hz)

        _grasp_tick(start)

        try:
            max_err_m = _track_carrot_path(
                waypoints,
                bridge,
                kp=ctrl._kp,
                max_joint_velocity=ctrl._max_joint_velocity,
                max_joint_acc=ctrl.max_joint_acc,
                race_speed=ctrl.race_speed,
                max_carrot_lag=ctrl.max_carrot_lag,
                dt=dt,
                fault_threshold_m=ctrl.fault_threshold,
                goal_tolerance=float(tracking_cfg["goal_tolerance"]),
                on_step=_grasp_tick,
            )
            if max_err_m > ee_error_limit_m:
                controller_faulted = True
        except RuntimeError:
            controller_faulted = True

        if not controller_faulted:
            for _ in range(settle_steps):
                q = bridge.get_positions()
                _grasp_tick(q)
                joint_error = goal - q
                err_m = float(np.linalg.norm(joint_error))
                max_err_m = max(max_err_m, err_m)
                q_dot = np.clip(
                    ctrl._kp * joint_error,
                    -ctrl._max_joint_velocity,
                    ctrl._max_joint_velocity,
                )
                bridge.step(q_dot, dt)

        _grasp_tick(bridge.get_positions())

        if grasp_captured:
            cargo_free = _path_is_collision_free(
                kin,
                box_occ,
                transit_result.path,
                include_cargo=True,
                grasp_config=grasp_cfg,
                collision_backend=collision_backend,
                scenario=scenario_id,
                contact_radius=contact_radius,
                workspace_bounds=preview_bounds,
            )
            path_collision_free = path_collision_free and cargo_free

        return PPPWarehouseRunResult(
            planning_status=transit_result.status,
            planning_duration_s=planning_duration,
            max_tracking_error_m=max_err_m,
            grasp_captured=grasp_captured,
            grasp_released=grasp_released,
            path_collision_free=path_collision_free,
            controller_faulted=controller_faulted,
            n_path_waypoints=len(operational_path),
        )
