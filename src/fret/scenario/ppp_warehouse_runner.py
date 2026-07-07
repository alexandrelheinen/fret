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

from fret.control.controller_ppp import PPPControllerNode
from fret.control.grasp_magnet import GraspConfig, GraspState, MagneticGraspFSM
from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.cspace_checker import make_cspace_checker
from fret.planning.planner_node import PlannerNode
from fret.planning.ppp_obstacles import (
    BoxObstacleOccupancy,
    boxes_to_point_cloud,
    load_ppp_warehouse_preview_obstacles,
)
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.ros.mujoco_bridge import make_mujoco_bridge_core
from fret.scene.occupancy_adapter import OccupancyAdapter
from fret.sitl_config import load_scenario_parameters

_DEFAULT_SCENARIO = (
    pathlib.Path(__file__).resolve().parents[1]
    / "config"
    / "scenarios"
    / "ppp_warehouse.yml"
)
_DEFAULT_CONTROLLER = (
    pathlib.Path(__file__).resolve().parents[1]
    / "config"
    / "controllers"
    / "ppp.yml"
)
_EE_ERROR_LIMIT_M: float = 0.010
_PLANNING_TIMEOUT_S: float = 30.0


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
    capture_radius = float(grasp.get("capture_radius", 0.3))
    goal_radius = float(grasp.get("goal_radius", 0.5))

    weld_raw = grasp.get("weld_offset", [0.0, 0.0, 0.25])
    if isinstance(weld_raw, (int, float)):
        weld_offset = np.array([0.0, 0.0, float(weld_raw)], dtype=np.float64)
    else:
        weld_offset = np.asarray(weld_raw, dtype=np.float64).reshape(3)

    half_raw = grasp.get("box_half_extent", 0.25)
    if isinstance(half_raw, (int, float)):
        box_half = np.full(3, float(half_raw), dtype=np.float64)
    else:
        box_half = np.asarray(half_raw, dtype=np.float64).reshape(3)

    return GraspConfig(
        capture_radius=capture_radius,
        goal_radius=goal_radius,
        weld_offset=weld_offset,
        box_half_extent=box_half,
    )


def _build_occupancy_adapter(
    obstacle_path: pathlib.Path | None,
) -> tuple[OccupancyAdapter, BoxObstacleOccupancy]:
    """Load preview obstacles into point-cloud and box occupancy models."""
    boxes = load_ppp_warehouse_preview_obstacles(obstacle_path)
    cloud = boxes_to_point_cloud(boxes, samples_per_edge=4)
    adapter = OccupancyAdapter()
    adapter.update(
        OccupancyUpdatePayload(
            obstacle_points=cloud,
            timestamp=0.0,
            frame_id="world",
        )
    )
    return adapter, BoxObstacleOccupancy(boxes)


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
) -> bool:
    """Return True when every waypoint is collision-free."""
    checker = make_cspace_checker(
        kin,
        occupancy,
        include_cargo=include_cargo,
        grasp_config=grasp_config,
    )
    return all(checker.is_collision_free(q) for q in path)


def _track_prismatic_path(
    waypoints: list[npt.NDArray[np.float64]],
    kin: Kinematics,
    bridge: Any,
    *,
    kp: float,
    max_joint_velocity: npt.NDArray[np.float64],
    dt: float,
    fault_threshold_m: float,
    on_step: Any | None = None,
    convergence_m: float = 0.004,
    max_inner_steps: int = 50,
) -> float:
    """Track dense waypoints with per-axis P-control (FR-CTL-02).

    Mirrors ``PPPControllerNode`` control law while advancing references
    only after the gantry converges, avoiding spurious fault trips.

    Returns:
        Maximum joint-space tracking error observed [m].
    """
    max_err_m = 0.0
    for wp_idx, q_ref in enumerate(waypoints):
        for _ in range(max_inner_steps):
            q = bridge.get_positions()
            joint_error = q_ref - q
            err_m = float(np.linalg.norm(joint_error))
            max_err_m = max(max_err_m, err_m)
            if err_m > fault_threshold_m:
                msg = (
                    f"Tracking error {err_m * 1000:.2f} mm exceeds "
                    f"{fault_threshold_m * 1000:.0f} mm at waypoint {wp_idx}"
                )
                raise RuntimeError(msg)

            q_dot = np.clip(
                kp * joint_error,
                -max_joint_velocity,
                max_joint_velocity,
            )
            bridge.step(q_dot, dt)
            if on_step is not None:
                on_step(bridge.get_positions())
            if err_m <= convergence_m:
                break
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
        return load_scenario_parameters(self._scenario_path)

    def run(self) -> PPPWarehouseRunResult:
        """Execute planning, tracking, grasp, and collision validation."""
        params = self.load_parameters()
        start = np.asarray(params["start_configuration"], dtype=np.float64)
        goal = np.asarray(params["goal_configuration"], dtype=np.float64)
        timeout = float(params.get("planning_timeout", _PLANNING_TIMEOUT_S))
        grasp_cfg = _parse_grasp_config(dict(params.get("grasp", {})))

        occ_adapter, box_occ = _build_occupancy_adapter(self._obstacle_path)
        kin = Kinematics("ppp")
        planner = PlannerNode(model="ppp", occupancy_adapter=occ_adapter)
        traj_gen = TrajectoryGenerator(kin)

        req = PlanningRequest(
            start_configuration=start.copy(),
            goal_configuration=goal.copy(),
            planning_timeout=timeout,
            scenario_id=str(params.get("scenario_id", "ppp_warehouse")),
        )
        t0 = time.monotonic()
        plan_result = planner.plan(req)
        planning_duration = time.monotonic() - t0

        if plan_result.status != PlanningStatus.SUCCESS:
            return PPPWarehouseRunResult(
                planning_status=plan_result.status,
                planning_duration_s=planning_duration,
                max_tracking_error_m=0.0,
                grasp_captured=False,
                grasp_released=False,
                path_collision_free=False,
                controller_faulted=False,
                n_path_waypoints=len(plan_result.path),
            )

        path_collision_free = _path_is_collision_free(
            kin,
            box_occ,
            plan_result.path,
            include_cargo=False,
            grasp_config=grasp_cfg,
        )

        waypoints = _trajectory_waypoints(traj_gen, plan_result.path)
        ctrl = PPPControllerNode(str(self._controller_config_path))

        bridge = make_mujoco_bridge_core(
            "ppp",
            "ppp_warehouse",
            initial_positions=start.copy(),
        )

        grasp = MagneticGraspFSM(grasp_cfg)
        box_anchor = start + grasp_cfg.weld_offset
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
            max_err_m = _track_prismatic_path(
                waypoints,
                kin,
                bridge,
                kp=ctrl._kp,
                max_joint_velocity=ctrl._max_joint_velocity,
                dt=dt,
                fault_threshold_m=ctrl.fault_threshold,
                on_step=_grasp_tick,
            )
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
                plan_result.path,
                include_cargo=True,
                grasp_config=grasp_cfg,
            )
            path_collision_free = path_collision_free and cargo_free

        return PPPWarehouseRunResult(
            planning_status=plan_result.status,
            planning_duration_s=planning_duration,
            max_tracking_error_m=max_err_m,
            grasp_captured=grasp_captured,
            grasp_released=grasp_released,
            path_collision_free=path_collision_free,
            controller_faulted=controller_faulted,
            n_path_waypoints=len(plan_result.path),
        )
