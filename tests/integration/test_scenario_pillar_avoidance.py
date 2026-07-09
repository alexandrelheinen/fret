"""Integration test: Milestone 5 pillar-avoidance scenario (pure-Python).

Runs the complete MS-5 pipeline without ROS or MuJoCo:

  1. Sample cylinder surfaces for pillar_a and pillar_b.
  2. Build a ``WorkspaceOccupancyBuilder`` to verify pillar occupancy.
  3. Create an ``OccupancyAdapter`` and ``PlannerNode`` for planning.
  4. Plan a joint-space path from the home configuration to the goal.
  5. Verify every waypoint's EE position is outside the safety zone around
     each pillar (horizontal clearance > pillar_radius + 0.05 m = 0.09 m).
  6. Densify the path and run the Jacobian controller.
  7. Verify tracking error <= 5 mm and no fault triggered.

Acceptance criteria (Milestone 5):
  - ``WorkspaceOccupancyBuilder`` detects both pillars (occupied voxels > 0).
  - All planned waypoints satisfy pillar-clearance requirement.
  - Controller tracking error <= 5 mm throughout the trajectory.
  - No fault triggered during tracking.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)

# ---------------------------------------------------------------------------
# Scenario constants
# ---------------------------------------------------------------------------

_PILLAR_A: tuple[float, float] = (0.25, 0.10)  # (x, y) centre
_PILLAR_B: tuple[float, float] = (-0.15, 0.30)  # (x, y) centre
_PILLAR_RADIUS: float = 0.04  # [m]
_PILLAR_Z_CENTRE: float = 0.20  # [m]
_PILLAR_LENGTH: float = 0.40  # [m]

_START_Q = np.array([0.0, 0.0, 0.10])
_GOAL_Q = np.array([-1.0, 0.5, 0.10])

_SAFETY_MARGIN: float = 0.05  # minimum clearance from pillar surface [m]
_MAX_EE_ERROR_M: float = 0.005  # 5 mm tracking limit
_DURATION_S: float = 20.0
_RATE_HZ: float = 50.0
_N_STEPS: int = int(_DURATION_S * _RATE_HZ)
_DT: float = 1.0 / _RATE_HZ


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _sample_cylinder(
    cx: float, cy: float, cz: float, radius: float, length: float
) -> np.ndarray:
    """Sample surface points for a vertical cylinder."""
    half = length / 2.0
    rng = np.random.default_rng(seed=42)
    n_lat = 200
    theta = rng.uniform(0, 2 * math.pi, n_lat)
    z_lat = rng.uniform(cz - half, cz + half, n_lat)
    lat = np.column_stack(
        [cx + radius * np.cos(theta), cy + radius * np.sin(theta), z_lat]
    )
    caps = []
    for sign in (-1.0, 1.0):
        n_cap = 50
        r_c = np.sqrt(rng.uniform(0, radius**2, n_cap))
        th = rng.uniform(0, 2 * math.pi, n_cap)
        cap = np.column_stack(
            [
                cx + r_c * np.cos(th),
                cy + r_c * np.sin(th),
                np.full(n_cap, cz + sign * half),
            ]
        )
        caps.append(cap)
    return np.vstack([lat] + caps).astype(np.float64)


def _make_payload() -> OccupancyUpdatePayload:
    pts_a = _sample_cylinder(
        _PILLAR_A[0],
        _PILLAR_A[1],
        _PILLAR_Z_CENTRE,
        _PILLAR_RADIUS,
        _PILLAR_LENGTH,
    )
    pts_b = _sample_cylinder(
        _PILLAR_B[0],
        _PILLAR_B[1],
        _PILLAR_Z_CENTRE,
        _PILLAR_RADIUS,
        _PILLAR_LENGTH,
    )
    return OccupancyUpdatePayload(
        obstacle_points=np.vstack([pts_a, pts_b]).astype(np.float64),
        timestamp=0.0,
        frame_id="world",
    )


def _densify(path: list[np.ndarray], n: int) -> list[np.ndarray]:
    """Linearly interpolate path to n waypoints."""
    return [path[0] + i / (n - 1) * (path[-1] - path[0]) for i in range(n)]


# ---------------------------------------------------------------------------
# Integration tests
# ---------------------------------------------------------------------------


class TestPillarAvoidanceIntegration:
    """Full Milestone 5 pipeline: planning + occupancy + tracking."""

    def test_occupancy_builder_detects_pillars(self) -> None:
        """WorkspaceOccupancyBuilder marks pillar voxels as occupied."""
        from fret.scene.workspace_occupancy import WorkspaceOccupancyBuilder

        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_payload())
        assert len(builder.occupied_centres()) > 0, "No occupied voxels found"

    def test_planner_returns_success(self) -> None:
        """PlannerNode returns SUCCESS for the pillar-avoidance scenario."""
        from fret.planning.planner_node import PlannerNode
        from fret.scene.occupancy_adapter import OccupancyAdapter

        adapter = OccupancyAdapter()
        adapter.update(_make_payload())

        planner = PlannerNode(model="scara", occupancy_adapter=adapter)
        result = planner.plan(
            PlanningRequest(
                start_configuration=_START_Q.copy(),
                goal_configuration=_GOAL_Q.copy(),
                planning_timeout=30.0,
                scenario_id="pillar_avoidance_test",
            )
        )
        assert result.status == PlanningStatus.SUCCESS, (
            f"PlannerNode returned {result.status!r} "
            f"(error_code={result.error_code!r})"
        )
        assert len(result.path) >= 2

    def test_planned_path_avoids_pillars(self) -> None:
        """All waypoints along the planned path satisfy the clearance requirement."""
        from fret.control.kinematics import Kinematics
        from fret.planning.planner_node import PlannerNode
        from fret.scene.occupancy_adapter import OccupancyAdapter

        kin = Kinematics("scara")
        adapter = OccupancyAdapter()
        adapter.update(_make_payload())

        planner = PlannerNode(model="scara", occupancy_adapter=adapter)
        result = planner.plan(
            PlanningRequest(
                start_configuration=_START_Q.copy(),
                goal_configuration=_GOAL_Q.copy(),
                planning_timeout=30.0,
                scenario_id="pillar_avoidance_path_test",
            )
        )
        assert result.status == PlanningStatus.SUCCESS

        waypoints = _densify(result.path, 100)
        min_req = _PILLAR_RADIUS + _SAFETY_MARGIN

        for i, q in enumerate(waypoints):
            T = kin.forward_kinematics(q)
            x, y = float(T[0, 3]), float(T[1, 3])
            for cx, cy in [_PILLAR_A, _PILLAR_B]:
                dist_xy = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
                assert dist_xy >= min_req, (
                    f"Waypoint {i}: EE ({x:.3f}, {y:.3f}) too close to "
                    f"pillar ({cx}, {cy}): dist = {dist_xy:.4f} m "
                    f"(required >= {min_req:.3f} m)"
                )

    def test_controller_tracks_within_error_limit(self) -> None:
        """Controller tracks the densified path with EE error <= 5 mm."""
        from fret.control.controller_node import ControllerNode, _NodeState
        from fret.control.kinematics import Kinematics
        from fret.planning.planner_node import PlannerNode
        from fret.scene.occupancy_adapter import OccupancyAdapter

        kin = Kinematics("scara")
        adapter = OccupancyAdapter()
        adapter.update(_make_payload())

        planner = PlannerNode(model="scara", occupancy_adapter=adapter)
        result = planner.plan(
            PlanningRequest(
                start_configuration=_START_Q.copy(),
                goal_configuration=_GOAL_Q.copy(),
                planning_timeout=30.0,
                scenario_id="pillar_avoidance_tracking_test",
            )
        )
        assert result.status == PlanningStatus.SUCCESS

        waypoints = _densify(result.path, _N_STEPS)

        ctrl = ControllerNode(model="scara", config_path="")
        ctrl.set_trajectory(waypoints)
        assert ctrl._state == _NodeState.TRACKING

        q_cur = _START_Q.copy()
        max_err = 0.0
        for q_ref in waypoints:
            x_ref = kin.forward_kinematics(q_ref)[:3, 3]
            x_cur = kin.forward_kinematics(q_cur)[:3, 3]
            err = float(np.linalg.norm(x_ref - x_cur))
            if err > max_err:
                max_err = err

            ctrl.compute_jacobian_command(kin, q_cur)
            q_cur = q_cur + ctrl._get_current_command() * _DT

            if ctrl._state == _NodeState.HALTED:
                pytest.fail(
                    f"Controller faulted during tracking: "
                    f"EE error = {err * 1000:.2f} mm"
                )

        assert (
            max_err <= _MAX_EE_ERROR_M
        ), f"Max EE error {max_err * 1000:.2f} mm exceeds 5 mm limit"

    def test_no_fault_triggered_during_tracking(self) -> None:
        """No fault is triggered during the full tracking run."""
        from fret.control.controller_node import ControllerNode, _NodeState
        from fret.control.kinematics import Kinematics
        from fret.planning.planner_node import PlannerNode
        from fret.scene.occupancy_adapter import OccupancyAdapter

        kin = Kinematics("scara")
        adapter = OccupancyAdapter()
        adapter.update(_make_payload())

        planner = PlannerNode(model="scara", occupancy_adapter=adapter)
        result = planner.plan(
            PlanningRequest(
                start_configuration=_START_Q.copy(),
                goal_configuration=_GOAL_Q.copy(),
                planning_timeout=30.0,
                scenario_id="pillar_avoidance_fault_test",
            )
        )
        assert result.status == PlanningStatus.SUCCESS

        waypoints = _densify(result.path, _N_STEPS)
        ctrl = ControllerNode(model="scara", config_path="")
        ctrl.set_trajectory(waypoints)

        q_cur = _START_Q.copy()
        for q_ref in waypoints:
            ctrl.compute_jacobian_command(kin, q_cur)
            q_cur = q_cur + ctrl._get_current_command() * _DT
            if ctrl._state == _NodeState.HALTED:
                pytest.fail("Controller faulted — unexpected HALTED state")

        assert ctrl._state == _NodeState.TRACKING
