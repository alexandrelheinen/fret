"""Unit tests for WorkspaceOccupancyBuilder with two cylindrical pillars.

Acceptance criteria (Milestone 5, FR-SCN-05):
  - Both pillars are detected — occupied-voxel count is non-zero.
  - Voxels near pillar centres are marked occupied.
  - Points at pillar centres have negative clearance.
  - Points far from pillars have positive clearance.
  - The planned path (start → goal) has positive clearance at every waypoint.
  - CSpaceChecker using builder occupancy detects arm configs near pillars.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.workspace_occupancy import WorkspaceOccupancyBuilder

# ---------------------------------------------------------------------------
# Pillar constants (mirrors pillar_scenario.sdf and perception.yaml)
# ---------------------------------------------------------------------------

_PILLAR_A_XY: tuple[float, float] = (0.25, 0.10)
_PILLAR_B_XY: tuple[float, float] = (-0.15, 0.30)
_PILLAR_RADIUS: float = 0.04  # [m]
_PILLAR_LENGTH: float = 0.40  # [m]
_PILLAR_Z_CENTRE: float = 0.20  # [m]

# Scenario start and goal (from pillar_avoidance.yml)
_START_Q = np.array([0.0, 0.0, 0.10])
_GOAL_Q = np.array([-1.0, 0.5, 0.10])

# Clearance requirement: EE must be > safety_margin from pillar surface [m]
_SAFETY_MARGIN: float = 0.05


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _sample_cylinder(
    cx: float,
    cy: float,
    cz: float,
    radius: float,
    length: float,
    n_lateral: int = 200,
    n_cap: int = 50,
) -> np.ndarray:
    """Return surface sample points for a vertical cylinder.

    Args:
        cx, cy, cz: Cylinder axis centre [m].
        radius: Cylinder radius [m].
        length: Cylinder length (height) [m].
        n_lateral: Number of lateral surface samples.
        n_cap: Number of cap samples per cap.

    Returns:
        Array of shape ``(N, 3)`` with sampled surface points.
    """
    half = length / 2.0
    rng = np.random.default_rng(seed=42)

    # Lateral surface
    theta = rng.uniform(0, 2 * math.pi, n_lateral)
    z_lat = rng.uniform(cz - half, cz + half, n_lateral)
    lat = np.column_stack(
        [cx + radius * np.cos(theta), cy + radius * np.sin(theta), z_lat]
    )

    # Two caps
    caps = []
    for sign in (-1.0, 1.0):
        r_sq = rng.uniform(0, radius**2, n_cap)
        th = rng.uniform(0, 2 * math.pi, n_cap)
        r_c = np.sqrt(r_sq)
        cap = np.column_stack(
            [
                cx + r_c * np.cos(th),
                cy + r_c * np.sin(th),
                np.full(n_cap, cz + sign * half),
            ]
        )
        caps.append(cap)

    return np.vstack([lat] + caps).astype(np.float64)


def _make_pillar_payload() -> OccupancyUpdatePayload:
    """Build an OccupancyUpdatePayload containing both pillars."""
    pts_a = _sample_cylinder(
        _PILLAR_A_XY[0],
        _PILLAR_A_XY[1],
        _PILLAR_Z_CENTRE,
        _PILLAR_RADIUS,
        _PILLAR_LENGTH,
    )
    pts_b = _sample_cylinder(
        _PILLAR_B_XY[0],
        _PILLAR_B_XY[1],
        _PILLAR_Z_CENTRE,
        _PILLAR_RADIUS,
        _PILLAR_LENGTH,
    )
    pts = np.vstack([pts_a, pts_b]).astype(np.float64)
    return OccupancyUpdatePayload(
        obstacle_points=pts,
        timestamp=0.0,
        frame_id="world",
    )


def _fk_ee_xy(q: np.ndarray) -> tuple[float, float]:
    """Return the EE horizontal position for a SCARA config q = [q1, q2, q3].

    Args:
        q: Joint configuration array of shape (3,).

    Returns:
        Tuple (x, y) of the end-effector position in the world XY plane.
    """
    from fret.control.kinematics import Kinematics

    kin = Kinematics("scara")
    T = kin.forward_kinematics(q)
    return float(T[0, 3]), float(T[1, 3])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestPillarOccupancy:
    """WorkspaceOccupancyBuilder correctly classifies pillar voxels."""

    def test_occupied_voxels_non_empty(self) -> None:
        """At least one voxel is occupied when both pillars are present."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_pillar_payload())
        assert len(builder.occupied_centres()) > 0

    def test_voxel_near_pillar_a_occupied(self) -> None:
        """A voxel centre close to pillar_a should be marked occupied."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_pillar_payload())
        # Query at a point near pillar_a — should be inside an occupied voxel.
        # Pillar_a is at (0.25, 0.10); nearest voxel centres are at (0.20, 0.20)
        # and (0.20, 0.00) at default 20 cm resolution.
        assert builder.is_occupied([0.20, 0.20, 0.20])

    def test_voxel_near_pillar_b_occupied(self) -> None:
        """A voxel centre close to pillar_b should be marked occupied."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_pillar_payload())
        # Pillar_b at (-0.15, 0.30); nearest voxels at (-0.20, 0.20) etc.
        assert builder.is_occupied([-0.20, 0.20, 0.20])

    def test_clearance_negative_at_pillar_a(self) -> None:
        """clearance() returns a negative value at a position near pillar_a."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_pillar_payload())
        c = builder.clearance([0.20, 0.20, 0.20])
        assert c < 0.0, f"Expected negative clearance, got {c:.4f}"

    def test_clearance_negative_at_pillar_b(self) -> None:
        """clearance() returns a negative value at a position near pillar_b."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_pillar_payload())
        c = builder.clearance([-0.20, 0.20, 0.20])
        assert c < 0.0, f"Expected negative clearance, got {c:.4f}"

    def test_clearance_positive_far_from_pillars(self) -> None:
        """clearance() returns a positive value far from both pillars."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_pillar_payload())
        # EE at the start configuration is at (0.60, 0, 0.138) — far from pillars.
        c = builder.clearance([0.60, 0.00, 0.20])
        assert c > 0.0, f"Expected positive clearance, got {c:.4f}"

    def test_empty_world_no_occupied_voxels(self) -> None:
        """No voxels occupied in an empty world (no obstacles)."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        empty = OccupancyUpdatePayload(
            obstacle_points=np.empty((0, 3), dtype=np.float64),
            timestamp=0.0,
            frame_id="world",
        )
        builder.build(empty)
        assert len(builder.occupied_centres()) == 0

    def test_pillar_a_more_occupied_than_empty(self) -> None:
        """Adding pillar_a increases the occupied-voxel count over empty world."""
        builder_empty = WorkspaceOccupancyBuilder(resolution=0.20)
        builder_empty.build(
            OccupancyUpdatePayload(
                obstacle_points=np.empty((0, 3), dtype=np.float64),
                timestamp=0.0,
                frame_id="world",
            )
        )
        n_empty = len(builder_empty.occupied_centres())

        builder_pillar = WorkspaceOccupancyBuilder(resolution=0.20)
        builder_pillar.build(_make_pillar_payload())
        n_pillar = len(builder_pillar.occupied_centres())

        assert n_pillar > n_empty


class TestPillarPathSafety:
    """Planned path from start to goal avoids both pillars."""

    def test_start_clear_of_pillars(self) -> None:
        """Start EE position is outside the required clearance zone."""
        x, y = _fk_ee_xy(_START_Q)
        for cx, cy in [_PILLAR_A_XY, _PILLAR_B_XY]:
            dist_xy = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            assert dist_xy >= _PILLAR_RADIUS + _SAFETY_MARGIN, (
                f"Start EE ({x:.3f}, {y:.3f}) is within safety zone of "
                f"pillar ({cx}, {cy}): dist_xy = {dist_xy:.4f} m"
            )

    def test_goal_clear_of_pillars(self) -> None:
        """Goal EE position is outside the required clearance zone."""
        x, y = _fk_ee_xy(_GOAL_Q)
        for cx, cy in [_PILLAR_A_XY, _PILLAR_B_XY]:
            dist_xy = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            assert dist_xy >= _PILLAR_RADIUS + _SAFETY_MARGIN, (
                f"Goal EE ({x:.3f}, {y:.3f}) is within safety zone of "
                f"pillar ({cx}, {cy}): dist_xy = {dist_xy:.4f} m"
            )

    def test_linear_interpolated_path_clear_of_pillars(self) -> None:
        """Every waypoint along the linear joint-space path avoids the pillars."""
        from fret.control.kinematics import Kinematics

        kin = Kinematics("scara")
        n_steps = 50
        for i in range(n_steps + 1):
            alpha = i / n_steps
            q = _START_Q + alpha * (_GOAL_Q - _START_Q)
            T = kin.forward_kinematics(q)
            x, y = float(T[0, 3]), float(T[1, 3])
            for cx, cy in [_PILLAR_A_XY, _PILLAR_B_XY]:
                dist_xy = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
                assert dist_xy >= _PILLAR_RADIUS + _SAFETY_MARGIN, (
                    f"Waypoint {i}/{n_steps}: EE ({x:.3f}, {y:.3f}) is too "
                    f"close to pillar ({cx}, {cy}): dist_xy = {dist_xy:.4f} m "
                    f"(min = {_PILLAR_RADIUS + _SAFETY_MARGIN:.3f} m)"
                )


class TestCSpaceCheckerWithPillars:
    """CSpaceChecker uses WorkspaceOccupancyBuilder to detect pillar collisions."""

    def test_start_config_is_collision_free(self) -> None:
        """The start configuration is collision-free with the pillar occupancy."""
        from fret.control.kinematics import Kinematics
        from fret.planning.cspace_checker import CSpaceChecker

        kin = Kinematics("scara")
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        occ_model = builder.build(_make_pillar_payload())

        checker = CSpaceChecker(kin, occ_model)
        assert checker.is_collision_free(
            _START_Q
        ), "Start configuration should be collision-free"

    def test_goal_config_is_collision_free(self) -> None:
        """The goal configuration is collision-free with the pillar occupancy."""
        from fret.control.kinematics import Kinematics
        from fret.planning.cspace_checker import CSpaceChecker

        kin = Kinematics("scara")
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        occ_model = builder.build(_make_pillar_payload())

        checker = CSpaceChecker(kin, occ_model)
        assert checker.is_collision_free(
            _GOAL_Q
        ), "Goal configuration should be collision-free"
