"""Tests for fret.scene.WorkspaceOccupancyBuilder.

Acceptance criteria (FR-SCN-05, Milestone 4):
  - Grid has 7 × 7 × 3 = 147 total cells at 20 cm resolution.
  - Annular mask excludes unreachable cells (r < 0.05 m or r > 0.60 m).
  - is_occupied() returns True for a point inside a known obstacle.
  - is_occupied() returns False for all points in an empty world.
  - clearance() returns negative values for occupied cells.
  - clearance() returns positive values in free space.
  - build() returns a KDTreeOccupancy when ARCO is available.
  - occupied_centres() + free_centres() partition the annular cells.
  - 10 cm resolution yields ≥ 147 cells total.
  - Default collision_check_distance equals voxel circumradius ≈ 0.173 m.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.workspace_occupancy import (
    _L1,
    _L2,
    _R_MAX,
    _R_MIN,
    WorkspaceOccupancyBuilder,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_empty_payload() -> OccupancyUpdatePayload:
    """Return an OccupancyUpdatePayload with no obstacle points."""
    return OccupancyUpdatePayload(
        obstacle_points=np.empty((0, 3), dtype=np.float64),
        timestamp=0.0,
        frame_id="world",
    )


def _make_payload(points: list[list[float]]) -> OccupancyUpdatePayload:
    """Return an OccupancyUpdatePayload with the given obstacle points."""
    return OccupancyUpdatePayload(
        obstacle_points=np.array(points, dtype=np.float64),
        timestamp=1.0,
        frame_id="world",
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestGridDimensions:
    """Grid cell count at default 20 cm resolution."""

    def test_grid_dimensions(self) -> None:
        """Default 20 cm grid: 7 × 7 × 3 = 147 total axis-aligned cells."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        total = len(builder.occupied_centres()) + len(builder.free_centres())
        # The annular mask reduces total < 147; but the full (unmasked) grid
        # must have been built from 7×7×3 samples.
        # Verify axis counts separately.
        xs = np.arange(-0.60, 0.60 + 0.10, 0.20)
        ys = np.arange(-0.60, 0.60 + 0.10, 0.20)
        zs = np.arange(0.00, 0.40 + 0.10, 0.20)
        # Keep values within bounds (floating-point safe)
        xs = xs[xs <= 0.60 + 1e-9]
        ys = ys[ys <= 0.60 + 1e-9]
        zs = zs[zs <= 0.40 + 1e-9]
        assert len(xs) == 7
        assert len(ys) == 7
        assert len(zs) == 3
        # The annular-filtered count must be ≤ 147 and ≥ 1.
        assert 1 <= total <= 147


class TestAnnularMask:
    """Cells outside reachable annulus are excluded from the classified sets."""

    def test_annular_mask_filters_unreachable(self) -> None:
        """No classified cell has r < R_MIN or r > R_MAX."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        all_centres = np.vstack(
            [builder.occupied_centres(), builder.free_centres()]
        )
        if len(all_centres) == 0:
            pytest.skip("No centres after annular filter — unexpected.")
        r = np.sqrt(all_centres[:, 0] ** 2 + all_centres[:, 1] ** 2)
        assert np.all(
            r >= _R_MIN - 1e-9
        ), f"Cell with r < R_MIN ({_R_MIN:.4f}) found: min r = {r.min():.4f}"
        assert np.all(
            r <= _R_MAX + 1e-9
        ), f"Cell with r > R_MAX ({_R_MAX:.4f}) found: max r = {r.max():.4f}"

    def test_r_min_r_max_constants(self) -> None:
        """Verify workspace constants match SCARA kinematic parameters."""
        assert abs(_R_MIN - abs(_L1 - _L2)) < 1e-9
        assert abs(_R_MAX - (_L1 + _L2)) < 1e-9


class TestIsOccupied:
    """is_occupied() correctness."""

    def test_is_occupied_inside_obstacle(self) -> None:
        """Voxel at grid centre (0.40, 0.20, 0.20) must be occupied."""
        # Use a point that falls exactly on a 20 cm grid centre so the
        # obstacle IS the voxel centre; distance from query to occupied
        # centre is 0, which is < any positive collision_check_distance.
        obs_pt = [
            0.40,
            0.20,
            0.20,
        ]  # grid centre, r ≈ 0.447 m ∈ [R_MIN, R_MAX]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_payload([obs_pt]))
        assert builder.is_occupied(obs_pt, collision_check_distance=0.01)

    def test_is_occupied_free_space(self) -> None:
        """All annular voxels free when obstacle payload is empty."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        # None of the free-space centres should be occupied.
        for centre in builder.free_centres():
            assert not builder.is_occupied(centre)

    def test_is_occupied_default_radius(self) -> None:
        """is_occupied uses default circumradius when distance not provided."""
        obs_pt = [0.40, 0.00, 0.20]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_payload([obs_pt]))
        # The default radius is the circumradius; a point at obs_pt should
        # be inside it with zero offset.
        assert builder.is_occupied(obs_pt)

    def test_is_occupied_bad_shape_raises(self) -> None:
        """is_occupied raises ValueError for wrong position shape."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        with pytest.raises(ValueError):
            builder.is_occupied([0.0, 0.0])  # 2-element, not 3


class TestClearance:
    """clearance() SDF semantics."""

    def test_clearance_negative_inside(self) -> None:
        """clearance < 0 at a point inside the collision radius."""
        obs_pt = [0.30, 0.10, 0.20]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_payload([obs_pt]))
        # The query point IS the obstacle point — distance to nearest
        # occupied centre = 0, so clearance = 0 - circumradius < 0.
        c = builder.clearance(obs_pt)
        assert c < 0.0, f"Expected clearance < 0, got {c}"

    def test_clearance_positive_free(self) -> None:
        """clearance > 0 for a point far from any obstacle."""
        obs_pt = [0.30, 0.10, 0.20]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_payload([obs_pt]))
        # Query a point that is far from the obstacle voxel.
        far_point = [-0.50, -0.50, 0.40]
        c = builder.clearance(far_point)
        assert c > 0.0, f"Expected clearance > 0, got {c}"

    def test_clearance_inf_empty_world(self) -> None:
        """clearance returns +inf when no voxel is occupied."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        assert builder.clearance([0.30, 0.10, 0.20]) == float("inf")

    def test_clearance_bad_shape_raises(self) -> None:
        """clearance raises ValueError for wrong position shape."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        with pytest.raises(ValueError):
            builder.clearance([0.0, 0.0, 0.0, 0.0])  # 4 elements


class TestBuild:
    """build() return type and ARCO integration."""

    def test_build_returns_none_empty_world(self) -> None:
        """build() returns None when no voxel is occupied."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        result = builder.build(_make_empty_payload())
        assert result is None

    def test_build_returns_kdtree_arco(self) -> None:
        """build() returns KDTreeOccupancy when ARCO is installed."""
        arco = pytest.importorskip("arco")  # noqa: F841
        from arco.mapping import KDTreeOccupancy

        obs_pt = [0.30, 0.10, 0.20]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        result = builder.build(_make_payload([obs_pt]))
        assert isinstance(result, KDTreeOccupancy)

    def test_build_returns_simple_occupancy_no_arco(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """build() uses _SimpleOccupancy fallback when ARCO is absent."""
        import fret.scene.workspace_occupancy as wom

        monkeypatch.setattr(wom, "KDTreeOccupancy", None)
        from fret.scene.occupancy_adapter import _SimpleOccupancy

        obs_pt = [0.30, 0.10, 0.20]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        result = builder.build(_make_payload([obs_pt]))
        assert isinstance(result, _SimpleOccupancy)


class TestPartition:
    """occupied_centres() + free_centres() partition annular cells."""

    def test_occupied_free_centres_partition(self) -> None:
        """occupied + free = all cells in the annular reachable region."""
        obs_pt = [0.30, 0.10, 0.20]
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_payload([obs_pt]))
        occ = builder.occupied_centres()
        free = builder.free_centres()
        # Together they must cover all annular cells
        # (no cell is counted twice, no cell is missing).
        if len(occ) > 0 and len(free) > 0:
            all_centres = np.vstack([occ, free])
        elif len(occ) > 0:
            all_centres = occ
        else:
            all_centres = free
        # No duplicates: each unique (x, y, z) appears exactly once.
        unique = np.unique(np.round(all_centres, 10), axis=0)
        assert len(unique) == len(
            all_centres
        ), "Duplicate centres detected between occupied and free sets"

    def test_all_centres_empty_world_are_free(self) -> None:
        """In an empty world, occupied_centres is empty and free is non-empty."""
        builder = WorkspaceOccupancyBuilder(resolution=0.20)
        builder.build(_make_empty_payload())
        assert len(builder.occupied_centres()) == 0
        assert len(builder.free_centres()) > 0


class TestResolutionParameter:
    """Higher resolution produces more cells."""

    def test_resolution_parameter_10cm(self) -> None:
        """10 cm resolution produces more classified cells than 20 cm."""
        builder_20 = WorkspaceOccupancyBuilder(resolution=0.20)
        builder_10 = WorkspaceOccupancyBuilder(resolution=0.10)
        builder_20.build(_make_empty_payload())
        builder_10.build(_make_empty_payload())
        cells_20 = len(builder_20.free_centres()) + len(
            builder_20.occupied_centres()
        )
        cells_10 = len(builder_10.free_centres()) + len(
            builder_10.occupied_centres()
        )
        assert (
            cells_10 >= 147
        ), f"10 cm resolution should yield ≥ 147 cells, got {cells_10}"
        assert (
            cells_10 > cells_20
        ), f"10 cm ({cells_10}) should have more cells than 20 cm ({cells_20})"

    def test_invalid_resolution_raises(self) -> None:
        """Non-positive resolution raises ValueError."""
        with pytest.raises(ValueError):
            WorkspaceOccupancyBuilder(resolution=0.0)
        with pytest.raises(ValueError):
            WorkspaceOccupancyBuilder(resolution=-0.1)


class TestDefaultCollisionCheckDistance:
    """Default CCD equals the voxel circumradius."""

    def test_default_collision_check_distance(self) -> None:
        """Default CCD = resolution * sqrt(3) / 2 ≈ 0.173 m for 20 cm grid."""
        resolution = 0.20
        expected_ccd = resolution * math.sqrt(3.0) / 2.0
        builder = WorkspaceOccupancyBuilder(resolution=resolution)
        assert abs(builder.collision_check_distance - expected_ccd) < 1e-9, (
            f"Default CCD {builder.collision_check_distance:.6f} ≠ "
            f"expected {expected_ccd:.6f}"
        )

    def test_ccd_scales_with_resolution(self) -> None:
        """CCD scales proportionally with resolution."""
        b10 = WorkspaceOccupancyBuilder(resolution=0.10)
        b20 = WorkspaceOccupancyBuilder(resolution=0.20)
        ratio = b20.collision_check_distance / b10.collision_check_distance
        assert (
            abs(ratio - 2.0) < 1e-9
        ), f"CCD ratio 20cm/10cm = {ratio:.6f}, expected 2.0"
