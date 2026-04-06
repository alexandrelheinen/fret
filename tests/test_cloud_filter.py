"""Unit tests for CloudFilter (Issue 03: Point Cloud Acquisition).

Validates:
- Floor-plane removal at configurable height and margin.
- Range-window filtering by Euclidean distance.
- Spherical self-exclusion zone removal.
- Composite apply() pipeline.
- Constructor validation for invalid parameters.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import math
import os
import sys
import unittest

# ---------------------------------------------------------------------------
# Make the fret package importable when running tests without a full ROS build
# (e.g., locally with ``python3 -m unittest discover tests``).
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.perception.cloud_filter import CloudFilter


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


def _pt(x, y, z):
    """Return a point as a list."""
    return [float(x), float(y), float(z)]


# ---------------------------------------------------------------------------
# Constructor validation tests
# ---------------------------------------------------------------------------


class TestCloudFilterConstructor(unittest.TestCase):
    """Validate CloudFilter constructor argument checking."""

    def test_default_construction_succeeds(self):
        """Default constructor should succeed with sensible defaults."""
        filt = CloudFilter()
        self.assertEqual(filt.floor_z, 0.0)
        self.assertEqual(filt.floor_margin, 0.02)
        self.assertEqual(filt.min_range, 0.1)
        self.assertEqual(filt.max_range, 5.0)
        self.assertEqual(filt.exclusion_zones, [])

    def test_explicit_construction_succeeds(self):
        """All parameters can be set explicitly."""
        filt = CloudFilter(
            floor_z=1.0,
            floor_margin=0.05,
            min_range=0.2,
            max_range=10.0,
            exclusion_zones=[(0.0, 0.0, 0.5, 0.3)],
        )
        self.assertEqual(filt.floor_z, 1.0)
        self.assertEqual(filt.floor_margin, 0.05)
        self.assertEqual(filt.min_range, 0.2)
        self.assertEqual(filt.max_range, 10.0)
        self.assertEqual(len(filt.exclusion_zones), 1)

    def test_negative_floor_margin_raises(self):
        """Negative floor_margin must raise ValueError."""
        with self.assertRaises(ValueError):
            CloudFilter(floor_margin=-0.01)

    def test_negative_min_range_raises(self):
        """Negative min_range must raise ValueError."""
        with self.assertRaises(ValueError):
            CloudFilter(min_range=-1.0)

    def test_max_range_not_greater_than_min_range_raises(self):
        """max_range <= min_range must raise ValueError."""
        with self.assertRaises(ValueError):
            CloudFilter(min_range=5.0, max_range=5.0)
        with self.assertRaises(ValueError):
            CloudFilter(min_range=5.0, max_range=1.0)

    def test_zero_floor_margin_is_valid(self):
        """floor_margin == 0 is permitted (exact floor plane removal)."""
        filt = CloudFilter(floor_margin=0.0)
        self.assertEqual(filt.floor_margin, 0.0)

    def test_zero_min_range_is_valid(self):
        """min_range == 0 is permitted (include points at origin)."""
        filt = CloudFilter(min_range=0.0)
        self.assertEqual(filt.min_range, 0.0)


# ---------------------------------------------------------------------------
# Floor filter tests
# ---------------------------------------------------------------------------


class TestFilterFloor(unittest.TestCase):
    """Unit tests for CloudFilter.filter_floor."""

    def setUp(self):
        self.filt = CloudFilter(floor_z=0.0, floor_margin=0.02)

    def test_point_above_floor_band_is_kept(self):
        """A point well above the floor band must pass the filter."""
        pts = [_pt(1.0, 0.0, 0.1)]
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], pts[0])

    def test_point_on_floor_plane_is_removed(self):
        """A point exactly at floor_z must be removed."""
        pts = [_pt(0.0, 0.0, 0.0)]
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 0)

    def test_point_within_margin_is_removed(self):
        """A point within floor_margin above floor_z must be removed."""
        pts = [_pt(0.0, 0.0, 0.01)]  # 0.01 < 0.02 (floor_margin)
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 0)

    def test_point_at_margin_boundary_is_removed(self):
        """A point exactly at floor_z + floor_margin must be removed."""
        pts = [_pt(0.0, 0.0, 0.02)]  # == floor_z + floor_margin
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 0)

    def test_point_just_above_margin_is_kept(self):
        """A point just above floor_z + floor_margin must be retained."""
        pts = [_pt(0.0, 0.0, 0.021)]
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 1)

    def test_below_floor_is_removed(self):
        """A point below the floor plane must also be removed."""
        pts = [_pt(0.0, 0.0, -0.5)]
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 0)

    def test_empty_input_returns_empty(self):
        """Empty input must return an empty list."""
        self.assertEqual(self.filt.filter_floor([]), [])

    def test_mixed_points_correct_split(self):
        """Mixed input: only above-margin points are kept."""
        pts = [
            _pt(0.0, 0.0, -0.1),  # below floor
            _pt(0.0, 0.0, 0.0),  # at floor
            _pt(0.0, 0.0, 0.01),  # inside margin
            _pt(0.0, 0.0, 0.02),  # at margin (excluded)
            _pt(0.0, 0.0, 0.05),  # above margin (kept)
            _pt(1.0, 2.0, 1.0),  # well above (kept)
        ]
        result = self.filt.filter_floor(pts)
        self.assertEqual(len(result), 2)
        self.assertIn(_pt(0.0, 0.0, 0.05), result)
        self.assertIn(_pt(1.0, 2.0, 1.0), result)

    def test_non_zero_floor_z(self):
        """Floor filter respects a non-zero floor_z."""
        filt = CloudFilter(floor_z=1.0, floor_margin=0.05)
        pts = [_pt(0.0, 0.0, 1.04), _pt(0.0, 0.0, 1.06)]
        result = filt.filter_floor(pts)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], _pt(0.0, 0.0, 1.06))

    def test_input_list_is_not_modified(self):
        """filter_floor must not mutate its input."""
        pts = [_pt(0.0, 0.0, 0.0), _pt(0.0, 0.0, 1.0)]
        original = [list(p) for p in pts]
        self.filt.filter_floor(pts)
        self.assertEqual(pts, original)


# ---------------------------------------------------------------------------
# Range filter tests
# ---------------------------------------------------------------------------


class TestFilterRange(unittest.TestCase):
    """Unit tests for CloudFilter.filter_range."""

    def setUp(self):
        self.filt = CloudFilter(min_range=0.1, max_range=5.0)

    def test_in_range_point_is_kept(self):
        """A point inside [min_range, max_range] must be retained."""
        pts = [_pt(1.0, 0.0, 0.0)]  # distance = 1.0
        result = self.filt.filter_range(pts)
        self.assertEqual(len(result), 1)

    def test_too_close_point_is_removed(self):
        """A point closer than min_range must be removed."""
        pts = [_pt(0.05, 0.0, 0.0)]  # distance = 0.05 < 0.1
        result = self.filt.filter_range(pts)
        self.assertEqual(len(result), 0)

    def test_too_far_point_is_removed(self):
        """A point farther than max_range must be removed."""
        pts = [_pt(6.0, 0.0, 0.0)]  # distance = 6.0 > 5.0
        result = self.filt.filter_range(pts)
        self.assertEqual(len(result), 0)

    def test_point_at_min_range_is_kept(self):
        """A point exactly at min_range must be retained."""
        pts = [_pt(0.1, 0.0, 0.0)]  # distance = 0.1
        result = self.filt.filter_range(pts)
        self.assertEqual(len(result), 1)

    def test_point_at_max_range_is_kept(self):
        """A point exactly at max_range must be retained."""
        pts = [_pt(5.0, 0.0, 0.0)]  # distance = 5.0
        result = self.filt.filter_range(pts)
        self.assertEqual(len(result), 1)

    def test_distance_computed_correctly_3d(self):
        """Euclidean distance is computed across all three axes."""
        # sqrt(3^2 + 4^2 + 0^2) = 5.0 — exactly at max_range
        pts = [_pt(3.0, 4.0, 0.0)]
        result = self.filt.filter_range(pts)
        self.assertEqual(len(result), 1)

    def test_empty_input_returns_empty(self):
        """Empty input must return an empty list."""
        self.assertEqual(self.filt.filter_range([]), [])

    def test_input_list_is_not_modified(self):
        """filter_range must not mutate its input."""
        pts = [_pt(0.05, 0.0, 0.0), _pt(1.0, 0.0, 0.0)]
        original = [list(p) for p in pts]
        self.filt.filter_range(pts)
        self.assertEqual(pts, original)


# ---------------------------------------------------------------------------
# Self filter tests
# ---------------------------------------------------------------------------


class TestFilterSelf(unittest.TestCase):
    """Unit tests for CloudFilter.filter_self."""

    def _filt_with_zone(self, cx, cy, cz, radius):
        return CloudFilter(
            exclusion_zones=[(cx, cy, cz, radius)]
        )

    def test_point_outside_zone_is_kept(self):
        """A point outside the exclusion sphere must be retained."""
        filt = self._filt_with_zone(0.0, 0.0, 0.5, 0.3)
        pts = [_pt(1.0, 0.0, 0.5)]  # distance from center = 1.0 > 0.3
        result = filt.filter_self(pts)
        self.assertEqual(len(result), 1)

    def test_point_inside_zone_is_removed(self):
        """A point inside the exclusion sphere must be removed."""
        filt = self._filt_with_zone(0.0, 0.0, 0.5, 0.3)
        pts = [_pt(0.0, 0.0, 0.5)]  # at center — distance = 0 < 0.3
        result = filt.filter_self(pts)
        self.assertEqual(len(result), 0)

    def test_point_on_zone_surface_is_removed(self):
        """A point on the sphere surface (distance == radius) is removed."""
        filt = self._filt_with_zone(0.0, 0.0, 0.0, 1.0)
        pts = [_pt(1.0, 0.0, 0.0)]  # distance = 1.0 == radius
        result = filt.filter_self(pts)
        self.assertEqual(len(result), 0)

    def test_no_exclusion_zones_keeps_all(self):
        """With no zones, all points are retained."""
        filt = CloudFilter(exclusion_zones=[])
        pts = [_pt(0.0, 0.0, 0.0), _pt(1.0, 1.0, 1.0)]
        result = filt.filter_self(pts)
        self.assertEqual(len(result), 2)

    def test_multiple_exclusion_zones(self):
        """A point inside any zone is removed."""
        filt = CloudFilter(
            exclusion_zones=[
                (0.0, 0.0, 0.0, 0.5),  # zone A
                (2.0, 0.0, 0.0, 0.5),  # zone B
            ]
        )
        pts = [
            _pt(0.1, 0.0, 0.0),  # inside zone A
            _pt(2.1, 0.0, 0.0),  # inside zone B
            _pt(1.0, 1.0, 0.0),  # outside both
        ]
        result = filt.filter_self(pts)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], _pt(1.0, 1.0, 0.0))

    def test_empty_input_returns_empty(self):
        """Empty input must return an empty list."""
        filt = self._filt_with_zone(0.0, 0.0, 0.0, 1.0)
        self.assertEqual(filt.filter_self([]), [])

    def test_input_list_is_not_modified(self):
        """filter_self must not mutate its input."""
        filt = self._filt_with_zone(0.0, 0.0, 0.0, 0.5)
        pts = [_pt(0.0, 0.0, 0.0), _pt(2.0, 0.0, 0.0)]
        original = [list(p) for p in pts]
        filt.filter_self(pts)
        self.assertEqual(pts, original)


# ---------------------------------------------------------------------------
# Composite apply tests
# ---------------------------------------------------------------------------


class TestFilterApply(unittest.TestCase):
    """Unit tests for the composite CloudFilter.apply() pipeline."""

    def setUp(self):
        self.filt = CloudFilter(
            floor_z=0.0,
            floor_margin=0.02,
            min_range=0.1,
            max_range=5.0,
            exclusion_zones=[(0.0, 0.0, 1.0, 0.2)],
        )

    def test_obstacle_point_passes_all_filters(self):
        """A genuine obstacle point must survive all three filter stages."""
        pts = [_pt(1.0, 0.0, 1.0)]  # z=1.0 > 0.02, dist≈1.41, outside zone
        result = self.filt.apply(pts)
        self.assertEqual(len(result), 1)

    def test_floor_point_removed_by_apply(self):
        """A floor-level point must be removed during apply()."""
        pts = [_pt(1.0, 0.0, 0.01)]  # z=0.01 inside floor band
        result = self.filt.apply(pts)
        self.assertEqual(len(result), 0)

    def test_out_of_range_point_removed_by_apply(self):
        """A point beyond max_range must be removed during apply()."""
        pts = [_pt(6.0, 0.0, 0.5)]  # dist=6.02 > 5.0
        result = self.filt.apply(pts)
        self.assertEqual(len(result), 0)

    def test_self_point_removed_by_apply(self):
        """A point inside an exclusion zone must be removed during apply()."""
        pts = [_pt(0.0, 0.0, 1.0)]  # center of zone
        result = self.filt.apply(pts)
        self.assertEqual(len(result), 0)

    def test_apply_on_empty_list_returns_empty(self):
        """apply() on an empty list must return an empty list."""
        self.assertEqual(self.filt.apply([]), [])

    def test_apply_returns_correct_point_count(self):
        """apply() must keep only obstacle points from a mixed input."""
        pts = [
            _pt(0.0, 0.0, 0.0),  # floor
            _pt(6.0, 0.0, 1.0),  # too far
            _pt(0.0, 0.0, 1.0),  # inside exclusion zone
            _pt(1.0, 0.5, 1.5),  # valid obstacle point
            _pt(2.0, 1.0, 0.8),  # valid obstacle point
        ]
        result = self.filt.apply(pts)
        self.assertEqual(len(result), 2)
        self.assertIn(_pt(1.0, 0.5, 1.5), result)
        self.assertIn(_pt(2.0, 1.0, 0.8), result)

    def test_apply_does_not_mutate_input(self):
        """apply() must not modify the original input list."""
        pts = [_pt(1.0, 0.0, 1.0), _pt(0.0, 0.0, 0.0)]
        original = [list(p) for p in pts]
        self.filt.apply(pts)
        self.assertEqual(pts, original)

    def test_exclusion_zones_property_returns_copy(self):
        """exclusion_zones property must return a copy, not a live reference."""
        filt = CloudFilter(exclusion_zones=[(1.0, 0.0, 0.0, 0.5)])
        zones = filt.exclusion_zones
        zones.clear()
        self.assertEqual(len(filt.exclusion_zones), 1)


if __name__ == "__main__":
    unittest.main()
