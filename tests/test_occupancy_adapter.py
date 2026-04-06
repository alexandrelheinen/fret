"""Unit tests for OccupancyAdapter (Issue 04: KD-Tree Occupancy Adapter).

Validates:
- Constructor argument checking.
- Tree build / update from point cloud snapshots.
- Occupancy query correctness against known fixtures.
- is_free / is_occupied / clearance / nearest_distance semantics.
- Edge cases: empty cloud, single point, duplicate points.
- Staleness detection via is_stale / last_update_stamp.
- Monotonic safety: larger inflation radius never frees an occupied point.
- max_points truncation behavior.
- Micro-benchmarks: update and query timing (informational; not pass/fail thresholds).

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import math
import os
import sys
import time
import unittest

# ---------------------------------------------------------------------------
# Make the fret package importable without a full ROS build
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.perception.occupancy_adapter import OccupancyAdapter

# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


def _pt(x: float, y: float, z: float) -> list:
    """Return a point as a list of floats."""
    return [float(x), float(y), float(z)]


def _grid(n: int, spacing: float = 0.1) -> list:
    """Return a regular grid of n×n×1 points on the z=0 plane."""
    return [
        _pt(i * spacing, j * spacing, 0.0) for i in range(n) for j in range(n)
    ]


# ---------------------------------------------------------------------------
# Constructor validation
# ---------------------------------------------------------------------------


class TestOccupancyAdapterConstructor(unittest.TestCase):
    """Validate OccupancyAdapter constructor argument checking."""

    def test_default_construction_succeeds(self):
        """Default constructor should succeed with sensible defaults."""
        adapter = OccupancyAdapter()
        self.assertAlmostEqual(adapter.inflation_radius, 0.05)
        self.assertEqual(adapter.max_points, 100_000)
        self.assertAlmostEqual(adapter.max_rebuild_age, 2.0)
        self.assertEqual(adapter.point_count, 0)
        self.assertIsNone(adapter.last_update_stamp)
        self.assertTrue(adapter.is_stale)

    def test_explicit_construction_succeeds(self):
        """All parameters can be set explicitly."""
        adapter = OccupancyAdapter(
            inflation_radius=0.1,
            max_points=500,
            max_rebuild_age=5.0,
        )
        self.assertAlmostEqual(adapter.inflation_radius, 0.1)
        self.assertEqual(adapter.max_points, 500)
        self.assertAlmostEqual(adapter.max_rebuild_age, 5.0)

    def test_zero_inflation_radius_raises(self):
        """inflation_radius=0.0 must be rejected."""
        with self.assertRaises(ValueError):
            OccupancyAdapter(inflation_radius=0.0)

    def test_negative_inflation_radius_raises(self):
        """Negative inflation_radius must be rejected."""
        with self.assertRaises(ValueError):
            OccupancyAdapter(inflation_radius=-0.01)

    def test_zero_max_points_raises(self):
        """max_points=0 must be rejected."""
        with self.assertRaises(ValueError):
            OccupancyAdapter(max_points=0)

    def test_negative_max_points_raises(self):
        """Negative max_points must be rejected."""
        with self.assertRaises(ValueError):
            OccupancyAdapter(max_points=-1)

    def test_zero_max_rebuild_age_raises(self):
        """max_rebuild_age=0.0 must be rejected."""
        with self.assertRaises(ValueError):
            OccupancyAdapter(max_rebuild_age=0.0)

    def test_negative_max_rebuild_age_raises(self):
        """Negative max_rebuild_age must be rejected."""
        with self.assertRaises(ValueError):
            OccupancyAdapter(max_rebuild_age=-1.0)

    def test_small_positive_inflation_radius_succeeds(self):
        """Very small but positive inflation_radius is valid."""
        adapter = OccupancyAdapter(inflation_radius=1e-6)
        self.assertAlmostEqual(adapter.inflation_radius, 1e-6)

    def test_large_inflation_radius_succeeds(self):
        """Large inflation_radius is valid."""
        adapter = OccupancyAdapter(inflation_radius=10.0)
        self.assertAlmostEqual(adapter.inflation_radius, 10.0)


# ---------------------------------------------------------------------------
# Update / rebuild tests
# ---------------------------------------------------------------------------


class TestOccupancyAdapterUpdate(unittest.TestCase):
    """Validate tree build and update behavior."""

    def test_update_sets_point_count(self):
        """After update, point_count matches the number of input points."""
        adapter = OccupancyAdapter()
        points = [_pt(1, 0, 0), _pt(2, 0, 0), _pt(3, 0, 0)]
        adapter.update(points)
        self.assertEqual(adapter.point_count, 3)

    def test_update_sets_last_update_stamp(self):
        """After update, last_update_stamp is a recent POSIX timestamp."""
        adapter = OccupancyAdapter()
        before = time.time()
        adapter.update([_pt(0, 0, 1)])
        after = time.time()
        self.assertIsNotNone(adapter.last_update_stamp)
        self.assertGreaterEqual(adapter.last_update_stamp, before)
        self.assertLessEqual(adapter.last_update_stamp, after)

    def test_update_clears_previous_tree(self):
        """A second update replaces the previous tree entirely."""
        adapter = OccupancyAdapter(inflation_radius=0.05)
        adapter.update([_pt(0, 0, 0)])
        self.assertTrue(adapter.is_occupied(_pt(0, 0, 0)))

        # Replace with a cloud that has no point near the origin.
        adapter.update([_pt(10, 10, 10)])
        self.assertFalse(adapter.is_occupied(_pt(0, 0, 0)))

    def test_update_empty_cloud_yields_zero_count(self):
        """Updating with an empty cloud leaves the tree empty."""
        adapter = OccupancyAdapter()
        adapter.update([])
        self.assertEqual(adapter.point_count, 0)

    def test_update_does_not_modify_input_list(self):
        """update() must not mutate the caller's point list."""
        adapter = OccupancyAdapter()
        original = [_pt(1, 2, 3), _pt(4, 5, 6)]
        copy_before = [list(p) for p in original]
        adapter.update(original)
        self.assertEqual(original, copy_before)

    def test_update_truncates_to_max_points(self):
        """Clouds larger than max_points are truncated."""
        adapter = OccupancyAdapter(max_points=5)
        points = [_pt(float(i), 0, 0) for i in range(20)]
        adapter.update(points)
        self.assertEqual(adapter.point_count, 5)

    def test_update_exactly_max_points_not_truncated(self):
        """Clouds exactly at max_points are not truncated."""
        adapter = OccupancyAdapter(max_points=10)
        points = [_pt(float(i), 0, 0) for i in range(10)]
        adapter.update(points)
        self.assertEqual(adapter.point_count, 10)


# ---------------------------------------------------------------------------
# Occupancy query correctness — known fixtures
# ---------------------------------------------------------------------------


class TestOccupancyAdapterQueries(unittest.TestCase):
    """Fixture-based correctness tests for all occupancy query methods."""

    def setUp(self):
        """Build a simple adapter used by most test cases."""
        # Single obstacle at the origin.
        self.adapter = OccupancyAdapter(inflation_radius=0.1)
        self.adapter.update([_pt(0, 0, 0)])

    # -- nearest_distance -----------------------------------------------

    def test_nearest_distance_exact_hit(self):
        """Query exactly on an obstacle point returns distance 0."""
        d = self.adapter.nearest_distance(_pt(0, 0, 0))
        self.assertAlmostEqual(d, 0.0)

    def test_nearest_distance_known_offset(self):
        """Query at a known offset returns the correct Euclidean distance."""
        # Point at (1, 0, 0) is exactly 1.0 m from the obstacle at origin.
        d = self.adapter.nearest_distance(_pt(1, 0, 0))
        self.assertAlmostEqual(d, 1.0, places=10)

    def test_nearest_distance_diagonal(self):
        """Nearest distance uses Euclidean (not Manhattan) distance."""
        # (1, 1, 1) is sqrt(3) from origin.
        d = self.adapter.nearest_distance(_pt(1, 1, 1))
        self.assertAlmostEqual(d, math.sqrt(3), places=10)

    def test_nearest_distance_empty_tree_returns_inf(self):
        """Querying an empty tree returns math.inf."""
        adapter = OccupancyAdapter()
        d = adapter.nearest_distance(_pt(0, 0, 0))
        self.assertEqual(d, math.inf)

    # -- is_occupied / is_free ------------------------------------------

    def test_is_occupied_inside_inflation(self):
        """Point inside inflation radius is occupied."""
        # 0.05 m from origin; inflation_radius = 0.1 m → occupied.
        self.assertTrue(self.adapter.is_occupied(_pt(0.05, 0, 0)))

    def test_is_occupied_on_inflation_boundary(self):
        """Point exactly on the inflation boundary is occupied (inclusive)."""
        self.assertTrue(self.adapter.is_occupied(_pt(0.1, 0, 0)))

    def test_is_free_outside_inflation(self):
        """Point just beyond inflation radius is free."""
        # 0.1001 m > inflation_radius 0.1 m → free.
        self.assertTrue(self.adapter.is_free(_pt(0.1001, 0, 0)))

    def test_is_occupied_far_point_is_free(self):
        """Point far from obstacles is free."""
        self.assertFalse(self.adapter.is_occupied(_pt(5, 5, 5)))

    def test_is_free_and_is_occupied_are_complements(self):
        """is_free always equals not is_occupied."""
        for x in [-1.0, -0.05, 0.0, 0.05, 0.1, 0.2, 1.0]:
            pt = _pt(x, 0, 0)
            self.assertEqual(
                self.adapter.is_free(pt),
                not self.adapter.is_occupied(pt),
            )

    def test_empty_tree_all_points_free(self):
        """All queries return is_free=True when the tree is empty."""
        adapter = OccupancyAdapter()
        for pt in [_pt(0, 0, 0), _pt(1, 2, 3), _pt(-5, 0, 0.5)]:
            self.assertTrue(adapter.is_free(pt))
            self.assertFalse(adapter.is_occupied(pt))

    # -- clearance -------------------------------------------------------

    def test_clearance_inside_inflation_is_zero(self):
        """Clearance is 0.0 when inside the inflation radius."""
        self.assertAlmostEqual(self.adapter.clearance(_pt(0.05, 0, 0)), 0.0)

    def test_clearance_on_inflation_boundary_is_zero(self):
        """Clearance is 0.0 on the inflation boundary."""
        self.assertAlmostEqual(
            self.adapter.clearance(_pt(0.1, 0, 0)), 0.0, places=10
        )

    def test_clearance_outside_inflation(self):
        """Clearance equals nearest_distance minus inflation_radius."""
        # Query at (0.5, 0, 0); nearest obstacle at origin; dist = 0.5.
        expected = 0.5 - 0.1  # 0.4 m clearance
        self.assertAlmostEqual(
            self.adapter.clearance(_pt(0.5, 0, 0)), expected, places=10
        )

    def test_clearance_empty_tree_returns_inf(self):
        """Clearance returns math.inf when the tree is empty."""
        adapter = OccupancyAdapter()
        self.assertEqual(adapter.clearance(_pt(1, 2, 3)), math.inf)

    def test_clearance_is_non_negative(self):
        """Clearance is always non-negative for any query point."""
        points = [_pt(float(i) * 0.5, 0.0, 0.0) for i in range(5)]
        self.adapter.update(points)
        for x in [-2.0, -0.5, 0.0, 0.1, 0.25, 1.0, 5.0]:
            c = self.adapter.clearance(_pt(x, 0, 0))
            self.assertGreaterEqual(c, 0.0)


# ---------------------------------------------------------------------------
# Multiple-obstacle fixtures
# ---------------------------------------------------------------------------


class TestOccupancyAdapterMultipleObstacles(unittest.TestCase):
    """Correctness tests with more than one obstacle point."""

    def test_nearest_of_two_obstacles(self):
        """nearest_distance returns the distance to the closer of two points."""
        adapter = OccupancyAdapter(inflation_radius=0.05)
        adapter.update([_pt(1, 0, 0), _pt(3, 0, 0)])
        # Query at (2, 0, 0): equidistant (1 m each).
        self.assertAlmostEqual(adapter.nearest_distance(_pt(2, 0, 0)), 1.0)
        # Query at (0.5, 0, 0): closer to (1, 0, 0) → 0.5 m.
        self.assertAlmostEqual(adapter.nearest_distance(_pt(0.5, 0, 0)), 0.5)

    def test_occupied_near_second_obstacle(self):
        """is_occupied returns True near any obstacle, not just the first."""
        adapter = OccupancyAdapter(inflation_radius=0.1)
        adapter.update([_pt(0, 0, 0), _pt(5, 0, 0)])
        # Point near first obstacle.
        self.assertTrue(adapter.is_occupied(_pt(0.05, 0, 0)))
        # Point near second obstacle.
        self.assertTrue(adapter.is_occupied(_pt(5.05, 0, 0)))
        # Point far from both.
        self.assertFalse(adapter.is_occupied(_pt(2.5, 0, 0)))

    def test_large_grid_nearest_distance(self):
        """KD-tree returns correct nearest for a grid of 100 points."""
        n = 10
        adapter = OccupancyAdapter(inflation_radius=0.05)
        adapter.update(_grid(n, spacing=1.0))
        # Query exactly on grid point (3, 4, 0) → distance 0.
        self.assertAlmostEqual(
            adapter.nearest_distance(_pt(3, 4, 0)), 0.0, places=10
        )
        # Query at (3.4, 4, 0) → nearest is (3, 4, 0) or (4, 4, 0), dist 0.4.
        self.assertAlmostEqual(
            adapter.nearest_distance(_pt(3.4, 4, 0)), 0.4, places=10
        )


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------


class TestOccupancyAdapterEdgeCases(unittest.TestCase):
    """Edge-case and boundary tests."""

    def test_single_point_cloud(self):
        """A single-point cloud supports all query methods correctly."""
        adapter = OccupancyAdapter(inflation_radius=0.1)
        adapter.update([_pt(2, 3, 4)])
        self.assertAlmostEqual(adapter.nearest_distance(_pt(2, 3, 4)), 0.0)
        self.assertTrue(adapter.is_occupied(_pt(2, 3, 4)))
        self.assertFalse(adapter.is_free(_pt(2, 3, 4)))
        self.assertAlmostEqual(adapter.clearance(_pt(2, 3, 4)), 0.0)

    def test_duplicate_points_accepted(self):
        """Duplicate points do not raise an error."""
        adapter = OccupancyAdapter(inflation_radius=0.05)
        adapter.update([_pt(1, 1, 1)] * 10)
        self.assertEqual(adapter.point_count, 10)
        self.assertAlmostEqual(adapter.nearest_distance(_pt(1, 1, 1)), 0.0)

    def test_negative_coordinates_handled(self):
        """Points with negative coordinates are queried correctly."""
        adapter = OccupancyAdapter(inflation_radius=0.1)
        adapter.update([_pt(-3, -2, -1)])
        self.assertAlmostEqual(adapter.nearest_distance(_pt(-3, -2, -1)), 0.0)
        self.assertTrue(adapter.is_occupied(_pt(-3, -2, -1)))

    def test_update_then_empty_update(self):
        """Updating with empty cloud after a populated one resets everything."""
        adapter = OccupancyAdapter(inflation_radius=0.05)
        adapter.update([_pt(0, 0, 0)])
        self.assertTrue(adapter.is_occupied(_pt(0, 0, 0)))

        adapter.update([])
        self.assertEqual(adapter.point_count, 0)
        self.assertTrue(adapter.is_free(_pt(0, 0, 0)))

    def test_large_cloud_single_query(self):
        """KD-tree handles a cloud of 5000 points without error."""
        adapter = OccupancyAdapter(inflation_radius=0.05)
        cloud = _grid(70, spacing=0.1)  # 4900 points on a 70×70 grid
        adapter.update(cloud)
        self.assertEqual(adapter.point_count, 4900)
        # Query at grid point (0, 0, 0) → nearest distance must be 0.
        self.assertAlmostEqual(adapter.nearest_distance(_pt(0, 0, 0)), 0.0)


# ---------------------------------------------------------------------------
# Staleness detection
# ---------------------------------------------------------------------------


class TestOccupancyAdapterStaleness(unittest.TestCase):
    """Validate is_stale and last_update_stamp behavior."""

    def test_is_stale_before_any_update(self):
        """A freshly constructed adapter is stale."""
        adapter = OccupancyAdapter()
        self.assertTrue(adapter.is_stale)

    def test_is_not_stale_immediately_after_update(self):
        """An adapter is not stale immediately after update."""
        adapter = OccupancyAdapter(max_rebuild_age=10.0)
        adapter.update([_pt(0, 0, 1)])
        self.assertFalse(adapter.is_stale)

    def test_is_stale_after_age_exceeded(self):
        """An adapter becomes stale once max_rebuild_age is exceeded."""
        adapter = OccupancyAdapter(max_rebuild_age=0.05)
        adapter.update([_pt(0, 0, 1)])
        time.sleep(0.1)
        self.assertTrue(adapter.is_stale)

    def test_last_update_stamp_none_before_update(self):
        """last_update_stamp is None before any update."""
        adapter = OccupancyAdapter()
        self.assertIsNone(adapter.last_update_stamp)

    def test_last_update_stamp_advances_on_second_update(self):
        """last_update_stamp advances monotonically with successive updates."""
        adapter = OccupancyAdapter()
        adapter.update([_pt(0, 0, 0)])
        stamp_1 = adapter.last_update_stamp
        time.sleep(0.01)
        adapter.update([_pt(1, 1, 1)])
        stamp_2 = adapter.last_update_stamp
        self.assertGreater(stamp_2, stamp_1)


# ---------------------------------------------------------------------------
# Monotonic safety property tests
# ---------------------------------------------------------------------------


class TestOccupancyAdapterMonotonicSafety(unittest.TestCase):
    """Property tests: inflation radius changes must satisfy monotonic safety.

    Safety monotonicity guarantees:
    - Increasing inflation_radius never converts an occupied point to free.
    - Decreasing inflation_radius never converts a free point to occupied.
    """

    _OBSTACLE = _pt(0.0, 0.0, 0.0)
    _QUERY_POINTS = [
        _pt(0.0, 0.0, 0.0),  # on obstacle
        _pt(0.05, 0.0, 0.0),
        _pt(0.1, 0.0, 0.0),
        _pt(0.15, 0.0, 0.0),
        _pt(0.5, 0.0, 0.0),
        _pt(1.0, 0.0, 0.0),
        _pt(1.0, 1.0, 0.0),
    ]
    _RADII = [0.01, 0.05, 0.1, 0.15, 0.2, 0.5, 1.0]

    def _occupied_set(self, radius: float) -> set:
        """Return the set of query-point indices that are occupied."""
        adapter = OccupancyAdapter(inflation_radius=radius)
        adapter.update([self._OBSTACLE])
        return {
            i
            for i, pt in enumerate(self._QUERY_POINTS)
            if adapter.is_occupied(pt)
        }

    def test_occupied_set_grows_with_inflation(self):
        """Larger inflation radius always produces a superset of occupied points."""
        prev_occupied = self._occupied_set(self._RADII[0])
        for radius in self._RADII[1:]:
            curr_occupied = self._occupied_set(radius)
            self.assertTrue(
                prev_occupied.issubset(curr_occupied),
                f"Occupied set shrank when radius increased to {radius}: "
                f"prev={prev_occupied}, curr={curr_occupied}",
            )
            prev_occupied = curr_occupied

    def test_free_set_shrinks_with_inflation(self):
        """Larger inflation radius never creates new free points."""
        all_indices = set(range(len(self._QUERY_POINTS)))
        prev_free = all_indices - self._occupied_set(self._RADII[0])
        for radius in self._RADII[1:]:
            curr_free = all_indices - self._occupied_set(radius)
            self.assertTrue(
                curr_free.issubset(prev_free),
                f"Free set grew when radius increased to {radius}: "
                f"prev={prev_free}, curr={curr_free}",
            )
            prev_free = curr_free

    def test_clearance_decreases_with_inflation(self):
        """Clearance is non-increasing as inflation radius grows."""
        prev_adapter = OccupancyAdapter(inflation_radius=self._RADII[0])
        prev_adapter.update([self._OBSTACLE])
        for radius in self._RADII[1:]:
            curr_adapter = OccupancyAdapter(inflation_radius=radius)
            curr_adapter.update([self._OBSTACLE])
            for pt in self._QUERY_POINTS:
                prev_c = prev_adapter.clearance(pt)
                curr_c = curr_adapter.clearance(pt)
                self.assertLessEqual(
                    curr_c,
                    prev_c + 1e-12,  # tolerance for floating-point rounding
                    f"Clearance increased from {prev_c} to {curr_c} at {pt} "
                    f"when radius grew to {radius}",
                )
            prev_adapter = curr_adapter


# ---------------------------------------------------------------------------
# Micro-benchmarks (informational — no timing thresholds)
# ---------------------------------------------------------------------------


class TestOccupancyAdapterBenchmark(unittest.TestCase):
    """Micro-benchmarks for update and query timing.

    These tests always pass; they report timings to stdout when run with
    verbosity enabled (``-v``).  Formal performance thresholds are tracked
    in Issue 09 and enforced separately.
    """

    def _time_update(self, n_points: int) -> float:
        """Return the time in seconds to build a KD-tree from n_points."""
        cloud = _grid(int(math.ceil(math.sqrt(n_points))), spacing=0.05)
        cloud = cloud[:n_points]
        adapter = OccupancyAdapter()
        t0 = time.perf_counter()
        adapter.update(cloud)
        return time.perf_counter() - t0

    def _time_queries(self, n_points: int, n_queries: int) -> float:
        """Return the time in seconds to execute n_queries is_occupied calls."""
        cloud = _grid(int(math.ceil(math.sqrt(n_points))), spacing=0.05)
        cloud = cloud[:n_points]
        adapter = OccupancyAdapter()
        adapter.update(cloud)
        queries = [_pt(i * 0.03, i * 0.03, 0.0) for i in range(n_queries)]
        t0 = time.perf_counter()
        for q in queries:
            adapter.is_occupied(q)
        return time.perf_counter() - t0

    def test_benchmark_update_1k(self):
        """Update benchmark: 1_000 points (informational)."""
        elapsed = self._time_update(1_000)
        print(f"\n[BENCH] update(1k pts): {elapsed * 1000:.2f} ms")
        self.assertGreater(elapsed, 0.0)  # trivially true; validates run

    def test_benchmark_update_10k(self):
        """Update benchmark: 10_000 points (informational)."""
        elapsed = self._time_update(10_000)
        print(f"\n[BENCH] update(10k pts): {elapsed * 1000:.2f} ms")
        self.assertGreater(elapsed, 0.0)

    def test_benchmark_query_1k(self):
        """Query benchmark: 1_000 queries against a 1_000-point tree (informational)."""
        elapsed = self._time_queries(1_000, 1_000)
        print(f"\n[BENCH] query(1k pts, 1k queries): {elapsed * 1000:.2f} ms")
        self.assertGreater(elapsed, 0.0)

    def test_benchmark_query_1k_10k_tree(self):
        """Query benchmark: 1_000 queries against a 10_000-point tree (informational)."""
        elapsed = self._time_queries(10_000, 1_000)
        print(f"\n[BENCH] query(10k pts, 1k queries): {elapsed * 1000:.2f} ms")
        self.assertGreater(elapsed, 0.0)


# ---------------------------------------------------------------------------
# Package-level import smoke test
# ---------------------------------------------------------------------------


class TestOccupancyAdapterPackageImport(unittest.TestCase):
    """Verify OccupancyAdapter is accessible via the fret.perception package."""

    def test_import_from_package(self):
        """OccupancyAdapter is re-exported by fret.perception.__init__."""
        from fret.perception import OccupancyAdapter as _OA

        self.assertIs(_OA, OccupancyAdapter)

    def test_all_exports_present(self):
        """fret.perception.__all__ includes OccupancyAdapter."""
        import fret.perception as perception

        self.assertIn("OccupancyAdapter", perception.__all__)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main()
