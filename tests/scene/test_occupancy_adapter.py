"""Tests for fret.scene.OccupancyAdapter.

Acceptance criteria (FR-SCN-03, FR-SCN-04):
  - ``update()`` accepts an ``OccupancyUpdatePayload`` without raising.
  - ``get_occupancy()`` returns a KDTreeOccupancy-compatible object after
    at least one ``update()`` call.
  - ``get_occupancy()`` raises ``RuntimeError`` if called before ``update()``.
  - Subsequent ``update()`` calls atomically replace the previous occupancy.
  - The fallback ``_SimpleOccupancy`` model returns positive clearance for
    free positions and ``inf`` when the cloud is empty.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.occupancy_adapter import OccupancyAdapter, _SimpleOccupancy


def _make_payload(n: int = 5) -> OccupancyUpdatePayload:
    return OccupancyUpdatePayload(
        obstacle_points=np.random.default_rng(42).random((n, 3)),
        timestamp=1.0,
        frame_id="world",
    )


def test_construction() -> None:
    OccupancyAdapter()


def test_raises_before_update() -> None:
    """get_occupancy() must raise RuntimeError when called before update()."""
    adapter = OccupancyAdapter()
    with pytest.raises(RuntimeError):
        adapter.get_occupancy()


def test_update_then_get_occupancy() -> None:
    """get_occupancy() returns an object after a successful update()."""
    adapter = OccupancyAdapter()
    adapter.update(_make_payload())
    occ = adapter.get_occupancy()
    assert occ is not None


def test_sequential_updates_replace_occupancy() -> None:
    """A second update() replaces the previous occupancy model."""
    adapter = OccupancyAdapter()
    adapter.update(_make_payload(n=3))
    first = adapter.get_occupancy()
    adapter.update(_make_payload(n=10))
    second = adapter.get_occupancy()
    # After a new update the returned object is new (different identity).
    assert first is not second


# ---------------------------------------------------------------------------
# _SimpleOccupancy unit tests (fallback model used when ARCO is absent)
# ---------------------------------------------------------------------------


def test_simple_occupancy_empty_cloud_returns_inf() -> None:
    """Clearance must be inf when the point cloud is empty."""
    occ = _SimpleOccupancy(np.empty((0, 3), dtype=np.float64))
    assert occ.clearance(np.array([0.0, 0.0, 0.0])) == float("inf")


def test_simple_occupancy_clearance_positive_when_far() -> None:
    """Clearance must be positive when the query point is far from obstacles."""
    pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
    occ = _SimpleOccupancy(pts)
    # Query 1 m away; clearance = 1.0 - contact_radius = 0.99
    clr = occ.clearance(np.array([1.0, 0.0, 0.0]))
    assert clr > 0.0


def test_simple_occupancy_clearance_negative_when_inside() -> None:
    """Clearance must be negative when the query point is inside the cloud."""
    pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
    occ = _SimpleOccupancy(pts)
    # Query at the exact obstacle point; dist = 0, clearance = -contact_radius
    clr = occ.clearance(np.array([0.0, 0.0, 0.0]))
    assert clr < 0.0


def test_simple_occupancy_multiple_query_points() -> None:
    """Clearance with (N, 3) query array returns minimum across all rows."""
    pts = np.array([[0.5, 0.0, 0.0]], dtype=np.float64)
    occ = _SimpleOccupancy(pts)
    queries = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    clr = occ.clearance(queries)
    # Distance from obstacle to q1 = 0.5, to q2 = 0.5 → same clearance
    assert abs(clr - (0.5 - _SimpleOccupancy._CONTACT_RADIUS)) < 1e-9

