"""Tests for fret.scene.OccupancyAdapter.

Acceptance criteria (FR-SCN-03, FR-SCN-04):
  - ``update()`` accepts an ``OccupancyUpdatePayload`` without raising.
  - ``get_occupancy()`` returns a KDTreeOccupancy-compatible object after
    at least one ``update()`` call.
  - ``get_occupancy()`` raises ``RuntimeError`` if called before ``update()``.
  - Subsequent ``update()`` calls atomically replace the previous occupancy.
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.occupancy_adapter import OccupancyAdapter


def _make_payload(n: int = 5) -> OccupancyUpdatePayload:
    return OccupancyUpdatePayload(
        obstacle_points=np.random.default_rng(42).random((n, 3)),
        timestamp=1.0,
        frame_id="world",
    )


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction() -> None:
    OccupancyAdapter()


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_raises_before_update() -> None:
    """get_occupancy() must raise RuntimeError when called before update()."""
    adapter = OccupancyAdapter()
    with pytest.raises(RuntimeError):
        adapter.get_occupancy()


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_update_then_get_occupancy() -> None:
    """get_occupancy() returns an object after a successful update()."""
    adapter = OccupancyAdapter()
    adapter.update(_make_payload())
    occ = adapter.get_occupancy()
    assert occ is not None


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_sequential_updates_replace_occupancy() -> None:
    """A second update() replaces the previous occupancy model."""
    adapter = OccupancyAdapter()
    adapter.update(_make_payload(n=3))
    first = adapter.get_occupancy()
    adapter.update(_make_payload(n=10))
    second = adapter.get_occupancy()
    # After a new update the returned object is new (different identity).
    assert first is not second
