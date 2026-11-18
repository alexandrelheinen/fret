"""GoalSegment geometry: a drop region is a segment, not a point."""

from __future__ import annotations

import numpy as np
import pytest

from fret.planning.goal_segment import GoalSegment


def test_vertical_segment_endpoints_and_midpoint() -> None:
    segment = GoalSegment.vertical(np.array([0.2, 0.08]), 0.16, 0.26)
    assert segment.length == pytest.approx(0.10)
    np.testing.assert_allclose(segment.point_at(0.0), [0.2, 0.08, 0.16])
    np.testing.assert_allclose(segment.point_at(1.0), [0.2, 0.08, 0.26])
    np.testing.assert_allclose(segment.point_at(0.5), [0.2, 0.08, 0.21])


def test_distance_is_zero_on_the_segment_and_positive_beside_it() -> None:
    segment = GoalSegment.vertical(np.array([0.2, 0.08]), 0.16, 0.26)
    assert segment.distance(np.array([0.2, 0.08, 0.20])) == pytest.approx(0.0)
    assert segment.distance(np.array([0.2, 0.08, 0.10])) == pytest.approx(0.06)
    assert segment.distance(np.array([0.23, 0.08, 0.20])) == pytest.approx(
        0.03
    )


def test_zero_length_segment_is_rejected() -> None:
    with pytest.raises(ValueError, match="positive"):
        GoalSegment(
            start=np.zeros(3),
            end=np.zeros(3),
        )
