"""Tests for arc-length carrot path tracking helpers."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.path_tracking import (
    cumulative_arc_lengths,
    densify_polyline,
    sample_path_at_distance,
    simulate_joint_carrot_tracking,
)


def test_densify_polyline_splits_long_segments() -> None:
    path = [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
    dense = densify_polyline(path, max_step=0.25)
    assert len(dense) >= 5
    assert np.allclose(dense[0], path[0])
    assert np.allclose(dense[-1], path[-1])


def test_sample_path_at_distance_interpolates_midpoint() -> None:
    path = [
        np.array([0.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
    ]
    arcs = cumulative_arc_lengths(path)
    pos, at_goal = sample_path_at_distance(path, arcs, 1.0)
    assert not at_goal
    assert pos[0] == pytest.approx(1.0)


def test_carrot_tracking_reaches_goal_without_waypoint_holds() -> None:
  path = [
      np.array([0.0, 0.0, 0.0]),
      np.array([1.0, 0.0, 0.0]),
      np.array([2.0, 1.0, 0.5]),
  ]
  dense = densify_polyline(path, max_step=0.2)
  history, max_err = simulate_joint_carrot_tracking(
      dense,
      start=path[0],
      race_speed=1.0,
      max_joint_velocity=np.array([2.0, 2.0, 2.0]),
      max_joint_acc=np.array([4.0, 4.0, 4.0]),
      proportional_gain=2.0,
      max_carrot_lag=0.2,
      dt=0.02,
      goal=dense[-1],
      goal_tolerance=0.05,
  )
  assert len(history) > 5
  assert max_err < 0.5
  assert np.linalg.norm(history[-1] - dense[-1]) < 0.08
