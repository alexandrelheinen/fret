"""Regression tests for PPP kinematic showcase (v1.0 reference)."""

from __future__ import annotations

import numpy as np
import pytest


def test_ppp_kinematic_showcase_has_pick_place_dwell_and_transit() -> None:
    """Release PPP clips must match v1.0: horizontal transit + visible Z motion."""
    pytest.importorskip("mujoco")
    import scripts.render_mujoco as rm

    waypoints = rm.build_showcase_waypoints(
        "ppp_warehouse",
        collision_backend="mujoco",
        planner_algorithm="rrt_star",
    )
    trajectory, sim_time_s = rm.build_showcase_trajectory(
        waypoints,
        scenario="ppp_warehouse",
        duration_s=None,
        fps=30,
        collision_backend="mujoco",
        use_tracking=False,
    )
    spans = trajectory.max(axis=0) - trajectory.min(axis=0)
    assert sim_time_s >= 30.0
    assert float(spans[0]) >= 5.0
    assert float(spans[2]) >= 1.0

    durations = rm.pick_place_segment_durations(waypoints)
    vertical_dwell_s = 0.0
    for idx, duration in enumerate(durations):
        dz = abs(float(waypoints[idx + 1][2]) - float(waypoints[idx][2]))
        if dz > 0.2:
            vertical_dwell_s += duration
    assert vertical_dwell_s >= 10.0
