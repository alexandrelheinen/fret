"""Unit tests for PPP physics grasp release gating (v1.1.5)."""

from __future__ import annotations

import numpy as np

from fret.scenario.ppp_warehouse_runner import _ppp_physics_grasp_released


def test_grasp_release_xy_uses_goal_radius_not_tracking_limit() -> None:
    """Release XY gate must use scenario goal_radius, not V12-2 tracking limit."""
    goal = np.array([10.5, 2.8, 2.65])
    ee_within_radius = np.array([10.35, 2.75, 2.65])
    ee_outside_radius = np.array([10.0, 2.4, 2.65])

    assert _ppp_physics_grasp_released(
        ee_within_radius, goal, goal_radius=0.5
    )
    assert not _ppp_physics_grasp_released(
        ee_outside_radius, goal, goal_radius=0.5
    )
    # 200 mm XY offset is inside 0.5 m goal_radius but would fail a 10 mm gate.
    ee_200mm_xy = np.array([10.7, 2.8, 2.65])
    assert _ppp_physics_grasp_released(ee_200mm_xy, goal, goal_radius=0.5)
    assert not _ppp_physics_grasp_released(
        ee_200mm_xy, goal, goal_radius=0.10
    )
