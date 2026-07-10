"""Unit tests for PPP physics grasp release gating (v1.1.5)."""

from __future__ import annotations

import numpy as np

from fret.scenario.ppp_warehouse_runner import _ppp_physics_grasp_released


def test_grasp_release_xy_uses_goal_radius_not_tracking_limit() -> None:
    """Release XY gate must use scenario goal_radius, not V12-2 tracking limit."""
    goal = np.array([10.5, 1.2, 0.59])
    ee_within_radius = np.array([10.35, 1.15, 0.59])
    ee_outside_radius = np.array([10.0, 0.8, 0.59])

    assert _ppp_physics_grasp_released(
        ee_within_radius, goal, goal_radius=0.5
    )
    assert not _ppp_physics_grasp_released(
        ee_outside_radius, goal, goal_radius=0.5
    )
    # 200 mm XY offset is inside 0.5 m goal_radius but would fail a 10 mm gate.
    ee_200mm_xy = np.array([10.7, 1.2, 0.59])
    assert _ppp_physics_grasp_released(ee_200mm_xy, goal, goal_radius=0.5)
    assert not _ppp_physics_grasp_released(
        ee_200mm_xy, goal, goal_radius=0.10
    )
