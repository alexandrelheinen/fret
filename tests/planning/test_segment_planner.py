"""plan_to_segment reaches a vertical drop segment above the SO-101 bin."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from fret.control.kinematics_so101 import (
    HeldPointKinematics,
    SO101Kinematics,
    grasp_offset_in_gripper,
    link_positions,
)
from fret.control.so101_pick_place_sim import (
    place_obstacles_from_params,
    release_segment_from_params,
)
from fret.planning.box_obstacle import points_clear_of_boxes
from fret.planning.goal_segment import sample_polyline
from fret.planning.segment_planner import plan_to_segment
from fret.sitl_config import load_scenario_parameters

_SCENARIO = Path("src/fret/config/scenarios/so101_pick_place.yml")


def test_place_plan_ends_on_the_segment_and_clears_the_bin() -> None:
    params = load_scenario_parameters(_SCENARIO)
    segment = release_segment_from_params(params)
    boxes = place_obstacles_from_params(params)
    margin = float(params["place_arm_margin_m"])
    kinematics = SO101Kinematics()
    lift = np.asarray(params["lift_hover_configuration"], dtype=np.float64)

    def is_free(configuration: np.ndarray) -> bool:
        samples = sample_polyline(link_positions(configuration), per_edge=4)
        return points_clear_of_boxes(samples, boxes, margin=margin)

    path = plan_to_segment(
        lift,
        segment,
        kinematics=kinematics,
        is_free=is_free,
        seed=int(params["planner_rng_seed"]),
        timeout_s=2.0,
        goal_tolerance_m=float(params["place_segment_tolerance_m"]),
    )
    assert len(path) >= 2
    end = kinematics.forward_kinematics(path[-1])[:3, 3]
    assert segment.distance(end) <= float(params["place_segment_tolerance_m"])
    # Lowest feasible height: the release is near the bottom of the segment,
    # not an arbitrary point and not the old pose on the far wall.
    assert float(end[2]) <= float(segment.start[2]) + 0.02
    old = np.asarray(params["place_hover_configuration"], dtype=np.float64)
    old_ee = kinematics.forward_kinematics(old)[:3, 3]
    assert segment.distance(old_ee) > 0.02
    for configuration in path:
        assert is_free(configuration)


def test_place_plan_puts_the_grasped_ball_on_the_segment() -> None:
    params = load_scenario_parameters(_SCENARIO)
    segment = release_segment_from_params(params)
    pick = np.asarray(params["pick_grasp_configuration"], dtype=np.float64)
    pick_xy = np.asarray(params["pick_xy"], dtype=np.float64).reshape(-1)
    ball = np.array(
        [float(pick_xy[0]), float(pick_xy[1]), float(params["ball_radius_m"])],
        dtype=np.float64,
    )
    held = HeldPointKinematics(grasp_offset_in_gripper(pick, ball))
    np.testing.assert_allclose(
        held.forward_kinematics(pick)[:3, 3], ball, atol=1e-9
    )
    lift = np.asarray(params["lift_hover_configuration"], dtype=np.float64)
    path = plan_to_segment(
        lift,
        segment,
        kinematics=held,
        is_free=lambda _configuration: True,
        seed=int(params["planner_rng_seed"]),
        timeout_s=2.0,
        goal_tolerance_m=float(params["place_segment_tolerance_m"]),
    )
    end = held.forward_kinematics(path[-1])[:3, 3]
    assert segment.distance(end) <= float(params["place_segment_tolerance_m"])
    assert float(end[2]) <= float(segment.start[2]) + 0.02
