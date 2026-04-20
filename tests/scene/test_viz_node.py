"""Unit tests for VizNode trajectory visualisation logic.

Tests the pure-Python ``VizNode`` class in isolation — no ROS context needed.

Acceptance criteria:
  - ``set_trajectory`` populates ``waypoint_positions`` with FK-computed
    Cartesian positions, one per waypoint.
  - The start and goal markers refer to the first and last waypoints.
  - Intermediate waypoints appear as a SPHERE_LIST marker.
  - ``append_ee`` accumulates FK EE positions in ``ee_trace``.
  - A LINE_STRIP trace marker is emitted only after at least 2 EE updates.
  - ``set_trajectory`` resets the EE trace.
  - Calling ``build_marker_descriptors`` with no trajectory returns [].
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.ros.viz_node import (
    _ID_GOAL,
    _ID_START,
    _ID_TRACE,
    _ID_WAYPOINTS,
    VizNode,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

_DOF = 3


@pytest.fixture()
def node() -> VizNode:
    return VizNode(model="scara")


def _zero_config() -> np.ndarray:
    return np.zeros(_DOF, dtype=np.float64)


def _nonzero_config() -> np.ndarray:
    return np.array([0.3, -0.2, 0.05], dtype=np.float64)


# ---------------------------------------------------------------------------
# set_trajectory
# ---------------------------------------------------------------------------


def test_set_trajectory_populates_waypoint_positions(node: VizNode) -> None:
    waypoints = [_zero_config(), _nonzero_config()]
    node.set_trajectory(waypoints)
    assert len(node.waypoint_positions) == 2


def test_set_trajectory_produces_3d_positions(node: VizNode) -> None:
    waypoints = [_zero_config(), _nonzero_config()]
    node.set_trajectory(waypoints)
    for pos in node.waypoint_positions:
        assert pos.shape == (3,)


def test_set_trajectory_resets_ee_trace(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    node.append_ee(_nonzero_config())
    # Loading a new trajectory must clear the old trace
    node.set_trajectory([_zero_config(), _nonzero_config()])
    assert len(node.ee_trace) == 0


# ---------------------------------------------------------------------------
# build_marker_descriptors — no trajectory
# ---------------------------------------------------------------------------


def test_no_markers_without_trajectory(node: VizNode) -> None:
    assert node.build_marker_descriptors() == []


# ---------------------------------------------------------------------------
# build_marker_descriptors — start / goal markers
# ---------------------------------------------------------------------------


def test_start_marker_id(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    ids = {d["id"] for d in node.build_marker_descriptors()}
    assert _ID_START in ids


def test_goal_marker_id(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    ids = {d["id"] for d in node.build_marker_descriptors()}
    assert _ID_GOAL in ids


def test_start_marker_position_matches_first_waypoint(node: VizNode) -> None:
    waypoints = [_zero_config(), _nonzero_config()]
    node.set_trajectory(waypoints)
    descriptors = {d["id"]: d for d in node.build_marker_descriptors()}
    start_pos = np.array(descriptors[_ID_START]["points"][0])
    expected = node.waypoint_positions[0]
    np.testing.assert_allclose(start_pos, expected, atol=1e-9)


def test_goal_marker_position_matches_last_waypoint(node: VizNode) -> None:
    waypoints = [_zero_config(), _nonzero_config()]
    node.set_trajectory(waypoints)
    descriptors = {d["id"]: d for d in node.build_marker_descriptors()}
    goal_pos = np.array(descriptors[_ID_GOAL]["points"][0])
    expected = node.waypoint_positions[-1]
    np.testing.assert_allclose(goal_pos, expected, atol=1e-9)


# ---------------------------------------------------------------------------
# build_marker_descriptors — intermediate waypoints
# ---------------------------------------------------------------------------


def test_no_intermediate_markers_for_two_waypoint_trajectory(
    node: VizNode,
) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    ids = {d["id"] for d in node.build_marker_descriptors()}
    assert _ID_WAYPOINTS not in ids


def test_intermediate_markers_for_three_waypoint_trajectory(
    node: VizNode,
) -> None:
    mid = np.array([0.15, 0.1, 0.02], dtype=np.float64)
    node.set_trajectory([_zero_config(), mid, _nonzero_config()])
    ids = {d["id"] for d in node.build_marker_descriptors()}
    assert _ID_WAYPOINTS in ids


def test_intermediate_sphere_list_count(node: VizNode) -> None:
    configs = (
        [_zero_config()]
        + [np.array([0.1 * i, 0.0, 0.01], dtype=np.float64) for i in range(3)]
        + [_nonzero_config()]
    )
    node.set_trajectory(configs)
    descriptors = {d["id"]: d for d in node.build_marker_descriptors()}
    # 3 intermediate configs → 3 points in the SPHERE_LIST
    assert len(descriptors[_ID_WAYPOINTS]["points"]) == 3


# ---------------------------------------------------------------------------
# append_ee / trace marker
# ---------------------------------------------------------------------------


def test_no_trace_marker_with_one_ee_update(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    node.append_ee(_zero_config())
    ids = {d["id"] for d in node.build_marker_descriptors()}
    assert _ID_TRACE not in ids


def test_trace_marker_appears_after_two_ee_updates(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    node.append_ee(_zero_config())
    node.append_ee(_nonzero_config())
    ids = {d["id"] for d in node.build_marker_descriptors()}
    assert _ID_TRACE in ids


def test_trace_grows_with_each_ee_update(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    for i in range(5):
        node.append_ee(np.array([0.05 * i, 0.0, 0.01], dtype=np.float64))
    descriptors = {d["id"]: d for d in node.build_marker_descriptors()}
    assert len(descriptors[_ID_TRACE]["points"]) == 5


def test_ee_trace_length_matches_append_count(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    for _ in range(7):
        node.append_ee(_zero_config())
    assert len(node.ee_trace) == 7


# ---------------------------------------------------------------------------
# Color / scale sanity
# ---------------------------------------------------------------------------


def test_start_marker_is_green(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    descriptors = {d["id"]: d for d in node.build_marker_descriptors()}
    r, g, b, a = descriptors[_ID_START]["color"]
    assert g > r and g > b, "Start marker should be predominantly green"


def test_goal_marker_is_red(node: VizNode) -> None:
    node.set_trajectory([_zero_config(), _nonzero_config()])
    descriptors = {d["id"]: d for d in node.build_marker_descriptors()}
    r, g, b, a = descriptors[_ID_GOAL]["color"]
    assert r > g and r > b, "Goal marker should be predominantly red"
