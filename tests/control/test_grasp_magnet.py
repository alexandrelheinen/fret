"""Tests for fret.control.grasp_magnet — PPP magnetic grasp FSM.

Acceptance criteria (T10-04, FR-GSP-01, FR-GSP-03, FR-GSP-04):
  - FSM states: IDLE, APPROACH, CAPTURE, TRANSPORT, RELEASE.
  - Weld latches when ``‖p_ee − p_box‖ < capture_radius``.
  - Cargo tracks EE + weld offset during TRANSPORT.
  - Weld releases at goal; cargo pose frozen (FR-GSP-03).
  - ``cargo_corners()`` returns 8 world-frame corners when welded.

Default parameters (docs/scenarios.md, docs/robots/ppp.md):
  capture_radius = 0.3 m, goal_radius = 0.5 m
  box_half_extent = (0.25, 0.25, 0.25) m
  weld_offset = (0, 0, 0.25) m
"""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.grasp_magnet import GraspConfig, GraspState, MagneticGraspFSM

_CAPTURE_RADIUS = 0.3
_GOAL_RADIUS = 0.5
_WELD_OFFSET = np.array([0.0, 0.0, 0.25])
_BOX_HALF = np.array([0.25, 0.25, 0.25])

_BOX_POS = np.array([3.0, 2.0, 1.0])
_GOAL_POS = np.array([10.0, 3.0, 1.0])


def _fsm() -> MagneticGraspFSM:
    return MagneticGraspFSM(
        GraspConfig(
            capture_radius=_CAPTURE_RADIUS,
            goal_radius=_GOAL_RADIUS,
            weld_offset=_WELD_OFFSET,
            box_half_extent=_BOX_HALF,
        )
    )


def test_construction_defaults_to_idle() -> None:
    fsm = _fsm()
    assert fsm.state == GraspState.IDLE
    assert fsm.is_welded is False


def test_invalid_capture_radius_raises() -> None:
    with pytest.raises(ValueError, match="capture_radius"):
        GraspConfig(capture_radius=-0.1)


def test_begin_transport_moves_to_approach() -> None:
    fsm = _fsm()
    fsm.begin_transport()
    assert fsm.state == GraspState.APPROACH


def test_approach_stays_until_capture_radius() -> None:
    fsm = _fsm()
    fsm.begin_transport()
    ee_far = _BOX_POS + np.array([1.0, 0.0, 0.0])
    fsm.update(ee_far, _BOX_POS, _GOAL_POS)
    assert fsm.state == GraspState.APPROACH
    assert fsm.is_welded is False


def test_capture_when_within_radius() -> None:
    fsm = _fsm()
    fsm.begin_transport()
    ee_near = _BOX_POS + np.array([0.1, 0.0, 0.0])
    fsm.update(ee_near, _BOX_POS, _GOAL_POS)
    assert fsm.state == GraspState.TRANSPORT
    assert fsm.is_welded is True


def test_cargo_tracks_ee_plus_offset_in_transport() -> None:
    fsm = _fsm()
    fsm.begin_transport()
    ee = _BOX_POS.copy()
    fsm.update(ee, _BOX_POS, _GOAL_POS)
    expected = ee + _WELD_OFFSET
    np.testing.assert_allclose(fsm.cargo_position, expected)

    ee_moved = ee + np.array([1.0, 0.5, 0.0])
    fsm.update(ee_moved, _BOX_POS, _GOAL_POS)
    np.testing.assert_allclose(fsm.cargo_position, ee_moved + _WELD_OFFSET)


def test_release_at_goal_freezes_cargo() -> None:
    fsm = _fsm()
    fsm.begin_transport()
    fsm.update(_BOX_POS, _BOX_POS, _GOAL_POS)
    assert fsm.state == GraspState.TRANSPORT

    ee_at_goal = _GOAL_POS.copy()
    fsm.update(ee_at_goal, _BOX_POS, _GOAL_POS)
    assert fsm.state == GraspState.IDLE
    assert fsm.is_welded is False

    frozen = fsm.cargo_position.copy()
    ee_retract = _GOAL_POS + np.array([2.0, 0.0, 0.0])
    fsm.update(ee_retract, _BOX_POS, _GOAL_POS)
    np.testing.assert_allclose(fsm.cargo_position, frozen)


def test_cargo_corners_shape_during_transport() -> None:
    fsm = _fsm()
    fsm.begin_transport()
    fsm.update(_BOX_POS, _BOX_POS, _GOAL_POS)
    corners = fsm.cargo_corners()
    assert corners.shape == (8, 3)
    centre = corners.mean(axis=0)
    np.testing.assert_allclose(centre, fsm.cargo_position, atol=1e-9)


def test_cargo_corners_empty_when_not_welded() -> None:
    fsm = _fsm()
    corners = fsm.cargo_corners()
    assert corners.shape == (0, 3)


def test_update_from_idle_without_begin_is_noop() -> None:
    fsm = _fsm()
    fsm.update(_BOX_POS, _BOX_POS, _GOAL_POS)
    assert fsm.state == GraspState.IDLE
