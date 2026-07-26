"""Adhesion enable must respect OM-X vs OMY gripper polarity."""

from __future__ import annotations

from fret.control.pick_place_common import adhesion_command
from fret.control.pick_place_fsm import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    OMY_GRIPPER_CLOSED,
    OMY_GRIPPER_OPEN,
    PickPlaceState,
)


def test_omx_adhesion_off_when_gripper_open() -> None:
    """Regression: open=0.019 > closed=0.006 used to keep adhesion stuck on."""
    assert (
        adhesion_command(
            PickPlaceState.GRASP,
            0.0,
            gripper=GRIPPER_OPEN,
            gripper_closed=GRIPPER_CLOSED,
            gripper_open=GRIPPER_OPEN,
        )
        == 0.0
    )
    assert (
        adhesion_command(
            PickPlaceState.GRASP,
            1.0,
            gripper=GRIPPER_OPEN,
            gripper_closed=GRIPPER_CLOSED,
            gripper_open=GRIPPER_OPEN,
        )
        == 0.0
    )


def test_omx_adhesion_on_when_gripper_closed() -> None:
    assert (
        adhesion_command(
            PickPlaceState.GRASP,
            0.0,
            gripper=GRIPPER_CLOSED,
            gripper_closed=GRIPPER_CLOSED,
            gripper_open=GRIPPER_OPEN,
        )
        == 1.0
    )


def test_omy_adhesion_polarity_still_works() -> None:
    assert (
        adhesion_command(
            PickPlaceState.GRASP,
            0.0,
            gripper=OMY_GRIPPER_OPEN,
            gripper_closed=OMY_GRIPPER_CLOSED,
            gripper_open=OMY_GRIPPER_OPEN,
        )
        == 0.0
    )
    assert (
        adhesion_command(
            PickPlaceState.GRASP,
            0.0,
            gripper=OMY_GRIPPER_CLOSED,
            gripper_closed=OMY_GRIPPER_CLOSED,
            gripper_open=OMY_GRIPPER_OPEN,
        )
        == 1.0
    )


def test_lift_keeps_adhesion() -> None:
    assert adhesion_command(PickPlaceState.LIFT, 0.0) == 1.0
    assert adhesion_command(PickPlaceState.IDLE, 0.0) == 0.0
