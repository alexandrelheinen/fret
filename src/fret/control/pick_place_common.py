"""Shared pick-and-place simulation types (no ARCO MPC dependency)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from fret.control.pick_place_fsm import PickPlaceState

_ADHERE_STATES = frozenset(
    {
        PickPlaceState.LIFT,
        PickPlaceState.MOVE_PLACE,
        PickPlaceState.DESCEND_PLACE,
    }
)
_GRASP_ADHERE_AFTER_S = 0.6


def adhesion_command(
    state: PickPlaceState,
    grasp_hold_t: float,
    *,
    gripper: float | None = None,
    gripper_closed: float = 0.85,
) -> float:
    """Return adhesion ctrl in ``[0, 1]`` for the current FSM phase."""
    if state == PickPlaceState.GRASP:
        if float(grasp_hold_t) < _GRASP_ADHERE_AFTER_S:
            return 0.0
        if gripper is not None and float(gripper) < float(gripper_closed):
            return 0.0
        return 1.0
    return 1.0 if state in _ADHERE_STATES else 0.0


@dataclass(frozen=True)
class PickPlaceSample:
    """One recorded sample of a pick-and-place cycle."""

    q_arm: npt.NDArray[np.float64]
    gripper: float
    box_qpos: npt.NDArray[np.float64]
    state: PickPlaceState
