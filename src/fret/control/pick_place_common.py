"""Shared pick-and-place simulation types (no ARCO MPC dependency)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.pick_place_fsm import PickPlaceState

_OMY_GRASP_PAD_MID_MAX_M = 0.050

_ADHERE_STATES = frozenset(
    {
        PickPlaceState.LIFT,
        PickPlaceState.MOVE_PLACE,
        PickPlaceState.DESCEND_PLACE,
    }
)
_GRASP_ADHERE_AFTER_S = 0.15


def adhesion_command(
    state: PickPlaceState,
    grasp_hold_t: float,
    *,
    gripper: float | None = None,
    gripper_closed: float = 0.85,
) -> float:
    """Return adhesion ctrl in ``[0, 1]`` for the current FSM phase."""
    if state == PickPlaceState.GRASP:
        if (
            gripper is not None
            and float(gripper) >= float(gripper_closed) - 0.05
        ):
            return 1.0
        if float(grasp_hold_t) < _GRASP_ADHERE_AFTER_S:
            if (
                gripper is not None
                and float(gripper) >= float(gripper_closed) - 0.08
            ):
                return 0.8
            return 0.0
        if gripper is not None and float(gripper) < float(gripper_closed):
            return 0.0
        return 1.0
    return 1.0 if state in _ADHERE_STATES else 0.0


def pad_midpoint(
    data: Any,
    *,
    pad_right_id: int,
    pad_left_id: int,
) -> npt.NDArray[np.float64]:
    """World-frame midpoint between injected finger pads."""
    return 0.5 * (
        np.asarray(data.geom_xpos[pad_right_id], dtype=np.float64)
        + np.asarray(data.geom_xpos[pad_left_id], dtype=np.float64)
    )


def ball_grasp_contact(
    mj: Any,
    model: Any,
    data: Any,
    *,
    box_body_id: int,
    pad_right_id: int,
    pad_left_id: int,
    max_pad_mid_dist_m: float = _OMY_GRASP_PAD_MID_MAX_M,
) -> bool:
    """True when the free ball touches a pad (MuJoCo contact or tight midpoint)."""
    box_geom_adr = int(model.body_geomadr[box_body_id])
    box_geom_num = int(model.body_geomnum[box_body_id])
    box_geoms = frozenset(range(box_geom_adr, box_geom_adr + box_geom_num))
    pad_contact = False
    for ci in range(int(data.ncon)):
        contact = data.contact[ci]
        g1 = int(contact.geom1)
        g2 = int(contact.geom2)
        if g1 not in box_geoms and g2 not in box_geoms:
            continue
        other = g2 if g1 in box_geoms else g1
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, other) or ""
        if name.startswith("pad_"):
            pad_contact = True
            break
    if pad_contact:
        return True
    mid = pad_midpoint(
        data, pad_right_id=pad_right_id, pad_left_id=pad_left_id
    )
    ball = np.asarray(data.xpos[box_body_id], dtype=np.float64)
    return float(np.linalg.norm(ball - mid)) <= float(max_pad_mid_dist_m)


@dataclass(frozen=True)
class PickPlaceSample:
    """One recorded sample of a pick-and-place cycle."""

    q_arm: npt.NDArray[np.float64]
    gripper: float
    box_qpos: npt.NDArray[np.float64]
    state: PickPlaceState
