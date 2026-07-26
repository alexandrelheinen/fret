"""OMY cluttered pick-and-place (SC-v14c) — thin wrapper over the modular stack.

Reuses :func:`fret.control.pick_place_clutter_sim.simulate_pick_place_clutter`
(FSM → planner → JointSpaceMPC → joints). OMY pad-mid carry during lift /
transfer / descend is handled inside that shared runner when ``model: omy``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

from fret.control.pick_place_clutter_sim import (
    ClutterPickPlaceResult,
    run_pick_place_clutter,
    simulate_pick_place_clutter,
)
from fret.control.pick_place_common import PickPlaceSample
from fret.control.pick_place_fsm import PickPlaceState

_SCENARIO = Path("src/fret/config/scenarios/omy_clutter.yml")


@dataclass(frozen=True)
class OmyClutterResult:
    """Outcome of one OMY cluttered pick-place cycle."""

    state: PickPlaceState
    box_pos: npt.NDArray[np.float64]
    transfer_path: list[npt.NDArray[np.float64]]
    straight_line_collides: bool
    samples: list[PickPlaceSample]
    wall_contact_steps: int = 0
    faulted_on_wall_contact: bool = False


def _as_omy(result: ClutterPickPlaceResult) -> OmyClutterResult:
    return OmyClutterResult(
        state=result.state,
        box_pos=result.box_pos,
        transfer_path=result.transfer_path,
        straight_line_collides=result.straight_line_collides,
        samples=result.samples,
        wall_contact_steps=int(result.wall_contact_steps),
        faulted_on_wall_contact=bool(result.faulted_on_wall_contact),
    )


def simulate_omy_clutter_pick_place(
    *,
    duration_s: float = 90.0,
    joint_tol_rad: float = 0.22,
    record_every_steps: int = 1,
    scenario_path: str | Path | None = None,
    seed_offset: int = 0,
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
) -> OmyClutterResult:
    """Run one OMY clutter cycle via the shared modular clutter runner."""
    result = simulate_pick_place_clutter(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        record_every_steps=record_every_steps,
        scenario_path=scenario_path or _SCENARIO,
        seed_offset=seed_offset,
        telemetry_enabled=telemetry_enabled,
        telemetry_output_dir=telemetry_output_dir,
        telemetry_csv_basename=telemetry_csv_basename,
    )
    return _as_omy(result)


def run_omy_clutter_pick_place(
    *,
    duration_s: float = 90.0,
    joint_tol_rad: float = 0.22,
    scenario_path: str | Path | None = None,
    seed_offset: int = 0,
    max_attempts: int = 4,
    telemetry_enabled: bool | None = None,
    telemetry_output_dir: Path | None = None,
    telemetry_csv_basename: str | None = None,
) -> OmyClutterResult:
    """Execute OMY clutter pick-place with place-xy retries."""
    del seed_offset  # retries use seed_offset internally
    result = run_pick_place_clutter(
        duration_s=duration_s,
        joint_tol_rad=joint_tol_rad,
        scenario_path=scenario_path or _SCENARIO,
        max_attempts=max_attempts,
        telemetry_enabled=telemetry_enabled,
        telemetry_output_dir=telemetry_output_dir,
        telemetry_csv_basename=telemetry_csv_basename,
    )
    return _as_omy(result)
