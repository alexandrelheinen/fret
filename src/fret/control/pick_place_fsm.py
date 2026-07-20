"""OpenMANIPULATOR-X pick-and-place FSM (SC-v13b).

Pure-Python state machine: idle → approach pick → grasp → lift → move →
descend place → release → retreat → done. Controllers/SITL drive the commanded
joint and gripper setpoints; this module only encodes phase logic and timeouts.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

# Menagerie Gripper slide (OMX): more positive ⇒ wider fingers.
GRIPPER_OPEN: float = 0.019
GRIPPER_CLOSED: float = -0.01

# Menagerie OMY revolute gripper (rh_r1): 0 ≈ open, ~1 rad ≈ fully closed.
# Pinch for the ground ball (Ø ≈ 75 mm at 65 % of max opening) — not a full close.
OMY_GRIPPER_OPEN: float = 0.05
OMY_GRIPPER_CLOSED: float = 0.24


class PickPlaceState(enum.IntEnum):
    """Manipulation phases for SC-v13b."""

    IDLE = 0
    APPROACH_PICK = 1
    DESCEND_PICK = 2
    GRASP = 3
    LIFT = 4
    MOVE_PLACE = 5
    DESCEND_PLACE = 6
    RELEASE = 7
    RETREAT = 8
    DONE = 9
    FAULT = 10


@dataclass(frozen=True)
class PickPlaceWaypoints:
    """Named arm configurations (4,) for one pick-and-place cycle."""

    idle: npt.NDArray[np.float64]
    pick_hover: npt.NDArray[np.float64]
    pick_grasp: npt.NDArray[np.float64]
    place_hover: npt.NDArray[np.float64]
    place_grasp: npt.NDArray[np.float64]
    # Optional post-grasp lift pose (defaults to ``pick_hover``).
    lift_hover: npt.NDArray[np.float64] | None = None
    # Optional fold after place (defaults to ``idle`` when omitted).
    retreat: npt.NDArray[np.float64] | None = None

    def __post_init__(self) -> None:
        if self.lift_hover is None:
            object.__setattr__(self, "lift_hover", self.pick_hover.copy())
        if self.retreat is None:
            object.__setattr__(self, "retreat", self.idle.copy())


@dataclass(frozen=True)
class PickPlaceObservation:
    """Sensors the FSM needs each tick."""

    q: npt.NDArray[np.float64]
    object_pos: npt.NDArray[np.float64]
    ee_pos: npt.NDArray[np.float64]
    # When set, GRASP waits for pad contact before advancing (OMY physics grasp).
    grasp_contact: bool | None = None


@dataclass(frozen=True)
class PickPlaceCommand:
    """Actuator setpoints for the current phase."""

    q_des: npt.NDArray[np.float64]
    gripper: float
    state: PickPlaceState


class PickPlaceFSM:
    """Tabletop pick-and-place state machine (FR / SC-v13b).

    Args:
        waypoints: Joint-space poses for each phase.
        joint_tol_rad: Configuration reach tolerance.
        grasp_hold_s: Time to keep closing at the pick pose.
        release_hold_s: Time to keep opening at the place pose.
        lift_height_m: Object z that counts as successfully lifted.
        phase_timeout_s: Max time in one phase before ``FAULT``.
    """

    def __init__(
        self,
        waypoints: PickPlaceWaypoints,
        *,
        dof: int = 4,
        gripper_open: float = GRIPPER_OPEN,
        gripper_closed: float = GRIPPER_CLOSED,
        joint_tol_rad: float = 0.08,
        grasp_hold_s: float = 0.6,
        release_hold_s: float = 0.5,
        lift_height_m: float = 0.055,
        phase_timeout_s: float = 8.0,
        drop_fault_enabled: bool = True,
        require_grasp_contact: bool = False,
        approach_joint_tol_rad: float | None = None,
    ) -> None:
        self._wp = waypoints
        self._dof = int(dof)
        self._gripper_open = float(gripper_open)
        self._gripper_closed = float(gripper_closed)
        self._joint_tol = float(joint_tol_rad)
        self._grasp_hold_s = float(grasp_hold_s)
        self._release_hold_s = float(release_hold_s)
        self._lift_height_m = float(lift_height_m)
        self._phase_timeout_s = float(phase_timeout_s)
        self._drop_fault_enabled = bool(drop_fault_enabled)
        self._require_grasp_contact = bool(require_grasp_contact)
        self._approach_tol = (
            float(approach_joint_tol_rad)
            if approach_joint_tol_rad is not None
            else self._joint_tol
        )
        self._state = PickPlaceState.IDLE
        self._phase_t = 0.0
        self._hold_t = 0.0
        self._retreat_cleared = False

    @property
    def state(self) -> PickPlaceState:
        """Current FSM state."""
        return self._state

    @property
    def hold_t(self) -> float:
        """Elapsed time in the current grasp/release hold phase."""
        return self._hold_t

    def start(self) -> None:
        """Leave idle and begin the pick approach."""
        if self._state in {PickPlaceState.IDLE, PickPlaceState.DONE}:
            self._enter(PickPlaceState.APPROACH_PICK)

    def reset(self) -> None:
        """Return to idle and clear timers."""
        self._enter(PickPlaceState.IDLE)

    def force_state(self, state: PickPlaceState) -> None:
        """Jump to ``state`` (simulation harness hook for physics grasp)."""
        self._enter(state)

    def tick(self, obs: PickPlaceObservation, dt: float) -> PickPlaceCommand:
        """Advance the FSM and return the active setpoints."""
        dt = float(dt)
        self._phase_t += dt

        if self._state == PickPlaceState.IDLE:
            return self._cmd(self._wp.idle, self._gripper_open)

        if self._state == PickPlaceState.DONE:
            return self._cmd(self._wp.idle, self._gripper_open)

        if self._state == PickPlaceState.FAULT:
            return self._cmd(obs.q, self._gripper_open)

        if self._phase_t > self._phase_timeout_s:
            self._enter(PickPlaceState.FAULT)
            return self._cmd(obs.q, self._gripper_open)

        if self._state == PickPlaceState.APPROACH_PICK:
            if self._reached(
                obs.q, self._wp.pick_hover, tol=self._approach_tol
            ):
                self._enter(PickPlaceState.DESCEND_PICK)
            return self._cmd(self._wp.pick_hover, self._gripper_open)

        if self._state == PickPlaceState.DESCEND_PICK:
            if self._reached(obs.q, self._wp.pick_grasp):
                if (
                    not self._require_grasp_contact
                    or obs.grasp_contact is True
                ):
                    self._enter(PickPlaceState.GRASP)
            return self._cmd(self._wp.pick_grasp, self._gripper_open)

        if self._state == PickPlaceState.GRASP:
            self._hold_t += dt
            grasp_ready = self._hold_t >= self._grasp_hold_s
            if self._require_grasp_contact:
                if obs.grasp_contact is not True:
                    grasp_ready = False
            if grasp_ready:
                self._enter(PickPlaceState.LIFT)
            return self._cmd(self._wp.pick_grasp, self._gripper_closed)

        if self._state == PickPlaceState.LIFT:
            lift_target = (
                self._wp.lift_hover
                if self._wp.lift_hover is not None
                else self._wp.pick_hover
            )
            if self._reached(obs.q, lift_target):
                if float(obs.object_pos[2]) >= self._lift_height_m:
                    self._enter(PickPlaceState.MOVE_PLACE)
                elif self._phase_t > 0.5 * self._phase_timeout_s:
                    self._enter(PickPlaceState.FAULT)
            return self._cmd(lift_target, self._gripper_closed)

        if self._state == PickPlaceState.MOVE_PLACE:
            if self._reached(obs.q, self._wp.place_hover):
                self._enter(PickPlaceState.DESCEND_PLACE)
            # Drop detection mid-transfer.
            if (
                self._drop_fault_enabled
                and float(obs.object_pos[2]) < 0.5 * self._lift_height_m
            ):
                self._enter(PickPlaceState.FAULT)
            return self._cmd(self._wp.place_hover, self._gripper_closed)

        if self._state == PickPlaceState.DESCEND_PLACE:
            if self._reached(obs.q, self._wp.place_grasp):
                self._enter(PickPlaceState.RELEASE)
            return self._cmd(self._wp.place_grasp, self._gripper_closed)

        if self._state == PickPlaceState.RELEASE:
            self._hold_t += dt
            if self._hold_t >= self._release_hold_s:
                self._enter(PickPlaceState.RETREAT)
            return self._cmd(self._wp.place_grasp, self._gripper_open)

        if self._state == PickPlaceState.RETREAT:
            # Lift clear of the placed box before slewing to the retreat fold.
            retreat = (
                self._wp.retreat
                if self._wp.retreat is not None
                else self._wp.idle
            )
            if not self._retreat_cleared:
                if self._reached(obs.q, self._wp.place_hover):
                    self._retreat_cleared = True
                return self._cmd(self._wp.place_hover, self._gripper_open)
            if self._reached(obs.q, retreat):
                self._enter(PickPlaceState.DONE)
            return self._cmd(retreat, self._gripper_open)

        return self._cmd(self._wp.idle, self._gripper_open)

    def _enter(self, state: PickPlaceState) -> None:
        self._state = state
        self._phase_t = 0.0
        self._hold_t = 0.0
        if state != PickPlaceState.RETREAT:
            self._retreat_cleared = False

    def _reached(
        self,
        q: npt.NDArray[np.float64],
        target: npt.NDArray[np.float64],
        *,
        tol: float | None = None,
    ) -> bool:
        limit = self._joint_tol if tol is None else float(tol)
        return float(np.linalg.norm(q - target)) <= limit

    def _cmd(
        self, q_des: npt.NDArray[np.float64], gripper: float
    ) -> PickPlaceCommand:
        return PickPlaceCommand(
            q_des=np.asarray(q_des, dtype=np.float64)
            .reshape(self._dof)
            .copy(),
            gripper=float(gripper),
            state=self._state,
        )
