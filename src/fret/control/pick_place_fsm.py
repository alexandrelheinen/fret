"""Robot-agnostic pick-and-place FSM.

Pure-Python state machine shared by OM-X, OMY, and future manipulators.
See ``docs/modules/pick_place_fsm.md`` for the logical cycle (Mermaid) and the
mapping onto these implementation substates.

Controllers / SITL drive joint and gripper setpoints; this module encodes phase
logic, timeouts, and **which joint goal the path planner must target** on each
arm motion. It does not call MuJoCo, ROS, or a robot-specific IK model.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

# ---------------------------------------------------------------------------
# Gripper presets (robot-specific numbers; FSM only sees GripperSpec)
# ---------------------------------------------------------------------------

# Menagerie Gripper slide (OM-X): more positive ⇒ wider fingers.
GRIPPER_OPEN: float = 0.019
GRIPPER_CLOSED: float = -0.01

# Menagerie OMY revolute gripper (rh_r1): 0 ≈ open, ~1.05 pinches Ø 86 mm ball.
OMY_GRIPPER_OPEN: float = 0.0
OMY_GRIPPER_CLOSED: float = 1.05


@dataclass(frozen=True)
class GripperSpec:
    """Open / closed actuator setpoints for one gripper model."""

    open: float
    closed: float


OMX_GRIPPER = GripperSpec(open=GRIPPER_OPEN, closed=GRIPPER_CLOSED)
OMY_GRIPPER = GripperSpec(open=OMY_GRIPPER_OPEN, closed=OMY_GRIPPER_CLOSED)


class PickPlaceState(enum.IntEnum):
    """Implementation substates (physics-grade hover / descend splits).

    Logical product cycle (IDLE → MOVE_PICK → GRASP → MOVE_PLACE → RELEASE →
    MOVE_HOME → IDLE) is documented in ``docs/modules/pick_place_fsm.md``.
    """

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


class MotionKind(enum.Enum):
    """What the runner should do with ``PickPlaceCommand.plan_goal``."""

    HOLD = "hold"
    """Hold / track the current ``q_des``; no new plan."""

    PLAN_TO_GOAL = "plan_to_goal"
    """Request a path plan from ``q_now`` to ``plan_goal`` (joint space)."""


# States that represent arm motions and therefore need a path plan on entry.
_MOTION_STATES: frozenset[PickPlaceState] = frozenset(
    {
        PickPlaceState.APPROACH_PICK,
        PickPlaceState.DESCEND_PICK,
        PickPlaceState.LIFT,
        PickPlaceState.MOVE_PLACE,
        PickPlaceState.DESCEND_PLACE,
        PickPlaceState.RETREAT,
    }
)


@dataclass(frozen=True)
class PickPlaceWaypoints:
    """Named arm configurations for one pick-and-place cycle (any DOF)."""

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

    def with_pick(
        self,
        *,
        pick_hover: npt.NDArray[np.float64],
        pick_grasp: npt.NDArray[np.float64],
        lift_hover: npt.NDArray[np.float64] | None = None,
    ) -> PickPlaceWaypoints:
        """Return a copy with updated pick-side joints (vision / IK refresh)."""
        return PickPlaceWaypoints(
            idle=self.idle.copy(),
            pick_hover=np.asarray(pick_hover, dtype=np.float64).copy(),
            pick_grasp=np.asarray(pick_grasp, dtype=np.float64).copy(),
            place_hover=self.place_hover.copy(),
            place_grasp=self.place_grasp.copy(),
            lift_hover=(
                np.asarray(lift_hover, dtype=np.float64).copy()
                if lift_hover is not None
                else (
                    None if self.lift_hover is None else self.lift_hover.copy()
                )
            ),
            retreat=(None if self.retreat is None else self.retreat.copy()),
        )


@dataclass(frozen=True)
class PickPlaceObservation:
    """Sensors the FSM needs each tick (robot-agnostic)."""

    q: npt.NDArray[np.float64]
    object_pos: npt.NDArray[np.float64]
    ee_pos: npt.NDArray[np.float64]
    # When set, GRASP waits for pad contact before advancing (physics grasp).
    grasp_contact: bool | None = None
    # When True while IDLE/DONE, starts a new cycle (vision / harness).
    ball_detected: bool = False


@dataclass(frozen=True)
class PickPlaceCommand:
    """Actuator setpoints + planning hint for the current phase."""

    q_des: npt.NDArray[np.float64]
    gripper: float
    state: PickPlaceState
    motion: MotionKind = MotionKind.HOLD
    """``PLAN_TO_GOAL`` when the runner must (re)plan to ``plan_goal``."""

    plan_goal: npt.NDArray[np.float64] | None = None
    """Joint-space goal for the path planner (same as ``q_des`` on motion)."""

    needs_plan: bool = False
    """True on the first tick after entering a motion state (edge trigger)."""


class PickPlaceFSM:
    """Tabletop pick-and-place state machine (robot-agnostic).

    Args:
        waypoints: Joint-space poses for each phase (any DOF).
        gripper: Open/closed setpoints (``OMX_GRIPPER`` / ``OMY_GRIPPER``).
        dof: Arm DOF (defaults to ``len(waypoints.idle)``).
        joint_tol_rad: Configuration reach tolerance.
        grasp_hold_s: Time to keep closing at the pick pose.
        release_hold_s: Drop delay (open gripper) at the place pose.
        lift_height_m: Object z that counts as successfully lifted.
        phase_timeout_s: Max time in one phase before ``FAULT``.
        auto_start_on_ball: If True, IDLE/DONE + ``ball_detected`` → cycle.
    """

    def __init__(
        self,
        waypoints: PickPlaceWaypoints,
        *,
        gripper: GripperSpec | None = None,
        dof: int | None = None,
        gripper_open: float | None = None,
        gripper_closed: float | None = None,
        joint_tol_rad: float = 0.08,
        grasp_hold_s: float = 0.6,
        release_hold_s: float = 0.5,
        lift_height_m: float = 0.055,
        phase_timeout_s: float = 8.0,
        drop_fault_enabled: bool = True,
        require_grasp_contact: bool = False,
        approach_joint_tol_rad: float | None = None,
        transfer_joint_tol_rad: float | None = None,
        auto_start_on_ball: bool = True,
    ) -> None:
        self._wp = waypoints
        self._dof = int(dof if dof is not None else waypoints.idle.shape[0])
        if gripper is not None:
            self._gripper_open = float(gripper.open)
            self._gripper_closed = float(gripper.closed)
        else:
            # Backward-compatible kwargs (OM-X defaults).
            self._gripper_open = float(
                GRIPPER_OPEN if gripper_open is None else gripper_open
            )
            self._gripper_closed = float(
                GRIPPER_CLOSED if gripper_closed is None else gripper_closed
            )
        self._joint_tol = float(joint_tol_rad)
        self._grasp_hold_s = float(grasp_hold_s)
        self._release_hold_s = float(release_hold_s)
        self._lift_height_m = float(lift_height_m)
        self._phase_timeout_s = float(phase_timeout_s)
        self._drop_fault_enabled = bool(drop_fault_enabled)
        self._require_grasp_contact = bool(require_grasp_contact)
        self._auto_start_on_ball = bool(auto_start_on_ball)
        self._approach_tol = (
            float(approach_joint_tol_rad)
            if approach_joint_tol_rad is not None
            else self._joint_tol
        )
        # Loaded transfer often saturates distal joints short of place_hover.
        self._transfer_tol = (
            float(transfer_joint_tol_rad)
            if transfer_joint_tol_rad is not None
            else self._joint_tol
        )
        self._state = PickPlaceState.IDLE
        self._phase_t = 0.0
        self._hold_t = 0.0
        self._retreat_cleared = False
        self._plan_edge = False
        self._cycles_completed = 0

    @property
    def state(self) -> PickPlaceState:
        """Current FSM state."""
        return self._state

    @property
    def hold_t(self) -> float:
        """Elapsed time in the current grasp/release hold phase."""
        return self._hold_t

    @property
    def cycles_completed(self) -> int:
        """Number of successful pick→place→home cycles finished."""
        return self._cycles_completed

    @property
    def waypoints(self) -> PickPlaceWaypoints:
        """Active joint waypoints (pick side may be refreshed from vision)."""
        return self._wp

    def set_waypoints(self, waypoints: PickPlaceWaypoints) -> None:
        """Replace waypoints (e.g. after IK from ``BallObservation``)."""
        if int(waypoints.idle.shape[0]) != self._dof:
            raise ValueError(
                f"waypoint DOF {waypoints.idle.shape[0]} != fsm dof {self._dof}"
            )
        self._wp = waypoints

    def start(self) -> None:
        """Leave idle/done and begin the pick approach (manual / harness)."""
        if self._state in {PickPlaceState.IDLE, PickPlaceState.DONE}:
            self._enter(PickPlaceState.APPROACH_PICK)

    def reset(self) -> None:
        """Return to idle and clear timers."""
        self._enter(PickPlaceState.IDLE)

    def force_state(self, state: PickPlaceState) -> None:
        """Jump to ``state`` (simulation harness hook for physics grasp)."""
        self._enter(state)

    def fault(self) -> None:
        """Latch ``FAULT`` (collision monitor / external safety stop)."""
        self._enter(PickPlaceState.FAULT)

    def tick(self, obs: PickPlaceObservation, dt: float) -> PickPlaceCommand:
        """Advance the FSM and return the active setpoints + plan hint."""
        dt = float(dt)
        self._phase_t += dt

        if self._state in {PickPlaceState.IDLE, PickPlaceState.DONE}:
            if self._auto_start_on_ball and obs.ball_detected:
                self.start()
            elif self._state == PickPlaceState.IDLE:
                return self._cmd(self._wp.idle, self._gripper_open)
            else:
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
            return self._cmd(
                self._wp.pick_hover,
                self._gripper_open,
                motion=MotionKind.PLAN_TO_GOAL,
            )

        if self._state == PickPlaceState.DESCEND_PICK:
            # Enter GRASP when the open-jaw descend pose is reached. Contact is
            # gated on GRASP→LIFT so the jaw can close before pads must touch
            # (floor pinch: open pads often clear the sphere until close).
            if self._reached(obs.q, self._wp.pick_grasp):
                self._enter(PickPlaceState.GRASP)
            return self._cmd(
                self._wp.pick_grasp,
                self._gripper_open,
                motion=MotionKind.PLAN_TO_GOAL,
            )

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
            lifted = float(obs.object_pos[2]) >= self._lift_height_m
            if lifted and self._reached(obs.q, lift_target):
                self._enter(PickPlaceState.MOVE_PLACE)
            elif (
                not lifted
                and self._reached(obs.q, lift_target)
                and self._phase_t > 0.5 * self._phase_timeout_s
            ):
                self._enter(PickPlaceState.FAULT)
            return self._cmd(
                lift_target,
                self._gripper_closed,
                motion=MotionKind.PLAN_TO_GOAL,
            )

        if self._state == PickPlaceState.MOVE_PLACE:
            if self._reached(
                obs.q, self._wp.place_hover, tol=self._transfer_tol
            ):
                self._enter(PickPlaceState.DESCEND_PLACE)
            # Drop detection mid-transfer.
            if (
                self._drop_fault_enabled
                and float(obs.object_pos[2]) < 0.5 * self._lift_height_m
            ):
                self._enter(PickPlaceState.FAULT)
            return self._cmd(
                self._wp.place_hover,
                self._gripper_closed,
                motion=MotionKind.PLAN_TO_GOAL,
            )

        if self._state == PickPlaceState.DESCEND_PLACE:
            if self._reached(obs.q, self._wp.place_grasp):
                self._enter(PickPlaceState.RELEASE)
            return self._cmd(
                self._wp.place_grasp,
                self._gripper_closed,
                motion=MotionKind.PLAN_TO_GOAL,
            )

        if self._state == PickPlaceState.RELEASE:
            self._hold_t += dt
            if self._hold_t >= self._release_hold_s:
                self._enter(PickPlaceState.RETREAT)
            return self._cmd(self._wp.place_grasp, self._gripper_open)

        if self._state == PickPlaceState.RETREAT:
            # Lift clear of the placed object before slewing to the idle fold.
            retreat = (
                self._wp.retreat
                if self._wp.retreat is not None
                else self._wp.idle
            )
            if not self._retreat_cleared:
                if self._reached(obs.q, self._wp.place_hover):
                    self._retreat_cleared = True
                return self._cmd(
                    self._wp.place_hover,
                    self._gripper_open,
                    motion=MotionKind.PLAN_TO_GOAL,
                )
            if self._reached(obs.q, retreat):
                self._cycles_completed += 1
                self._enter(PickPlaceState.DONE)
            return self._cmd(
                retreat,
                self._gripper_open,
                motion=MotionKind.PLAN_TO_GOAL,
            )

        return self._cmd(self._wp.idle, self._gripper_open)

    def _enter(self, state: PickPlaceState) -> None:
        self._state = state
        self._phase_t = 0.0
        self._hold_t = 0.0
        if state != PickPlaceState.RETREAT:
            self._retreat_cleared = False
        self._plan_edge = state in _MOTION_STATES

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
        self,
        q_des: npt.NDArray[np.float64],
        gripper: float,
        *,
        motion: MotionKind = MotionKind.HOLD,
    ) -> PickPlaceCommand:
        goal = np.asarray(q_des, dtype=np.float64).reshape(self._dof).copy()
        needs_plan = bool(
            self._plan_edge and motion is MotionKind.PLAN_TO_GOAL
        )
        if needs_plan:
            self._plan_edge = False
        return PickPlaceCommand(
            q_des=goal,
            gripper=float(gripper),
            state=self._state,
            motion=motion,
            plan_goal=goal if motion is MotionKind.PLAN_TO_GOAL else None,
            needs_plan=needs_plan,
        )
