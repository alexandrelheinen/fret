"""Magnetic grasp finite-state machine for PPP warehouse pick-and-place (v1.0).

Models cargo handling without a gripper: when the gantry end-effector enters a
capture zone the box welds to the EE frame; during transport the welded cargo
envelope is queryable for planning; at the goal the weld releases and the box
remains at the release pose.

Satisfies requirements FR-GSP-01, FR-GSP-03, and FR-GSP-04.
Provides FR-GSP-02 hooks via ``is_welded`` and ``cargo_corners``.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

# Default scenario parameters (docs/scenarios.md, docs/robots/ppp.md).
_DEFAULT_CAPTURE_RADIUS: float = 0.3
_DEFAULT_GOAL_RADIUS: float = 0.5
_DEFAULT_WELD_OFFSET: npt.NDArray[np.float64] = np.array(
    [0.0, 0.0, 0.25], dtype=np.float64
)
_DEFAULT_BOX_HALF_EXTENT: npt.NDArray[np.float64] = np.array(
    [0.25, 0.25, 0.25], dtype=np.float64
)


class GraspState(enum.IntEnum):
    """Magnetic grasp FSM states (FR-GSP-04)."""

    IDLE = 0
    APPROACH = 1
    CAPTURE = 2
    TRANSPORT = 3
    RELEASE = 4


@dataclass(frozen=True)
class GraspConfig:
    """Configuration for the magnetic grasp FSM.

    Attributes:
        capture_radius: Distance threshold for welding [m].
        goal_radius: Distance threshold for release at goal [m].
        weld_offset: Cargo centre offset from EE in world axes [m].
        box_half_extent: Half-sizes of the axis-aligned cargo box [m].
    """

    capture_radius: float = _DEFAULT_CAPTURE_RADIUS
    goal_radius: float = _DEFAULT_GOAL_RADIUS
    weld_offset: npt.NDArray[np.float64] = field(
        default_factory=lambda: _DEFAULT_WELD_OFFSET.copy()
    )
    box_half_extent: npt.NDArray[np.float64] = field(
        default_factory=lambda: _DEFAULT_BOX_HALF_EXTENT.copy()
    )

    def __post_init__(self) -> None:
        if self.capture_radius <= 0.0:
            raise ValueError(
                f"capture_radius must be positive, got {self.capture_radius}"
            )
        if self.goal_radius <= 0.0:
            raise ValueError(
                f"goal_radius must be positive, got {self.goal_radius}"
            )
        if np.any(self.box_half_extent <= 0.0):
            raise ValueError("box_half_extent components must be positive")


class MagneticGraspFSM:
    """Pure-Python magnetic weld / release state machine.

    Level-3 logic core mirroring ``ControllerNode``: testable without ROS.
    The FSM is driven by periodic calls to ``update`` with world-frame EE,
    box, and goal positions.

    Args:
        config: Grasp radii, weld offset, and cargo geometry.
    """

    def __init__(self, config: GraspConfig | None = None) -> None:
        self._config = config if config is not None else GraspConfig()
        self._state: GraspState = GraspState.IDLE
        self._cargo_position: npt.NDArray[np.float64] = np.zeros(
            3, dtype=np.float64
        )

    @property
    def state(self) -> GraspState:
        """Current FSM state."""
        return self._state

    @property
    def is_welded(self) -> bool:
        """Whether cargo is currently welded to the end-effector."""
        return self._state in (GraspState.CAPTURE, GraspState.TRANSPORT)

    @property
    def cargo_position(self) -> npt.NDArray[np.float64]:
        """World-frame centre of the cargo box [m]."""
        return self._cargo_position.copy()

    def begin_transport(self) -> None:
        """Signal task start: transition IDLE → APPROACH."""
        if self._state == GraspState.IDLE:
            self._state = GraspState.APPROACH

    def update(
        self,
        ee_position: npt.NDArray[np.float64],
        box_position: npt.NDArray[np.float64],
        goal_position: npt.NDArray[np.float64],
    ) -> GraspState:
        """Advance the FSM by one tick.

        Args:
            ee_position: End-effector position in ``world``, shape ``(3,)``.
            box_position: Nominal box position before weld, shape ``(3,)``.
            goal_position: Goal position for release check, shape ``(3,)``.

        Returns:
            The FSM state after this update.
        """
        ee = np.asarray(ee_position, dtype=np.float64).reshape(3)
        box = np.asarray(box_position, dtype=np.float64).reshape(3)
        goal = np.asarray(goal_position, dtype=np.float64).reshape(3)
        cfg = self._config

        if self._state == GraspState.IDLE:
            return self._state

        if self._state == GraspState.APPROACH:
            if np.linalg.norm(ee - box) < cfg.capture_radius:
                self._state = GraspState.CAPTURE
                self._cargo_position = ee + cfg.weld_offset
                self._state = GraspState.TRANSPORT
            return self._state

        if self._state == GraspState.TRANSPORT:
            self._cargo_position = ee + cfg.weld_offset
            if np.linalg.norm(ee - goal) < cfg.goal_radius:
                self._state = GraspState.RELEASE
                self._state = GraspState.IDLE
            return self._state

        return self._state

    def cargo_corners(self) -> npt.NDArray[np.float64]:
        """Return eight world-frame corners of the welded cargo AABB.

        Used as an FR-GSP-02 hook for ``CSpaceChecker`` integration.
        Returns an empty ``(0, 3)`` array when not welded.

        Returns:
            Array of shape ``(8, 3)`` or ``(0, 3)``.
        """
        if not self.is_welded:
            return np.empty((0, 3), dtype=np.float64)

        centre = self._cargo_position
        hx, hy, hz = self._config.box_half_extent
        offsets = np.array(
            [
                [-hx, -hy, -hz],
                [hx, -hy, -hz],
                [-hx, hy, -hz],
                [hx, hy, -hz],
                [-hx, -hy, hz],
                [hx, -hy, hz],
                [-hx, hy, hz],
                [hx, hy, hz],
            ],
            dtype=np.float64,
        )
        return centre + offsets
