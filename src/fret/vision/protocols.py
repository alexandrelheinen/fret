"""Algorithm-agnostic protocols for ball vision stages.

Implementations plug in after algorithm selection. The scaffold defines only
the call shapes so inputs/outputs stay stable across candidates.
"""

from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

from fret.vision.types import BallDetection, BallObservation, CameraFrame


@runtime_checkable
class BallDetector(Protocol):
    """Produce image-space ball hypotheses from one or more frames."""

    def detect(self, frames: Sequence[CameraFrame]) -> list[BallDetection]:
        """Return zero or more detections (may be empty)."""
        ...


@runtime_checkable
class BallTracker(Protocol):
    """Optional temporal association over detections."""

    def update(
        self,
        detections: Sequence[BallDetection],
        timestamp: float,
    ) -> list[BallDetection]:
        """Return tracked / filtered detections for this timestamp."""
        ...


@runtime_checkable
class PoseLifter(Protocol):
    """Lift image-space detections to a world-frame ball observation."""

    def lift(
        self,
        detections: Sequence[BallDetection],
        frames: Sequence[CameraFrame],
    ) -> BallObservation | None:
        """Return one observation, or ``None`` if lift is impossible."""
        ...
