"""Ball vision pipeline: frames in → optional world observation out.

Composes detector → optional tracker → pose lifter. Concrete algorithms are
injected; this module does not select or embed one.
"""

from __future__ import annotations

from typing import Sequence

from fret.vision.protocols import BallDetector, BallTracker, PoseLifter
from fret.vision.types import BallObservation, CameraFrame, VisionConfig


class BallVisionPipeline:
    """Orchestrates vision stages behind a stable I/O boundary.

    Args:
        detector: Image-space ball detector (required).
        lifter: Pixel → world pose lifter (required).
        tracker: Optional temporal filter between detect and lift.
        config: Shared camera calibration (algorithm-agnostic).
        source: Opaque id written onto successful :class:`BallObservation`
            values by a future Level-4 implementation (stored for API shape).
    """

    def __init__(
        self,
        detector: BallDetector,
        lifter: PoseLifter,
        *,
        tracker: BallTracker | None = None,
        config: VisionConfig | None = None,
        source: str = "ball_vision_pipeline",
    ) -> None:
        if not source:
            raise ValueError("source must be non-empty")
        self._detector = detector
        self._lifter = lifter
        self._tracker = tracker
        self._config = config
        self._source = source

    @property
    def config(self) -> VisionConfig | None:
        """Shared vision config, if provided at construction."""
        return self._config

    @property
    def source(self) -> str:
        """Opaque producer id for observations emitted by this pipeline."""
        return self._source

    def process(self, frames: Sequence[CameraFrame]) -> BallObservation | None:
        """Run detect → (track) → lift.

        Returns:
            A world-frame :class:`BallObservation`, or ``None`` when no ball
            can be reported (never fabricates a pose).

        Raises:
            ValueError: If ``frames`` is empty.
        """
        if not frames:
            raise ValueError("frames must be non-empty")
        detections = self._detector.detect(frames)
        if self._tracker is not None:
            timestamp = float(frames[0].timestamp)
            detections = self._tracker.update(detections, timestamp)
        observation = self._lifter.lift(detections, frames)
        if observation is None:
            return None
        if observation.source == self._source:
            return observation
        # Normalize producer id to the pipeline source when lifter used another.
        return BallObservation(
            position_world=observation.position_world,
            radius_m=observation.radius_m,
            timestamp=observation.timestamp,
            source=self._source,
            covariance=observation.covariance,
            pickable=observation.pickable,
        )
