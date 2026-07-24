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
            NotImplementedError: Level-3 stub — orchestration not filled yet.
            ValueError: If ``frames`` is empty.
        """
        if not frames:
            raise ValueError("frames must be non-empty")
        raise NotImplementedError(
            "BallVisionPipeline.process is a Level-3 stub; "
            "wire detector/tracker/lifter in the v1.3 implementation"
        )
