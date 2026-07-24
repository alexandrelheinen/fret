"""Tests that vision protocols are structural (algorithm-agnostic)."""

from __future__ import annotations

from typing import Sequence

import numpy as np

from fret.vision.protocols import BallDetector, BallTracker, PoseLifter
from fret.vision.types import BallDetection, BallObservation, CameraFrame


class _AnyDetector:
    """Stand-in with no algorithm semantics — only the call shape."""

    def detect(self, frames: Sequence[CameraFrame]) -> list[BallDetection]:
        assert frames
        return [
            BallDetection(
                camera_id=frames[0].camera_id,
                centre_px=(1.0, 2.0),
                radius_px=3.0,
                confidence=0.9,
            )
        ]


class _AnyTracker:
    def update(
        self,
        detections: Sequence[BallDetection],
        timestamp: float,
    ) -> list[BallDetection]:
        del timestamp
        return list(detections)


class _AnyLifter:
    def lift(
        self,
        detections: Sequence[BallDetection],
        frames: Sequence[CameraFrame],
    ) -> BallObservation | None:
        del frames
        if not detections:
            return None
        return BallObservation(
            position_world=np.zeros(3, dtype=np.float64),
            radius_m=0.01,
            timestamp=0.0,
            source="protocol_test",
        )


def test_structural_protocol_instances() -> None:
    assert isinstance(_AnyDetector(), BallDetector)
    assert isinstance(_AnyTracker(), BallTracker)
    assert isinstance(_AnyLifter(), PoseLifter)


def test_detector_lifter_round_trip_shape() -> None:
    frame = CameraFrame(
        camera_id="cam0",
        image=np.zeros((4, 4), dtype=np.uint8),
        timestamp=0.0,
        intrinsics_id="cam0",
    )
    detections = _AnyDetector().detect([frame])
    tracked = _AnyTracker().update(detections, timestamp=0.0)
    obs = _AnyLifter().lift(tracked, [frame])
    assert obs is not None
    assert obs.position_world.shape == (3,)
