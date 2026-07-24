"""Unit tests for BallVisionPipeline scaffolding (Level 3)."""

from __future__ import annotations

from typing import Sequence

import numpy as np
import pytest

from fret.vision.pipeline import BallVisionPipeline
from fret.vision.protocols import BallDetector, PoseLifter
from fret.vision.types import (
    BallDetection,
    BallObservation,
    CameraFrame,
    CameraIntrinsics,
    VisionConfig,
)


class _StubDetector:
    def detect(self, frames: Sequence[CameraFrame]) -> list[BallDetection]:
        del frames
        return []


class _StubLifter:
    def lift(
        self,
        detections: Sequence[BallDetection],
        frames: Sequence[CameraFrame],
    ) -> BallObservation | None:
        del detections, frames
        return None


def _frame() -> CameraFrame:
    return CameraFrame(
        camera_id="cam0",
        image=np.zeros((8, 8, 3), dtype=np.uint8),
        timestamp=0.0,
        intrinsics_id="cam0",
    )


def _config() -> VisionConfig:
    return VisionConfig(
        intrinsics=(
            CameraIntrinsics(
                camera_id="cam0",
                width=8,
                height=8,
                fx=10.0,
                fy=10.0,
                cx=4.0,
                cy=4.0,
            ),
        )
    )


def test_pipeline_stores_dependencies() -> None:
    detector: BallDetector = _StubDetector()
    lifter: PoseLifter = _StubLifter()
    pipe = BallVisionPipeline(
        detector,
        lifter,
        config=_config(),
        source="scaffold",
    )
    assert pipe.config is not None
    assert pipe.source == "scaffold"
    assert isinstance(detector, BallDetector)
    assert isinstance(lifter, PoseLifter)


def test_pipeline_rejects_empty_source() -> None:
    with pytest.raises(ValueError, match="source"):
        BallVisionPipeline(_StubDetector(), _StubLifter(), source="")


def test_pipeline_process_returns_none_when_no_detection() -> None:
    pipe = BallVisionPipeline(_StubDetector(), _StubLifter())
    assert pipe.process([_frame()]) is None


def test_pipeline_process_rejects_empty_frames() -> None:
    pipe = BallVisionPipeline(_StubDetector(), _StubLifter())
    with pytest.raises(ValueError, match="non-empty"):
        pipe.process([])
