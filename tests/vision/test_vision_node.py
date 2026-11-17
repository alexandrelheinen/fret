"""FR-HW-31: vision_node logic with a stub pipeline."""

from __future__ import annotations

import numpy as np

from fret.vision.ros.vision_node import VisionNodeLogic
from fret.vision.types import BallObservation, CameraFrame


class _StubPipeline:
    def process(self, frames: list[CameraFrame]) -> BallObservation | None:
        assert frames
        return BallObservation(
            position_world=np.array([0.20, -0.12, 0.02]),
            radius_m=0.02,
            timestamp=frames[0].timestamp,
            source="stub",
        )


def test_vision_node_publishes_world_point() -> None:
    logic = VisionNodeLogic(_StubPipeline(), rate_hz=5.0)
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    obs = logic.process_bgr(image, stamp_s=1.0, frame_id="cam0")
    assert obs is not None
    assert obs["x"] == 0.20
    assert obs["frame_id"] == "world"
