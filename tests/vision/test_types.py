"""Unit tests for fret.vision typed contracts (algorithm-agnostic)."""

from __future__ import annotations

import numpy as np
import pytest

from fret.vision.types import (
    BallDetection,
    BallObservation,
    CameraExtrinsics,
    CameraFrame,
    CameraIntrinsics,
    PlaceTarget,
    VisionConfig,
)


def _rgb(h: int = 4, w: int = 6) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_camera_intrinsics_rejects_non_positive_focal() -> None:
    with pytest.raises(ValueError, match="fx/fy"):
        CameraIntrinsics(
            camera_id="cam0",
            width=640,
            height=480,
            fx=0.0,
            fy=500.0,
            cx=320.0,
            cy=240.0,
        )


def test_camera_frame_accepts_rgb_and_gray() -> None:
    rgb = CameraFrame(
        camera_id="cam0",
        image=_rgb(),
        timestamp=1.0,
        intrinsics_id="cam0",
    )
    assert rgb.image.shape == (4, 6, 3)
    gray = CameraFrame(
        camera_id="cam0",
        image=np.zeros((4, 6), dtype=np.uint8),
        timestamp=1.0,
        intrinsics_id="cam0",
    )
    assert gray.image.ndim == 2


def test_camera_frame_rejects_float_image() -> None:
    with pytest.raises(ValueError, match="uint8"):
        CameraFrame(
            camera_id="cam0",
            image=np.zeros((4, 6, 3), dtype=np.float32),
            timestamp=0.0,
            intrinsics_id="cam0",
        )


def test_ball_detection_confidence_bounds() -> None:
    with pytest.raises(ValueError, match="confidence"):
        BallDetection(
            camera_id="cam0",
            centre_px=(10.0, 20.0),
            radius_px=5.0,
            confidence=1.5,
        )


def test_ball_observation_world_position_shape() -> None:
    obs = BallObservation(
        position_world=np.array([0.1, 0.2, 0.03], dtype=np.float64),
        radius_m=0.025,
        timestamp=2.0,
        source="unit_test",
    )
    assert obs.position_world.shape == (3,)
    assert obs.pickable is None
    with pytest.raises(ValueError, match="shape \\(3,\\)"):
        BallObservation(
            position_world=np.array([0.1, 0.2], dtype=np.float64),
            radius_m=0.025,
            timestamp=2.0,
            source="unit_test",
        )


def test_place_target_requires_world_frame() -> None:
    target = PlaceTarget(
        position_world=np.array([0.3, 0.0, 0.05], dtype=np.float64)
    )
    assert target.frame_id == "world"
    with pytest.raises(ValueError, match="world"):
        PlaceTarget(
            position_world=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            frame_id="base_link",
        )


def test_vision_config_lookup() -> None:
    intr = CameraIntrinsics(
        camera_id="cam0",
        width=640,
        height=480,
        fx=500.0,
        fy=500.0,
        cx=320.0,
        cy=240.0,
    )
    ext = CameraExtrinsics(
        camera_id="cam0",
        t_world_cam=np.eye(4, dtype=np.float64),
    )
    cfg = VisionConfig(intrinsics=(intr,), extrinsics=(ext,))
    assert cfg.intrinsics_for("cam0") is intr
    assert cfg.extrinsics_for("cam0") is ext
    with pytest.raises(KeyError):
        cfg.intrinsics_for("missing")


def test_vision_config_rejects_empty_intrinsics() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        VisionConfig(intrinsics=())
