"""HSV blob detector + table-plane lifter + wired pipeline tests."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from fret.vision.config import load_hsv_plane_pipeline_config
from fret.vision.detect.hsv_blob import HsvBlobBallDetector
from fret.vision.factory import build_hsv_plane_pipeline
from fret.vision.geometry.plane_lifter import (
    TablePlanePoseLifter,
    look_at_extrinsics,
)
from fret.vision.types import (
    CameraFrame,
    CameraIntrinsics,
    VisionConfig,
)

_CFG = Path("src/fret/config/vision/hsv_blob_overhead.yml")


def _orange_ball_rgb(
    *,
    width: int = 640,
    height: int = 480,
    centre: tuple[int, int] = (320, 240),
    radius: int = 40,
) -> np.ndarray:
    """Synthetic RGB image with an orange disk on a dark background."""
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (30, 30, 30)
    # RGB orange roughly matching OpenCV HSV lower/upper in the YAML.
    cv2.circle(image, centre, radius, (255, 140, 20), thickness=-1)
    return image


def test_load_default_hsv_plane_config() -> None:
    cfg = load_hsv_plane_pipeline_config(_CFG)
    assert cfg.detector.camera_id == "overhead"
    assert cfg.lifter.ball_radius_m == pytest.approx(0.025)
    assert cfg.vision.intrinsics_for("overhead").width == 640


def test_hsv_detector_finds_synthetic_ball() -> None:
    cfg = load_hsv_plane_pipeline_config(_CFG)
    detector = HsvBlobBallDetector(cfg.detector)
    centre = (400, 200)
    frame = CameraFrame(
        camera_id="overhead",
        image=_orange_ball_rgb(centre=centre, radius=35),
        timestamp=1.0,
        intrinsics_id="overhead",
    )
    detections = detector.detect([frame])
    assert len(detections) == 1
    u, v = detections[0].centre_px
    assert abs(u - centre[0]) <= 3.0
    assert abs(v - centre[1]) <= 3.0
    assert detections[0].confidence >= 0.55


def test_hsv_detector_returns_empty_without_ball() -> None:
    cfg = load_hsv_plane_pipeline_config(_CFG)
    detector = HsvBlobBallDetector(cfg.detector)
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    frame = CameraFrame(
        camera_id="overhead",
        image=blank,
        timestamp=0.0,
        intrinsics_id="overhead",
    )
    assert detector.detect([frame]) == []


def test_table_plane_lifter_nadir_camera() -> None:
    """Camera above table looking down: pixel centre maps near world origin."""
    cfg = load_hsv_plane_pipeline_config(_CFG)
    extrinsics = look_at_extrinsics(
        camera_id="overhead",
        eye_world=(0.0, 0.0, 1.0),
        target_world=(0.0, 0.0, 0.0),
        up_world=(0.0, 0.0, 1.0),
    )
    vision = VisionConfig(
        intrinsics=cfg.vision.intrinsics,
        extrinsics=(extrinsics,),
    )
    lifter = TablePlanePoseLifter(cfg.lifter, vision, source="test")
    # Principal point should hit near (0, 0, r) for nadir camera at z=1.
    from fret.vision.types import BallDetection

    det = BallDetection(
        camera_id="overhead",
        centre_px=(320.0, 240.0),
        radius_px=20.0,
        confidence=0.9,
    )
    obs = lifter.lift([det], [])
    assert obs is not None
    assert abs(float(obs.position_world[0])) < 0.02
    assert abs(float(obs.position_world[1])) < 0.02
    assert float(obs.position_world[2]) == pytest.approx(
        cfg.lifter.table_z_m + cfg.lifter.ball_radius_m, abs=1e-6
    )


def test_build_pipeline_end_to_end_synthetic() -> None:
    cfg = load_hsv_plane_pipeline_config(_CFG)
    extrinsics = look_at_extrinsics(
        camera_id="overhead",
        eye_world=(0.0, 0.0, 1.2),
        target_world=(0.0, 0.0, 0.0),
    )
    # Rebuild config with overhead look-at extrinsics for a meaningful lift.
    from fret.vision.config import HsvPlanePipelineConfig
    from fret.vision.factory import pipeline_from_hsv_plane_config

    wired = HsvPlanePipelineConfig(
        vision=VisionConfig(
            intrinsics=cfg.vision.intrinsics, extrinsics=(extrinsics,)
        ),
        detector=cfg.detector,
        lifter=cfg.lifter,
        source=cfg.source,
    )
    pipe = pipeline_from_hsv_plane_config(wired)
    # Project world point (0.1, -0.05, r) into the camera for a consistent GT.
    r = cfg.lifter.ball_radius_m
    point_w = np.array([0.1, -0.05, r], dtype=np.float64)
    t_world_cam = extrinsics.t_world_cam
    t_cam_world = np.linalg.inv(t_world_cam)
    point_c = t_cam_world[:3, :3] @ point_w + t_cam_world[:3, 3]
    assert point_c[2] > 0.0
    intr = cfg.vision.intrinsics_for("overhead")
    u = intr.fx * (point_c[0] / point_c[2]) + intr.cx
    v = intr.fy * (point_c[1] / point_c[2]) + intr.cy
    centre = (int(round(u)), int(round(v)))
    frame = CameraFrame(
        camera_id="overhead",
        image=_orange_ball_rgb(centre=centre, radius=28),
        timestamp=3.0,
        intrinsics_id="overhead",
    )
    obs = pipe.process([frame])
    assert obs is not None
    assert float(np.linalg.norm(obs.position_world[:2] - point_w[:2])) < 0.03
    assert obs.source == cfg.source


def test_build_hsv_plane_pipeline_from_default_yaml() -> None:
    pipe = build_hsv_plane_pipeline(_CFG)
    assert pipe.config is not None
    assert pipe.source == "hsv_blob_table_plane"
