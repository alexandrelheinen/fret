"""Computer-vision package for graspable-ball observation (v1.3+).

Public contracts stay algorithm-agnostic. Concrete MVP: HSV blob + table-plane
lift via :func:`~fret.vision.factory.build_hsv_plane_pipeline`.

Pure Python core: no ``rclpy`` or MuJoCo imports.
"""

from fret.vision.detect import HsvBlobBallDetector
from fret.vision.factory import (
    build_hsv_plane_pipeline,
    pipeline_from_hsv_plane_config,
)
from fret.vision.geometry import (
    TablePlanePoseLifter,
    intersect_ray_horizontal_plane,
    look_at_extrinsics,
)
from fret.vision.pipeline import BallVisionPipeline
from fret.vision.protocols import BallDetector, BallTracker, PoseLifter
from fret.vision.types import (
    BallDetection,
    BallObservation,
    CameraExtrinsics,
    CameraFrame,
    CameraIntrinsics,
    PlaceTarget,
    VisionConfig,
)

__all__ = [
    "BallDetection",
    "BallDetector",
    "BallObservation",
    "BallTracker",
    "BallVisionPipeline",
    "CameraExtrinsics",
    "CameraFrame",
    "CameraIntrinsics",
    "HsvBlobBallDetector",
    "PlaceTarget",
    "PoseLifter",
    "TablePlanePoseLifter",
    "VisionConfig",
    "build_hsv_plane_pipeline",
    "intersect_ray_horizontal_plane",
    "look_at_extrinsics",
    "pipeline_from_hsv_plane_config",
]
