"""Computer-vision package for graspable-ball observation (v1.3+).

Algorithm-agnostic contracts and pipeline scaffolding. Concrete detectors /
lifters are selected and implemented separately; this package must not encode
a preferred algorithm.

Pure Python: no ``rclpy`` or MuJoCo imports.
"""

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
    "PlaceTarget",
    "PoseLifter",
    "VisionConfig",
]
