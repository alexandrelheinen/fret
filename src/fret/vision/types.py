"""Typed I/O contracts for the vision → manipulation boundary.

Algorithm-agnostic: these types describe images, detections, and world-frame
observations only. They must not encode colour spaces, feature methods, or
network architectures. See ``docs/interfaces.md`` § Vision.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics for one sensor (metres / pixels).

    Distortion models are intentionally omitted from the scaffold; add them
    when a concrete calibration path requires them.
    """

    camera_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        if not self.camera_id:
            raise ValueError("camera_id must be non-empty")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(
                f"width/height must be positive, got {self.width}x{self.height}"
            )
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(
                f"fx/fy must be positive, got fx={self.fx}, fy={self.fy}"
            )


@dataclass(frozen=True)
class CameraExtrinsics:
    """Rigid transform from camera frame to ``world`` (4×4 homogeneous)."""

    camera_id: str
    t_world_cam: npt.NDArray[np.float64]

    def __post_init__(self) -> None:
        if not self.camera_id:
            raise ValueError("camera_id must be non-empty")
        matrix = np.asarray(self.t_world_cam, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(
                f"t_world_cam must have shape (4, 4), got {matrix.shape}"
            )
        object.__setattr__(self, "t_world_cam", matrix)


@dataclass(frozen=True)
class CameraFrame:
    """One camera image sample for the vision pipeline.

    ``image`` is RGB ``(H, W, 3)`` uint8 or grayscale ``(H, W)`` uint8.
    ``intrinsics_id`` names the calibration entry in :class:`VisionConfig`
    (may equal ``camera_id``).
    """

    camera_id: str
    image: npt.NDArray[np.uint8]
    timestamp: float
    intrinsics_id: str

    def __post_init__(self) -> None:
        if not self.camera_id:
            raise ValueError("camera_id must be non-empty")
        if not self.intrinsics_id:
            raise ValueError("intrinsics_id must be non-empty")
        image = np.asarray(self.image)
        if image.dtype != np.uint8:
            raise ValueError(f"image dtype must be uint8, got {image.dtype}")
        if image.ndim == 2:
            pass
        elif image.ndim == 3 and image.shape[2] == 3:
            pass
        else:
            raise ValueError(
                "image must be (H, W) or (H, W, 3), "
                f"got shape {image.shape}"
            )
        object.__setattr__(self, "image", image)


@dataclass(frozen=True)
class BallDetection:
    """Image-space ball hypothesis from one or more views.

    Geometry is expressed in pixels only. How the centre / radius were
    estimated is left to the detector implementation.
    """

    camera_id: str
    centre_px: tuple[float, float]
    radius_px: float
    confidence: float

    def __post_init__(self) -> None:
        if not self.camera_id:
            raise ValueError("camera_id must be non-empty")
        if self.radius_px < 0.0:
            raise ValueError(f"radius_px must be >= 0, got {self.radius_px}")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(
                f"confidence must be in [0, 1], got {self.confidence}"
            )
        if len(self.centre_px) != 2:
            raise ValueError(
                f"centre_px must be length 2, got {self.centre_px!r}"
            )


@dataclass(frozen=True)
class BallObservation:
    """World-frame ball estimate for manipulation consumers.

    ``pickable`` is ``None`` until a pickability classifier exists (v1.5).
    ``source`` is an opaque producer id (pipeline / stage name), not an
    algorithm recipe.
    """

    position_world: npt.NDArray[np.float64]
    radius_m: float
    timestamp: float
    source: str
    covariance: npt.NDArray[np.float64] | None = None
    pickable: bool | None = None

    def __post_init__(self) -> None:
        position = np.asarray(self.position_world, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(
                f"position_world must have shape (3,), got {position.shape}"
            )
        object.__setattr__(self, "position_world", position)
        if self.radius_m < 0.0:
            raise ValueError(f"radius_m must be >= 0, got {self.radius_m}")
        if not self.source:
            raise ValueError("source must be non-empty")
        if self.covariance is not None:
            cov = np.asarray(self.covariance, dtype=np.float64)
            if cov.shape != (3, 3):
                raise ValueError(
                    f"covariance must have shape (3, 3), got {cov.shape}"
                )
            object.__setattr__(self, "covariance", cov)


@dataclass(frozen=True)
class PlaceTarget:
    """Known place / dispenser pose from scenario parameters (not from CV)."""

    position_world: npt.NDArray[np.float64]
    frame_id: str = "world"

    def __post_init__(self) -> None:
        if self.frame_id != "world":
            raise ValueError(
                f"PlaceTarget frame_id must be 'world', got '{self.frame_id}'"
            )
        position = np.asarray(self.position_world, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(
                f"position_world must have shape (3,), got {position.shape}"
            )
        object.__setattr__(self, "position_world", position)


@dataclass(frozen=True)
class VisionConfig:
    """Shared, algorithm-agnostic vision configuration.

    Holds camera calibration only. Detector / lifter / tracker knobs belong
    with those implementations (and their YAML), not here.
    """

    intrinsics: tuple[CameraIntrinsics, ...]
    extrinsics: tuple[CameraExtrinsics, ...] = ()

    def __post_init__(self) -> None:
        if not self.intrinsics:
            raise ValueError("VisionConfig.intrinsics must be non-empty")
        ids = [item.camera_id for item in self.intrinsics]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate camera_id in intrinsics: {ids}")
        ext_ids = [item.camera_id for item in self.extrinsics]
        if len(ext_ids) != len(set(ext_ids)):
            raise ValueError(f"duplicate camera_id in extrinsics: {ext_ids}")

    def intrinsics_for(self, camera_id: str) -> CameraIntrinsics:
        """Return intrinsics for ``camera_id`` or raise ``KeyError``."""
        for item in self.intrinsics:
            if item.camera_id == camera_id:
                return item
        raise KeyError(f"no intrinsics for camera_id={camera_id!r}")

    def extrinsics_for(self, camera_id: str) -> CameraExtrinsics:
        """Return extrinsics for ``camera_id`` or raise ``KeyError``."""
        for item in self.extrinsics:
            if item.camera_id == camera_id:
                return item
        raise KeyError(f"no extrinsics for camera_id={camera_id!r}")
