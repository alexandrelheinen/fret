"""Pixel → world lift via ray ∩ horizontal table plane."""

from __future__ import annotations

from typing import Sequence

import numpy as np
import numpy.typing as npt

from fret.vision.config import TablePlaneLifterConfig
from fret.vision.types import (
    BallDetection,
    BallObservation,
    CameraExtrinsics,
    CameraFrame,
    CameraIntrinsics,
    VisionConfig,
)


def _pixel_ray_cam(
    u: float,
    v: float,
    intrinsics: CameraIntrinsics,
) -> npt.NDArray[np.float64]:
    """Unit ray in the camera optical frame (X right, Y down, Z forward)."""
    x = (u - intrinsics.cx) / intrinsics.fx
    y = (v - intrinsics.cy) / intrinsics.fy
    direction = np.array([x, y, 1.0], dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-12:
        raise ValueError("degenerate pixel ray")
    return direction / norm


def intersect_ray_horizontal_plane(
    origin_world: npt.NDArray[np.float64],
    direction_world: npt.NDArray[np.float64],
    plane_z_m: float,
) -> npt.NDArray[np.float64] | None:
    """Intersect ray with ``z = plane_z_m``. None if parallel or behind camera."""
    dz = float(direction_world[2])
    if abs(dz) < 1e-12:
        return None
    t = (plane_z_m - float(origin_world[2])) / dz
    if t <= 0.0:
        return None
    return origin_world + t * direction_world


class TablePlanePoseLifter:
    """Lift the best detection using a known horizontal support plane.

    Ball centre plane: ``z = table_z_m + ball_radius_m``.
    Uses :class:`~fret.vision.types.VisionConfig` extrinsics/intrinsics for the
    configured camera. Does not use robot proprioception.
    """

    def __init__(
        self,
        config: TablePlaneLifterConfig,
        vision: VisionConfig,
        *,
        source: str = "table_plane",
    ) -> None:
        if not source:
            raise ValueError("source must be non-empty")
        self._cfg = config
        self._intrinsics = vision.intrinsics_for(config.camera_id)
        self._extrinsics = vision.extrinsics_for(config.camera_id)
        self._source = source

    @property
    def config(self) -> TablePlaneLifterConfig:
        return self._cfg

    def lift(
        self,
        detections: Sequence[BallDetection],
        frames: Sequence[CameraFrame],
    ) -> BallObservation | None:
        if not detections:
            return None
        # Prefer highest confidence among this camera's detections.
        candidates = [
            d for d in detections if d.camera_id == self._cfg.camera_id
        ]
        if not candidates:
            return None
        detection = max(candidates, key=lambda d: d.confidence)

        t_world_cam = self._extrinsics.t_world_cam
        rotation = t_world_cam[:3, :3]
        origin = t_world_cam[:3, 3]
        direction_cam = _pixel_ray_cam(
            detection.centre_px[0],
            detection.centre_px[1],
            self._intrinsics,
        )
        direction_world = rotation @ direction_cam
        plane_z = self._cfg.table_z_m + self._cfg.ball_radius_m
        point = intersect_ray_horizontal_plane(
            origin, direction_world, plane_z
        )
        if point is None:
            return None

        timestamp = 0.0
        matching = [f for f in frames if f.camera_id == detection.camera_id]
        if matching:
            timestamp = float(matching[0].timestamp)

        return BallObservation(
            position_world=np.asarray(point, dtype=np.float64),
            radius_m=float(self._cfg.ball_radius_m),
            timestamp=timestamp,
            source=self._source,
            pickable=None,
        )


def look_at_extrinsics(
    *,
    camera_id: str,
    eye_world: Sequence[float],
    target_world: Sequence[float],
    up_world: Sequence[float] = (0.0, 0.0, 1.0),
) -> CameraExtrinsics:
    """Build ``T_world_cam`` for a camera looking from ``eye`` toward ``target``.

    Optical axes: camera +Z toward target, +X right, +Y down (OpenCV).
    Useful for tests and MuJoCo-style overhead mounts.
    """
    eye = np.asarray(eye_world, dtype=np.float64).reshape(3)
    target = np.asarray(target_world, dtype=np.float64).reshape(3)
    up = np.asarray(up_world, dtype=np.float64).reshape(3)
    forward = target - eye
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm <= 1e-12:
        raise ValueError("eye and target must differ")
    z_cam = forward / forward_norm
    # OpenCV Y down: prefer world -up when looking down so +Y image aligns.
    x_cam = np.cross(z_cam, -up if abs(float(z_cam[2])) > 0.9 else up)
    x_norm = float(np.linalg.norm(x_cam))
    if x_norm <= 1e-12:
        x_cam = np.cross(z_cam, np.array([0.0, 1.0, 0.0], dtype=np.float64))
        x_norm = float(np.linalg.norm(x_cam))
    x_cam = x_cam / x_norm
    y_cam = np.cross(z_cam, x_cam)
    rotation = np.column_stack((x_cam, y_cam, z_cam))
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = eye
    return CameraExtrinsics(camera_id=camera_id, t_world_cam=matrix)
