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
    """Lift detections via ray ∩ horizontal table plane (mono or multi-view).

    Ball centre plane: ``z = table_z_m + ball_radius_m``.
    When several configured cameras report a ball, fuses world XY by
    confidence-weighted mean (Z stays on the plane). A single surviving view
    is enough — obstruction of one gate camera is OK.
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
        self._vision = vision
        self._source = source

    @property
    def config(self) -> TablePlaneLifterConfig:
        return self._cfg

    def _lift_one(
        self, detection: BallDetection
    ) -> npt.NDArray[np.float64] | None:
        intrinsics = self._vision.intrinsics_for(detection.camera_id)
        extrinsics = self._vision.extrinsics_for(detection.camera_id)
        t_world_cam = extrinsics.t_world_cam
        rotation = t_world_cam[:3, :3]
        origin = t_world_cam[:3, 3]
        direction_cam = _pixel_ray_cam(
            detection.centre_px[0],
            detection.centre_px[1],
            intrinsics,
        )
        direction_world = rotation @ direction_cam
        plane_z = self._cfg.table_z_m + self._cfg.ball_radius_m
        return intersect_ray_horizontal_plane(origin, direction_world, plane_z)

    def lift(
        self,
        detections: Sequence[BallDetection],
        frames: Sequence[CameraFrame],
    ) -> BallObservation | None:
        if not detections:
            return None
        allowed = set(self._cfg.camera_ids)
        # Best detection per camera (highest confidence).
        best_by_cam: dict[str, BallDetection] = {}
        for det in detections:
            if det.camera_id not in allowed:
                continue
            prev = best_by_cam.get(det.camera_id)
            if prev is None or det.confidence > prev.confidence:
                best_by_cam[det.camera_id] = det
        if not best_by_cam:
            return None

        points: list[npt.NDArray[np.float64]] = []
        weights: list[float] = []
        for det in best_by_cam.values():
            point = self._lift_one(det)
            if point is None:
                continue
            points.append(point)
            weights.append(max(float(det.confidence), 1e-3))
        if not points:
            return None

        w = np.asarray(weights, dtype=np.float64)
        w = w / float(np.sum(w))
        stacked = np.stack(points, axis=0)
        fused = np.sum(stacked * w[:, None], axis=0)
        # Keep fused point on the support plane.
        fused[2] = self._cfg.table_z_m + self._cfg.ball_radius_m

        timestamp = 0.0
        for det in best_by_cam.values():
            matching = [f for f in frames if f.camera_id == det.camera_id]
            if matching:
                timestamp = max(timestamp, float(matching[0].timestamp))

        n_views = len(points)
        source = (
            f"{self._source}_fused_{n_views}v"
            if n_views > 1
            else f"{self._source}_mono"
        )
        return BallObservation(
            position_world=np.asarray(fused, dtype=np.float64),
            radius_m=float(self._cfg.ball_radius_m),
            timestamp=timestamp,
            source=source,
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
