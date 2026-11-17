"""``fret-calib intrinsics`` — checkerboard camera matrix (FR-HW-33)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class IntrinsicsResult:
    camera_matrix: npt.NDArray[np.float64]
    dist_coeffs: npt.NDArray[np.float64]
    rms_px: float
    views_accepted: int


def calibrate_intrinsics_from_corners(
    object_points: Sequence[npt.NDArray[np.float64]],
    image_points: Sequence[npt.NDArray[np.float64]],
    image_size: tuple[int, int],
    *,
    min_views: int = 15,
) -> IntrinsicsResult:
    """Run OpenCV calibrateCamera on pre-detected corner sets."""
    if len(object_points) < min_views:
        raise ValueError(
            f"need at least {min_views} accepted views, got {len(object_points)}"
        )
    import cv2

    obj_pts = [
        np.asarray(p, dtype=np.float32).reshape(-1, 1, 3)
        for p in object_points
    ]
    img_pts = [
        np.asarray(p, dtype=np.float32).reshape(-1, 1, 2) for p in image_points
    ]
    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        obj_pts,
        img_pts,
        image_size,
        None,
        None,
    )
    return IntrinsicsResult(
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64).reshape(-1),
        rms_px=float(rms),
        views_accepted=len(object_points),
    )


def synthesize_checkerboard_views(
    *,
    views: int = 16,
    cols: int = 9,
    rows: int = 6,
    square_m: float = 0.025,
    image_size: tuple[int, int] = (1280, 720),
    fx: float = 900.0,
    fy: float = 900.0,
) -> tuple[
    list[npt.NDArray[np.float64]],
    list[npt.NDArray[np.float64]],
    tuple[int, int],
]:
    """Generate synthetic corner observations for unit tests (no images)."""
    obj = np.zeros((cols * rows, 3), dtype=np.float64)
    obj[:, :2] = (
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float64) * square_m
    )
    cx, cy = image_size[0] / 2.0, image_size[1] / 2.0
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    object_points: list[npt.NDArray[np.float64]] = []
    image_points: list[npt.NDArray[np.float64]] = []
    rng = np.random.default_rng(0)
    for i in range(views):
        rvec = np.array(
            [0.05 * (i % 3), -0.04 * (i % 4), 0.02 * (i % 5)],
            dtype=np.float64,
        )
        tvec = np.array(
            [0.02 * (i % 3), 0.01 * (i % 2), 0.45 + 0.01 * i],
            dtype=np.float64,
        )
        import cv2

        img_pts, _ = cv2.projectPoints(
            obj, rvec, tvec, k, np.zeros(5, dtype=np.float64)
        )
        pts2 = img_pts.reshape(-1, 2)
        noise = rng.normal(0.0, 0.05, size=pts2.shape)
        image_points.append((pts2 + noise).astype(np.float64))
        object_points.append(obj.copy())
    return object_points, image_points, image_size
