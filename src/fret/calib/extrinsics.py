"""``fret-calib extrinsics`` — ChArUco ``t_world_cam`` (FR-HW-33)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class ExtrinsicsResult:
    t_world_cam: npt.NDArray[np.float64]  # 4x4
    reprojection_error_px: float


def estimate_extrinsics_from_pose(
    rvec: npt.NDArray[np.float64],
    tvec: npt.NDArray[np.float64],
    *,
    reprojection_error_px: float,
) -> ExtrinsicsResult:
    """Build ``t_world_cam`` from an OpenCV board pose (board in world)."""
    import cv2

    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = rot
    t[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    # OpenCV gives t_cam_board; for a board fixed in world we invert.
    t_world_cam = np.linalg.inv(t)
    return ExtrinsicsResult(
        t_world_cam=t_world_cam,
        reprojection_error_px=float(reprojection_error_px),
    )
