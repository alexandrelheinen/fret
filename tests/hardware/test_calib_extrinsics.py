"""FR-HW-33: synthetic extrinsics pose inversion."""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from fret.calib.extrinsics import estimate_extrinsics_from_pose  # noqa: E402


def test_extrinsics_roundtrip_translation() -> None:
    result = estimate_extrinsics_from_pose(
        np.zeros(3),
        np.array([0.1, -0.2, 0.5]),
        reprojection_error_px=0.25,
    )
    assert result.t_world_cam.shape == (4, 4)
    assert result.reprojection_error_px == pytest.approx(0.25)
    # Inverse of pure translation t should put camera at -R^T t ≈ -t.
    assert result.t_world_cam[2, 3] == pytest.approx(-0.5)
