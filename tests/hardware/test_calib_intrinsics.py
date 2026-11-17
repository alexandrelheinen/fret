"""FR-HW-33: synthetic checkerboard intrinsics recovery."""

from __future__ import annotations

import pytest

cv2 = pytest.importorskip("cv2")

from fret.calib.intrinsics import (  # noqa: E402
    calibrate_intrinsics_from_corners,
    synthesize_checkerboard_views,
)


def test_synthetic_intrinsics_rms_within_tolerance() -> None:
    objs, imgs, size = synthesize_checkerboard_views(views=16)
    result = calibrate_intrinsics_from_corners(objs, imgs, size, min_views=15)
    assert result.views_accepted == 16
    assert result.rms_px <= 1.0
