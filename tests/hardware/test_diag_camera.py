"""FR-HW-30/32: camera diag with fake V4L2 info."""

from __future__ import annotations

from fret.diag.camera import run_camera_checks
from fret.diag.core import CheckStatus


def test_camera_fake_720p_pass() -> None:
    results = run_camera_checks(
        fake_info={"width": 1280, "height": 720, "fps": 15.0, "format": "MJPG"}
    )
    assert results[0].status is CheckStatus.PASS


def test_camera_fake_low_res_fail() -> None:
    results = run_camera_checks(
        fake_info={"width": 640, "height": 480, "fps": 30.0, "format": "YUYV"}
    )
    assert results[0].status is CheckStatus.FAIL
