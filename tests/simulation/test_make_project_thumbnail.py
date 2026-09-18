"""Tests for scripts/make_project_thumbnail.py (FR-SIM-14 composite).

The composite is deliberately separate from the renders: these tests pin
the geometry it uses (equal halves, the seam, the clearance check) and the
fact that it refuses to place a subject across the cut.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "scripts"
sys.path.insert(0, str(_SCRIPTS))

import make_project_thumbnail as mt  # noqa: E402

_WIDTH = 1920
_HEIGHT = 1080


def _half(
    profile: list[list[float]], *, width: int = 2560, height: int = _HEIGHT
) -> mt.Half:
    image = np.full((height, width, 3), 200, dtype=np.uint8)
    return mt.Half(image=image, profile=profile, label="probe")


def test_seam_runs_from_top_fraction_to_bottom_fraction() -> None:
    """The seam is a straight line between the two configured fractions."""
    top = mt.seam_x_at(0.0, height=_HEIGHT, width=_WIDTH)
    bottom = mt.seam_x_at(float(_HEIGHT), height=_HEIGHT, width=_WIDTH)
    assert top == pytest.approx(mt._SEAM_TOP * _WIDTH)
    assert bottom == pytest.approx(mt._SEAM_BOTTOM * _WIDTH)


def test_the_two_halves_have_equal_area() -> None:
    """A 50/50 cut is the point; the seam crosses the midline at mid height."""
    middle = mt.seam_x_at(_HEIGHT / 2.0, height=_HEIGHT, width=_WIDTH)
    assert middle == pytest.approx(_WIDTH / 2.0)
    assert (mt._SEAM_TOP + mt._SEAM_BOTTOM) / 2.0 == pytest.approx(0.5)


def test_region_centroids_are_mirrored_about_the_midline() -> None:
    """Equal areas means the two centroids sit symmetrically."""
    left_x, left_y = mt.region_centroid(
        width=_WIDTH, height=_HEIGHT, side="left"
    )
    right_x, right_y = mt.region_centroid(
        width=_WIDTH, height=_HEIGHT, side="right"
    )
    assert left_x + right_x == pytest.approx(float(_WIDTH), abs=1.0)
    # The left region is bottom heavy and the right one top heavy by the
    # same amount, so their vertical centroids straddle the middle row.
    assert left_y + right_y == pytest.approx(float(_HEIGHT), abs=1.0)


def test_a_narrow_band_may_sit_where_a_wide_one_could_not() -> None:
    """The per-band check is what a bounding box would get wrong."""
    seam_top = mt.seam_x_at(0.05 * _HEIGHT, height=_HEIGHT, width=_WIDTH)
    narrow = [[0.0, 0.1 * _HEIGHT, seam_top - 200.0, seam_top - 120.0]]
    wide = [[0.0, 0.1 * _HEIGHT, seam_top - 900.0, seam_top - 60.0]]
    ok_narrow, _ = mt.subject_clears_seam(
        narrow, width=_WIDTH, height=_HEIGHT, side="left", clearance_px=40.0
    )
    ok_wide, _ = mt.subject_clears_seam(
        wide, width=_WIDTH, height=_HEIGHT, side="left", clearance_px=40.0
    )
    assert ok_narrow
    assert not ok_wide


def test_a_subject_crossing_the_seam_is_rejected() -> None:
    """Nothing is allowed to straddle the cut."""
    middle = _WIDTH / 2.0
    band = [[0.4 * _HEIGHT, 0.6 * _HEIGHT, middle - 50.0, middle + 50.0]]
    ok, gap = mt.subject_clears_seam(
        band, width=_WIDTH, height=_HEIGHT, side="left", clearance_px=0.0
    )
    assert not ok
    assert gap < 0.0


def test_the_edge_margin_is_enforced_as_well_as_the_seam() -> None:
    """A subject hard against the outer border fails even if the seam is far."""
    band = [[0.4 * _HEIGHT, 0.6 * _HEIGHT, 5.0, 200.0]]
    ok, _ = mt.subject_clears_seam(
        band,
        width=_WIDTH,
        height=_HEIGHT,
        side="left",
        clearance_px=0.0,
        edge_px=60.0,
    )
    assert not ok


def test_solve_offset_refuses_a_subject_too_wide_for_its_half() -> None:
    """Fail loudly rather than quietly cutting the robot in half."""
    half = _half([[0.0, 1.0, 0.05, 0.95]])
    with pytest.raises(mt.ThumbnailError, match="clear of the seam"):
        mt.solve_offset(
            half,
            width=_WIDTH,
            height=_HEIGHT,
            side="left",
            clearance_px=40.0,
            edge_px=40.0,
        )


def test_solve_offset_places_a_narrow_subject_inside_its_half() -> None:
    """A subject that fits is panned until it clears."""
    half = _half([[0.2, 0.8, 0.45, 0.60]])
    offset = mt.solve_offset(
        half,
        width=_WIDTH,
        height=_HEIGHT,
        side="left",
        clearance_px=40.0,
        edge_px=40.0,
    )
    ok, _ = mt.subject_clears_seam(
        half.bands_px(offset),
        width=_WIDTH,
        height=_HEIGHT,
        side="left",
        clearance_px=40.0,
        edge_px=40.0,
    )
    assert ok


def test_the_accent_shard_is_a_real_polygon() -> None:
    """The shard is copied from CSS, so its winding must not silently empty."""
    mask = mt._shard_mask(_WIDTH, _HEIGHT)
    assert 0.02 < float(mask.mean()) < 0.30


def test_compose_returns_a_full_frame_and_its_measurements() -> None:
    """The composite is the requested size and reports how it was placed."""
    left = _half([[0.2, 0.8, 0.30, 0.45]])
    right = _half([[0.2, 0.8, 0.55, 0.70]])
    canvas, report = mt.compose(left, right, clearance_px=40.0, edge_px=40.0)
    assert canvas.shape == (_HEIGHT, _WIDTH, 3)
    assert canvas.dtype == np.uint8
    assert report["left_within_margins"] is True
    assert report["right_within_margins"] is True
    assert report["shard"] is False


def test_compose_honors_hand_chosen_offsets() -> None:
    """A hand-placed pan is reported as such rather than silently solved."""
    left = _half([[0.2, 0.8, 0.30, 0.45]])
    right = _half([[0.2, 0.8, 0.55, 0.70]])
    _, report = mt.compose(
        left,
        right,
        clearance_px=40.0,
        edge_px=40.0,
        left_offset=-120,
        right_offset=90,
    )
    assert report["left_offset_px"] == -120
    assert report["right_offset_px"] == 90


def test_load_half_requires_the_extent_profile(tmp_path) -> None:
    """Old metrics files are refused instead of guessing a subject shape."""
    metrics = tmp_path / "metrics.json"
    metrics.write_text('[{"camera": "cam", "extent": {}}]', encoding="utf-8")
    with pytest.raises(mt.ThumbnailError, match="extent_profile"):
        mt.load_half(tmp_path / "missing.png", metrics, "cam", "probe")


def test_load_half_reports_an_unknown_camera(tmp_path) -> None:
    """A camera that is not in the metrics is named in the error."""
    metrics = tmp_path / "metrics.json"
    metrics.write_text('[{"camera": "other"}]', encoding="utf-8")
    with pytest.raises(mt.ThumbnailError, match="not in"):
        mt.load_half(tmp_path / "missing.png", metrics, "cam", "probe")
