"""Tests for scripts/release/gallery_manifest.py."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "scripts" / "release"))

from gallery_manifest import (
    GradeSpec,
    assert_sixteen_by_nine,
    box_downscale,
    ghost_indices,
    ghost_weights,
)
from gallery_manifest import grade_image  # noqa: E402
from gallery_manifest import grade_image as _grade
from gallery_manifest import (
    hero_index,
    load_gallery_manifest,
    pyramid_blur,
    screen,
    vignette_mask,
)


def test_manifest_lists_the_gallery_plates() -> None:
    manifest = load_gallery_manifest()
    ids = [plate.id for plate in manifest.plates]
    assert ids == [
        "omy_transfer",
        "omy_grasp",
        "omx_wall_maze",
        "omy_clutter",
        "dubins_duel",
        "dubins_atlas",
    ]
    assert (manifest.width, manifest.height) == (1920, 1080)
    assert manifest.supersample >= 1


def test_every_plate_is_sixteen_by_nine_and_writes_png() -> None:
    manifest = load_gallery_manifest()
    for plate in manifest.plates:
        width = plate.effective_width(manifest.width)
        height = plate.effective_height(manifest.height)
        assert_sixteen_by_nine(width, height, context=plate.id)
        assert plate.output.endswith(".png")
        assert plate.caption, f"{plate.id} has no caption"


def test_plate_lookup_by_id() -> None:
    manifest = load_gallery_manifest()
    assert manifest.by_id("omy_transfer").scenario == "omy_pick_place"
    with pytest.raises(KeyError):
        manifest.by_id("nope")


def test_ghost_bodies_are_declared_for_every_plate() -> None:
    manifest = load_gallery_manifest()
    for plate in manifest.plates:
        if plate.ghosts.count:
            assert plate.ghosts.enabled, f"{plate.id} ghosts have no bodies"


@pytest.mark.parametrize(
    ("width", "height"),
    [(1920, 1080), (3840, 2160), (1280, 720)],
)
def test_sixteen_by_nine_accepts_valid_frames(width: int, height: int) -> None:
    assert_sixteen_by_nine(width, height)


@pytest.mark.parametrize(
    ("width", "height"),
    [(1920, 1200), (1000, 1000), (1920, 0)],
)
def test_sixteen_by_nine_rejects_other_ratios(width: int, height: int) -> None:
    with pytest.raises(ValueError):
        assert_sixteen_by_nine(width, height)


def test_hero_index_without_fsm_uses_the_whole_run() -> None:
    assert hero_index(None, n_samples=101, state=None, at=0.0) == 0
    assert hero_index(None, n_samples=101, state=None, at=1.0) == 100
    assert hero_index(None, n_samples=101, state=None, at=0.5) == 50


def test_hero_index_lands_inside_the_requested_phase() -> None:
    states = np.array([1, 1, 3, 3, 3, 5, 5, 9])
    assert hero_index(states, n_samples=8, state=3, at=0.0) == 2
    assert hero_index(states, n_samples=8, state=3, at=1.0) == 4
    assert hero_index(states, n_samples=8, state=5, at=0.5) == 5


def test_hero_index_rejects_missing_phase_and_empty_runs() -> None:
    states = np.array([1, 1, 2])
    with pytest.raises(ValueError):
        hero_index(states, n_samples=3, state=7, at=0.5)
    with pytest.raises(ValueError):
        hero_index(None, n_samples=0, state=None, at=0.5)


def test_ghost_indices_stay_before_the_hero_frame() -> None:
    indices = ghost_indices(hero=100, count=5, span=(0.0, 1.0))
    assert indices == [0, 25, 50, 74, 99]
    assert all(i < 100 for i in indices)


def test_ghost_indices_honour_the_span_window() -> None:
    indices = ghost_indices(hero=200, count=3, span=(0.5, 0.9))
    assert indices == [100, 140, 179]


def test_ghost_indices_are_empty_without_history() -> None:
    assert ghost_indices(hero=0, count=8, span=(0.0, 1.0)) == []
    assert ghost_indices(hero=50, count=0, span=(0.0, 1.0)) == []


def test_ghost_weights_ramp_from_faint_to_strong() -> None:
    weights = ghost_weights(5, weight_min=0.02, weight_max=0.10)
    assert weights.shape == (5,)
    assert weights[0] == pytest.approx(0.02)
    assert weights[-1] == pytest.approx(0.10)
    assert np.all(np.diff(weights) > 0)


def test_screen_blend_never_darkens() -> None:
    base = np.full((4, 4, 3), 0.3)
    layer = np.full((4, 4, 3), 1.0)
    out = screen(base, layer, 0.5)
    assert np.all(out >= base)
    assert np.all(out <= 1.0)


def test_screen_with_black_layer_is_a_no_op() -> None:
    base = np.random.default_rng(0).random((8, 8, 3))
    out = screen(base, np.zeros_like(base), 0.4)
    assert np.allclose(out, base)


def test_box_downscale_averages_blocks() -> None:
    image = np.arange(16, dtype=np.float64).reshape(4, 4, 1)
    out = box_downscale(image, 2)
    assert out.shape == (2, 2, 1)
    assert out[0, 0, 0] == pytest.approx(2.5)


def test_box_downscale_rejects_non_divisible_sizes() -> None:
    with pytest.raises(ValueError):
        box_downscale(np.zeros((5, 4, 3)), 2)


def test_vignette_mask_is_bright_at_the_centre() -> None:
    mask = vignette_mask(90, 160, strength=0.8, start=0.5)
    assert mask.shape == (90, 160)
    assert mask[45, 80] == pytest.approx(1.0)
    assert mask[0, 0] < mask[45, 80]


def test_grade_image_stays_in_range_and_keeps_shape() -> None:
    rng = np.random.default_rng(1)
    image = rng.random((90, 160, 3))
    out = grade_image(image, GradeSpec())
    assert out.shape == image.shape
    assert out.min() >= 0.0 and out.max() <= 1.0


def test_grade_image_brightens_a_highlight() -> None:
    image = np.zeros((90, 160, 3))
    image[40:50, 70:90] = 1.0
    spec = GradeSpec(vignette=0.0, contrast=1.0, pivot=0.0, bloom_gain=2.0)
    out = _grade(image, spec)
    assert out[38, 72].max() > image[38, 72].max()


def test_pyramid_blur_spreads_energy_without_changing_shape() -> None:
    image = np.zeros((90, 160, 3))
    image[44:46, 78:82] = 1.0
    out = pyramid_blur(image, 12.0)
    assert out.shape == image.shape
    assert out[30, 60].max() > 0.0
    assert out.max() < image.max()
