"""Tests for scripts/render_still.py (FR-SIM-14 presentation stills).

Covers AC-STILL-01 .. AC-STILL-06 from docs/presentation_stills.md. The
full 1920x1080 render is a release-time command, not a unit test; these
tests exercise the pure-Python helpers plus a small MuJoCo load.
"""

from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import numpy.typing as npt
import pytest

# render_still.py lives under scripts/, not src/
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS = _REPO_ROOT / "scripts"
sys.path.insert(0, str(_SCRIPTS))

import render_still as rs  # noqa: E402

_SHARED_SCENES = (
    "src/fret/mjcf/omx_pick_place.xml",
    "src/fret/mjcf/omx_desk_clutter.xml",
    "src/fret/mjcf/omx_wall_maze.xml",
    "src/fret/mjcf/omx_tabletop.xml",
    "src/fret/mjcf/dubins_race.xml",
)


def test_shared_scenes_carry_no_presentation_elements() -> None:
    """AC-STILL-01: framing and fill light stay out of the shared scenes."""
    for rel in _SHARED_SCENES:
        text = (_REPO_ROOT / rel).read_text(encoding="utf-8")
        assert "thumb_" not in text, f"{rel} gained a presentation element"


@pytest.mark.parametrize(
    ("scenario", "included"),
    [
        ("omx_pick_place", ".generated/omx_pick_place.xml"),
        ("dubins_race", "dubins_race.xml"),
    ],
)
def test_thumbnail_wrapper_includes_the_shared_cell(
    scenario: str, included: str
) -> None:
    """AC-STILL-01: the wrapper reuses the cell instead of copying it."""
    root = ET.parse(rs.thumbnail_template(scenario)).getroot()
    includes = [node.get("file") for node in root.iter("include")]
    assert includes == [included]
    assert not list(root.iter("geom"))


@pytest.mark.parametrize("scenario", ["omx_pick_place", "dubins_race"])
def test_thumbnail_wrapper_adds_only_thumb_named_lights(
    scenario: str,
) -> None:
    """AC-STILL-01: no unnamed or cell-shadowing light sneaks in."""
    root = ET.parse(rs.thumbnail_template(scenario)).getroot()
    names = [node.get("name") for node in root.iter("light")]
    assert names
    assert all(
        name is not None and name.startswith("thumb_") for name in names
    )


@pytest.mark.parametrize("scenario", ["omx_pick_place", "dubins_race"])
def test_thumbnail_cameras_are_three_distinct_viewpoints(
    scenario: str,
) -> None:
    """AC-STILL-02: three viewpoints, three positions, three directions.

    ``thumb_split`` is excluded: it is the composite's pulled-back copy of
    one of these eyes, not a fourth point of view.
    """
    cameras = [
        cam
        for cam in rs.thumbnail_cameras(rs.thumbnail_template(scenario))
        if cam.name != "thumb_split"
    ]
    assert len(cameras) >= 3
    positions = {tuple(cam.position) for cam in cameras}
    assert len(positions) == len(cameras)
    forwards = [cam.forward for cam in cameras]
    for i, first in enumerate(forwards):
        for second in forwards[i + 1 :]:
            assert float(np.dot(first, second)) < 0.9


def test_thumb_split_reuses_an_eye_with_a_wider_field() -> None:
    """The composite camera changes the field of view, nothing else."""
    cameras = rs.thumbnail_cameras(rs.thumbnail_template("omx_pick_place"))
    by_name = {cam.name: cam for cam in cameras}
    split = by_name["thumb_split"]
    twins = [
        cam
        for cam in cameras
        if cam.name != "thumb_split"
        and np.allclose(cam.position, split.position)
        and np.allclose(cam.forward, split.forward)
    ]
    assert len(twins) == 1
    assert split.fovy > twins[0].fovy


@pytest.mark.parametrize("scenario", ["omx_pick_place", "dubins_race"])
def test_thumbnail_cameras_look_down_from_above_the_work_surface(
    scenario: str,
) -> None:
    """AC-STILL-02: above the floor plane, tilted down, not square across."""
    for cam in rs.thumbnail_cameras(rs.thumbnail_template(scenario)):
        assert cam.position[2] > 0.15
        assert cam.forward[2] < -0.05


def test_select_still_sample_takes_the_last_frame_of_the_state() -> None:
    """AC-STILL-03: the instant is searched, never hand-entered."""
    samples = [
        _FakeSample("APPROACH_PICK", 0),
        _FakeSample("DESCEND_PICK", 1),
        _FakeSample("DESCEND_PICK", 2),
        _FakeSample("GRASP", 3),
    ]
    index, sample = rs.select_still_sample(samples, "DESCEND_PICK")
    assert index == 2
    assert sample.tag == 2


def test_select_still_sample_rejects_a_state_the_run_never_reached() -> None:
    """AC-STILL-03: fail loudly instead of falling back to a rest pose."""
    samples = [_FakeSample("IDLE", 0)]
    with pytest.raises(ValueError, match="DESCEND_PICK"):
        rs.select_still_sample(samples, "DESCEND_PICK")


def test_frame_tone_stats_measures_crush_and_blowout() -> None:
    """AC-STILL-05: tone is measured, not asserted."""
    frame = np.full((10, 10, 3), 128, dtype=np.uint8)
    frame[0, :, :] = 0
    frame[1, :, :] = 255
    stats = rs.frame_tone_stats(frame)
    assert stats["crushed_fraction"] == pytest.approx(0.10)
    assert stats["blown_fraction"] == pytest.approx(0.10)
    assert 120.0 < stats["mean_luma"] < 140.0


def test_frame_tone_stats_rejects_a_near_black_frame() -> None:
    """AC-STILL-05: a hole punched in a light page fails the gate."""
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    stats = rs.frame_tone_stats(frame)
    assert not rs.tone_is_acceptable(stats)


def test_frame_tone_stats_accepts_a_mid_tone_frame() -> None:
    """AC-STILL-05: an evenly lit frame passes the same gate."""
    frame = np.linspace(40, 210, 64, dtype=np.uint8)
    stats = rs.frame_tone_stats(np.repeat(frame.reshape(8, 8, 1), 3, axis=2))
    assert rs.tone_is_acceptable(stats)


def test_subject_extent_is_read_from_a_segmentation_render() -> None:
    """AC-STILL-06: containment comes from geom ids, not from eyeballing."""
    seg = np.full((100, 100), -1, dtype=np.int32)
    seg[40:60, 30:70] = 7
    extent = rs.subject_extent(seg, background_ids={-1})
    assert extent["min_x_fraction"] == pytest.approx(0.30)
    assert extent["max_x_fraction"] == pytest.approx(0.70)
    assert extent["min_y_fraction"] == pytest.approx(0.40)
    assert extent["max_y_fraction"] == pytest.approx(0.60)
    assert rs.subject_is_inside_central(extent, fraction=0.85)


def test_subject_extent_flags_a_subject_touching_the_edge() -> None:
    """AC-STILL-06: a subject running off frame is reported, not accepted."""
    seg = np.full((100, 100), -1, dtype=np.int32)
    seg[0:50, 0:50] = 3
    extent = rs.subject_extent(seg, background_ids={-1})
    assert not rs.subject_is_inside_central(extent, fraction=0.85)


def test_subject_extent_profile_narrows_where_the_subject_narrows() -> None:
    """AC-STILL-06: the profile tracks width band by band, not one box."""
    seg = np.full((96, 100), -1, dtype=np.int32)
    seg[0:48, 40:60] = 5
    seg[48:96, 10:90] = 5
    profile = rs.subject_extent_profile(seg, background_ids={-1}, bands=2)
    assert len(profile) == 2
    top, bottom = profile
    assert top[2] == pytest.approx(0.40)
    assert top[3] == pytest.approx(0.60)
    assert bottom[2] == pytest.approx(0.10)
    assert bottom[3] == pytest.approx(0.90)


def test_subject_extent_profile_skips_empty_bands() -> None:
    """AC-STILL-06: bands with no subject are dropped, not reported wide."""
    seg = np.full((96, 100), -1, dtype=np.int32)
    seg[48:96, 10:90] = 5
    profile = rs.subject_extent_profile(seg, background_ids={-1}, bands=2)
    assert len(profile) == 1
    assert profile[0][0] == pytest.approx(0.5)


def test_subject_extent_profile_rejects_an_empty_segmentation() -> None:
    """AC-STILL-06: an empty frame is an error, not an empty profile."""
    seg = np.full((16, 16), -1, dtype=np.int32)
    with pytest.raises(ValueError, match="no subject"):
        rs.subject_extent_profile(seg, background_ids={-1})


def test_subject_extent_rejects_an_empty_segmentation() -> None:
    """AC-STILL-06: an empty frame is an error, not a passing measurement."""
    seg = np.full((16, 16), -1, dtype=np.int32)
    with pytest.raises(ValueError, match="no subject"):
        rs.subject_extent(seg, background_ids={-1})


def test_write_still_png_has_no_alpha_and_carries_srgb(tmp_path) -> None:
    """AC-STILL-04: exact size, 8-bit RGB, sRGB tagged, no alpha."""
    Image = pytest.importorskip("PIL.Image")
    frame = np.random.default_rng(0).integers(
        0, 256, size=(12, 20, 3), dtype=np.uint8
    )
    out = tmp_path / "still.png"
    rs.write_still_png(out, frame)
    with Image.open(out) as image:
        assert image.format == "PNG"
        assert image.mode == "RGB"
        assert image.size == (20, 12)
        assert image.info.get("srgb") == 0


def test_write_still_png_is_byte_identical_across_calls(tmp_path) -> None:
    """AC-STILL-07: the encoder writes no timestamp, so runs match."""
    frame = np.random.default_rng(1).integers(
        0, 256, size=(9, 16, 3), dtype=np.uint8
    )
    first, second = tmp_path / "a.png", tmp_path / "b.png"
    rs.write_still_png(first, frame)
    rs.write_still_png(second, frame)
    assert first.read_bytes() == second.read_bytes()


def test_write_still_png_rejects_an_alpha_frame(tmp_path) -> None:
    """AC-STILL-04: a four-channel frame is refused rather than flattened."""
    frame = np.zeros((4, 4, 4), dtype=np.uint8)
    with pytest.raises(ValueError, match="RGB"):
        rs.write_still_png(tmp_path / "bad.png", frame)


def test_cli_requires_a_camera_selection() -> None:
    """The CLI fails fast instead of guessing a viewpoint."""
    result = subprocess.run(
        [
            sys.executable,
            str(_SCRIPTS / "render_still.py"),
            "--scenario",
            "omx_pick_place",
            "--output-dir",
            "/tmp/fret_still_cli_check",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "--camera" in result.stderr or "--all-cameras" in result.stderr


def test_thumbnail_model_keeps_the_shared_cell_intact() -> None:
    """AC-STILL-01: the wrapper adds lights and cameras and nothing else."""
    mujoco = pytest.importorskip("mujoco")
    sys.path.insert(0, str(_REPO_ROOT / "src"))
    from fret.mjcf.omx import ensure_omx_pick_place_mjcf

    cell = mujoco.MjModel.from_xml_path(str(ensure_omx_pick_place_mjcf()))
    wrapper = mujoco.MjModel.from_xml_path(
        str(rs.resolve_thumbnail_mjcf("omx_pick_place"))
    )
    assert wrapper.ngeom == cell.ngeom
    assert wrapper.nbody == cell.nbody
    assert wrapper.njnt == cell.njnt
    cell_lights = _names(mujoco, cell, mujoco.mjtObj.mjOBJ_LIGHT)
    wrapper_lights = _names(mujoco, wrapper, mujoco.mjtObj.mjOBJ_LIGHT)
    assert cell_lights <= wrapper_lights
    assert all(
        name.startswith("thumb_") for name in wrapper_lights - cell_lights
    )
    cell_cams = _names(mujoco, cell, mujoco.mjtObj.mjOBJ_CAMERA)
    wrapper_cams = _names(mujoco, wrapper, mujoco.mjtObj.mjOBJ_CAMERA)
    assert cell_cams <= wrapper_cams
    assert all(name.startswith("thumb_") for name in wrapper_cams - cell_cams)


class _FakeSample:
    """Stand-in for PickPlaceSample carrying only the searched field."""

    def __init__(self, state_name: str, tag: int) -> None:
        self.state = _FakeState(state_name)
        self.tag = tag


class _FakeState:
    def __init__(self, name: str) -> None:
        self.name = name


def _names(mujoco: object, model: object, obj_type: object) -> set[str]:
    counts = {
        mujoco.mjtObj.mjOBJ_LIGHT: model.nlight,  # type: ignore[attr-defined]
        mujoco.mjtObj.mjOBJ_CAMERA: model.ncam,  # type: ignore[attr-defined]
    }
    return {
        name
        for i in range(counts[obj_type])
        if (name := _id2name(mujoco, model, obj_type, i)) is not None
    }


def _id2name(mujoco: object, model: object, obj_type: object, i: int):
    """Name lookup wrapper that keeps the call site inside 79 columns."""
    return mujoco.mj_id2name(model, obj_type, i)  # type: ignore[attr-defined]


def _pose(x: float, y: float) -> npt.NDArray[np.float64]:
    return np.array([x, y, 0.0], dtype=np.float64)


def _straight_down_camera() -> "rs.ThumbnailCamera":
    """A camera 4 m up at the origin, looking straight down the -z axis."""
    return rs.ThumbnailCamera(
        name="probe",
        position=np.array([0.0, 0.0, 4.0]),
        forward=np.array([0.0, 0.0, -1.0]),
        right=np.array([1.0, 0.0, 0.0]),
        up=np.array([0.0, 1.0, 0.0]),
        fovy=90.0,
    )


def test_project_to_frame_puts_the_look_at_point_in_the_center() -> None:
    """AC-STILL-03: the projection the instant search relies on is correct."""
    camera = _straight_down_camera()
    x_fraction, y_fraction, depth = rs.project_to_frame(
        camera, np.array([0.0, 0.0, 0.0]), aspect=16 / 9
    )
    assert x_fraction == pytest.approx(0.5)
    assert y_fraction == pytest.approx(0.5)
    assert depth == pytest.approx(4.0)


def test_project_to_frame_reports_a_point_behind_the_camera() -> None:
    """AC-STILL-03: a point behind the camera is rejected, not wrapped."""
    camera = _straight_down_camera()
    _, _, depth = rs.project_to_frame(
        camera, np.array([0.0, 0.0, 9.0]), aspect=16 / 9
    )
    assert depth < 0.0


def test_project_to_frame_moves_right_along_the_camera_right_axis() -> None:
    """AC-STILL-03: horizontal fraction follows the camera's right axis."""
    camera = _straight_down_camera()
    x_fraction, y_fraction, _ = rs.project_to_frame(
        camera, np.array([1.0, 0.0, 0.0]), aspect=16 / 9
    )
    assert x_fraction > 0.5
    assert y_fraction == pytest.approx(0.5)


def test_ranking_prefers_the_instant_that_centers_both_agvs() -> None:
    """AC-STILL-03: the race instant is ranked, never hand-picked."""
    camera = _straight_down_camera()
    rrt = np.array([_pose(0.4, 0.0), _pose(8.0, 0.0), _pose(0.0, 0.0)])
    sst = np.array([_pose(-0.4, 0.0), _pose(-8.0, 0.0), _pose(0.0, 0.0)])
    ranked = rs.rank_dubins_still_samples(rrt, sst, camera, aspect=16 / 9)
    assert ranked[0][0] == 2
    assert [index for index, _ in ranked] == [2, 0]


def test_ranking_drops_instants_that_push_an_agv_out_of_frame() -> None:
    """AC-STILL-03: an AGV off the edge disqualifies its instant."""
    camera = _straight_down_camera()
    rrt = np.array([_pose(0.0, 0.0)])
    sst = np.array([_pose(40.0, 0.0)])
    ranked = rs.rank_dubins_still_samples(rrt, sst, camera, aspect=16 / 9)
    assert ranked == []


def test_select_dubins_still_sample_rejects_a_camera_that_never_frames() -> (
    None
):
    """AC-STILL-03: fail loudly instead of rendering one AGV off screen."""
    camera = _straight_down_camera()
    rrt = np.array([_pose(0.0, 0.0)])
    sst = np.array([_pose(40.0, 0.0)])
    with pytest.raises(ValueError, match="never frames both AGVs"):
        rs.select_dubins_still_sample(rrt, sst, camera, aspect=16 / 9)


def test_dubins_subject_is_the_two_racing_agvs() -> None:
    """AC-STILL-06: the dummy foil and the racks are not the subject."""
    assert rs._SUBJECT_ROOT_BODIES["dubins_race"] == ("car_rrt", "car_sst")


def test_dubins_backdrop_excludes_everything_outside_the_agvs() -> None:
    """AC-STILL-06: subject roots drive the mask, not a name prefix."""
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(
        str(rs.resolve_thumbnail_mjcf("dubins_race"))
    )
    backdrop = rs.backdrop_geom_ids(
        mujoco, model, subject_roots=("car_rrt", "car_sst")
    )
    groups = rs.subject_geom_ids_by_root(mujoco, model, ("car_rrt", "car_sst"))
    assert groups["car_rrt"] and groups["car_sst"]
    for geoms in groups.values():
        assert not (geoms & backdrop)
    dummy = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "dummy_collision"
    )
    assert dummy in backdrop


def test_backdrop_rejects_an_unknown_subject_root() -> None:
    """AC-STILL-06: a typo in a root fails instead of measuring all."""
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(
        str(rs.resolve_thumbnail_mjcf("dubins_race"))
    )
    with pytest.raises(ValueError, match="not in the model"):
        rs.backdrop_geom_ids(mujoco, model, subject_roots=("car_missing",))
