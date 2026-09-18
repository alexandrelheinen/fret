#!/usr/bin/env python3
"""Headless MuJoCo still renderer for FRET presentation frames (FR-SIM-14).

The frame this writes is what MuJoCo produced. Nothing is drawn on top of
it: no text, no boxes, no legend, no watermark, and no compositing. The
scenes carry no chrome of their own either, so a still here is the raw
render. Specification: docs/presentation_stills.md.

Framing and fill light live in presentation-only wrappers
(``src/fret/mjcf/<scenario>_thumbnail.xml``) that include the shared scene
rather than edit it, which keeps the vision pipeline's gate cameras and the
release showcase matrix on exactly the pixels they had before.

Simulation, pose application, and MuJoCo import are reused from
``scripts/render_mujoco.py``; this module adds only the still path.

Example::

    MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python3 scripts/render_still.py \\
        --scenario omx_pick_place --all-cameras \\
        --output-dir artifacts/stills --width 1920 --height 1080

Dependencies (not required for core FRET algorithms)::

    pip install mujoco imageio pillow
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

sys.path.insert(0, str(Path(__file__).resolve().parent))

from render_mujoco import (  # noqa: E402
    _apply_dubins_poses,
    _apply_omx_pick_place_sample,
    _ensure_fret_importable,
    _require_mujoco,
)

# Scenarios that own a presentation wrapper, and the FSM state whose last
# recorded sample reads as "committed to the approach".
_DEFAULT_STILL_STATE: dict[str, str] = {
    "omx_pick_place": "DESCEND_PICK",
}

# TurtleBot3 Burger body center used when projecting an AGV into the frame.
_DUBINS_BODY_HEIGHT_M: float = 0.12
# Geometry alone cannot tell that a shelf stands between the camera and an
# AGV, so the best-framed instants are checked against a segmentation render
# until one shows both AGVs. The cap bounds the work on a race where one
# agent is behind a rack for most of the well-framed instants.
_DUBINS_VISIBILITY_CANDIDATES: int = 400
_DUBINS_MIN_VISIBLE_FRACTION: float = 1e-3
# Both agents are the same TurtleBot3, so pixel count times the square of
# the viewing distance is the same for both when neither is occluded. A gap
# in that product means a pallet or a rack upright is covering one of them.
_DUBINS_MIN_VISIBLE_BALANCE: float = 0.70

# AC-STILL-05 tone window. A web page is light, so a near-black frame reads
# as a hole punched in it; a blown frame loses the printed texture.
_MIN_MEAN_LUMA: float = 60.0
_MAX_MEAN_LUMA: float = 200.0
_CRUSH_LEVEL: int = 8
_BLOWOUT_LEVEL: int = 247
_MAX_CLIPPED_FRACTION: float = 0.02

# AC-STILL-06 framing window, and the props that count as backdrop rather
# than subject. The v1.4 vision gate is a 0.72 m portal the arm stands
# inside, so it crosses the frame in any shot close enough to read; it is
# staging, not the subject the framing gate measures.
_CENTRAL_FRACTION: float = 0.85
# Bands used by the per-row subject profile written to the metrics JSON.
_EXTENT_PROFILE_BANDS: int = 48
_BACKDROP_PREFIXES: dict[str, tuple[str, ...]] = {
    "omx_pick_place": ("gate_",),
}
# Where a scene's subject is a few named bodies rather than everything that
# is not scenery, name the roots instead. The Dubins still is about the two
# racing AGVs; the warehouse structures they weave through are staging, and
# the grey dummy foil is neither planner's result.
_SUBJECT_ROOT_BODIES: dict[str, tuple[str, ...]] = {
    "dubins_race": ("car_rrt", "car_sst"),
}

# PNG sRGB chunk payload: rendering intent 0, perceptual.
_SRGB_RENDERING_INTENT: bytes = b"\x00"

# Rec. 709 luma weights, matching how a browser reads an sRGB PNG.
_LUMA_WEIGHTS: tuple[float, float, float] = (0.2126, 0.7152, 0.0722)


class StillRenderError(RuntimeError):
    """A still could not be produced, or failed its measured gate."""


@dataclass(frozen=True)
class ThumbnailCamera:
    """One presentation camera read from a wrapper MJCF."""

    name: str
    position: npt.NDArray[np.float64]
    forward: npt.NDArray[np.float64]
    right: npt.NDArray[np.float64]
    up: npt.NDArray[np.float64]
    fovy: float


@dataclass
class StillResult:
    """One written still plus the measurements that justify it."""

    camera: str
    path: Path
    width: int
    height: int
    sample_index: int
    sample_count: int
    state: str
    sim_time_s: float
    tone: dict[str, float] = field(default_factory=dict)
    subject_tone: dict[str, float] = field(default_factory=dict)
    extent: dict[str, float] = field(default_factory=dict)
    extent_profile: list[list[float]] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable view of this result."""
        return {
            "camera": self.camera,
            "path": str(self.path),
            "width": self.width,
            "height": self.height,
            "sample_index": self.sample_index,
            "sample_count": self.sample_count,
            "state": self.state,
            "sim_time_s": self.sim_time_s,
            "tone": self.tone,
            "subject_tone": self.subject_tone,
            "extent": self.extent,
            "extent_profile": self.extent_profile,
        }


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def thumbnail_template(scenario: str) -> Path:
    """Return the committed presentation wrapper MJCF for ``scenario``.

    Args:
        scenario: Scenario stem (e.g. ``omx_pick_place``).

    Returns:
        Path to ``src/fret/mjcf/<scenario>_thumbnail.xml``.

    Raises:
        FileNotFoundError: If no wrapper exists for the scenario.
    """
    path = _project_root() / "src/fret/mjcf" / f"{scenario}_thumbnail.xml"
    if not path.is_file():
        raise FileNotFoundError(
            f"No presentation wrapper for scenario {scenario!r}: {path}"
        )
    return path


def resolve_thumbnail_mjcf(scenario: str) -> Path:
    """Return a loadable wrapper path, building its included cell first.

    Args:
        scenario: Scenario stem (e.g. ``omx_pick_place``).

    Returns:
        Path to the wrapper MJCF, ready for ``MjModel.from_xml_path``.

    Raises:
        ValueError: If the scenario has no known cell builder.
    """
    template = thumbnail_template(scenario)
    _ensure_fret_importable()
    if scenario == "omx_pick_place":
        from fret.mjcf.omx import ensure_omx_pick_place_mjcf

        ensure_omx_pick_place_mjcf()
        return template
    if scenario == "dubins_race":
        # The race scene is committed as-is; nothing to build first.
        return template
    raise ValueError(f"Unknown presentation scenario: {scenario!r}")


def thumbnail_cameras(template: Path) -> list[ThumbnailCamera]:
    """Read the presentation cameras declared in a wrapper MJCF.

    Args:
        template: Path to a ``*_thumbnail.xml`` wrapper.

    Returns:
        Cameras in declaration order.

    Raises:
        ValueError: If a camera lacks ``pos`` or ``xyaxes``.
    """
    root = ET.parse(template).getroot()
    cameras: list[ThumbnailCamera] = []
    for node in root.iter("camera"):
        name = node.get("name")
        pos_text = node.get("pos")
        axes_text = node.get("xyaxes")
        if name is None:
            continue
        if pos_text is None or axes_text is None:
            raise ValueError(
                f"Presentation camera {name!r} needs pos and xyaxes"
            )
        position = np.fromstring(pos_text, sep=" ", dtype=np.float64)
        axes = np.fromstring(axes_text, sep=" ", dtype=np.float64)
        if position.size != 3 or axes.size != 6:
            raise ValueError(f"Malformed camera {name!r} in {template}")
        right = axes[:3] / float(np.linalg.norm(axes[:3]))
        up = axes[3:] / float(np.linalg.norm(axes[3:]))
        forward = -np.cross(right, up)
        forward = forward / float(np.linalg.norm(forward))
        cameras.append(
            ThumbnailCamera(
                name=name,
                position=position,
                forward=forward,
                right=right,
                up=up,
                fovy=float(node.get("fovy", "45")),
            )
        )
    return cameras


def select_still_sample(
    samples: list[Any], state_name: str
) -> tuple[int, Any]:
    """Search a recorded run for the last sample in an FSM state.

    The instant is found in the run, never typed in by hand: the last frame
    of ``DESCEND_PICK`` is the arm committed to the approach, gripper open,
    one step short of the grasp.

    Args:
        samples: Recorded samples carrying a ``state`` with a ``name``.
        state_name: FSM state to search for (e.g. ``DESCEND_PICK``).

    Returns:
        The index and the sample itself.

    Raises:
        ValueError: If the run never entered that state.
    """
    matches = [
        index
        for index, sample in enumerate(samples)
        if sample.state.name == state_name
    ]
    if not matches:
        reached = sorted({sample.state.name for sample in samples})
        raise ValueError(f"Run never entered {state_name}; reached {reached}")
    index = matches[-1]
    return index, samples[index]


def frame_tone_stats(frame: npt.NDArray[np.uint8]) -> dict[str, float]:
    """Measure the tone distribution of a rendered frame.

    Args:
        frame: ``(H, W, 3)`` uint8 RGB frame.

    Returns:
        Mean, percentile, and clipping statistics of the Rec. 709 luma.
    """
    luma = (
        frame[..., 0].astype(np.float64) * _LUMA_WEIGHTS[0]
        + frame[..., 1].astype(np.float64) * _LUMA_WEIGHTS[1]
        + frame[..., 2].astype(np.float64) * _LUMA_WEIGHTS[2]
    )
    percentiles = np.percentile(luma, [1, 5, 50, 95, 99])
    return {
        "mean_luma": float(luma.mean()),
        "p01_luma": float(percentiles[0]),
        "p05_luma": float(percentiles[1]),
        "median_luma": float(percentiles[2]),
        "p95_luma": float(percentiles[3]),
        "p99_luma": float(percentiles[4]),
        "crushed_fraction": float((luma <= _CRUSH_LEVEL).mean()),
        "blown_fraction": float((luma >= _BLOWOUT_LEVEL).mean()),
    }


def tone_is_acceptable(stats: dict[str, float]) -> bool:
    """Return whether measured tone sits in the AC-STILL-05 window."""
    return (
        _MIN_MEAN_LUMA <= stats["mean_luma"] <= _MAX_MEAN_LUMA
        and stats["crushed_fraction"] <= _MAX_CLIPPED_FRACTION
        and stats["blown_fraction"] <= _MAX_CLIPPED_FRACTION
    )


def subject_extent(
    segmentation: npt.NDArray[np.int32],
    *,
    background_ids: set[int],
) -> dict[str, float]:
    """Measure where the subject sits in the frame, as fractions of it.

    Args:
        segmentation: ``(H, W)`` array of geom ids, background negative.
        background_ids: Ids that are backdrop rather than subject.

    Returns:
        Bounding-box edges and covered area, all as frame fractions.

    Raises:
        ValueError: If no subject pixel survives the background mask.
    """
    mask = ~np.isin(segmentation, list(background_ids))
    if not mask.any():
        raise ValueError("Segmentation render contains no subject pixels")
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    height, width = segmentation.shape
    return {
        "min_x_fraction": float(cols[0]) / float(width),
        "max_x_fraction": float(cols[-1] + 1) / float(width),
        "min_y_fraction": float(rows[0]) / float(height),
        "max_y_fraction": float(rows[-1] + 1) / float(height),
        "covered_fraction": float(mask.mean()),
    }


def subject_extent_profile(
    segmentation: npt.NDArray[np.int32],
    *,
    background_ids: set[int],
    bands: int = _EXTENT_PROFILE_BANDS,
) -> list[list[float]]:
    """Measure the subject's horizontal extent band by band down the frame.

    A single bounding box is a blunt instrument for anything that is wide at
    one end and narrow at the other, which is every one of these subjects.
    The band profile is what lets a downstream consumer, such as the
    thumbnail composer, fit a subject against a sloped edge without assuming
    it is as wide at the top as it is at the bottom.

    Args:
        segmentation: ``(H, W)`` array of geom ids, background negative.
        background_ids: Ids that are backdrop rather than subject.
        bands: Number of equal-height bands to measure.

    Returns:
        One ``[y_start, y_end, min_x, max_x]`` row per occupied band, all as
        frame fractions, ordered top to bottom.

    Raises:
        ValueError: If no subject pixel survives the background mask.
    """
    mask = ~np.isin(segmentation, list(background_ids))
    if not mask.any():
        raise ValueError("Segmentation render contains no subject pixels")
    height, width = segmentation.shape
    edges = np.linspace(0, height, bands + 1).astype(int)
    profile: list[list[float]] = []
    for start, end in zip(edges[:-1], edges[1:]):
        if end <= start:
            continue
        band = mask[start:end]
        if not band.any():
            continue
        columns = np.flatnonzero(band.any(axis=0))
        profile.append(
            [
                float(start) / float(height),
                float(end) / float(height),
                float(columns[0]) / float(width),
                float(columns[-1] + 1) / float(width),
            ]
        )
    return profile


def subject_is_inside_central(
    extent: dict[str, float], *, fraction: float = _CENTRAL_FRACTION
) -> bool:
    """Return whether the measured extent fits the central ``fraction``."""
    margin = (1.0 - fraction) / 2.0
    return (
        extent["min_x_fraction"] >= margin
        and extent["max_x_fraction"] <= 1.0 - margin
        and extent["min_y_fraction"] >= margin
        and extent["max_y_fraction"] <= 1.0 - margin
    )


def write_still_png(path: Path, frame: npt.NDArray[np.uint8]) -> None:
    """Write one frame as an 8-bit sRGB PNG with no alpha channel.

    The encoder receives the renderer's pixels unchanged; the only thing
    added to the file is the sRGB profile that names how those pixels
    should be read.

    Args:
        path: Destination PNG path.
        frame: ``(H, W, 3)`` uint8 RGB frame.

    Raises:
        ValueError: If the frame is not 8-bit three-channel RGB.
    """
    from PIL import Image, PngImagePlugin

    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(
            f"Still frames must be RGB with 3 channels, got {frame.shape}"
        )
    if frame.dtype != np.uint8:
        raise ValueError(f"Still frames must be uint8, got {frame.dtype}")
    # The PNG sRGB chunk names the color space in one byte of rendering
    # intent. An embedded ICC profile would say the same thing but carries a
    # creation timestamp, which would make two runs of the same command
    # differ byte for byte (AC-STILL-07).
    info = PngImagePlugin.PngInfo()
    info.add(b"sRGB", _SRGB_RENDERING_INTENT)
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(frame, mode="RGB").save(
        path, format="PNG", optimize=True, pnginfo=info
    )


def _geom_alpha(model: Any, gid: int) -> float:
    """Return the alpha a geom actually renders with."""
    matid = int(model.geom_matid[gid])
    if matid >= 0:
        return float(model.mat_rgba[matid][3])
    return float(model.geom_rgba[gid][3])


def _root_body_id(model: Any, body_id: int) -> int:
    """Walk up the body tree to the child of the world body."""
    current = int(body_id)
    while current > 0 and int(model.body_parentid[current]) != 0:
        current = int(model.body_parentid[current])
    return current


def backdrop_geom_ids(
    mujoco: Any,
    model: Any,
    *,
    prefixes: tuple[str, ...] = (),
    subject_roots: tuple[str, ...] = (),
) -> set[int]:
    """Return geom ids that count as backdrop rather than subject.

    Backdrop is the ground plane, anything the renderer draws fully
    transparent (the SC-v13b catcher funnel and the CV ghost), and any geom
    whose name starts with one of ``prefixes``. Everything else is subject,
    so AC-STILL-06 measures the arm, the ball, and the place bin.

    Args:
        mujoco: Imported ``mujoco`` module.
        model: Loaded ``MjModel``.
        prefixes: Geom name prefixes to treat as backdrop.
        subject_roots: Root body names whose geoms are the subject; when
            given, every geom outside those subtrees is backdrop.

    Returns:
        Geom ids, including ``-1`` for the empty background.

    Raises:
        ValueError: If a named subject root is not in the model.
    """
    ids: set[int] = {-1}
    root_ids: set[int] = set()
    for root in subject_roots:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root)
        if bid < 0:
            raise ValueError(f"Subject root body {root!r} is not in the model")
        root_ids.add(int(bid))
    for gid in range(int(model.ngeom)):
        if root_ids:
            body = _root_body_id(model, int(model.geom_bodyid[gid]))
            if body not in root_ids:
                ids.add(gid)
                continue
        if int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_PLANE):
            ids.add(gid)
            continue
        if _geom_alpha(model, gid) <= 0.0:
            ids.add(gid)
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if name is not None and name.startswith(prefixes):
            ids.add(gid)
    return ids


def _omx_pick_place_samples(
    duration_s: float, record_every_steps: int
) -> tuple[list[Any], float]:
    """Run the SC-v13b cycle and return its samples plus the step period."""
    _ensure_fret_importable()
    from fret.control.pick_place_fsm import PickPlaceState
    from fret.control.pick_place_sim import simulate_pick_place

    state, samples = simulate_pick_place(
        duration_s=duration_s,
        joint_tol_rad=0.22,
        record_every_steps=record_every_steps,
    )
    if state not in {PickPlaceState.DONE}:
        raise StillRenderError(
            f"Pick-place run ended in {state.name}, expected DONE"
        )
    if len(samples) < 2:
        raise StillRenderError("Pick-place run recorded too few samples")
    return list(samples), 0.002 * record_every_steps


def _dubins_race_samples(
    fps: int,
) -> tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    float,
]:
    """Run the SC-v11 race under the seeded showcase planner RNG.

    Args:
        fps: Pose sampling rate; the still is picked from these samples.

    Returns:
        RRT* poses, SST poses, dummy poses, and the simulated race time.

    Raises:
        StillRenderError: If the race produced too few poses to search.
    """
    from render_mujoco import simulate_dubins_race_poses

    rrt, sst, dummy, sim_time_s = simulate_dubins_race_poses(
        "dubins_race",
        duration_s=None,
        fps=fps,
        physics_mode=True,
    )
    if len(rrt) < 2:
        raise StillRenderError("Dubins race recorded too few poses")
    return rrt, sst, dummy, float(sim_time_s)


def project_to_frame(
    camera: ThumbnailCamera,
    point: npt.NDArray[np.float64],
    *,
    aspect: float,
) -> tuple[float, float, float]:
    """Project a world point into frame fractions for a fixed camera.

    Args:
        camera: Presentation camera read from a wrapper MJCF.
        point: World-space point.
        aspect: Frame width divided by frame height.

    Returns:
        Horizontal fraction, vertical fraction (0 at the top), and the
        distance along the viewing direction. A point behind the camera
        comes back with a non-positive distance.
    """
    offset = np.asarray(point, dtype=np.float64) - camera.position
    depth = float(np.dot(offset, camera.forward))
    if depth <= 0.0:
        return (float("nan"), float("nan"), depth)
    half_height = np.tan(np.radians(camera.fovy) / 2.0) * depth
    half_width = half_height * aspect
    x_fraction = 0.5 + float(np.dot(offset, camera.right)) / (2.0 * half_width)
    y_fraction = 0.5 - float(np.dot(offset, camera.up)) / (2.0 * half_height)
    return (x_fraction, y_fraction, depth)


def rank_dubins_still_samples(
    rrt_poses: npt.NDArray[np.float64],
    sst_poses: npt.NDArray[np.float64],
    camera: ThumbnailCamera,
    *,
    aspect: float,
    fraction: float = _CENTRAL_FRACTION,
    body_height_m: float = _DUBINS_BODY_HEIGHT_M,
) -> list[tuple[int, float]]:
    """Rank every race instant by how comfortably it frames both AGVs.

    The score of an instant is the distance, in frame fractions, from the
    worse-placed AGV to the nearest edge of the central window. Instants
    that put either AGV outside that window, or behind the camera, are
    dropped rather than ranked low.

    Args:
        rrt_poses: ``(N, 3)`` SE(2) poses of the RRT* agent.
        sst_poses: ``(N, 3)`` SE(2) poses of the SST agent.
        camera: Camera the instants are scored against.
        aspect: Frame width divided by frame height.
        fraction: Central window both AGVs must fall inside.
        body_height_m: Height used as the AGV's projected center.

    Returns:
        ``(index, score)`` pairs, best first.

    Raises:
        ValueError: If the two pose histories differ in length.
    """
    if len(rrt_poses) != len(sst_poses):
        raise ValueError("RRT* and SST pose histories differ in length")
    margin = (1.0 - fraction) / 2.0
    ranked: list[tuple[int, float]] = []
    for index in range(len(rrt_poses)):
        score = np.inf
        for pose in (rrt_poses[index], sst_poses[index]):
            point = np.array(
                [float(pose[0]), float(pose[1]), body_height_m],
                dtype=np.float64,
            )
            x_fraction, y_fraction, depth = project_to_frame(
                camera, point, aspect=aspect
            )
            if depth <= 0.0 or not np.isfinite(x_fraction):
                score = -np.inf
                break
            score = min(
                score,
                x_fraction - margin,
                1.0 - margin - x_fraction,
                y_fraction - margin,
                1.0 - margin - y_fraction,
            )
        if score > 0.0:
            ranked.append((index, float(score)))
    ranked.sort(key=lambda pair: (-pair[1], pair[0]))
    return ranked


def select_dubins_still_sample(
    rrt_poses: npt.NDArray[np.float64],
    sst_poses: npt.NDArray[np.float64],
    camera: ThumbnailCamera,
    *,
    aspect: float,
    fraction: float = _CENTRAL_FRACTION,
    body_height_m: float = _DUBINS_BODY_HEIGHT_M,
) -> tuple[int, float]:
    """Search the race for the instant that frames both AGVs best.

    Every recorded instant is scored by how far the worse-placed AGV sits
    from the nearest frame edge, and the best-scoring instant wins. The
    camera stays where the wrapper MJCF puts it, so the search moves the
    moment, not the viewpoint, and it never hand-places an agent.

    Args:
        rrt_poses: ``(N, 3)`` SE(2) poses of the RRT* agent.
        sst_poses: ``(N, 3)`` SE(2) poses of the SST agent.
        camera: Camera the instant is searched against.
        aspect: Frame width divided by frame height.
        fraction: Central window both AGVs must fall inside.
        body_height_m: Height used as the AGV's projected center.

    Returns:
        The winning index and its margin, in frame fractions.

    Raises:
        ValueError: If no instant puts both AGVs inside the window.
    """
    ranked = rank_dubins_still_samples(
        rrt_poses,
        sst_poses,
        camera,
        aspect=aspect,
        fraction=fraction,
        body_height_m=body_height_m,
    )
    if not ranked:
        raise ValueError(
            f"Camera {camera.name!r} never frames both AGVs inside the "
            f"central {fraction:.0%} of the race"
        )
    return ranked[0]


def _measure_still(
    mujoco: Any,
    renderer: Any,
    data: Any,
    *,
    camera: str,
    background: set[int],
    output_dir: Path,
    scenario: str,
    sample_index: int,
    sample_count: int,
    state: str,
    sim_time_s: float,
) -> StillResult:
    """Render one camera, write its PNG, and measure what was written."""
    renderer.disable_segmentation_rendering()
    renderer.update_scene(data, camera=camera)
    frame = renderer.render()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera)
    segmentation = renderer.render()[..., 0].astype(np.int32)
    subject_mask = ~np.isin(segmentation, list(background))

    path = output_dir / f"{scenario}_{camera}.png"
    write_still_png(path, frame)
    return StillResult(
        camera=camera,
        path=path,
        width=int(frame.shape[1]),
        height=int(frame.shape[0]),
        sample_index=sample_index,
        sample_count=sample_count,
        state=state,
        sim_time_s=sim_time_s,
        tone=frame_tone_stats(frame),
        subject_tone=frame_tone_stats(frame[subject_mask].reshape(-1, 1, 3)),
        extent=subject_extent(segmentation, background_ids=background),
        extent_profile=subject_extent_profile(
            segmentation, background_ids=background
        ),
    )


def subject_geom_ids_by_root(
    mujoco: Any, model: Any, roots: tuple[str, ...]
) -> dict[str, set[int]]:
    """Group geom ids by the root body they hang from.

    Args:
        mujoco: Imported ``mujoco`` module.
        model: Loaded ``MjModel``.
        roots: Root body names to group by.

    Returns:
        One geom id set per root body name.

    Raises:
        ValueError: If a named root body is not in the model.
    """
    groups: dict[str, set[int]] = {}
    for root in roots:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root)
        if bid < 0:
            raise ValueError(f"Subject root body {root!r} is not in the model")
        groups[root] = {
            gid
            for gid in range(int(model.ngeom))
            if _root_body_id(model, int(model.geom_bodyid[gid])) == int(bid)
            and _geom_alpha(model, gid) > 0.0
        }
    return groups


def _scene_backdrop(mujoco: Any, model: Any, scenario: str) -> set[int]:
    """Return the backdrop geom ids configured for ``scenario``."""
    return backdrop_geom_ids(
        mujoco,
        model,
        prefixes=_BACKDROP_PREFIXES.get(scenario, ()),
        subject_roots=_SUBJECT_ROOT_BODIES.get(scenario, ()),
    )


def _render_omx_pick_place_stills(
    mujoco: Any,
    mjcf_path: Path,
    output_dir: Path,
    *,
    cameras: list[str],
    width: int,
    height: int,
    state_name: str,
    duration_s: float,
    record_every_steps: int,
) -> list[StillResult]:
    """Render OM-X stills from one searched instant of the SC-v13b cycle."""
    samples, sample_period_s = _omx_pick_place_samples(
        duration_s, record_every_steps
    )
    index, sample = select_still_sample(samples, state_name)

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    if box_jid < 0:
        raise StillRenderError("pick_box_joint missing from presentation MJCF")
    _apply_omx_pick_place_sample(
        mujoco,
        model,
        data,
        q_arm=sample.q_arm,
        gripper=sample.gripper,
        box_qpos=sample.box_qpos,
        box_qadr=int(model.jnt_qposadr[box_jid]),
    )
    _require_cameras(mujoco, model, cameras, mjcf_path)

    background = _scene_backdrop(mujoco, model, "omx_pick_place")
    output_dir.mkdir(parents=True, exist_ok=True)
    results: list[StillResult] = []
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        for camera in cameras:
            results.append(
                _measure_still(
                    mujoco,
                    renderer,
                    data,
                    camera=camera,
                    background=background,
                    output_dir=output_dir,
                    scenario="omx_pick_place",
                    sample_index=index,
                    sample_count=len(samples),
                    state=state_name,
                    sim_time_s=float(index) * sample_period_s,
                )
            )
    finally:
        renderer.close()
    return results


def _render_dubins_race_stills(
    mujoco: Any,
    mjcf_path: Path,
    output_dir: Path,
    *,
    cameras: list[str],
    width: int,
    height: int,
    fps: int,
) -> list[StillResult]:
    """Render Dubins stills, each from the instant its own camera frames best.

    The two racing AGVs separate and rejoin as the race runs, so one shared
    instant cannot frame all three viewpoints. Each camera therefore keeps
    its own searched instant, reported alongside the frame.
    """
    rrt, sst, dummy, _race_time_s = _dubins_race_samples(fps)
    declared = {
        cam.name: cam
        for cam in thumbnail_cameras(thumbnail_template("dubins_race"))
    }
    aspect = float(width) / float(height)

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    _require_cameras(mujoco, model, cameras, mjcf_path)
    background = _scene_backdrop(mujoco, model, "dubins_race")
    agent_geoms = subject_geom_ids_by_root(
        mujoco, model, _SUBJECT_ROOT_BODIES["dubins_race"]
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    results: list[StillResult] = []
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        for camera in cameras:
            if camera not in declared:
                raise StillRenderError(
                    f"Camera {camera!r} is not declared in {mjcf_path}"
                )
            index, margin = _first_visible_dubins_sample(
                mujoco,
                model,
                data,
                renderer,
                camera=camera,
                declared=declared[camera],
                rrt=rrt,
                sst=sst,
                dummy=dummy,
                aspect=aspect,
                agent_geoms=agent_geoms,
            )
            _apply_dubins_poses(
                mujoco, model, data, rrt[index], sst[index], dummy[index]
            )
            results.append(
                _measure_still(
                    mujoco,
                    renderer,
                    data,
                    camera=camera,
                    background=background,
                    output_dir=output_dir,
                    scenario="dubins_race",
                    sample_index=index,
                    sample_count=len(rrt),
                    state=f"RACE(margin={margin:.3f})",
                    sim_time_s=float(index) / float(fps),
                )
            )
    finally:
        renderer.close()
    return results


def _first_visible_dubins_sample(
    mujoco: Any,
    model: Any,
    data: Any,
    renderer: Any,
    *,
    camera: str,
    declared: ThumbnailCamera,
    rrt: npt.NDArray[np.float64],
    sst: npt.NDArray[np.float64],
    dummy: npt.NDArray[np.float64],
    aspect: float,
    agent_geoms: dict[str, set[int]],
) -> tuple[int, float]:
    """Return the best-framed instant that also shows both AGVs unoccluded.

    Args:
        mujoco: Imported ``mujoco`` module.
        model: Loaded ``MjModel``.
        data: Loaded ``MjData``, written in place.
        renderer: Renderer used for the segmentation probe.
        camera: Camera name to probe through.
        declared: The same camera as read from the wrapper MJCF.
        rrt: RRT* pose history.
        sst: SST pose history.
        dummy: Dummy-agent pose history.
        aspect: Frame width divided by frame height.
        agent_geoms: Geom ids per racing agent root body.

    Returns:
        The winning index and its framing margin.

    Raises:
        StillRenderError: If no ranked instant shows both AGVs.
    """
    ranked = rank_dubins_still_samples(rrt, sst, declared, aspect=aspect)
    if not ranked:
        raise StillRenderError(
            f"Camera {camera!r} never frames both AGVs inside the central "
            f"{_CENTRAL_FRACTION:.0%} of the race"
        )
    renderer.enable_segmentation_rendering()
    for index, margin in ranked[:_DUBINS_VISIBILITY_CANDIDATES]:
        _apply_dubins_poses(
            mujoco, model, data, rrt[index], sst[index], dummy[index]
        )
        renderer.update_scene(data, camera=camera)
        segmentation = renderer.render()[..., 0].astype(np.int32)
        shares: list[float] = []
        scaled: list[float] = []
        for root, geoms in agent_geoms.items():
            share = float(np.isin(segmentation, list(geoms)).mean())
            pose = rrt[index] if root == "car_rrt" else sst[index]
            _, _, depth = project_to_frame(
                declared,
                np.array(
                    [float(pose[0]), float(pose[1]), _DUBINS_BODY_HEIGHT_M],
                    dtype=np.float64,
                ),
                aspect=aspect,
            )
            shares.append(share)
            scaled.append(share * depth * depth)
        if (
            min(shares) >= _DUBINS_MIN_VISIBLE_FRACTION
            and max(scaled) > 0.0
            and min(scaled) >= _DUBINS_MIN_VISIBLE_BALANCE * max(scaled)
        ):
            return index, margin
    raise StillRenderError(
        f"Camera {camera!r}: the best {_DUBINS_VISIBILITY_CANDIDATES} framed "
        "instants all hide an AGV behind warehouse structure"
    )


def _require_cameras(
    mujoco: Any, model: Any, cameras: list[str], mjcf_path: Path
) -> None:
    """Fail before rendering when a requested camera is not in the model."""
    for camera in cameras:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
            raise StillRenderError(
                f"Camera {camera!r} missing from {mjcf_path}"
            )


def render_stills(
    scenario: str,
    output_dir: Path,
    *,
    cameras: list[str],
    width: int,
    height: int,
    state_name: str | None = None,
    duration_s: float = 20.0,
    record_every_steps: int = 10,
    fps: int = 30,
    enforce_gates: bool = True,
) -> list[StillResult]:
    """Render one PNG per camera from a single instant of a seeded run.

    Args:
        scenario: Scenario stem with a presentation wrapper.
        output_dir: Directory for the PNG files.
        cameras: Presentation camera names to render.
        width: Output width in pixels.
        height: Output height in pixels.
        state_name: FSM state to search for; OM-X only, defaults per scenario.
        duration_s: Simulated cycle budget in seconds; OM-X only.
        record_every_steps: Sample stride in physics steps; OM-X only.
        fps: Pose sampling rate for the Dubins race search.
        enforce_gates: Fail when tone or framing misses its window.

    Returns:
        One result per camera, carrying the measurements.

    Raises:
        StillRenderError: If a camera is missing or a gate fails.
    """
    if not cameras:
        raise StillRenderError("At least one presentation camera is required")

    mujoco, _iio = _require_mujoco()
    mjcf_path = resolve_thumbnail_mjcf(scenario)

    if scenario == "omx_pick_place":
        results = _render_omx_pick_place_stills(
            mujoco,
            mjcf_path,
            output_dir,
            cameras=cameras,
            width=width,
            height=height,
            state_name=state_name or _DEFAULT_STILL_STATE[scenario],
            duration_s=duration_s,
            record_every_steps=record_every_steps,
        )
    elif scenario == "dubins_race":
        results = _render_dubins_race_stills(
            mujoco,
            mjcf_path,
            output_dir,
            cameras=cameras,
            width=width,
            height=height,
            fps=fps,
        )
    else:
        raise StillRenderError(f"Unsupported still scenario: {scenario!r}")

    if enforce_gates:
        _enforce_gates(results, width=width, height=height)
    return results


def _enforce_gates(
    results: list[StillResult], *, width: int, height: int
) -> None:
    """Fail the run when a measured still misses its acceptance window."""
    problems: list[str] = []
    for result in results:
        if (result.width, result.height) != (width, height):
            problems.append(
                f"{result.camera}: rendered {result.width}x{result.height}, "
                f"expected {width}x{height}"
            )
        if not tone_is_acceptable(result.tone):
            problems.append(
                f"{result.camera}: tone outside window "
                f"(mean={result.tone['mean_luma']:.1f}, "
                f"crushed={result.tone['crushed_fraction']:.4f}, "
                f"blown={result.tone['blown_fraction']:.4f})"
            )
        if not subject_is_inside_central(result.extent):
            problems.append(
                f"{result.camera}: subject leaves the central "
                f"{_CENTRAL_FRACTION:.0%} "
                f"(x {result.extent['min_x_fraction']:.3f}"
                f"..{result.extent['max_x_fraction']:.3f}, "
                f"y {result.extent['min_y_fraction']:.3f}"
                f"..{result.extent['max_y_fraction']:.3f})"
            )
    if problems:
        raise StillRenderError("; ".join(problems))


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Render presentation stills from a FRET simulator run "
            "(real renderer output; nothing is drawn on top)."
        ),
    )
    parser.add_argument(
        "--scenario",
        default="omx_pick_place",
        help="Scenario stem with a presentation wrapper MJCF",
    )
    parser.add_argument(
        "--camera",
        action="append",
        dest="cameras",
        metavar="NAME",
        help="Presentation camera name (repeatable)",
    )
    parser.add_argument(
        "--all-cameras",
        action="store_true",
        help="Render every camera the presentation wrapper declares",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory for the PNG files",
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument(
        "--state",
        default=None,
        help="FSM state whose last recorded sample is rendered",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Simulated cycle budget in seconds",
    )
    parser.add_argument(
        "--record-every-steps",
        type=int,
        default=10,
        help="Sample stride in physics steps",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Pose sampling rate searched for the Dubins still",
    )
    parser.add_argument(
        "--metrics-json",
        type=Path,
        default=None,
        help="Write measured tone and framing statistics here",
    )
    parser.add_argument(
        "--no-gates",
        action="store_true",
        help="Report measurements without failing on them (tuning only)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.all_cameras and not args.cameras:
        parser.error("one of --camera or --all-cameras is required")

    template = thumbnail_template(args.scenario)
    if args.all_cameras:
        cameras = [cam.name for cam in thumbnail_cameras(template)]
    else:
        cameras = list(args.cameras)

    results = render_stills(
        args.scenario,
        args.output_dir,
        cameras=cameras,
        width=args.width,
        height=args.height,
        state_name=args.state,
        duration_s=args.duration,
        record_every_steps=args.record_every_steps,
        fps=args.fps,
        enforce_gates=not args.no_gates,
    )
    for result in results:
        print(
            f"Wrote {result.path} "
            f"({result.width}x{result.height}, "
            f"state={result.state}, "
            f"sample={result.sample_index}/{result.sample_count - 1}, "
            f"t={result.sim_time_s:.2f}s, "
            f"mean_luma={result.tone['mean_luma']:.1f}, "
            f"crushed={result.tone['crushed_fraction']:.4f}, "
            f"blown={result.tone['blown_fraction']:.4f}, "
            f"subject_mean_luma={result.subject_tone['mean_luma']:.1f}, "
            f"subject_crushed={result.subject_tone['crushed_fraction']:.4f}, "
            f"subject_x={result.extent['min_x_fraction']:.3f}"
            f"..{result.extent['max_x_fraction']:.3f}, "
            f"subject_y={result.extent['min_y_fraction']:.3f}"
            f"..{result.extent['max_y_fraction']:.3f})"
        )
    if args.metrics_json is not None:
        args.metrics_json.parent.mkdir(parents=True, exist_ok=True)
        args.metrics_json.write_text(
            json.dumps([r.as_dict() for r in results], indent=2) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
