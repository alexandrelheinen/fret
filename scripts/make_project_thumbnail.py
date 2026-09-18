#!/usr/bin/env python3
"""Compose one website thumbnail from two FRET stills (FR-SIM-14).

This is a COMPOSITE, not a render. Unlike the frames
``scripts/render_still.py`` writes, the output here is two renderer frames
cut along a diagonal seam, plus the site's own accent shard. It is kept in
its own file, and written under its own name, so that a composite is never
mistaken for a frame the simulator produced. Nothing is retouched inside
either half: each half is the delivered PNG, panned and cropped, and the
only marks added are the seam and the shard.

House style is read from https://alexandrelheinen.pages.dev, whose project
cards are flat: square corners, no shadow, a hairline border, a 16:9 poster
plate, and one low-opacity angular "cubist" shard per plate. The site
applies ``filter: saturate(0.92) contrast(1.02)`` to poster images itself,
so no tone adjustment is baked in here.

Example::

    python3 scripts/make_project_thumbnail.py \\
        --left artifacts/stills/omx_pick_place_thumb_pick_three_quarter.png \\
        --left-metrics artifacts/stills/omx_pick_place_stills.json \\
        --left-camera thumb_pick_three_quarter \\
        --right artifacts/stills/dubins_race_thumb_aisle_pursuit.png \\
        --right-metrics artifacts/stills/dubins_race_stills.json \\
        --right-camera thumb_aisle_pursuit \\
        --output artifacts/stills/fret_project_thumbnail.png
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

sys.path.insert(0, str(Path(__file__).resolve().parent))

from render_still import write_still_png  # noqa: E402

# Light-theme tokens, lifted from the site's own CSS custom properties.
_SEAM_GAP_COLOR: tuple[int, int, int] = (
    0xFF,
    0xFF,
    0xFF,
)  # --color-background
_SEAM_RULE_COLOR: tuple[int, int, int] = (
    0xD6,
    0xD6,
    0xD6,
)  # --color-outline-variant
_SHARD_COLOR: tuple[int, int, int] = (0x5E, 0x5D, 0x42)  # --color-tertiary
_SHARD_OPACITY: float = 0.12  # --cubist-shard-opacity
# --shard-beta, verbatim from base.css.
_SHARD_BETA: tuple[tuple[float, float], ...] = (
    (0.00, 0.00),
    (0.74, 0.18),
    (1.00, 1.00),
    (0.18, 0.84),
)
# .projects-plate--mark::after sizes its shard at 9rem and offsets it
# -2.5rem, against a card poster that is roughly 25rem wide.
_SHARD_WIDTH_FRACTION: float = 0.36
_SHARD_OVERHANG_FRACTION: float = 0.10

# The seam runs from ``top`` at y=0 to ``bottom`` at y=H, both as fractions
# of the width. Their mean is 0.5, so the two halves have equal area. The
# tilt leans the same way as the site's shards rather than splitting the
# frame down a dead-vertical line.
_SEAM_TOP: float = 0.36
_SEAM_BOTTOM: float = 0.64
# Mutated once at startup by --seam-tilt; the seam always stays centered on
# the midline so the two halves keep equal area.
_SEAM_TILT_DEFAULT: float = 0.14
_SEAM_GAP_PX: int = 10
_SEAM_RULE_PX: int = 2

_CANVAS_WIDTH: int = 1920
_CANVAS_HEIGHT: int = 1080


class ThumbnailError(RuntimeError):
    """The composite could not be built, or a subject would be cut."""


@dataclass(frozen=True)
class Half:
    """One source frame plus the measured extent of its subject."""

    image: npt.NDArray[np.uint8]
    profile: list[list[float]]
    label: str

    def bands_px(
        self, offset_x: int
    ) -> list[tuple[float, float, float, float]]:
        """Return the subject's per-band extent in canvas pixels."""
        height, width = self.image.shape[:2]
        return [
            (
                y_start * height,
                y_end * height,
                min_x * width + offset_x,
                max_x * width + offset_x,
            )
            for y_start, y_end, min_x, max_x in self.profile
        ]


def seam_x_at(y: float, *, height: int, width: int) -> float:
    """Return the seam's x position, in pixels, at row ``y``."""
    fraction = _SEAM_TOP + (_SEAM_BOTTOM - _SEAM_TOP) * (y / float(height))
    return fraction * width


def region_centroid(
    *, width: int, height: int, side: str
) -> tuple[float, float]:
    """Return the centroid of one side of the seam, in pixels."""
    rows = np.arange(height, dtype=np.float64) + 0.5
    seam = np.array(
        [seam_x_at(y, height=height, width=width) for y in rows],
        dtype=np.float64,
    )
    spans = seam if side == "left" else (width - seam)
    centers = seam / 2.0 if side == "left" else (seam + width) / 2.0
    return (
        float((centers * spans).sum() / spans.sum()),
        float((rows * spans).sum() / spans.sum()),
    )


def subject_clears_seam(
    bands: list[tuple[float, float, float, float]],
    *,
    width: int,
    height: int,
    side: str,
    clearance_px: float,
    edge_px: float = 0.0,
) -> tuple[bool, float]:
    """Return whether the subject stays clear of the seam, and by how much.

    The check runs band by band rather than against one bounding box, so a
    subject that is narrow at the top can sit in the narrow end of a sloped
    region instead of being rejected for the width it has lower down.

    Args:
        bands: Per-band extents as ``(y_start, y_end, min_x, max_x)`` in px.
        width: Canvas width.
        height: Canvas height.
        side: ``left`` or ``right``.
        clearance_px: Gap the subject must keep from the seam.
        edge_px: Gap the subject must keep from the outer frame edge.

    Returns:
        Whether the subject clears, and the smallest signed gap in pixels.
    """
    worst = np.inf
    for y_start, y_end, min_x, max_x in bands:
        rows = np.linspace(y_start, y_end, 8)
        seam = np.array(
            [seam_x_at(y, height=height, width=width) for y in rows],
            dtype=np.float64,
        )
        if side == "left":
            gap = float((seam - max_x).min())
            edge = min_x
        else:
            gap = float((min_x - seam).min())
            edge = width - max_x
        worst = min(worst, gap - clearance_px, edge - edge_px)
    return (worst >= 0.0, float(worst))


def solve_offset(
    half: Half,
    *,
    width: int,
    height: int,
    side: str,
    clearance_px: float,
    edge_px: float,
) -> int:
    """Pan a half so its subject sits in its own region and clears the seam.

    The subject is first centered on the region's centroid, then pushed away
    from the seam while the source still covers the region.

    Args:
        half: Source frame and its measured subject extent.
        width: Canvas width.
        height: Canvas height.
        side: ``left`` or ``right``.
        clearance_px: Gap the subject must keep from the seam.

    Returns:
        Horizontal offset in pixels, applied to the source.

    Raises:
        ThumbnailError: If no offset keeps the subject clear.
    """
    source_width = half.image.shape[1]
    centroid_x, _ = region_centroid(width=width, height=height, side=side)
    spans = [(row[2], row[3]) for row in half.profile]
    subject_center = (
        (min(lo for lo, _ in spans) + max(hi for _, hi in spans))
        / 2.0
        * source_width
    )
    start = int(round(centroid_x - subject_center))

    # Coverage bounds: the source must still reach the far edge of its region.
    if side == "left":
        low, high = int(round(_SEAM_BOTTOM * width)) - source_width, 0
    else:
        low, high = 0, width - int(round(_SEAM_TOP * width))

    best: tuple[float, int] | None = None
    for offset in sorted(range(low, high + 1), key=lambda o: abs(o - start)):
        ok, gap = subject_clears_seam(
            half.bands_px(offset),
            width=width,
            height=height,
            side=side,
            clearance_px=clearance_px,
            edge_px=edge_px,
        )
        if ok:
            return offset
        if best is None or gap > best[0]:
            best = (gap, offset)
    shortfall = best[0] if best is not None else float("nan")
    raise ThumbnailError(
        f"{half.label}: no pan keeps the subject clear of the seam "
        f"(best gap {shortfall:.0f} px, need {clearance_px:.0f} px). "
        "Widen the seam tilt or pick a different source frame."
    )


def _paste(
    canvas: npt.NDArray[np.uint8],
    source: npt.NDArray[np.uint8],
    offset_x: int,
) -> None:
    """Blit a source frame onto the canvas at a horizontal offset."""
    height, width = canvas.shape[:2]
    src_height, src_width = source.shape[:2]
    if src_height != height:
        raise ThumbnailError(
            f"Source is {src_width}x{src_height}; expected height {height}"
        )
    dest_start = max(0, offset_x)
    dest_end = min(width, offset_x + src_width)
    if dest_end <= dest_start:
        raise ThumbnailError("Source panned entirely off the canvas")
    canvas[:, dest_start:dest_end] = source[
        :, dest_start - offset_x : dest_end - offset_x
    ]


def _seam_masks(
    width: int, height: int
) -> tuple[
    npt.NDArray[np.bool_], npt.NDArray[np.bool_], npt.NDArray[np.bool_]
]:
    """Return the left-of-seam, gap, and rule masks for the canvas."""
    columns = np.arange(width, dtype=np.float64)[None, :]
    rows = np.arange(height, dtype=np.float64)[:, None]
    seam = (
        _SEAM_TOP + (_SEAM_BOTTOM - _SEAM_TOP) * (rows / float(height))
    ) * width
    distance = columns - seam
    half_gap = _SEAM_GAP_PX / 2.0
    half_rule = half_gap + _SEAM_RULE_PX
    return (
        distance < 0.0,
        np.abs(distance) <= half_gap,
        (np.abs(distance) > half_gap) & (np.abs(distance) <= half_rule),
    )


def _shard_mask(width: int, height: int) -> npt.NDArray[np.bool_]:
    """Return the site's --shard-beta polygon, placed bottom right."""
    size = _SHARD_WIDTH_FRACTION * width
    overhang = _SHARD_OVERHANG_FRACTION * width
    origin_x = width - size + overhang
    origin_y = height - size + overhang
    points = np.array(
        [
            (origin_x + px * size, origin_y + py * size)
            for px, py in _SHARD_BETA
        ]
    )
    columns = np.arange(width, dtype=np.float64)[None, :]
    rows = np.arange(height, dtype=np.float64)[:, None]
    # Accept either winding: the polygon is copied verbatim from the site's
    # CSS, where the order is whatever the designer typed.
    negative = np.ones((height, width), dtype=bool)
    positive = np.ones((height, width), dtype=bool)
    count = len(points)
    for index in range(count):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % count]
        cross = (x1 - x0) * (rows - y0) - (y1 - y0) * (columns - x0)
        negative &= cross <= 0.0
        positive &= cross >= 0.0
    return negative | positive


def compose(
    left: Half,
    right: Half,
    *,
    width: int = _CANVAS_WIDTH,
    height: int = _CANVAS_HEIGHT,
    clearance_px: float = 48.0,
    edge_px: float = 48.0,
    left_offset: int | None = None,
    right_offset: int | None = None,
    shard: bool = False,
) -> tuple[npt.NDArray[np.uint8], dict[str, object]]:
    """Cut two stills along a diagonal seam and add the site's accent shard.

    Args:
        left: Frame that keeps the left of the seam.
        right: Frame that keeps the right of the seam.
        width: Canvas width.
        height: Canvas height.
        clearance_px: Gap each subject must keep from the seam.
        edge_px: Gap each subject must keep from the outer frame edge.
        left_offset: Hand-chosen pan for the left half; solved when omitted.
        right_offset: Hand-chosen pan for the right half; solved when omitted.
        shard: Overlay the site's accent shard (glyph-plate motif).

    Returns:
        The composite frame and the measurements behind it.
    """
    if left_offset is None:
        left_offset = solve_offset(
            left,
            width=width,
            height=height,
            side="left",
            clearance_px=clearance_px,
            edge_px=edge_px,
        )
    if right_offset is None:
        right_offset = solve_offset(
            right,
            width=width,
            height=height,
            side="right",
            clearance_px=clearance_px,
            edge_px=edge_px,
        )

    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    left_canvas = np.zeros_like(canvas)
    right_canvas = np.zeros_like(canvas)
    _paste(left_canvas, left.image, left_offset)
    _paste(right_canvas, right.image, right_offset)

    is_left, is_gap, is_rule = _seam_masks(width, height)
    canvas[:] = np.where(is_left[..., None], left_canvas, right_canvas)
    canvas[is_rule] = _SEAM_RULE_COLOR
    canvas[is_gap] = _SEAM_GAP_COLOR

    if shard:
        mask = _shard_mask(width, height)
        tint = np.array(_SHARD_COLOR, dtype=np.float64)
        blended = (
            canvas[mask].astype(np.float64) * (1.0 - _SHARD_OPACITY)
            + tint * _SHARD_OPACITY
        )
        canvas[mask] = np.clip(np.rint(blended), 0, 255).astype(np.uint8)

    _, left_gap = subject_clears_seam(
        left.bands_px(left_offset),
        width=width,
        height=height,
        side="left",
        clearance_px=clearance_px,
        edge_px=edge_px,
    )
    _, right_gap = subject_clears_seam(
        right.bands_px(right_offset),
        width=width,
        height=height,
        side="right",
        clearance_px=clearance_px,
        edge_px=edge_px,
    )
    left_ok, _ = subject_clears_seam(
        left.bands_px(left_offset),
        width=width,
        height=height,
        side="left",
        clearance_px=clearance_px,
        edge_px=edge_px,
    )
    right_ok, _ = subject_clears_seam(
        right.bands_px(right_offset),
        width=width,
        height=height,
        side="right",
        clearance_px=clearance_px,
        edge_px=edge_px,
    )
    report: dict[str, object] = {
        "width": width,
        "height": height,
        "seam_top_fraction": _SEAM_TOP,
        "seam_bottom_fraction": _SEAM_BOTTOM,
        "left_source": left.label,
        "left_offset_px": left_offset,
        "left_seam_clearance_px": round(left_gap, 1),
        "right_source": right.label,
        "right_offset_px": right_offset,
        "right_seam_clearance_px": round(right_gap, 1),
        "left_within_margins": left_ok,
        "right_within_margins": right_ok,
        "shard": shard,
    }
    return canvas, report


def load_half(
    image_path: Path,
    metrics_path: Path,
    camera: str,
    label: str,
    *,
    canvas_height: int = _CANVAS_HEIGHT,
) -> Half:
    """Read one source frame together with its measured subject extent.

    Args:
        image_path: PNG written by ``scripts/render_still.py``.
        metrics_path: The matching ``--metrics-json`` file.
        camera: Camera name to look up in the metrics.
        label: Human-readable name used in errors and the report.
        canvas_height: Height the source is resampled to.

    Returns:
        The frame and its subject extent.

    Raises:
        ThumbnailError: If the camera is not in the metrics file.
    """
    from PIL import Image

    entries = json.loads(metrics_path.read_text(encoding="utf-8"))
    for entry in entries:
        if entry["camera"] == camera:
            profile = entry.get("extent_profile")
            break
    else:
        raise ThumbnailError(f"Camera {camera!r} is not in {metrics_path}")
    if not profile:
        raise ThumbnailError(
            f"{metrics_path} has no extent_profile for {camera!r}; "
            "re-run scripts/render_still.py to regenerate it"
        )
    with Image.open(image_path) as handle:
        frame = handle.convert("RGB")
        if frame.height != canvas_height:
            # Sources are rendered taller than the canvas so the subject
            # lands smaller in it, which buys room to place it against the
            # seam. Downscaling is the only pixel operation applied to a
            # half, and it is resampling, not retouching.
            scale = canvas_height / float(frame.height)
            frame = frame.resize(
                (int(round(frame.width * scale)), canvas_height),
                Image.LANCZOS,
            )
        image = np.asarray(frame, dtype=np.uint8)
    return Half(
        image=image,
        profile=[[float(value) for value in row] for row in profile],
        label=label,
    )


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Compose two FRET stills into one website thumbnail. "
            "The result is a composite, not a render."
        ),
    )
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--left-metrics", type=Path, required=True)
    parser.add_argument("--left-camera", required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--right-metrics", type=Path, required=True)
    parser.add_argument("--right-camera", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--clearance",
        type=float,
        default=48.0,
        help="Pixels each subject must keep clear of the seam",
    )
    parser.add_argument(
        "--left-offset",
        type=int,
        default=None,
        help="Hand-chosen pan for the left half, in pixels",
    )
    parser.add_argument(
        "--right-offset",
        type=int,
        default=None,
        help="Hand-chosen pan for the right half, in pixels",
    )
    parser.add_argument(
        "--edge-margin",
        type=float,
        default=48.0,
        help="Pixels each subject must keep clear of the outer edges",
    )
    parser.add_argument(
        "--seam-tilt",
        type=float,
        default=_SEAM_TILT_DEFAULT,
        help=(
            "Half the seam's horizontal run, as a fraction of the width; "
            "0 is a vertical split, 0.14 leans like the site's shards"
        ),
    )
    parser.add_argument(
        "--shard",
        action="store_true",
        help=(
            "Overlay the site's accent shard. Off by default: the site "
            "puts that shard on .projects-plate--mark, the glyph plate a "
            "project falls back to when it has no thumbnail, and leaves "
            "real posters plain"
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    global _SEAM_TOP, _SEAM_BOTTOM

    args = build_parser().parse_args(argv)
    _SEAM_TOP = 0.5 - float(args.seam_tilt)
    _SEAM_BOTTOM = 0.5 + float(args.seam_tilt)
    left = load_half(
        args.left, args.left_metrics, args.left_camera, args.left.name
    )
    right = load_half(
        args.right, args.right_metrics, args.right_camera, args.right.name
    )
    canvas, report = compose(
        left,
        right,
        clearance_px=args.clearance,
        edge_px=args.edge_margin,
        left_offset=args.left_offset,
        right_offset=args.right_offset,
        shard=args.shard,
    )
    write_still_png(args.output, canvas)
    print(f"Wrote {args.output}")
    for key, value in report.items():
        print(f"  {key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
