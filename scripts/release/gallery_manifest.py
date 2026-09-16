#!/usr/bin/env python3
"""Load the illustration gallery matrix from config/release/gallery.yml.

The gallery renders *stills* (16:9 PNG plates) from the same simulations
the release videos use. This module holds the manifest schema plus the
pure helpers the renderer needs (hero-frame selection, ghost sampling,
colour grading), so they can be unit-tested without MuJoCo or a GPU.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import numpy as np
import numpy.typing as npt
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_MANIFEST = _REPO_ROOT / "src/fret/config/release/gallery.yml"

SourceKind = Literal["pick_place", "clutter", "dubins"]
_SOURCE_KINDS: frozenset[str] = frozenset({"pick_place", "clutter", "dubins"})
_ASPECT_RATIO = 16.0 / 9.0


@dataclass(frozen=True)
class CameraSpec:
    """Free-camera framing for one plate."""

    lookat: tuple[float, float, float]
    distance: float
    azimuth: float
    elevation: float
    fovy: float = 45.0
    # Optional subject lock: an agent name, "pack" (centroid of the
    # planner agents), or "tool". The hero-frame subject position then
    # replaces ``lookat`` in x and y.
    track: str = ""


@dataclass(frozen=True)
class GhostSpec:
    """Multi-exposure motion trail settings.

    ``count`` poses are sampled from the recorded run between the
    ``span`` fractions of the hero frame's own history, then screened
    over the beauty frame with weights ramping from ``weight_min`` to
    ``weight_max`` (oldest to newest).
    """

    count: int = 0
    span: tuple[float, float] = (0.0, 1.0)
    weight_min: float = 0.04
    weight_max: float = 0.10
    rgb: tuple[float, float, float] = (0.18, 0.62, 0.95)
    bodies: tuple[str, ...] = ()

    @property
    def enabled(self) -> bool:
        return self.count > 0 and bool(self.bodies)


@dataclass(frozen=True)
class TraceSpec:
    """Executed-trajectory ribbon settings."""

    width: float = 0.003
    future_width: float = 0.0018
    future_scale: float = 0.45
    marker_radius: float = 0.013
    phase_colors: bool = True
    show_future: bool = True


@dataclass(frozen=True)
class GradeSpec:
    """Filmic grade applied to the composited plate."""

    bloom_threshold: float = 0.70
    bloom_gain: float = 1.1
    bloom_sigma: float = 22.0
    contrast: float = 1.10
    pivot: float = 0.45
    vignette: float = 0.75
    vignette_start: float = 0.55


@dataclass(frozen=True)
class GalleryPlate:
    """One still illustration rendered from a real FRET run."""

    id: str
    model: str
    scenario: str
    source: SourceKind
    camera: CameraSpec
    output: str
    hero_state: int | None = None
    hero_at: float = 0.5
    ghosts: GhostSpec = field(default_factory=GhostSpec)
    trace: TraceSpec = field(default_factory=TraceSpec)
    grade: GradeSpec = field(default_factory=GradeSpec)
    caption: str = ""
    agents: tuple[str, ...] = ()
    width: int | None = None
    height: int | None = None

    def effective_width(self, default_width: int) -> int:
        """Return the per-plate width override or the manifest default."""
        return (
            int(self.width) if self.width is not None else int(default_width)
        )

    def effective_height(self, default_height: int) -> int:
        """Return the per-plate height override or the manifest default."""
        return (
            int(self.height)
            if self.height is not None
            else int(default_height)
        )


@dataclass(frozen=True)
class GalleryManifest:
    """Full illustration gallery matrix."""

    width: int
    height: int
    supersample: int
    plates: tuple[GalleryPlate, ...]

    def by_id(self, plate_id: str) -> GalleryPlate:
        """Return the plate with ``plate_id``."""
        for plate in self.plates:
            if plate.id == plate_id:
                return plate
        raise KeyError(f"Unknown gallery plate: {plate_id!r}")


def _tuple3(raw: Any, name: str) -> tuple[float, float, float]:
    values = [float(v) for v in raw]
    if len(values) != 3:
        raise ValueError(f"{name} must have 3 numbers, got {raw!r}")
    return (values[0], values[1], values[2])


def _parse_camera(raw: dict[str, Any]) -> CameraSpec:
    return CameraSpec(
        lookat=_tuple3(raw["lookat"], "camera.lookat"),
        distance=float(raw["distance"]),
        azimuth=float(raw["azimuth"]),
        elevation=float(raw["elevation"]),
        fovy=float(raw.get("fovy", 45.0)),
        track=str(raw.get("track", "")),
    )


def _parse_ghosts(raw: dict[str, Any] | None) -> GhostSpec:
    if not raw:
        return GhostSpec()
    span_raw = raw.get("span", [0.0, 1.0])
    span = (float(span_raw[0]), float(span_raw[1]))
    if not 0.0 <= span[0] < span[1] <= 1.0:
        raise ValueError(f"ghosts.span must be 0 <= a < b <= 1, got {span!r}")
    weights = raw.get("weight", [0.04, 0.10])
    return GhostSpec(
        count=int(raw.get("count", 0)),
        span=span,
        weight_min=float(weights[0]),
        weight_max=float(weights[1]),
        rgb=_tuple3(raw.get("rgb", [0.18, 0.62, 0.95]), "ghosts.rgb"),
        bodies=tuple(str(b) for b in raw.get("bodies", ())),
    )


def _parse_trace(raw: dict[str, Any] | None) -> TraceSpec:
    if not raw:
        return TraceSpec()
    return TraceSpec(
        width=float(raw.get("width", 0.003)),
        future_width=float(raw.get("future_width", 0.0018)),
        future_scale=float(raw.get("future_scale", 0.45)),
        marker_radius=float(raw.get("marker_radius", 0.013)),
        phase_colors=bool(raw.get("phase_colors", True)),
        show_future=bool(raw.get("show_future", True)),
    )


def _parse_grade(raw: dict[str, Any] | None) -> GradeSpec:
    if not raw:
        return GradeSpec()
    return GradeSpec(
        bloom_threshold=float(raw.get("bloom_threshold", 0.70)),
        bloom_gain=float(raw.get("bloom_gain", 1.1)),
        bloom_sigma=float(raw.get("bloom_sigma", 22.0)),
        contrast=float(raw.get("contrast", 1.10)),
        pivot=float(raw.get("pivot", 0.45)),
        vignette=float(raw.get("vignette", 0.75)),
        vignette_start=float(raw.get("vignette_start", 0.55)),
    )


def _parse_plate(raw: dict[str, Any]) -> GalleryPlate:
    source = str(raw["source"])
    if source not in _SOURCE_KINDS:
        raise ValueError(
            f"plate {raw.get('id')!r} source must be one of "
            f"{sorted(_SOURCE_KINDS)}, got {source!r}"
        )
    hero_raw = raw.get("hero", {}) or {}
    hero_state = hero_raw.get("state")
    hero_at = float(hero_raw.get("at", 0.5))
    if not 0.0 <= hero_at <= 1.0:
        raise ValueError(
            f"plate {raw.get('id')!r} hero.at must be in [0, 1], "
            f"got {hero_at!r}"
        )
    output = str(raw["output"])
    if not output.endswith(".png"):
        raise ValueError(
            f"plate {raw.get('id')!r} output must be a .png, got {output!r}"
        )
    return GalleryPlate(
        id=str(raw["id"]),
        model=str(raw["model"]),
        scenario=str(raw["scenario"]),
        source=source,  # type: ignore[arg-type]
        camera=_parse_camera(raw["camera"]),
        output=output,
        hero_state=int(hero_state) if hero_state is not None else None,
        hero_at=hero_at,
        ghosts=_parse_ghosts(raw.get("ghosts")),
        trace=_parse_trace(raw.get("trace")),
        grade=_parse_grade(raw.get("grade")),
        caption=str(raw.get("caption", "")),
        agents=tuple(str(a) for a in raw.get("agents", ())),
        width=int(raw["width"]) if raw.get("width") is not None else None,
        height=int(raw["height"]) if raw.get("height") is not None else None,
    )


def load_gallery_manifest(path: Path | None = None) -> GalleryManifest:
    """Load and validate the gallery matrix.

    Args:
        path: Manifest path; defaults to the in-tree gallery.yml.

    Returns:
        Parsed manifest.

    Raises:
        ValueError: If the matrix is malformed or not 16:9.
    """
    manifest_path = path or _DEFAULT_MANIFEST
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    width = int(raw["width"])
    height = int(raw["height"])
    supersample = int(raw.get("supersample", 2))
    if supersample < 1:
        raise ValueError(f"supersample must be >= 1, got {supersample!r}")
    plates = tuple(_parse_plate(item) for item in raw["plates"])
    if not plates:
        raise ValueError("gallery manifest defines no plates")
    ids = [plate.id for plate in plates]
    if len(set(ids)) != len(ids):
        raise ValueError(f"duplicate plate ids in manifest: {ids}")
    for plate in plates:
        assert_sixteen_by_nine(
            plate.effective_width(width),
            plate.effective_height(height),
            context=plate.id,
        )
    assert_sixteen_by_nine(width, height, context="manifest default")
    return GalleryManifest(
        width=width,
        height=height,
        supersample=supersample,
        plates=plates,
    )


def assert_sixteen_by_nine(
    width: int, height: int, *, context: str = "plate"
) -> None:
    """Raise when ``width`` x ``height`` is not a 16:9 frame.

    Every gallery plate ships 16:9 so one crop works for the website
    card, a slide, and a social preview.
    """
    if height <= 0 or width <= 0:
        raise ValueError(f"{context}: size must be positive")
    if abs(width / height - _ASPECT_RATIO) > 1e-6:
        raise ValueError(
            f"{context}: {width}x{height} is not 16:9 "
            f"(ratio {width / height:.4f})"
        )


def hero_index(
    states: npt.NDArray[np.int_] | None,
    *,
    n_samples: int,
    state: int | None,
    at: float,
) -> int:
    """Pick the sample index the sharp (non-ghosted) pose is rendered at.

    With ``state`` set, the index is taken inside that FSM phase at the
    ``at`` fraction of the phase; otherwise ``at`` indexes the whole run.

    Args:
        states: Per-sample FSM state codes, or None for sources that
            have no FSM (the Dubins race).
        n_samples: Number of recorded samples.
        state: FSM state to land in, or None.
        at: Fraction in [0, 1].

    Returns:
        Sample index.

    Raises:
        ValueError: If the run is empty or never visits ``state``.
    """
    if n_samples <= 0:
        raise ValueError("run has no recorded samples")
    if state is None or states is None:
        return int(round(at * (n_samples - 1)))
    matches = np.flatnonzero(np.asarray(states) == int(state))
    if matches.size == 0:
        raise ValueError(f"run never entered FSM state {state}")
    return int(matches[int(round(at * (matches.size - 1)))])


def ghost_indices(
    *, hero: int, count: int, span: tuple[float, float]
) -> list[int]:
    """Sample the ghost poses that precede the hero frame.

    Ghosts are spread over ``span`` of the history *before* the hero
    frame so the trail reads as motion instead of a blob around the
    sharp pose.
    """
    if count <= 0 or hero <= 0:
        return []
    last = hero - 1
    lo = int(round(span[0] * last))
    hi = int(round(span[1] * last))
    if hi <= lo:
        return []
    picks = np.linspace(lo, hi, num=count, endpoint=True)
    ordered = sorted({int(round(p)) for p in picks})
    return [i for i in ordered if 0 <= i < hero]


def ghost_weights(
    n_ghosts: int, *, weight_min: float, weight_max: float, gamma: float = 2.0
) -> npt.NDArray[np.float64]:
    """Return per-ghost screen weights, oldest (faintest) first."""
    if n_ghosts <= 0:
        return np.zeros(0, dtype=np.float64)
    if n_ghosts == 1:
        return np.array([weight_max], dtype=np.float64)
    ramp = np.linspace(0.0, 1.0, n_ghosts) ** float(gamma)
    return weight_min + (weight_max - weight_min) * ramp


def screen(
    base: npt.NDArray[np.float64],
    layer: npt.NDArray[np.float64],
    weight: float,
) -> npt.NDArray[np.float64]:
    """Screen-blend ``layer`` over ``base`` at ``weight`` (both in 0..1)."""
    return 1.0 - (1.0 - base) * (1.0 - np.clip(layer * weight, 0.0, 1.0))


def box_downscale(
    image: npt.NDArray[np.float64], factor: int
) -> npt.NDArray[np.float64]:
    """Average-pool an image by an integer ``factor`` (supersampling)."""
    if factor <= 1:
        return image
    height, width = image.shape[:2]
    if height % factor or width % factor:
        raise ValueError(
            f"{width}x{height} is not divisible by supersample {factor}"
        )
    reshaped = image.reshape(
        height // factor, factor, width // factor, factor, -1
    )
    return reshaped.mean(axis=(1, 3))


def blur(
    image: npt.NDArray[np.float64], sigma: float
) -> npt.NDArray[np.float64]:
    """Separable Gaussian blur (NumPy only, no OpenCV/SciPy dependency)."""
    if sigma <= 0.0:
        return image
    radius = max(1, int(round(3.0 * sigma)))
    taps = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-0.5 * (taps / sigma) ** 2)
    kernel /= kernel.sum()
    padded = np.pad(
        image, ((radius, radius), (radius, radius), (0, 0)), mode="edge"
    )
    rows = np.apply_along_axis(
        lambda m: np.convolve(m, kernel, mode="valid"), 0, padded
    )
    return np.apply_along_axis(
        lambda m: np.convolve(m, kernel, mode="valid"), 1, rows
    )


def pyramid_blur(
    image: npt.NDArray[np.float64], sigma: float, *, factor: int = 4
) -> npt.NDArray[np.float64]:
    """Blur at reduced resolution: the same bloom halo, far cheaper.

    The image is edge-padded to a multiple of ``factor``, box-filtered
    down, blurred, then nearest-upsampled and cropped back.
    """
    if sigma <= 0.0:
        return image
    height, width = image.shape[:2]
    if factor <= 1 or min(height, width) < 4 * factor:
        return blur(image, sigma)
    pad_y = (-height) % factor
    pad_x = (-width) % factor
    padded = np.pad(image, ((0, pad_y), (0, pad_x), (0, 0)), mode="edge")
    small = blur(box_downscale(padded, factor), sigma / factor)
    grown = np.repeat(np.repeat(small, factor, axis=0), factor, axis=1)
    return grown[:height, :width]


def vignette_mask(
    height: int, width: int, *, strength: float, start: float
) -> npt.NDArray[np.float64]:
    """Return a radial falloff mask in [0, 1] for the plate grade."""
    yy, xx = np.mgrid[0:height, 0:width]
    norm_x = (xx - width / 2.0) / (width / 2.0)
    norm_y = (yy - height / 2.0) / (height / 2.0)
    radius = np.sqrt(norm_x**2 + norm_y**2)
    falloff = strength * np.clip(radius - start, 0.0, None) ** 1.7
    mask: npt.NDArray[np.float64] = np.clip(1.0 - falloff, 0.25, 1.0)
    return mask


def grade_image(
    image: npt.NDArray[np.float64], spec: GradeSpec
) -> npt.NDArray[np.float64]:
    """Apply bloom, contrast, and vignette to a linear 0..1 RGB plate."""
    bright = np.clip(image - spec.bloom_threshold, 0.0, None)
    out = screen(
        image,
        np.clip(pyramid_blur(bright, spec.bloom_sigma), 0.0, 1.0),
        spec.bloom_gain,
    )
    out = np.clip((out - spec.pivot) * spec.contrast + spec.pivot, 0.0, 1.0)
    mask = vignette_mask(
        out.shape[0],
        out.shape[1],
        strength=spec.vignette,
        start=spec.vignette_start,
    )
    return np.clip(out * mask[..., None], 0.0, 1.0)


def build_parser() -> argparse.ArgumentParser:
    """Build the manifest inspection CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Gallery manifest path (default: config/release/gallery.yml)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print the plate matrix as JSON",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Print the gallery matrix (debug helper)."""
    args = build_parser().parse_args(argv)
    manifest = load_gallery_manifest(args.manifest)
    if args.json:
        payload = {
            "width": manifest.width,
            "height": manifest.height,
            "supersample": manifest.supersample,
            "plates": [
                {
                    "id": plate.id,
                    "scenario": plate.scenario,
                    "source": plate.source,
                    "output": plate.output,
                }
                for plate in manifest.plates
            ],
        }
        print(json.dumps(payload, indent=2))
        return 0
    for plate in manifest.plates:
        print(f"{plate.id:20s} {plate.scenario:20s} -> {plate.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
