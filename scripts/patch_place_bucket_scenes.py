#!/usr/bin/env python3
"""Replace tip-down cone visuals with the AWS-derived place bucket mesh.

Keeps analytic ``funnel_w*`` / ``funnel_tip`` catchers (mesh collision would
convex-hull shut). Updates OM-X / OMY pick-place, clutter, and maze templates.
"""

from __future__ import annotations

import re
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_MJCF = _REPO / "src/fret/mjcf"

# Native place_bucket.obj extents (bottom-centered).
_BUCKET_SX = 0.94106129
_BUCKET_SY = 1.21770874
_BUCKET_SZ = 1.40414841

# Match existing YAML place receptacle envelope.
_OMX_R = 0.050
_OMX_H = 0.098
_OMY_R = 0.140
_OMY_H = 0.280

_OMX_SCALE = (
    (2.0 * _OMX_R) / _BUCKET_SX,
    (2.0 * _OMX_R) / _BUCKET_SY,
    _OMX_H / _BUCKET_SZ,
)
_OMY_SCALE = (
    (2.0 * _OMY_R) / _BUCKET_SX,
    (2.0 * _OMY_R) / _BUCKET_SY,
    _OMY_H / _BUCKET_SZ,
)

_SCENES: tuple[tuple[str, tuple[float, float, float]], ...] = (
    ("omx_pick_place.xml", _OMX_SCALE),
    ("omx_desk_clutter.xml", _OMX_SCALE),
    ("omx_wall_maze.xml", _OMX_SCALE),
    ("omy_pick_place.xml", _OMY_SCALE),
    ("omy_clutter.xml", _OMY_SCALE),
)


def _fmt_scale(scale: tuple[float, float, float]) -> str:
    return f"{scale[0]:.5f} {scale[1]:.5f} {scale[2]:.5f}"


def _patch_header_comment(text: str) -> str:
    text = text.replace(
        "transparent non-colliding plate over a tip-down cone funnel\n"
        "    (visual mesh + wall collision) that catches the dropped ball.\n"
        "    Cone mesh: assets/cone.obj.",
        "AWS warehouse bucket visual (scaled) over analytic funnel walls\n"
        "    that catch the dropped ball. Mesh: assets/place_bucket.obj.",
    )
    text = text.replace(
        "Cone: assets/cone.obj (visual) + funnel_w* collision.",
        "Bucket: assets/place_bucket.obj (visual) + funnel_w* collision.",
    )
    text = text.replace(
        "retracts around the wall. Cone: assets/cone.obj + funnel_w*.",
        "retracts around the wall. Bucket: assets/place_bucket.obj + funnel_w*.",
    )
    text = re.sub(
        r"Place: tip-down cone funnel — Ø [0-9]+ mm, height [0-9]+ mm\.\n"
        r"    Cone mesh: assets/cone\.obj \(scaled in geom\)\.",
        "Place: AWS warehouse bucket visual (scaled) + funnel_w* collision.\n"
        "    Bucket mesh: assets/place_bucket.obj.",
        text,
    )
    text = text.replace(
        "Place: tip-down cone funnel — Ø 280 mm, height 280 mm.\n"
        "    Cone mesh: assets/cone.obj (scaled in geom).",
        "Place: AWS warehouse bucket visual (scaled) + funnel_w* collision.\n"
        "    Bucket mesh: assets/place_bucket.obj.",
    )
    return text


def _patch_assets(text: str, scale: tuple[float, float, float]) -> str:
    text = text.replace(
        '<material name="place_cone" rgba="0.55 0.42 0.35 1"/>',
        '<material name="place_bucket" rgba="0.58 0.60 0.63 1"/>',
    )
    # Drop translucent plate material — goal_zone marks the mouth.
    text = text.replace(
        '    <material name="place_plate" rgba="0.85 0.35 0.28 0.45"/>\n',
        "",
    )
    scale_s = _fmt_scale(scale)
    text = re.sub(
        r'<mesh name="place_cone" file="assets/cone\.obj"'
        r'(?: scale="[^"]*")?\s*/>',
        f'<mesh name="place_bucket" file="assets/place_bucket.obj" '
        f'scale="{scale_s}"/>',
        text,
    )
    return text


def _patch_worldbody(text: str) -> str:
    text = text.replace(
        "<!-- Place dispenser: tip-down cone + transparent plate (no collision). -->\n"
        "    <!-- Visual funnel (mesh is convex-hulled by MuJoCo; collision uses funnel_w*). -->\n",
        "<!-- Place receptacle: bucket visual + analytic funnel catcher. -->\n"
        "    <!-- Visual mesh is non-colliding; funnel_w*/tip catch the ball. -->\n",
    )
    text = text.replace(
        "<!-- Visual mesh only; funnel_w* / funnel_tip carry rigid collision. -->\n",
        "<!-- Bucket visual only; funnel_w* / funnel_tip carry rigid collision. -->\n",
    )
    text = text.replace(
        'material="place_cone"',
        'material="place_bucket"',
    )
    text = text.replace(
        'mesh="place_cone"',
        'mesh="place_bucket"',
    )
    text = re.sub(
        r'<geom name="place_cone" type="mesh" mesh="place_bucket"',
        '<geom name="place_bucket" type="mesh" mesh="place_bucket"',
        text,
    )
    # Remove mouth plate + rim (bucket mesh carries its own lip).
    text = re.sub(
        r"\n\s*<geom name=\"place_plate\"[^/]*/>\n",
        "\n",
        text,
    )
    text = re.sub(
        r"\n\s*<!-- Opaque lip:[^\n]*-->\n"
        r"\s*<geom name=\"place_cone_rim\"[^/]*/>\n",
        "\n",
        text,
    )
    text = re.sub(
        r"\n\s*<geom name=\"place_cone_rim\"[^/]*/>\n",
        "\n",
        text,
    )
    text = text.replace(
        "arm + pick ball + place cone",
        "arm + pick ball + place bucket",
    )
    text = text.replace(
        "Contact bits: arm=1, cone=2, pads=4, floor=1|8.\n"
        "         Ball=2|4|8 hits floor/pads/cone but not arm links.",
        "Contact bits: arm=1, place-catcher=2, pads=4, floor=1|8.\n"
        "         Ball=2|4|8 hits floor/pads/catcher but not arm links.",
    )
    return text


def patch_file(path: Path, scale: tuple[float, float, float]) -> None:
    text = path.read_text(encoding="utf-8")
    text = _patch_header_comment(text)
    text = _patch_assets(text, scale)
    text = _patch_worldbody(text)
    if 'name="place_bucket"' not in text:
        raise RuntimeError(f"place_bucket geom missing after patch: {path}")
    if 'file="assets/place_bucket.obj"' not in text:
        raise RuntimeError(f"place_bucket mesh missing after patch: {path}")
    if 'name="place_cone"' in text or "assets/cone.obj" in text:
        raise RuntimeError(f"stale cone references remain in {path}")
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} scale={_fmt_scale(scale)}")


def main() -> None:
    for name, scale in _SCENES:
        patch_file(_MJCF / name, scale)


if __name__ == "__main__":
    main()
