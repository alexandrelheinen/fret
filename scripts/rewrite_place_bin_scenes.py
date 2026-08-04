#!/usr/bin/env python3
"""Replace AWS mop-bucket place mesh with a single open square bin.

Visual: four wall boxes + bottom + mouth plate/rim (``contype=0``).
Ball catch: restored analytic tip-down ``funnel_w*`` (bit 2, ball only).
Arm avoidance: planner place-shell occupancy from ``place_xy`` / radius
(see ``pick_place_planning.place_shell_from_params``). Distal place drop must
enter the bin, so wall geoms are not bit-matched to proximal arm links.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_MJCF = _REPO / "src/fret/mjcf"

# (file, place_xy, radius, height, wall_friction for funnel)
_SCENES: tuple[tuple[str, tuple[float, float], float, float, str], ...] = (
    ("omx_pick_place.xml", (0.260, 0.000), 0.050, 0.098, "1.4 0.1 0.01"),
    ("omx_desk_clutter.xml", (0.260, 0.000), 0.050, 0.098, "1.4 0.1 0.01"),
    ("omx_wall_maze.xml", (0.260, 0.000), 0.050, 0.098, "1.4 0.1 0.01"),
    ("omy_pick_place.xml", (0.480, 0.000), 0.140, 0.280, "0.45 0.05 0.01"),
    ("omy_clutter.xml", (0.480, 0.000), 0.140, 0.280, "0.45 0.05 0.01"),
)

_OMX_FUNNEL_REF = "b835588:src/fret/mjcf/omx_pick_place.xml"


def _omx_funnel_lines() -> list[str]:
    raw = subprocess.check_output(
        ["git", "show", _OMX_FUNNEL_REF],
        cwd=_REPO,
        text=True,
    )
    lines = [
        ln
        for ln in raw.splitlines()
        if re.match(r'\s*<geom name="funnel_', ln)
    ]
    if len(lines) < 20:
        raise RuntimeError("failed to extract OMX funnel geoms from git")
    return lines


def _scale_funnel(
    omx_lines: list[str],
    *,
    place_xy: tuple[float, float],
    radius: float,
    height: float,
    friction: str,
) -> list[str]:
    """Map OMX funnel geoms to a new place pose / envelope."""
    omx_place = (0.260, 0.000)
    omx_r, omx_h = 0.050, 0.098
    sx = radius / omx_r
    sz = height / omx_h
    out: list[str] = []
    for line in omx_lines:
        m = re.search(
            r'<geom name="(funnel_w\d+|funnel_tip)" type="(box|sphere)" '
            r'size="([^"]+)" pos="([^"]+)"(?: quat="([^"]+)")?',
            line,
        )
        if not m:
            continue
        name, gtype, size_s, pos_s, quat = m.groups()
        parts = [float(v) for v in size_s.split()]
        px, py, pz = (float(v) for v in pos_s.split())
        nx = place_xy[0] + (px - omx_place[0]) * sx
        ny = place_xy[1] + (py - omx_place[1]) * sx
        nz = pz * sz
        if gtype == "sphere":
            ns = parts[0] * sx
            size_attr = f"{ns:.5f}"
        else:
            size_attr = (
                f"{parts[0] * sx:.5f} {parts[1] * sx:.5f} {parts[2] * sz:.5f}"
            )
        quat_attr = f' quat="{quat}"' if quat else ""
        out.append(
            f'    <geom name="{name}" type="{gtype}" size="{size_attr}" '
            f'pos="{nx:.5f} {ny:.5f} {nz:.5f}"{quat_attr} '
            f'rgba="0.55 0.42 0.35 0" friction="{friction}" '
            f'contype="2" conaffinity="2"/>'
        )
    return out


def _bin_block(
    cx: float,
    cy: float,
    radius: float,
    height: float,
    friction: str,
    funnel_lines: list[str],
) -> str:
    t = min(max(0.004, 0.10 * radius), 0.012)
    # Walls outside the place disk so the opening contains place_cone_radius.
    inner = radius
    outer = radius + t
    hz = 0.5 * height
    z_c = hz
    bot_h = min(0.004, 0.25 * t)
    lines = [
        "    <!-- Place bin: single open square receptacle (visual).",
        "         Ball catch = invisible funnel_w* (bit 2).",
        "         Arm avoidance = planner place-shell occupancy. -->",
        f'    <geom name="place_bin_bottom" type="box" '
        f'pos="{cx:.5f} {cy:.5f} {0.5 * bot_h:.5f}" '
        f'size="{inner:.5f} {inner:.5f} {0.5 * bot_h:.5f}" '
        f'material="place_bin" contype="0" conaffinity="0"/>',
        f'    <geom name="place_bin_wall_px" type="box" '
        f'pos="{cx + inner + 0.5 * t:.5f} {cy:.5f} {z_c:.5f}" '
        f'size="{0.5 * t:.5f} {outer:.5f} {hz:.5f}" '
        f'material="place_bin" contype="0" conaffinity="0"/>',
        f'    <geom name="place_bin_wall_nx" type="box" '
        f'pos="{cx - inner - 0.5 * t:.5f} {cy:.5f} {z_c:.5f}" '
        f'size="{0.5 * t:.5f} {outer:.5f} {hz:.5f}" '
        f'material="place_bin" contype="0" conaffinity="0"/>',
        f'    <geom name="place_bin_wall_py" type="box" '
        f'pos="{cx:.5f} {cy + inner + 0.5 * t:.5f} {z_c:.5f}" '
        f'size="{inner:.5f} {0.5 * t:.5f} {hz:.5f}" '
        f'material="place_bin" contype="0" conaffinity="0"/>',
        f'    <geom name="place_bin_wall_ny" type="box" '
        f'pos="{cx:.5f} {cy - inner - 0.5 * t:.5f} {z_c:.5f}" '
        f'size="{inner:.5f} {0.5 * t:.5f} {hz:.5f}" '
        f'material="place_bin" contype="0" conaffinity="0"/>',
        *funnel_lines,
        f'    <geom name="place_plate" type="cylinder" '
        f'pos="{cx:.5f} {cy:.5f} {height:.5f}" '
        f'size="{outer:.5f} 0.001" '
        f'material="place_plate" contype="0" conaffinity="0"/>',
        f'    <geom name="place_bin_rim" type="cylinder" '
        f'pos="{cx:.5f} {cy:.5f} {height:.5f}" '
        f'size="{outer:.5f} 0.0015" '
        f'material="place_bin" contype="0" conaffinity="0"/>',
    ]
    return "\n".join(lines) + "\n"


def _patch_assets(text: str) -> str:
    text = re.sub(r'[ \t]*<material name="place_bucket".*?/>\n?', "", text)
    text = re.sub(r'[ \t]*<mesh name="place_bucket".*?/>\n?', "", text)
    if 'name="place_bin"' not in text:
        text = text.replace(
            '<material name="start_zone"',
            '<material name="place_bin" rgba="0.55 0.58 0.62 1"/>\n'
            '    <material name="place_plate" rgba="0.85 0.35 0.28 0.45"/>\n'
            '    <material name="start_zone"',
        )
    elif 'name="place_plate"' not in text:
        text = text.replace(
            '<material name="place_bin" rgba="0.55 0.58 0.62 1"/>\n',
            '<material name="place_bin" rgba="0.55 0.58 0.62 1"/>\n'
            '    <material name="place_plate" rgba="0.85 0.35 0.28 0.45"/>\n',
        )
    return text


def _patch_header(text: str, radius: float, height: float) -> str:
    d_mm = 2.0 * radius * 1000.0
    h_mm = height * 1000.0
    text = re.sub(
        r"Place:.*?(?=\n  -->)",
        f"Place: single open square bin (Ø≈{d_mm:.0f} mm mouth, "
        f"h≈{h_mm:.0f} mm) + analytic funnel catcher; planner treats the "
        f"bin shell as an obstacle.",
        text,
        count=1,
        flags=re.DOTALL,
    )
    text = re.sub(
        r"    Place:.*?\n(?:    .*\n)*?(?:    Bucket mesh:.*\n)?",
        f"    Place: single open square bin (Ø≈{d_mm:.0f} mm mouth, "
        f"h≈{h_mm:.0f} mm) + funnel_w* catcher.\n",
        text,
        count=1,
    )
    text = text.replace(
        "Bucket: assets/place_bucket.obj (visual) + funnel_w* collision.",
        "Place: open square bin (visual) + funnel_w* ball catcher.",
    )
    text = text.replace(
        "retracts around the wall. Bucket: assets/place_bucket.obj + funnel_w*.",
        "retracts around the wall. Place: open square bin + funnel catcher.",
    )
    text = text.replace(
        "Contact bits: arm=1, place-bin=3 (arm|catcher), pads=4, floor=1|8.\n"
        "         Ball=2|4|8 hits floor/pads/bin; proximal arm hits bin walls.",
        "Contact bits: arm=1, place-catcher=2, pads=4, floor=1|8.\n"
        "         Ball=2|4|8 hits floor/pads/funnel; planner owns bin avoidance.",
    )
    text = text.replace(
        "Contact bits: arm=1, place-catcher=2, pads=4, floor=1|8.\n"
        "         Ball=2|4|8 hits floor/pads/catcher but not arm links.",
        "Contact bits: arm=1, place-catcher=2, pads=4, floor=1|8.\n"
        "         Ball=2|4|8 hits floor/pads/funnel; planner owns bin avoidance.",
    )
    return text


def _replace_place_block(
    text: str,
    cx: float,
    cy: float,
    radius: float,
    height: float,
    friction: str,
    funnel_lines: list[str],
) -> str:
    bin_xml = _bin_block(cx, cy, radius, height, friction, funnel_lines)
    new_text = re.sub(
        r"\n[ \t]*<!-- (?:Place|Bucket|Visual)[^\n]*-->",
        "",
        text,
    )
    new_text = re.sub(
        r"\n[ \t]*<geom name=\"(?:place_bucket|place_bin(?:_bottom|_wall_px|_wall_nx|"
        r"_wall_py|_wall_ny|_rim)?|place_plate|place_cone_rim|funnel_[^\"]+)\""
        r"[\s\S]*?/>",
        "",
        new_text,
    )
    anchor = '    <geom name="start_zone"'
    if anchor not in new_text:
        raise RuntimeError("start_zone geom missing; cannot insert place bin")
    new_text = new_text.replace(anchor, bin_xml + anchor, 1)
    new_text = re.sub(
        r'(<geom name="goal_zone"[^>]*?pos=")([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)',
        rf"\g<1>{cx:.5f} {cy:.5f} {height + 0.001:.5f}",
        new_text,
        count=1,
    )
    new_text = re.sub(
        r'(<geom name="goal_zone"[^>]*?size=")([0-9.eE+-]+)',
        rf"\g<1>{radius:.5f}",
        new_text,
        count=1,
    )
    return new_text


def patch_file(
    path: Path,
    place_xy: tuple[float, float],
    radius: float,
    height: float,
    friction: str,
    omx_funnel: list[str],
) -> None:
    funnel = _scale_funnel(
        omx_funnel,
        place_xy=place_xy,
        radius=radius,
        height=height,
        friction=friction,
    )
    text = path.read_text(encoding="utf-8")
    text = _patch_header(text, radius, height)
    text = _patch_assets(text)
    text = _replace_place_block(
        text, place_xy[0], place_xy[1], radius, height, friction, funnel
    )
    if "place_bin_wall_px" not in text or "funnel_w0" not in text:
        raise RuntimeError(f"bin/funnel missing in {path}")
    if "place_bucket" in text or "cone.obj" in text:
        raise RuntimeError(f"stale place_bucket references remain in {path}")
    if 'name="place_plate"' not in text:
        raise RuntimeError(f"place_plate missing in {path}")
    path.write_text(text, encoding="utf-8")
    print(f"rewrote {path.name}")


def main() -> None:
    omx_funnel = _omx_funnel_lines()
    for name, xy, radius, height, friction in _SCENES:
        patch_file(_MJCF / name, xy, radius, height, friction, omx_funnel)


if __name__ == "__main__":
    main()
