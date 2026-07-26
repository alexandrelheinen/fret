#!/usr/bin/env python3
"""Regenerate OMY pick-place / clutter MJCF templates at Menagerie scale.

Sizing rules (SC-v14b/c, matching docs/scenarios.md):
  - Pick object: Ø 86 mm sphere on the **floor** (no pedestal)
  - Cone diameter ≈ place funnel; cone height = diameter
  - Pick / place radial reach ~0.49 m (forward stretch, not full extension)
  - Clutter wall perpendicular to pick→place LOS at the midpoint

Note: committed templates also carry the v1.4 vision_gate + ``cv_ball_ghost``
bodies (see ``regen_cell_layout_v14.py``). Re-running this script alone will
not recreate those — prefer editing the committed XML when changing vision
geometry.
"""

from __future__ import annotations

import re
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_MJCF = _REPO / "src/fret/mjcf"

# SC-v14b/c: Ø 86 mm ball resting on the floor / table plane.
_BALL_RADIUS_M = 0.043
_BALL_Z_M = _BALL_RADIUS_M
# Funnel wider than 1.5× ball diameter — physics place has centimetre slip.
_CONE_RADIUS_M = 0.14
_CONE_HEIGHT_M = 2.0 * _CONE_RADIUS_M
_BALL_DENSITY = 150.0

# Open pick-place cell (SC-v14b) pick XY; clutter overrides in footer.
_PICK_XY = (0.350, -0.300)
# Place sits where the loaded 6-DOF arm can deliver past the mid-cell wall.
_PLACE_XY = (0.480, 0.000)
_CLUTTER_PICK_XY = (0.40, -0.28)
_CLUTTER_PLACE_XY = (0.50, 0.20)

# OMX reference cone (assets/cone.obj native size).
_OMX_CONE_R = 0.05
_OMX_CONE_H = 0.098
_OMX_PLACE = (0.273, 0.160)
_SCALE_XY = _CONE_RADIUS_M / _OMX_CONE_R
_SCALE_Z = _CONE_HEIGHT_M / _OMX_CONE_H

# Funnel walls: rigid ball catcher (OMX SC-v13b: contype=2 pedestal class).
# Visual mesh non-colliding; funnel_w*/tip collide with the ball (not ghost).
# Place EE stays above the rim and drops — never enters the funnel volume.
_FUNNEL_ATTR = (
    'rgba="0.55 0.42 0.35 0" friction="0.45 0.05 0.01" '
    'contype="2" conaffinity="2"'
)


def _xf_pos(
    omx_x: float, omx_y: float, omx_z: float
) -> tuple[float, float, float]:
    """Map OMX place-relative coordinates to OMY scene."""
    dx = (omx_x - _OMX_PLACE[0]) * _SCALE_XY
    dy = (omx_y - _OMX_PLACE[1]) * _SCALE_XY
    dz = omx_z * _SCALE_Z
    return (_PLACE_XY[0] + dx, _PLACE_XY[1] + dy, dz)


def _fmt_pos(x: float, y: float, z: float) -> str:
    return f"{x:.5f} {y:.5f} {z:.5f}"


def _funnel_geoms_from_omx(omx_text: str) -> list[str]:
    lines: list[str] = []
    for line in omx_text.splitlines():
        m = re.search(
            r'<geom name="(funnel_w\d+|funnel_tip)" type="(box|sphere)" '
            r'size="([^"]+)" pos="([^"]+)"(?: quat="([^"]+)")?',
            line,
        )
        if not m:
            continue
        name, gtype, size_s, pos_s, quat_vals = m.groups()
        parts = [float(v) for v in size_s.split()]
        if gtype == "sphere":
            sx = sy = sz = parts[0]
        else:
            sx, sy, sz = parts  # type: ignore[misc]
        px, py, pz = (float(v) for v in pos_s.split())
        nx, ny, nz = _xf_pos(px, py, pz)
        nsx, nsy, nsz = sx * _SCALE_XY, sy * _SCALE_XY, sz * _SCALE_Z
        quat_attr = f' quat="{quat_vals}"' if quat_vals else ""
        lines.append(
            f'    <geom name="{name}" type="{gtype}" '
            f'size="{nsx:.5f} {nsy:.5f} {nsz:.5f}" '
            f'pos="{_fmt_pos(nx, ny, nz)}"{quat_attr} {_FUNNEL_ATTR}/>'
        )
    return lines


def _scene_header(model_name: str, comment: str) -> str:
    ball_d_mm = 2.0 * _BALL_RADIUS_M * 1000.0
    cone_d_mm = 2.0 * _CONE_RADIUS_M * 1000.0
    return f"""<mujoco model="{model_name}">
  <!--
    {comment}

    Pick: Ø {ball_d_mm:.0f} mm sphere resting on the floor / table plane.
    Place: tip-down cone funnel — Ø {cone_d_mm:.0f} mm, height {cone_d_mm:.0f} mm.
    Cone mesh: assets/cone.obj (scaled in geom).

  -->
  <include file="omy.xml"/>

  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"
          cone="elliptic" impratio="10"/>

  <statistic center="0.40 0.0 0.18" extent="1.05"/>

  <visual>
    <headlight ambient="0.45 0.47 0.50" diffuse="0.30 0.32 0.35" specular="0 0 0"/>
    <rgba haze="0.75 0.78 0.82 0.2"/>
    <global azimuth="140" elevation="-25" offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient"
             rgb1="0.78 0.80 0.84" rgb2="0.55 0.58 0.62"
             width="512" height="1536"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.85 0.86 0.88" rgb2="0.72 0.74 0.76"
             markrgb="0.55 0.56 0.58" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="4 4" reflectance="0.05"/>
    <material name="place_cone" rgba="0.55 0.42 0.35 1"/>
    <material name="place_plate" rgba="0.85 0.35 0.28 0.45"/>
    <material name="start_zone" rgba="0.20 0.75 0.35 0.35"/>
    <material name="goal_zone" rgba="0.85 0.25 0.20 0.35"/>
    <material name="pick_ball" rgba="0.85 0.92 0.20 1" reflectance="0.05"/>
    <material name="vision_portal" rgba="0.35 0.38 0.42 1"/>
    <mesh name="place_cone" file="assets/cone.obj" scale="{_SCALE_XY:.5f} {_SCALE_XY:.5f} {_SCALE_Z:.5f}"/>
  </asset>

  <worldbody>
    <light name="key" pos="0.95 0.0 1.6" dir="0 0 -1" directional="true"
           diffuse="0.45 0.46 0.48" specular="0.05 0.05 0.05"/>
    <!-- Contact bits: arm=1, cone=2, pads=4, floor=1|8.
         Ball=2|4|8 hits floor/pads/cone but not arm links. -->
    <geom name="floor" type="plane" size="0 0 0.05" material="groundplane"
          friction="1.0 0.1 0.01" contype="9" conaffinity="9"/>
"""


def _scene_footer(
    *,
    clutter: bool,
    pick_xy: tuple[float, float],
    place_xy: tuple[float, float],
) -> str:
    zone_r = _CONE_RADIUS_M + 0.01
    plate_z = _CONE_HEIGHT_M
    goal_z = plate_z + 0.001
    start_z = 0.001
    # Funnel geoms are authored relative to the open pick-place place XY;
    # for clutter, shift them from the open-cell place to the clutter place.
    funnel_src = (_MJCF / "omx_pick_place.xml").read_text(encoding="utf-8")
    global _PLACE_XY  # noqa: PLW0603 — temporary override for funnel map
    saved_place = _PLACE_XY
    _PLACE_XY = place_xy
    try:
        funnel_lines = _funnel_geoms_from_omx(funnel_src)
    finally:
        _PLACE_XY = saved_place
    wall_block = ""
    if clutter:
        # Committed SC-v14c stub (v1.4.1): blocks the lift→place LOS but stays
        # short/thin enough for RRT* and SST under MPC wall honesty.
        # Keep AABB in sync with omy_clutter*.yml ``walls:``.
        wall_x, wall_y = 0.415, -0.120
        wall_half = (0.05, 0.025, 0.09)  # → 18 cm tall
        wall_block = f"""
    <geom name="transfer_wall_a" type="box"
          pos="{wall_x:.5f} {wall_y:.5f} {wall_half[2]:.5f}"
          size="{wall_half[0]:.5f} {wall_half[1]:.5f} {wall_half[2]:.5f}"
          material="transfer_wall"
          friction="1.0 0.1 0.01" contype="1" conaffinity="1"/>
"""
    return f"""
    <!-- Visual mesh only; funnel_w* / funnel_tip carry rigid collision. -->
    <geom name="place_cone" type="mesh" mesh="place_cone"
          pos="{_fmt_pos(place_xy[0], place_xy[1], 0.0)}"
          material="place_cone" contype="0" conaffinity="0"/>
{chr(10).join(funnel_lines)}

    <geom name="place_plate" type="cylinder"
          pos="{_fmt_pos(place_xy[0], place_xy[1], plate_z)}"
          size="{_CONE_RADIUS_M:.5f} 0.001"
          material="place_plate" contype="0" conaffinity="0"/>
    <geom name="place_cone_rim" type="cylinder"
          pos="{_fmt_pos(place_xy[0], place_xy[1], plate_z)}"
          size="{_CONE_RADIUS_M:.5f} 0.0015"
          material="place_cone" contype="0" conaffinity="0"/>

    <geom name="start_zone" type="cylinder"
          pos="{_fmt_pos(pick_xy[0], pick_xy[1], start_z)}"
          size="{zone_r:.5f} 0.001"
          material="start_zone" contype="0" conaffinity="0"/>
    <geom name="goal_zone" type="cylinder"
          pos="{_fmt_pos(place_xy[0], place_xy[1], goal_z)}"
          size="{zone_r:.5f} 0.001"
          material="goal_zone" contype="0" conaffinity="0"/>

    <body name="pick_box" pos="{_fmt_pos(pick_xy[0], pick_xy[1], _BALL_Z_M)}">
      <freejoint name="pick_box_joint"/>
      <geom name="pick_box_geom" type="sphere"
            size="{_BALL_RADIUS_M:.5f}"
            density="{_BALL_DENSITY:.1f}" material="pick_ball"
            friction="2.8 1.2 0.15" solref="0.015 1" solimp="0.9 0.95 0.001"
            condim="6" contype="14" conaffinity="14" priority="1"/>
    </body>
{wall_block}
    <!-- Vision gate / CV ghost are maintained on committed XML (v1.4). -->
    <camera name="overview" pos="1.05 -0.70 1.00"
            xyaxes="0.55 0.84 0 -0.40 0.26 0.88" fovy="40"/>
    <camera name="follow" pos="0.70 0.15 0.55"
            xyaxes="-0.7 0.7 0 -0.3 -0.3 0.9" fovy="50"/>
  </worldbody>
</mujoco>
"""


def _write_clutter_template() -> str:
    extra_asset = (
        '    <material name="transfer_wall" rgba="0.55 0.38 0.28 1" '
        'reflectance="0.05"/>\n'
    )
    header = _scene_header(
        "omy_clutter",
        "OpenMANIPULATOR-Y cluttered floor pick-and-place (SC-v14c).",
    )
    header = header.replace(
        '    <material name="pick_ball"',
        extra_asset + '    <material name="pick_ball"',
    )
    return header + _scene_footer(
        clutter=True,
        pick_xy=_CLUTTER_PICK_XY,
        place_xy=_CLUTTER_PLACE_XY,
    )


def main() -> None:
    pick = _scene_header(
        "omy_pick_place",
        "OpenMANIPULATOR-Y floor pick-and-place (SC-v14b).",
    ) + _scene_footer(
        clutter=False, pick_xy=_PICK_XY, place_xy=_PLACE_XY
    )
    clutter = _write_clutter_template()
    (_MJCF / "omy_pick_place.xml").write_text(pick, encoding="utf-8")
    (_MJCF / "omy_clutter.xml").write_text(clutter, encoding="utf-8")
    print(
        f"wrote pick_place ball_r={_BALL_RADIUS_M:.4f} "
        f"cone_r={_CONE_RADIUS_M:.4f} ball_z={_BALL_Z_M:.4f} (floor)"
    )
    print(f"pick={_PICK_XY} place={_PLACE_XY}")


if __name__ == "__main__":
    main()
