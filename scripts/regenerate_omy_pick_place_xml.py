#!/usr/bin/env python3
"""Regenerate OMY pick-place / clutter MJCF templates at Menagerie scale.

Sizing rules (SC-v14b/c):
  - Ball diameter = 75 % of OMY max pad opening (~86 mm); physics grasp uses pad contact + adhesion (no kinematic carry)
  - Cone diameter = 1.5 × ball diameter; cone height = diameter
  - Pick / place radial reach ~0.49 m (forward stretch, not full extension)
  - Short pick pedestal (OMX-style) so pad-mid grasps clear the floor
  - Clutter wall perpendicular to pick→place LOS at the midpoint
"""

from __future__ import annotations

import re
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_MJCF = _REPO / "src/fret/mjcf"

# Measured from Menagerie OMY + fingertip pads (grip=0 open).
_OMY_MAX_GRIPPER_OPENING_M = 0.1147
_BALL_RADIUS_M = 0.75 * _OMY_MAX_GRIPPER_OPENING_M / 2.0
_CONE_RADIUS_M = 1.5 * (2.0 * _BALL_RADIUS_M) / 2.0
_CONE_HEIGHT_M = 2.0 * _CONE_RADIUS_M
# Cylinder half-height; top = 2 * half-height; ball center = top + radius.
_PEDESTAL_HALF_H_M = 0.045
_PEDESTAL_RADIUS_M = 0.032
_BALL_Z_M = 2.0 * _PEDESTAL_HALF_H_M + _BALL_RADIUS_M

_PICK_XY = (0.40, -0.28)
_PLACE_XY = (0.40, 0.28)

# OMX reference cone (assets/cone.obj native size).
_OMX_CONE_R = 0.05
_OMX_CONE_H = 0.098
_OMX_PLACE = (0.273, 0.160)
_SCALE_XY = _CONE_RADIUS_M / _OMX_CONE_R
_SCALE_Z = _CONE_HEIGHT_M / _OMX_CONE_H

_FUNNEL_ATTR = (
    'rgba="0.55 0.42 0.35 0" friction="1.4 0.1 0.01" '
    "contype=\"2\" conaffinity=\"2\""
)


def _xf_pos(omx_x: float, omx_y: float, omx_z: float) -> tuple[float, float, float]:
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

    Pick: Ø {ball_d_mm:.0f} mm ball on a short pedestal (75 % of max gripper opening).
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
    <material name="pedestal" rgba="0.45 0.46 0.48 1"/>
    <mesh name="place_cone" file="assets/cone.obj" scale="{_SCALE_XY:.5f} {_SCALE_XY:.5f} {_SCALE_Z:.5f}"/>
  </asset>

  <worldbody>
    <light name="key" pos="0.95 0.0 1.6" dir="0 0 -1" directional="true"
           diffuse="0.45 0.46 0.48" specular="0.05 0.05 0.05"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="groundplane"
          friction="1.0 0.1 0.01" contype="9" conaffinity="9"/>
"""


def _scene_footer(*, clutter: bool) -> str:
    zone_r = _CONE_RADIUS_M + 0.01
    plate_z = _CONE_HEIGHT_M
    goal_z = plate_z + 0.001
    tip_z = 0.004 * _SCALE_Z
    wall_block = ""
    if clutter:
        mid_x = (_PICK_XY[0] + _PLACE_XY[0]) / 2.0
        mid_y = (_PICK_XY[1] + _PLACE_XY[1]) / 2.0
        wall_block = f"""
    <geom name="transfer_wall_a" type="box" pos="{mid_x:.5f} {mid_y:.5f} 0.20000"
          size="0.10000 0.02500 0.20000" material="transfer_wall"
          friction="1.0 0.1 0.01" contype="1" conaffinity="1"/>
"""
    return f"""
    <geom name="place_cone" type="mesh" mesh="place_cone"
          pos="{_fmt_pos(_PLACE_XY[0], _PLACE_XY[1], 0.0)}"
          material="place_cone" contype="0" conaffinity="0"/>
{chr(10).join(_funnel_geoms_from_omx((_MJCF / "omx_pick_place.xml").read_text()))}

    <geom name="place_plate" type="cylinder"
          pos="{_fmt_pos(_PLACE_XY[0], _PLACE_XY[1], plate_z)}"
          size="{_CONE_RADIUS_M:.5f} 0.001"
          material="place_plate" contype="0" conaffinity="0"/>
    <geom name="place_cone_rim" type="cylinder"
          pos="{_fmt_pos(_PLACE_XY[0], _PLACE_XY[1], plate_z)}"
          size="{_CONE_RADIUS_M:.5f} 0.0015"
          material="place_cone" contype="0" conaffinity="0"/>

    <geom name="start_zone" type="cylinder"
          pos="{_fmt_pos(_PICK_XY[0], _PICK_XY[1], 0.002)}"
          size="{zone_r:.5f} 0.001"
          material="start_zone" contype="0" conaffinity="0"/>
    <geom name="goal_zone" type="cylinder"
          pos="{_fmt_pos(_PLACE_XY[0], _PLACE_XY[1], goal_z)}"
          size="{zone_r:.5f} 0.001"
          material="goal_zone" contype="0" conaffinity="0"/>

    <geom name="pedestal_pick" type="cylinder"
          pos="{_fmt_pos(_PICK_XY[0], _PICK_XY[1], _PEDESTAL_HALF_H_M)}"
          size="{_PEDESTAL_RADIUS_M:.5f} {_PEDESTAL_HALF_H_M:.5f}"
          material="pedestal" friction="1.2 0.1 0.01"
          contype="2" conaffinity="2"/>

    <body name="pick_box" pos="{_fmt_pos(_PICK_XY[0], _PICK_XY[1], _BALL_Z_M)}">
      <freejoint name="pick_box_joint"/>
      <geom name="pick_box_geom" type="sphere" size="{_BALL_RADIUS_M:.5f}"
            density="400" material="pick_ball"
            friction="2.8 1.2 0.15" solref="0.015 1" solimp="0.9 0.95 0.001"
            condim="6" contype="14" conaffinity="14" priority="1"/>
    </body>
{wall_block}
    <camera name="topdown" pos="0.40 0.0 1.35"
            xyaxes="0 1 0 -1 0 0" fovy="50"/>
    <camera name="overview" pos="0.95 -0.95 1.05"
            xyaxes="0.82 0.57 0 -0.32 0.46 0.83" fovy="40"/>
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
        "OpenMANIPULATOR-Y cluttered ground pick-and-place (SC-v14c).",
    )
    header = header.replace(
        '    <material name="pick_ball"',
        extra_asset + '    <material name="pick_ball"',
    )
    return header + _scene_footer(clutter=True)


def main() -> None:
  pick = _scene_header(
      "omy_pick_place",
      "OpenMANIPULATOR-Y ground pick-and-place (SC-v14b).",
  ) + _scene_footer(clutter=False)
  clutter = _write_clutter_template()
  (_MJCF / "omy_pick_place.xml").write_text(pick, encoding="utf-8")
  (_MJCF / "omy_clutter.xml").write_text(clutter, encoding="utf-8")
  print(
      f"wrote pick_place ball_r={_BALL_RADIUS_M:.4f} "
      f"cone_r={_CONE_RADIUS_M:.4f} ball_z={_BALL_Z_M:.4f}"
  )
  print(f"pick={_PICK_XY} place={_PLACE_XY}")


if __name__ == "__main__":
    main()
