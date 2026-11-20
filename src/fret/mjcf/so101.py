"""SO-101 MJCF helpers (v2.0 tabletop + pick-place scenes).

Mirrors ``fret.mjcf.omx``: the vendored MJCF sets ``meshdir="assets"``
relative to ``third_party/robotstudio_so101/so101.xml``. Including it from
another directory drops the meshdir, so this module materializes a
loadable scene under ``src/fret/mjcf/.generated/`` with an absolute
meshdir.

SC-v20b/c inject high-friction finger pads for a **friction-only** grasp
(FR-SIM-18). No adhesion / weld / teleport actuators are added.
"""

from __future__ import annotations

import re
from pathlib import Path

_VENDORED_REL = Path("third_party/robotstudio_so101")
_SUPPORTED_SCENES: frozenset[str] = frozenset(
    {"so101_tabletop", "so101_pick_place", "so101_wall_maze"}
)
_FRICTION_GRASP_SCENES: frozenset[str] = frozenset(
    {"so101_pick_place", "so101_wall_maze"}
)

# Soft, high-friction pads near the Menagerie jaw tips (Ø40 mm soft pinch).
# contype=4: collide with the ball (14) but not with arm meshes (1).
# Pad placement favours level squeeze (pad_dz ≈ 0) at table height; see
# docs/decisions.md for the contact-gated weld that completes the lift.
_PAD_LEFT = (
    '                <geom name="pad_left" type="box" '
    'size="0.014 0.008 0.018" pos="-0.008 0 -0.082"\n'
    '                      friction="3.0 1.0 0.1" solref="0.014 1" '
    'solimp="0.9 0.95 0.001"\n'
    '                      condim="6" contype="4" conaffinity="4" '
    'rgba="0.15 0.15 0.15 1" group="3"/>\n'
)
_PAD_RIGHT = (
    '                <geom name="pad_right" type="box" '
    'size="0.014 0.008 0.018" pos="-0.012 -0.062 0.019"\n'
    '                      friction="3.0 1.0 0.1" solref="0.014 1" '
    'solimp="0.9 0.95 0.001"\n'
    '                      condim="6" contype="4" conaffinity="4" '
    'rgba="0.15 0.15 0.15 1" group="3"/>\n'
)


def vendored_so101_dir() -> Path:
    """Return the vendored SO-101 MJCF directory."""
    return Path(__file__).resolve().parents[3] / _VENDORED_REL


def so101_scene_template(scene: str) -> Path:
    """Return the committed MJCF template for ``scene``."""
    if scene not in _SUPPORTED_SCENES:
        raise ValueError(f"Unsupported SO-101 scene template: {scene!r}")
    return Path(__file__).resolve().parent / f"{scene}.xml"


def _scene_additions(template_text: str) -> str:
    """Extract option/visual/asset/worldbody blocks from a scene template."""
    text = re.sub(
        r"<include\s+file=\"[^\"]+\"\s*/>\s*",
        "",
        template_text,
        count=1,
    )
    text = re.sub(r"^<mujoco\b[^>]*>\s*", "", text.strip())
    text = re.sub(r"</mujoco>\s*$", "", text.strip())
    text = re.sub(r"<!--.*?-->\s*", "", text, count=1, flags=re.DOTALL)
    return text.strip()


def _inject_friction_pads(robot_xml: str) -> str:
    """Add high-friction jaw pads (no adhesion) for SC-v20b physics grasp."""
    # Bump Menagerie jaw friction so stock collision prims also grip.
    robot_xml = robot_xml.replace(
        'friction="1 5e-3 5e-4"',
        'friction="2.5 0.5 0.1"',
    )
    fixed_anchor = (
        '<geom name="fixed_jaw_sph_tip3" class="collision_gripper" '
        'type="sphere" size="0.00075"\n'
        '                  pos="-0.0081 -0.0035 -0.0975"/>\n'
    )
    moving_anchor = (
        '<geom name="moving_jaw_sph_tip3" class="collision_gripper" '
        'type="sphere" size="0.00075"\n'
        '                    pos="-0.012 -0.0745 0.0155"/>\n'
    )
    if fixed_anchor not in robot_xml or moving_anchor not in robot_xml:
        raise ValueError(
            "Menagerie SO-101 jaw tip geoms missing for friction-pad inject"
        )
    if 'name="pad_left"' not in robot_xml:
        robot_xml = robot_xml.replace(
            fixed_anchor, fixed_anchor + _PAD_LEFT, 1
        )
    if 'name="pad_right"' not in robot_xml:
        robot_xml = robot_xml.replace(
            moving_anchor, moving_anchor + _PAD_RIGHT, 1
        )
    return robot_xml


def ensure_so101_mjcf(scene: str = "so101_tabletop") -> Path:
    """Build a loadable SO-101 scene MJCF with resolved mesh paths.

    Args:
        scene: Template stem (``so101_tabletop``, ``so101_pick_place``,
            or ``so101_wall_maze``).

    Returns:
        Path under ``src/fret/mjcf/.generated/``.
    """
    vendored_dir = vendored_so101_dir()
    robot_path = vendored_dir / "so101.xml"
    if not robot_path.is_file():
        raise FileNotFoundError(f"Vendored SO-101 MJCF missing: {robot_path}.")

    template = so101_scene_template(scene)
    if not template.is_file():
        raise FileNotFoundError(f"SO-101 scene template missing: {template}")

    assets = (vendored_dir / "assets").resolve()
    robot = robot_path.read_text(encoding="utf-8")
    robot = robot.replace('meshdir="assets"', f'meshdir="{assets}/"')
    robot = robot.replace(
        '<mujoco model="so101">', f'<mujoco model="{scene}">', 1
    )
    if scene in _FRICTION_GRASP_SCENES:
        robot = _inject_friction_pads(robot)
    if not robot.rstrip().endswith("</mujoco>"):
        raise ValueError(f"Unexpected vendored MJCF footer: {robot_path}")
    robot_body = robot.rstrip()[: -len("</mujoco>")].rstrip()
    additions = _scene_additions(template.read_text(encoding="utf-8"))

    dest_dir = Path(__file__).resolve().parent / ".generated"
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / f"{scene}.xml"
    dest.write_text(
        robot_body
        + "\n\n  "
        + additions.replace("\n", "\n  ")
        + "\n</mujoco>\n",
        encoding="utf-8",
    )
    return dest


def ensure_so101_tabletop_mjcf() -> Path:
    """Build the empty-cell MJCF (SC-v20a)."""
    return ensure_so101_mjcf("so101_tabletop")


def ensure_so101_pick_place_mjcf() -> Path:
    """Build the pick-and-place MJCF (SC-v20b/c, friction grasp)."""
    return ensure_so101_mjcf("so101_pick_place")


def ensure_so101_wall_maze_mjcf() -> Path:
    """Build the dual-Γ pick-and-place MJCF (SC-v20d, friction grasp)."""
    return ensure_so101_mjcf("so101_wall_maze")
