"""OpenMANIPULATOR-X MJCF helpers (v1.2.3 tabletop / pick-place cells).

Menagerie sets ``meshdir="assets/"`` relative to ``open_manipulator_x.xml``.
Including that file from another directory drops the meshdir, so we materialize
a loadable scene under ``src/fret/mjcf/.generated/`` with an absolute meshdir.

SC-v13b/c also inject finger-pad geoms and MuJoCo adhesion actuators so the
parallel gripper can lift the free ball under full physics (stock Menagerie
finger meshes alone do not pinch reliably). Place-cone meshes in the scene
templates are rewritten to absolute paths (Menagerie ``meshdir`` would not
find ``src/fret/mjcf/assets/cone.obj``).
"""

from __future__ import annotations

import re
from pathlib import Path

_MENAGERIE_REL = Path(
    "third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x"
)
_SUPPORTED_SCENES: frozenset[str] = frozenset(
    {
        "omx_tabletop",
        "omx_pick_place",
        "omx_desk_clutter",
        "omx_wall_maze",
    }
)
_PHYSICAL_GRIPPER_SCENES: frozenset[str] = frozenset(
    {"omx_pick_place", "omx_desk_clutter", "omx_wall_maze"}
)
_CONE_MESH_PLACEHOLDER = 'file="assets/cone.obj"'

# Pad contype=4: collide with the ball (14) but not with arm meshes (1).
# Soft, high-friction pads (tennis-ball felt grip). Half-Y 0.007 pinches Ø40 mm
# at Gripper ctrl ≈ 0.006 (inner gap ≈ 37 mm).
_PAD_LEFT = (
    '                <geom name="pad_left" type="box" '
    'size="0.016 0.007 0.012" pos="0.028 0.0 0"\n'
    '                      friction="3.0 1.0 0.1" solref="0.014 1" '
    'solimp="0.9 0.95 0.001"\n'
    '                      condim="6" contype="4" conaffinity="4" '
    'rgba="0.15 0.15 0.15 1" group="3"/>\n'
)
_PAD_RIGHT = (
    '                <geom name="pad_right" type="box" '
    'size="0.016 0.007 0.012" pos="0.028 0.0 0"\n'
    '                      friction="3.0 1.0 0.1" solref="0.014 1" '
    'solimp="0.9 0.95 0.001"\n'
    '                      condim="6" contype="4" conaffinity="4" '
    'rgba="0.15 0.15 0.15 1" group="3"/>\n'
)
# Modest gain + delayed enable (see adhesion_command): assist pinch, not glue.
_ADHESION = (
    '    <adhesion name="grip_left" body="gripper_left" '
    'ctrlrange="0 1" gain="5"/>\n'
    '    <adhesion name="grip_right" body="gripper_right" '
    'ctrlrange="0 1" gain="5"/>\n'
)


def menagerie_omx_dir() -> Path:
    """Return the Menagerie OpenMANIPULATOR-X model directory."""
    return Path(__file__).resolve().parents[3] / _MENAGERIE_REL


def omx_scene_template(scene: str) -> Path:
    """Return the committed MJCF template for ``scene``."""
    if scene not in _SUPPORTED_SCENES:
        raise ValueError(f"Unsupported OM-X scene template: {scene!r}")
    return Path(__file__).resolve().parent / f"{scene}.xml"


def fret_mjcf_assets_dir() -> Path:
    """Return ``src/fret/mjcf/assets`` (cone primitive, …)."""
    return Path(__file__).resolve().parent / "assets"


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
    cone = (fret_mjcf_assets_dir() / "cone.obj").resolve()
    if _CONE_MESH_PLACEHOLDER in text:
        if not cone.is_file():
            raise FileNotFoundError(f"Place-cone mesh missing: {cone}")
        text = text.replace(_CONE_MESH_PLACEHOLDER, f'file="{cone}"')
    return text.strip()


def _inject_physical_gripper(robot_xml: str) -> str:
    """Add finger pads + adhesion actuators for SC-v13b physics grasp."""
    left_anchor = '<joint name="Gripper" class="Gripper"/>\n'
    right_anchor = '<joint name="Gripper_mimic" class="Gripper_mimic"/>\n'
    if left_anchor not in robot_xml or right_anchor not in robot_xml:
        raise ValueError(
            "Menagerie OM-X gripper joints missing for pad inject"
        )
    if 'name="pad_left"' not in robot_xml:
        robot_xml = robot_xml.replace(left_anchor, left_anchor + _PAD_LEFT, 1)
    if 'name="pad_right"' not in robot_xml:
        robot_xml = robot_xml.replace(
            right_anchor, right_anchor + _PAD_RIGHT, 1
        )
    grip_act = (
        '<position class="Gripper" name="Gripper" '
        'joint="Gripper" inheritrange="1"/>\n'
    )
    if 'name="grip_left"' not in robot_xml:
        if grip_act not in robot_xml:
            raise ValueError("Menagerie Gripper actuator missing for adhesion")
        robot_xml = robot_xml.replace(grip_act, grip_act + _ADHESION, 1)
    robot_xml = robot_xml.replace(
        'ctrl="0 0 0 0 0"',
        'ctrl="0 0 0 0 0 0 0"',
        1,
    )
    return robot_xml


_FLOOR_PICK_SCENES: frozenset[str] = frozenset(
    {"omx_pick_place", "omx_desk_clutter", "omx_wall_maze"}
)


def _enable_floor_pick_contacts(robot_xml: str) -> str:
    """Remap palm collision bits for floor-ball grasp (all OM-X pick cells).

    Palms match pad bit 4 so the STL does not fight the plane while closing
    on a table-resting ball (pick-place, desk clutter, Γ maze).
    """
    robot_xml = robot_xml.replace(
        '<geom mesh="gripper_left_palm" class="collision"/>',
        '<geom mesh="gripper_left_palm" class="collision" '
        'contype="4" conaffinity="4"/>',
        1,
    )
    robot_xml = robot_xml.replace(
        '<geom mesh="gripper_right_palm" class="collision"/>',
        '<geom mesh="gripper_right_palm" class="collision" '
        'contype="4" conaffinity="4"/>',
        1,
    )
    return robot_xml


def ensure_omx_mjcf(scene: str = "omx_tabletop") -> Path:
    """Build a loadable OM-X scene MJCF with resolved mesh paths.

    Args:
        scene: Template stem (``omx_tabletop``, ``omx_pick_place``,
            ``omx_desk_clutter``, or ``omx_wall_maze``).

    Returns:
        Path under ``src/fret/mjcf/.generated/``.
    """
    menagerie = menagerie_omx_dir()
    robot_path = menagerie / "open_manipulator_x.xml"
    if not robot_path.is_file():
        raise FileNotFoundError(
            f"Menagerie OM-X MJCF missing: {robot_path}. "
            "Run: git submodule update --init --recursive"
        )

    template = omx_scene_template(scene)
    if not template.is_file():
        raise FileNotFoundError(f"OM-X scene template missing: {template}")

    assets = (menagerie / "assets").resolve()
    robot = robot_path.read_text(encoding="utf-8")
    robot = robot.replace('meshdir="assets/"', f'meshdir="{assets}/"')
    robot = robot.replace(
        '<mujoco model="open_manipulator_x">',
        f'<mujoco model="{scene}">',
        1,
    )
    if scene in _PHYSICAL_GRIPPER_SCENES:
        robot = _inject_physical_gripper(robot)
    if scene in _FLOOR_PICK_SCENES:
        robot = _enable_floor_pick_contacts(robot)
    if not robot.rstrip().endswith("</mujoco>"):
        raise ValueError(f"Unexpected Menagerie MJCF footer: {robot_path}")
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


def ensure_omx_tabletop_mjcf() -> Path:
    """Build the empty-cell MJCF (SC-v13a)."""
    return ensure_omx_mjcf("omx_tabletop")


def ensure_omx_pick_place_mjcf() -> Path:
    """Build the pick-and-place MJCF with free ball + place cone (SC-v13b)."""
    return ensure_omx_mjcf("omx_pick_place")


def ensure_omx_desk_clutter_mjcf() -> Path:
    """Build the desk-clutter MJCF with mid-cell wall (SC-v13c)."""
    return ensure_omx_mjcf("omx_desk_clutter")


def ensure_omx_wall_maze_mjcf() -> Path:
    """Build the Γ-wall maze MJCF (SC-v13d retract → climb → place)."""
    return ensure_omx_mjcf("omx_wall_maze")
