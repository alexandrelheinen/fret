"""OpenMANIPULATOR-Y MJCF helpers (v1.2.4 tabletop / pick-place cells).

Same materialization pattern as :mod:`fret.mjcf.omx` — resolve Menagerie
``meshdir``, inject finger pads + adhesion for physics grasp on ``rh_r2`` /
``rh_l2``.

Pad local offsets sit on the **inner** fingertip faces (toward the grasp
volume). Outward ±Y offsets leave the pads outside the fingers so the ball
never contacts them.
"""

from __future__ import annotations

import re
from pathlib import Path

_MENAGERIE_REL = Path("third_party/robotis_mujoco_menagerie/robotis_omy")
_SUPPORTED_SCENES: frozenset[str] = frozenset(
    {
        "omy_tabletop",
        "omy_pick_place",
        "omy_clutter",
    }
)
_PHYSICAL_GRIPPER_SCENES: frozenset[str] = frozenset(
    {"omy_pick_place", "omy_clutter"}
)
_CONE_MESH_PLACEHOLDER = 'file="assets/cone.obj"'

# Inner fingertip pads (rh_r2 / rh_l2 open along ±Y). Sized for Ø86 mm ball.
# contype/conaffinity bit 4 matches the ball; do **not** include pedestal bit 2
# or pads weld to the stand and block lift.
_PAD_RIGHT = (
    '                      <geom name="pad_right" type="box" '
    'size="0.020 0.012 0.022" pos="0.0 -0.010 0.020"\n'
    '                            friction="4.0 1.5 0.2" solref="0.01 1" '
    'solimp="0.95 0.99 0.001"\n'
    '                            condim="6" contype="4" conaffinity="4" '
    'rgba="0.15 0.15 0.15 1" group="3"/>\n'
)
_PAD_LEFT = (
    '                      <geom name="pad_left" type="box" '
    'size="0.020 0.012 0.022" pos="0.0 0.010 0.020"\n'
    '                            friction="4.0 1.5 0.2" solref="0.01 1" '
    'solimp="0.95 0.99 0.001"\n'
    '                            condim="6" contype="4" conaffinity="4" '
    'rgba="0.15 0.15 0.15 1" group="3"/>\n'
)
_ADHESION = (
    '    <adhesion name="grip_right" body="rh_r2" '
    'ctrlrange="0 1" gain="40"/>\n'
    '    <adhesion name="grip_left" body="rh_l2" '
    'ctrlrange="0 1" gain="40"/>\n'
)
_GRIPPER_FORCE_OLD = '<position kp="50" dampratio="1" forcerange="-3.5 3.5"/>'
_GRIPPER_FORCE_NEW = '<position kp="100" dampratio="1" forcerange="-15 15"/>'


def menagerie_omy_dir() -> Path:
    """Return the Menagerie OpenMANIPULATOR-Y model directory."""
    return Path(__file__).resolve().parents[3] / _MENAGERIE_REL


def omy_scene_template(scene: str) -> Path:
    """Return the committed MJCF template for ``scene``."""
    if scene not in _SUPPORTED_SCENES:
        raise ValueError(f"Unsupported OMY scene template: {scene!r}")
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
    """Add finger pads + adhesion actuators for physics grasp."""
    r2_anchor = '<joint name="rh_r2" class="Gripper_mimic"/>\n'
    l2_anchor = '<joint name="rh_l2" class="Gripper_mimic_pos"/>\n'
    if r2_anchor not in robot_xml or l2_anchor not in robot_xml:
        raise ValueError("Menagerie OMY gripper joints missing for pad inject")
    if 'name="pad_right"' not in robot_xml:
        robot_xml = robot_xml.replace(r2_anchor, r2_anchor + _PAD_RIGHT, 1)
    if 'name="pad_left"' not in robot_xml:
        robot_xml = robot_xml.replace(l2_anchor, l2_anchor + _PAD_LEFT, 1)
    grip_act = (
        '<position class="Gripper" name="Gripper" '
        'joint="rh_r1" inheritrange="1"/>\n'
    )
    if 'name="grip_right"' not in robot_xml:
        if grip_act not in robot_xml:
            raise ValueError("Menagerie Gripper actuator missing for adhesion")
        robot_xml = robot_xml.replace(grip_act, grip_act + _ADHESION, 1)
    if _GRIPPER_FORCE_OLD in robot_xml:
        robot_xml = robot_xml.replace(
            _GRIPPER_FORCE_OLD, _GRIPPER_FORCE_NEW, 1
        )
    robot_xml = robot_xml.replace(
        'ctrl="0 0 0 0 0 0 0"',
        'ctrl="0 0 0 0 0 0 0 0 0"',
        1,
    )
    return robot_xml


def ensure_omy_mjcf(scene: str = "omy_tabletop") -> Path:
    """Build a loadable OMY scene MJCF with resolved mesh paths."""
    menagerie = menagerie_omy_dir()
    robot_path = menagerie / "omy.xml"
    if not robot_path.is_file():
        raise FileNotFoundError(
            f"Menagerie OMY MJCF missing: {robot_path}. "
            "Run: git submodule update --init --recursive"
        )

    template = omy_scene_template(scene)
    if not template.is_file():
        raise FileNotFoundError(f"OMY scene template missing: {template}")

    assets = (menagerie / "assets").resolve()
    robot = robot_path.read_text(encoding="utf-8")
    robot = robot.replace('meshdir="assets/"', f'meshdir="{assets}/"')
    robot = robot.replace(
        '<mujoco model="omy">',
        f'<mujoco model="{scene}">',
        1,
    )
    if scene in _PHYSICAL_GRIPPER_SCENES:
        robot = _inject_physical_gripper(robot)
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


def ensure_omy_tabletop_mjcf() -> Path:
    """Build the empty-cell MJCF (SC-v14a)."""
    return ensure_omy_mjcf("omy_tabletop")


def ensure_omy_pick_place_mjcf() -> Path:
    """Build the pedestal pick + cone place MJCF (SC-v14b)."""
    return ensure_omy_mjcf("omy_pick_place")


def ensure_omy_clutter_mjcf() -> Path:
    """Build the cluttered pick-place MJCF (SC-v14c)."""
    return ensure_omy_mjcf("omy_clutter")
