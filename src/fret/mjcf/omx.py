"""OpenMANIPULATOR-X MJCF helpers (v1.3 tabletop / pick-place cells).

Menagerie sets ``meshdir="assets/"`` relative to ``open_manipulator_x.xml``.
Including that file from another directory drops the meshdir, so we materialize
a loadable scene under ``src/fret/mjcf/.generated/`` with an absolute meshdir.
"""

from __future__ import annotations

import re
from pathlib import Path

_MENAGERIE_REL = Path(
    "third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x"
)
_SUPPORTED_SCENES: frozenset[str] = frozenset(
    {"omx_tabletop", "omx_pick_place"}
)


def menagerie_omx_dir() -> Path:
    """Return the Menagerie OpenMANIPULATOR-X model directory."""
    return Path(__file__).resolve().parents[3] / _MENAGERIE_REL


def omx_scene_template(scene: str) -> Path:
    """Return the committed MJCF template for ``scene``."""
    if scene not in _SUPPORTED_SCENES:
        raise ValueError(f"Unsupported OM-X scene template: {scene!r}")
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


def ensure_omx_mjcf(scene: str = "omx_tabletop") -> Path:
    """Build a loadable OM-X scene MJCF with resolved mesh paths.

    Args:
        scene: Template stem (``omx_tabletop`` or ``omx_pick_place``).

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
    """Build the pick-and-place MJCF with free box (SC-v13b)."""
    return ensure_omx_mjcf("omx_pick_place")
