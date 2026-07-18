"""OpenMANIPULATOR-X MJCF helpers (v1.3 tabletop cell).

Menagerie sets ``meshdir="assets/"`` relative to ``open_manipulator_x.xml``.
Including that file from another directory drops the meshdir, so we materialize
a loadable scene under ``src/fret/mjcf/.generated/`` with an absolute meshdir
and the empty-cell worldbody from ``omx_tabletop.xml``.
"""

from __future__ import annotations

import re
from pathlib import Path

_MENAGERIE_REL = Path(
    "third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x"
)
_GENERATED_NAME = "omx_tabletop.xml"


def menagerie_omx_dir() -> Path:
    """Return the Menagerie OpenMANIPULATOR-X model directory."""
    return Path(__file__).resolve().parents[3] / _MENAGERIE_REL


def omx_tabletop_template() -> Path:
    """Return the committed empty-cell scene fragment/template."""
    return Path(__file__).resolve().parent / _GENERATED_NAME


def _scene_additions(template_text: str) -> str:
    """Extract option/visual/asset/worldbody blocks from the tabletop template."""
    # Drop the include + outer mujoco wrapper; keep the cell décor blocks.
    text = re.sub(
        r"<include\s+file=\"[^\"]+\"\s*/>\s*",
        "",
        template_text,
        count=1,
    )
    text = re.sub(r"^<mujoco\b[^>]*>\s*", "", text.strip())
    text = re.sub(r"</mujoco>\s*$", "", text.strip())
    # Strip the leading HTML comment block if present.
    text = re.sub(r"<!--.*?-->\s*", "", text, count=1, flags=re.DOTALL)
    return text.strip()


def ensure_omx_tabletop_mjcf() -> Path:
    """Build a loadable OM-X empty-cell MJCF with resolved mesh paths.

    Returns:
        Path to ``src/fret/mjcf/.generated/omx_tabletop.xml``.

    Raises:
        FileNotFoundError: If the Menagerie submodule or template is missing.
    """
    menagerie = menagerie_omx_dir()
    robot_path = menagerie / "open_manipulator_x.xml"
    if not robot_path.is_file():
        raise FileNotFoundError(
            f"Menagerie OM-X MJCF missing: {robot_path}. "
            "Run: git submodule update --init --recursive"
        )

    template = omx_tabletop_template()
    if not template.is_file():
        raise FileNotFoundError(f"OM-X tabletop template missing: {template}")

    assets = (menagerie / "assets").resolve()
    robot = robot_path.read_text(encoding="utf-8")
    robot = robot.replace('meshdir="assets/"', f'meshdir="{assets}/"')
    robot = robot.replace(
        '<mujoco model="open_manipulator_x">',
        '<mujoco model="omx_tabletop">',
        1,
    )
    if not robot.rstrip().endswith("</mujoco>"):
        raise ValueError(f"Unexpected Menagerie MJCF footer: {robot_path}")
    robot_body = robot.rstrip()[: -len("</mujoco>")].rstrip()
    additions = _scene_additions(template.read_text(encoding="utf-8"))

    dest_dir = Path(__file__).resolve().parent / ".generated"
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / _GENERATED_NAME
    dest.write_text(
        robot_body
        + "\n\n  "
        + additions.replace("\n", "\n  ")
        + "\n</mujoco>\n",
        encoding="utf-8",
    )
    return dest
