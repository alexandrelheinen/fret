"""SO-101 MJCF helpers (v2.0 tabletop scene).

Mirrors ``fret.mjcf.omx``: the vendored MJCF sets ``meshdir="assets"``
relative to ``third_party/robotstudio_so101/so101.xml``. Including it from
another directory drops the meshdir, so this module materializes a
loadable scene under ``src/fret/mjcf/.generated/`` with an absolute
meshdir, the same way ``fret.mjcf.omx.ensure_omx_mjcf`` does for OM-X.
"""

from __future__ import annotations

import re
from pathlib import Path

_VENDORED_REL = Path("third_party/robotstudio_so101")
_SUPPORTED_SCENES: frozenset[str] = frozenset({"so101_tabletop"})


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


def ensure_so101_mjcf(scene: str = "so101_tabletop") -> Path:
    """Build a loadable SO-101 scene MJCF with resolved mesh paths.

    Args:
        scene: Template stem (``so101_tabletop``).

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
