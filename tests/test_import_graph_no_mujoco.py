"""FR-HW-61: hardware import graph must not require MuJoCo.

The Raspberry Pi 5 runtime installs the ``hw`` extra only. Importing
``fret.hardware``, ``fret.robots``, ``fret.control`` (minus ``*_sim``),
and ``fret.vision`` must succeed when ``mujoco`` is absent.
"""

from __future__ import annotations

import subprocess
import sys


_PROBE = r"""
import sys

class _BlockMujoco:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "mujoco" or fullname.startswith("mujoco."):
            raise ImportError("mujoco must not be required (FR-HW-61)")
        return None

sys.meta_path.insert(0, _BlockMujoco())

import fret.hardware  # noqa: F401
import fret.robots  # noqa: F401
import fret.robots.variants  # noqa: F401
import fret.control  # noqa: F401
import fret.vision  # noqa: F401
import fret.control.kinematics_so101  # noqa: F401

# Construct SO-101 kinematics (pure NumPy) without MuJoCo.
from fret.robots.registry import get_default_registry

kin = get_default_registry().get_kinematics("so101")
assert kin is not None
print("import_graph_ok")
"""


def test_hardware_import_graph_does_not_need_mujoco_fr_hw_61() -> None:
    """When mujoco is blocked, the Pi-side import graph still loads."""
    result = subprocess.run(
        [sys.executable, "-c", _PROBE],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    assert "import_graph_ok" in result.stdout
