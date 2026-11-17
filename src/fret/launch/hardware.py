"""Launch: hardware-in-the-loop pipeline — not available before v2.x.

The hardware bridge (:class:`fret.hardware.bridge_node.BridgeNode`) is an
unimplemented stub (FR-HW-01..03, superseded in v2.0, pending
redefinition) with no registered ``bridge_node`` executable in
``pyproject.toml`` ``[project.scripts]`` or ``CMakeLists.txt``. Running
``ros2 launch fret hardware.py`` previously tried to launch that missing
executable and failed with an unhelpful "executable not found" error;
this now fails fast with a clear message instead. The v2.x hardware
target is defined starting v2.0 (see ``docs/hardware/README.md``).
"""

from __future__ import annotations

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Raise: hardware launch is not wired up before v2.x.

    Raises:
        RuntimeError: Always, naming the missing bridge implementation and
            pointing to ``docs/hardware/README.md``.
    """
    raise RuntimeError(
        "fret hardware.py is not available before v2.x: the hardware "
        "bridge (fret.hardware.bridge_node.BridgeNode) is an "
        "unimplemented stub (FR-HW-01..03, superseded in v2.0, pending "
        "redefinition). See docs/hardware/README.md."
    )
