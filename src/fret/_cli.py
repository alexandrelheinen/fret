"""CLI entry points for the FRET toolkit.

Provides the ``fretsim`` command installed via ``pyproject.toml`` scripts.
"""

from __future__ import annotations

import argparse
import subprocess
import sys


def fretsim() -> None:
    """Launch the FRET SITL simulation pipeline.

    Thin wrapper around ``ros2 launch fret sitl.py`` that exposes
    ``--model`` and ``--scenario`` flags; any unrecognised arguments are
    forwarded verbatim to the launch system.

    Example::

        fretsim --model scara --scenario obstacle_avoidance
        fretsim --model scara --ros-args -p planning_timeout:=5.0
    """
    parser = argparse.ArgumentParser(
        prog="fretsim",
        description=(
            "FRET SITL simulation launcher. "
            "Alias for `ros2 launch fret sitl.py` with ergonomic flags."
        ),
    )
    parser.add_argument(
        "--model",
        default="scara",
        help="Robot model name (default: %(default)s)",
    )
    parser.add_argument(
        "--scenario",
        default="static_reach",
        help=(
            "Scenario YAML stem under config/scenarios/ "
            "(default: %(default)s)"
        ),
    )
    args, extra = parser.parse_known_args()

    cmd = [
        "ros2",
        "launch",
        "fret",
        "sitl.py",
        f"model:={args.model}",
        f"scenario:={args.scenario}",
        *extra,
    ]
    sys.exit(subprocess.call(cmd))
