#!/usr/bin/env python3
"""Interactive demo of the PPP magnetic grasp FSM (T10-04).

Runs a scripted pick-and-place sequence in pure Python — no ROS required.

Example::

    pip install -e ".[dev]"
    python3 scripts/demo_grasp.py
"""

from __future__ import annotations

import numpy as np

from fret.config_loader import load_algorithm_config
from fret.control import GraspState, MagneticGraspFSM
from fret.control.grasp_magnet import parse_grasp_config

_BOX = np.array([3.0, 2.0, 1.0])
_GOAL = np.array([10.0, 3.0, 1.0])


def main() -> None:
    """Run a minimal weld → transport → release sequence."""
    grasp_cfg = parse_grasp_config(load_algorithm_config("grasp/magnetic.yml"))
    fsm = MagneticGraspFSM(grasp_cfg)
    print("Magnetic grasp FSM demo (FR-GSP-01–04)")
    print(f"  initial state: {fsm.state.name}")

    fsm.begin_transport()
    print(f"  after begin_transport: {fsm.state.name}")

    steps = [
        ("approach (far)", _BOX + np.array([1.0, 0.0, 0.0])),
        ("capture", _BOX.copy()),
        ("transport", _BOX + np.array([4.0, 0.5, 0.0])),
        ("at goal", _GOAL.copy()),
        ("retract", _GOAL + np.array([1.0, 0.0, 0.5])),
    ]

    for label, ee in steps:
        state = fsm.update(ee, _BOX, _GOAL)
        welded = "welded" if fsm.is_welded else "free"
        cargo = fsm.cargo_position
        print(
            f"  {label:16s} ee={ee.round(2)} → {state.name:9s} "
            f"({welded}) cargo={cargo.round(2)}"
        )

    corners = fsm.cargo_corners()
    print(f"  cargo_corners after release: shape {corners.shape}")
    print("Done.")


if __name__ == "__main__":
    main()
