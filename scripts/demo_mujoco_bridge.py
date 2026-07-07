#!/usr/bin/env python3
"""Interactive demo of the MuJoCo bridge core (T10-03).

Simulates a short velocity-command sequence in pure Python — no ROS required.

Example::

    pip install -e ".[dev]"
    python3 scripts/demo_mujoco_bridge.py
"""

from __future__ import annotations

import numpy as np

from fret.ros.mujoco_bridge import make_mujoco_bridge_core


def main() -> None:
    """Step the PPP MuJoCo bridge with a simple pick-like motion."""
    core = make_mujoco_bridge_core("ppp", "ppp_warehouse")
    print("MuJoCo bridge core demo (T10-03)")
    print(f"  mjcf: {core.mjcf_path.name}")
    print(f"  mujoco runtime: {core.has_mujoco_runtime}")
    print(f"  joints: {core.joint_names}")
    print(f"  start q: {core.get_positions().round(3)}")

    dt = 0.02
    segments = [
        ("lower Z", np.array([0.0, 0.0, -0.5])),
        ("raise Z", np.array([0.0, 0.0, 0.5])),
        ("traverse +X", np.array([0.8, 0.0, 0.0])),
        ("traverse +Y", np.array([0.0, 0.4, 0.0])),
        ("stop", np.array([0.0, 0.0, 0.0])),
    ]

    for label, cmd in segments:
        for _ in range(25):
            core.step(cmd, dt)
        q = core.get_positions()
        print(f"  {label:12s} cmd={cmd} -> q={q.round(3)}")

    print("Done.")


if __name__ == "__main__":
    main()
