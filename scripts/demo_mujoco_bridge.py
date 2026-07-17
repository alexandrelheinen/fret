#!/usr/bin/env python3
"""Interactive demo of the MuJoCo bridge core (T10-03).

Simulates a short velocity-command sequence in pure Python — no ROS required.

Example::

    pip install -e ".[dev]"
    python3 scripts/demo_mujoco_bridge.py
"""

from __future__ import annotations

import numpy as np

from fret.ros.mujoco_bridge import make_dubins_race_bridge_core


def main() -> None:
    """Step the Dubins race MuJoCo bridge with a short SE(2) motion."""
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([6.0, 6.0, 0.0]),
        initial_sst=np.array([6.0, 6.4, 0.0]),
        initial_dummy=np.array([6.0, 5.6, 0.0]),
    )
    print("MuJoCo bridge core demo (T10-03)")
    print(f"  mjcf: {core.mjcf_path.name}")
    print(f"  mujoco runtime: {core.has_mujoco_runtime}")
    print(f"  start rrt: {core.get_rrt_pose().round(3)}")
    print(f"  start sst: {core.get_sst_pose().round(3)}")

    poses = [
        ("advance RRT +X", (8.0, 6.0, 0.0), (6.0, 6.4, 0.0)),
        ("advance SST +X", (8.0, 6.0, 0.0), (8.0, 6.4, 0.0)),
        ("yaw RRT", (8.0, 6.0, 0.5), (8.0, 6.4, 0.0)),
    ]

    for label, rrt, sst in poses:
        core.set_rrt_pose(rrt)
        core.set_sst_pose(sst)
        print(
            f"  {label:16s} "
            f"rrt={core.get_rrt_pose().round(3)} "
            f"sst={core.get_sst_pose().round(3)}"
        )

    print("Done.")


if __name__ == "__main__":
    main()
