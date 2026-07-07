#!/usr/bin/env python3
"""Interactive demo of the PPP prismatic velocity controller (T10-09).

Runs a short tracking sequence in pure Python — no ROS required.

Example::

    pip install -e ".[dev]"
    python3 scripts/demo_ppp_controller.py
"""

from __future__ import annotations

import tempfile

import numpy as np
import yaml

from fret.control.controller_node import make_controller_node
from fret.control.kinematics import Kinematics


def main() -> None:
    """Track a gantry move and print EE error samples."""
    kin = Kinematics("ppp")
    q_start = np.array([0.395, 0.195, 0.145])
    q_goal = np.array([0.4, 0.2, 0.15])

    demo_config = {
        "/**": {
            "ros__parameters": {
                "kp": 1.5,
                "max_joint_velocity": [3.0, 3.0, 1.5],
                "fault_threshold": 0.010,
                "update_rate": 50.0,
                "ticks_per_waypoint": 10_000,
            }
        }
    }
    with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as fh:
        yaml.dump(demo_config, fh)
        demo_path = fh.name

    ctrl = make_controller_node("ppp", demo_path)
    ctrl.set_trajectory([q_start.copy(), q_goal.copy()])
    ctrl._trajectory_index = 1

    print("PPP prismatic controller demo (T10-09)")
    print(f"  start: {q_start}")
    print(f"  goal:  {q_goal}")
    print(
        f"  rate:  {ctrl.update_rate} Hz, fault: {ctrl.fault_threshold * 1000:.0f} mm"
    )

    dt = 1.0 / ctrl.update_rate
    q = q_start.copy()
    for step in range(int(2.0 / dt)):
        q_dot = ctrl.compute_prismatic_command(kin, q)
        q = q + q_dot * dt
        if step % 25 == 0:
            err_mm = float(np.linalg.norm(q - q_goal)) * 1000.0
            print(
                f"  t={step * dt:4.2f}s  q={q.round(3)}  "
                f"state={ctrl.state.name}  ee_err={err_mm:5.1f} mm"
            )

    final_err_mm = float(np.linalg.norm(q - q_goal)) * 1000.0
    print(f"  final EE error: {final_err_mm:.2f} mm ({ctrl.state.name})")
    print("Done.")


if __name__ == "__main__":
    main()
