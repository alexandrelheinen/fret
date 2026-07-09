#!/usr/bin/env python3
"""Demo PPP C-space checker against warehouse obstacles (T10-05 / T10-08).

Pure Python — no ROS required.

Example::

    pip install -e ".[dev]"
    python3 scripts/demo_ppp_checker.py
"""

from __future__ import annotations

import numpy as np

from fret.config_loader import load_algorithm_config
from fret.control import Kinematics
from fret.control.grasp_magnet import parse_grasp_config
from fret.planning import (
    BoxObstacle,
    build_box_obstacle_occupancy,
    load_ppp_warehouse_obstacles,
    make_cspace_checker,
)


def main() -> None:
    """Probe free and colliding PPP configurations in the warehouse."""
    contact_radius = float(load_algorithm_config("planning/ppp.yml")["contact_radius"])
    boxes = load_ppp_warehouse_obstacles()
    occ = build_box_obstacle_occupancy(boxes, contact_radius=contact_radius)
    kin = Kinematics("ppp")

    checker_ee = make_cspace_checker(kin, occ, include_cargo=False)
    checker_cargo = make_cspace_checker(
        kin,
        occ,
        include_cargo=True,
        grasp_config=parse_grasp_config(
            load_algorithm_config("grasp/ppp_warehouse.yml")
        ),
    )

    probes = [
        ("open aisle", np.array([5.0, 5.0, 4.0])),
        ("inside barrier", np.array([16.0, 10.0, 1.0])),
        ("above barrier", np.array([16.0, 10.0, 4.0])),
    ]

    print(f"PPP warehouse checker demo ({len(boxes)} obstacles)")
    for label, q in probes:
        ee_free = checker_ee.is_collision_free(q)
        cargo_free = checker_cargo.is_collision_free(q)
        print(
            f"  {label:20s} q={q}  EE={'free' if ee_free else 'HIT':4s}  "
            f"cargo={'free' if cargo_free else 'HIT':4s}"
        )

    # FR-GSP-02: cargo envelope can collide when EE alone is free.
    slab = BoxObstacle(0.0, 0.0, 1.0, 60.0, 20.0, 1.8)
    slab_occ = build_box_obstacle_occupancy([slab], contact_radius=contact_radius)
    slab_ee = make_cspace_checker(kin, slab_occ, include_cargo=False)
    slab_cargo = make_cspace_checker(
        kin,
        slab_occ,
        include_cargo=True,
        grasp_config=parse_grasp_config(
            load_algorithm_config("grasp/ppp_warehouse.yml")
        ),
    )
    q_slab = np.array([5.0, 5.0, 0.6])
    print("  cargo-envelope slab (FR-GSP-02):")
    print(
        f"    q={q_slab}  EE={'free' if slab_ee.is_collision_free(q_slab) else 'HIT':4s}  "
        f"cargo={'free' if slab_cargo.is_collision_free(q_slab) else 'HIT':4s}"
    )
    print("Done.")


if __name__ == "__main__":
    main()
