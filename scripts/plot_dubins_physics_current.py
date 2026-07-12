#!/usr/bin/env python3
"""Plot current Dubins physics showcase trajectories (post PR #102 fix)."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from fret.scenario.dubins_race_runner import DubinsRaceRunner
from fret.scenario.planner_rng import SHOWCASE_PLANNER_RNG_SEED

OUT = Path("/opt/cursor/artifacts/dubins_physics_evidence")
GOAL_XY = (74.0, 74.0)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    kin = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=False,
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
    )
    phys = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=True,
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
    )

    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5), sharex=True, sharey=True)

    for ax, result, title in (
        (
            axes[0],
            kin,
            f"Kinematic (seed={SHOWCASE_PLANNER_RNG_SEED})",
        ),
        (
            axes[1],
            phys,
            f"Physics SITL (seed={SHOWCASE_PLANNER_RNG_SEED})",
        ),
    ):
        rrt = np.asarray(result.rrt_pose_history)
        sst = np.asarray(result.sst_pose_history)
        dummy = np.asarray(result.dummy_pose_history)
        ax.plot(rrt[:, 0], rrt[:, 1], color="#4477cc", lw=2.2, label="RRT*")
        ax.plot(sst[:, 0], sst[:, 1], color="#44aa66", lw=2.2, label="SST")
        ax.plot(
            dummy[:, 0],
            dummy[:, 1],
            color="#888888",
            lw=1.4,
            ls="--",
            label="dummy",
        )
        ax.scatter([rrt[0, 0]], [rrt[0, 1]], c="#4477cc", s=80, zorder=5)
        ax.scatter([GOAL_XY[0]], [GOAL_XY[1]], c="#cc3333", s=120, marker="*", zorder=5)
        subtitle = (
            f"finish {max(result.rrt_time_to_goal_s or 0, result.sst_time_to_goal_s or 0):.1f}s · "
            f"min_clr {result.min_obstacle_clearance_m:+.2f}m · "
            f"goal={'yes' if result.both_reached_goal else 'no'}"
        )
        ax.set_title(f"{title}\n{subtitle}", fontsize=11)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_xlim(0, 80)
        ax.set_ylim(0, 80)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="lower right", fontsize=9)

    fig.suptitle(
        "Dubins race — clearance_margin 1.0 m + physics planning bump 0.15 m",
        fontsize=13,
        fontweight="bold",
        y=1.02,
    )
    fig.tight_layout()
    out = OUT / "07_current_physics_trajectories.png"
    fig.savefig(out, dpi=150, facecolor="white")
    plt.close(fig)
    print(f"Wrote {out}")
    print(
        f"physics: goal={phys.both_reached_goal} "
        f"min_clr={phys.min_obstacle_clearance_m:+.3f}m "
        f"dur={phys.race_duration_s:.1f}s"
    )


if __name__ == "__main__":
    main()
