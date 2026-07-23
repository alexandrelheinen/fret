"""Showcase early-stop: clip export must not wait for both agents at goal."""

from __future__ import annotations

from fret.scenario.dubins_race_runner import DubinsRaceRunner
from fret.scenario.planner_rng import SHOWCASE_PLANNER_RNG_SEED


def test_max_sim_time_stops_before_race_timeout() -> None:
    """``max_sim_time_s`` ends the loop even when agents are still racing."""
    result = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=False,
        planner_rng_seed=SHOWCASE_PLANNER_RNG_SEED,
        max_sim_time_s=1.0,
    )
    assert result.race_duration_s <= 1.0 + 1e-6
    assert len(result.rrt_pose_history) >= 2
    assert len(result.sst_pose_history) >= 2
    # Full warehouse race is far longer than 1 s; early-stop must not finish.
    assert result.both_reached_goal is False


def test_simulate_dubins_poses_honours_duration_early_stop() -> None:
    """``simulate_dubins_race_poses(duration_s=…)`` must not run to goal."""
    import scripts.render_mujoco as rm

    rrt, sst, _dummy, sim_time_s = rm.simulate_dubins_race_poses(
        "dubins_race",
        duration_s=2.0,
        fps=20,
        physics_mode=False,
    )
    assert sim_time_s <= 2.0 + 1e-6
    assert len(rrt) == max(2, int(round(2.0 * 20)))
    assert len(sst) == len(rrt)
