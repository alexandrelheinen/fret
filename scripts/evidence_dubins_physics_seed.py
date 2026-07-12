#!/usr/bin/env python3
"""Generate technical evidence images for Dubins physics showcase seeding."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import mujoco
import numpy as np
from matplotlib.gridspec import GridSpec

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from fret.scenario.dubins_race_runner import DubinsRaceRunner
from fret.scenario.planner_rng import (
    SHOWCASE_PLANNER_RNG_SEED,
    deterministic_planner_rng,
)
from render_mujoco import (
    _apply_dubins_poses,
    _resample_pose_history,
    resolve_mjcf_path,
    resolve_scenario_simulation_dt,
)

OUT = Path("/opt/cursor/artifacts/dubins_physics_evidence")
MJCF = resolve_mjcf_path("dubins", "dubins_race", None)
GOAL_XY = (74.0, 74.0)
NOMINAL_CLIP_S = 35.0
FPS = 30


def _run_physics(*, seed: int | None) -> object:
    runner = DubinsRaceRunner()
    kwargs: dict[str, object] = {
        "record_poses": True,
        "physics_mode": True,
    }
    if seed is not None:
        kwargs["planner_rng_seed"] = seed
    return runner.run(**kwargs)


def _find_unseeded_failure(max_tries: int = 8) -> object:
    for _ in range(max_tries):
        result = _run_physics(seed=None)
        if not result.both_reached_goal:
            return result
    raise RuntimeError("could not sample an unseeded failure within max_tries")


def _xy(history: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return history[:, 0], history[:, 1]


def _mujoco_snapshot(
    rrt: np.ndarray,
    sst: np.ndarray,
    dummy: np.ndarray,
    *,
    title: str,
    path: Path,
) -> None:
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
    _apply_dubins_poses(mujoco, model, data, rrt, sst, dummy)
    renderer.update_scene(data, camera=cam_id)
    rgb = renderer.render()
    renderer.close()
    plt.figure(figsize=(12.8, 7.2), dpi=100)
    plt.imshow(rgb)
    plt.axis("off")
    plt.title(title, fontsize=14, fontweight="bold", pad=12)
    plt.tight_layout()
    plt.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close()


def plot_trajectories(failed: object, seeded: object) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5), sharex=True, sharey=True)
    for ax, result, title, subtitle in (
        (
            axes[0],
            failed,
            "Unseeded planner (CI failure)",
            f"timeout {failed.race_duration_s:.0f}s · "
            f"xtrack {failed.max_cross_track_error_m:.1f}m",
        ),
        (
            axes[1],
            seeded,
            f"Seeded planner (seed={SHOWCASE_PLANNER_RNG_SEED})",
            f"finish {max(seeded.rrt_time_to_goal_s, seeded.sst_time_to_goal_s):.1f}s · "
            f"xtrack {seeded.max_cross_track_error_m:.1f}m",
        ),
    ):
        rrt = np.asarray(result.rrt_pose_history)
        sst = np.asarray(result.sst_pose_history)
        dummy = np.asarray(result.dummy_pose_history)
        ax.plot(*_xy(rrt), color="#4477cc", lw=2.2, label="RRT*")
        ax.plot(*_xy(sst), color="#44aa66", lw=2.2, label="SST")
        ax.plot(*_xy(dummy), color="#888888", lw=1.4, ls="--", label="dummy")
        ax.scatter([rrt[0, 0]], [rrt[0, 1]], c="#4477cc", s=80, zorder=5)
        ax.scatter([GOAL_XY[0]], [GOAL_XY[1]], c="#cc3333", s=120, marker="*", zorder=5)
        ax.set_title(f"{title}\n{subtitle}", fontsize=11)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_xlim(0, 80)
        ax.set_ylim(0, 80)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="lower right", fontsize=9)
    fig.suptitle(
        "Same physics controller · different ARC sample trees",
        fontsize=13,
        fontweight="bold",
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(OUT / "01_trajectory_xy_comparison.png", dpi=150, facecolor="white")
    plt.close(fig)


def plot_time_compression(failed: object) -> None:
    sim_dt = resolve_scenario_simulation_dt("dubins_race")
    rrt = np.asarray(failed.rrt_pose_history)
    t = np.arange(rrt.shape[0]) * sim_dt
    n_frames = max(2, int(round(NOMINAL_CLIP_S * FPS)))

    fig = plt.figure(figsize=(14, 8))
    gs = GridSpec(2, 1, height_ratios=[1.2, 1], hspace=0.35)

    ax0 = fig.add_subplot(gs[0])
    ax0.plot(t, rrt[:, 0], color="#4477cc", lw=2, label="RRT* x(t) — real sim")
    ax0.axvline(NOMINAL_CLIP_S, color="#cc3333", ls="--", lw=1.5)
    ax0.axvline(failed.race_duration_s, color="#666666", ls=":", lw=1.5)
    ax0.annotate(
        f"nominal clip end ({NOMINAL_CLIP_S:.0f}s)",
        xy=(NOMINAL_CLIP_S, 8),
        fontsize=9,
        color="#cc3333",
    )
    ax0.annotate(
        f"race timeout ({failed.race_duration_s:.0f}s)",
        xy=(failed.race_duration_s - 35, rrt[-1, 0] - 4),
        fontsize=9,
        color="#666666",
    )
    ax0.set_xlim(0, failed.race_duration_s)
    ax0.set_ylim(0, 80)
    ax0.set_xlabel("simulation time [s]")
    ax0.set_ylabel("x position [m]")
    ax0.set_title(
        "Failed physics run never reaches goal — but pose log keeps growing",
        fontweight="bold",
    )
    ax0.grid(True, alpha=0.25)
    ax0.legend(loc="lower right")

    ax1 = fig.add_subplot(gs[1])
    video_t = np.linspace(0, NOMINAL_CLIP_S, n_frames)
    mapped_sim_t = video_t * (failed.race_duration_s / NOMINAL_CLIP_S)
    ax1.plot(
        video_t,
        mapped_sim_t,
        color="#aa2222",
        lw=3,
        label="old export mapping (resample full log)",
    )
    ax1.plot(
        video_t,
        video_t,
        color="#228822",
        lw=2,
        ls="--",
        label="honest mapping (success only)",
    )
    ax1.fill_between(
        video_t,
        video_t,
        mapped_sim_t,
        color="#aa2222",
        alpha=0.15,
        label="hidden speedup gap",
    )
    speedup = failed.race_duration_s / NOMINAL_CLIP_S
    ax1.text(
        18,
        210,
        f"{speedup:.1f}× apparent speedup\nwhen 300s log → 35s MP4",
        fontsize=11,
        color="#aa2222",
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="#aa2222"),
    )
    ax1.set_xlabel("viewer timeline [s] (MP4 duration)")
    ax1.set_ylabel("source sim time [s] (pose log index)")
    ax1.set_title(
        "Time-compression export: viewer clock ≠ physics clock",
        fontweight="bold",
    )
    ax1.set_xlim(0, NOMINAL_CLIP_S)
    ax1.set_ylim(0, failed.race_duration_s + 10)
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="upper left", fontsize=9)

    fig.savefig(OUT / "02_time_compression_manipulation.png", dpi=150, facecolor="white")
    plt.close(fig)


def plot_failure_rate() -> None:
    trials = 8
    unseeded_ok = 0
    for _ in range(trials):
        if _run_physics(seed=None).both_reached_goal:
            unseeded_ok += 1
    seeded_ok = 0
    for _ in range(trials):
        if _run_physics(seed=SHOWCASE_PLANNER_RNG_SEED).both_reached_goal:
            seeded_ok += 1

    labels = ["unseeded\n(release path)", f"seed={SHOWCASE_PLANNER_RNG_SEED}\n(PR #102)"]
    success = [unseeded_ok / trials * 100, seeded_ok / trials * 100]
    colors = ["#cc5544", "#44aa66"]

    fig, ax = plt.subplots(figsize=(8, 5.5))
    bars = ax.bar(labels, success, color=colors, width=0.55)
    ax.set_ylim(0, 105)
    ax.set_ylabel("physics race success rate [%]")
    ax.set_title(
        f"Dubins physics both_reached_goal over {trials} runs each",
        fontweight="bold",
    )
    ax.axhline(100, color="#888888", ls=":", lw=1)
    for bar, pct in zip(bars, success, strict=True):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 2,
            f"{pct:.0f}%",
            ha="center",
            fontweight="bold",
        )
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT / "03_failure_rate_bars.png", dpi=150, facecolor="white")
    plt.close(fig)


def render_snapshots(failed: object, seeded: object) -> None:
    sim_dt = resolve_scenario_simulation_dt("dubins_race")
    n_frames = max(2, int(round(NOMINAL_CLIP_S * FPS)))

    def _hist(result: object) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.asarray(result.rrt_pose_history),
            np.asarray(result.sst_pose_history),
            np.asarray(result.dummy_pose_history),
        )

    fail_rrt, fail_sst, fail_dummy = _hist(failed)
    seed_rrt, seed_sst, seed_dummy = _hist(seeded)

    # Honest t=35s sample during failed run (agents barely moved).
    idx_fail_real = min(fail_rrt.shape[0] - 1, int(round(NOMINAL_CLIP_S / sim_dt)))
    _mujoco_snapshot(
        fail_rrt[idx_fail_real],
        fail_sst[idx_fail_real],
        fail_dummy[idx_fail_real],
        title=(
            f"FAILED run at real t={NOMINAL_CLIP_S:.0f}s sim "
            f"(x≈{fail_rrt[idx_fail_real,0]:.0f}m — not at goal)"
        ),
        path=OUT / "04_snapshot_failed_real_t35.png",
    )

    # Dishonest last frame if we resample the full 300s log into 35s MP4.
    resampled_rrt = _resample_pose_history(fail_rrt, n_frames)
    resampled_sst = _resample_pose_history(fail_sst, n_frames)
    resampled_dummy = _resample_pose_history(fail_dummy, n_frames)
    _mujoco_snapshot(
        resampled_rrt[-1],
        resampled_sst[-1],
        resampled_dummy[-1],
        title=(
            "OLD export: last MP4 frame = t=300s poses squeezed into 35s clip "
            f"(x≈{resampled_rrt[-1,0]:.0f}m — looks like progress)"
        ),
        path=OUT / "05_snapshot_failed_compressed_last_frame.png",
    )

    seed_frames = _resample_pose_history(seed_rrt, n_frames)
    _mujoco_snapshot(
        seed_frames[-1],
        _resample_pose_history(seed_sst, n_frames)[-1],
        _resample_pose_history(seed_dummy, n_frames)[-1],
        title=(
            f"SEEDED success: last MP4 frame after real finish "
            f"(x≈{seed_frames[-1,0]:.0f}m — both agents at goal)"
        ),
        path=OUT / "06_snapshot_seeded_success_last_frame.png",
    )


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    print("Sampling unseeded failure …")
    failed = _find_unseeded_failure()
    print(
        f"  failed: duration={failed.race_duration_s:.1f}s "
        f"xtrack={failed.max_cross_track_error_m:.1f}m"
    )
    print("Running seeded success …")
    seeded = _run_physics(seed=SHOWCASE_PLANNER_RNG_SEED)
    assert seeded.both_reached_goal
    print(
        f"  seeded: duration={seeded.race_duration_s:.1f}s "
        f"xtrack={seeded.max_cross_track_error_m:.1f}m"
    )

    plot_trajectories(failed, seeded)
    plot_time_compression(failed)
    print("Measuring failure rates (16 physics runs) …")
    plot_failure_rate()
    print("Rendering MuJoCo overview snapshots …")
    render_snapshots(failed, seeded)
    print(f"Wrote evidence to {OUT}")


if __name__ == "__main__":
    main()
