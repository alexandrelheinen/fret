#!/usr/bin/env python3
"""Headless MuJoCo video renderer for FRET showcase scenarios.

Renders MP4s of the PPP gantry traversing a warehouse preview scene
without requiring a ROS runtime.  Intended for README / article assets
(releases.md V10-6, T10-07).

Example::

    python3 scripts/render_mujoco.py --scenario ppp_warehouse -o /tmp/v10.mp4
    ./scripts/video.sh --model ppp --duration 30
    ./scripts/video.sh --all-cameras --output-dir /tmp/showcase

Dependencies (not required for core FRET algorithms)::

    pip install mujoco imageio imageio-ffmpeg
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

# PPP joint limits at 1:5 scale (see mjcf/ppp_warehouse.xml).
_PPP_LIMITS = np.array(
    [
        [0.0, 12.0],
        [0.0, 5.0],
        [0.0, 3.0],
    ],
    dtype=np.float64,
)

# MJCF cargo body offset from EE (see ppp_warehouse.xml cargo body pos).
_PPP_CARGO_EE_OFFSET_Z: float = -0.34

# Per-segment dwell times so pick/place Z motion is visible in showcase clips.
_PPP_PICK_DESCEND_S: float = 4.0
_PPP_PICK_ASCEND_S: float = 3.5
_PPP_PLACE_DESCEND_S: float = 4.0
_PPP_PLACE_ASCEND_S: float = 3.0
# Legacy fallback when --collision-backend is not set.
_PPP_WAREHOUSE_WAYPOINTS: list[npt.NDArray[np.float64]] = [
    np.array([2.0, 1.0, 2.4]),
    np.array([2.0, 1.0, 0.95]),
    np.array([2.0, 1.0, 2.2]),
    np.array([4.5, 1.0, 2.2]),
    np.array([6.0, 2.6, 2.2]),
    np.array([9.5, 3.2, 2.2]),
    np.array([10.5, 2.8, 0.95]),
    np.array([10.5, 2.8, 2.65]),
]

# Release showcase exports: one oblique overview + one follow POV per scenario.
# Extra MJCF cameras remain available via explicit --camera flags.
RELEASE_SHOWCASE_CAMERAS: tuple[str, ...] = ("overview", "follow")

_PPP_WAREHOUSE_CAMERAS: tuple[str, ...] = RELEASE_SHOWCASE_CAMERAS
_DUBINS_RACE_CAMERAS: tuple[str, ...] = RELEASE_SHOWCASE_CAMERAS

_DUBINS_JOINT_NAMES: tuple[tuple[str, str, str], ...] = (
    ("rrt_joint_x", "rrt_joint_y", "rrt_joint_yaw"),
    ("sst_joint_x", "sst_joint_y", "sst_joint_yaw"),
)

_DUBINS_LIMITS = np.array(
    [
        [0.0, 80.0],
        [0.0, 80.0],
        [-np.pi, np.pi],
    ],
    dtype=np.float64,
)

_EGL_APT_HINT = (
    "sudo apt install libegl1 libegl-mesa0 libgles2 libgl1 "
    "libgl1-mesa-dri libosmesa6"
)


@dataclass(frozen=True)
class ShowcaseTiming:
    """Simulation vs render timing for real-time post-processing."""

    sim_time_s: float
    render_duration_s: float

    @property
    def real_time_factor(self) -> float:
        """Playback speedup needed so video duration matches ``sim_time_s``."""
        if self.sim_time_s <= 0.0:
            return 1.0
        return self.render_duration_s / self.sim_time_s


@dataclass(frozen=True)
class RenderResult:
    """One rendered showcase clip."""

    camera: str
    path: Path
    frame_mean: float
    timing: ShowcaseTiming


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def resolve_mjcf_path(
    model: str, scenario: str, mjcf_override: Path | None
) -> Path:
    """Return the MJCF file path for the requested model/scenario pair.

    Args:
        model: Robot model name (e.g. ``ppp``).
        scenario: Scenario stem (e.g. ``ppp_warehouse``).
        mjcf_override: Optional explicit MJCF path.

    Returns:
        Resolved path to an existing MJCF file.

    Raises:
        FileNotFoundError: If no MJCF file can be located.
        ValueError: If model/scenario combination is unsupported.
    """
    if mjcf_override is not None:
        if not mjcf_override.is_file():
            raise FileNotFoundError(f"MJCF not found: {mjcf_override}")
        return mjcf_override

    if model == "ppp" and scenario in {"ppp_warehouse", "ppp"}:
        candidate = _project_root() / "src/fret/mjcf/ppp_warehouse.xml"
        if candidate.is_file():
            return candidate

    if model == "dubins" and scenario in {"dubins_race", "dubins"}:
        candidate = _project_root() / "src/fret/mjcf/dubins_race.xml"
        if candidate.is_file():
            return candidate

    raise ValueError(
        f"Unsupported model/scenario combination: model={model!r}, "
        f"scenario={scenario!r}"
    )


def list_showcase_cameras(
    mjcf_path: Path,
    *,
    scenario: str = "ppp_warehouse",
) -> list[str]:
    """Return ordered showcase camera names for a scenario MJCF.

    Args:
        mjcf_path: Path to the MJCF scene file.
        scenario: Scenario stem used for fallback defaults.

    Returns:
        Camera names in release-export order (overview + follow).
    """
    root = ET.parse(mjcf_path).getroot()
    mjcf_cameras = {
        name
        for node in root.iter("camera")
        if (name := node.get("name")) is not None
    }
    if mjcf_cameras:
        release = [
            cam for cam in RELEASE_SHOWCASE_CAMERAS if cam in mjcf_cameras
        ]
        if release:
            return release
        raise ValueError(
            f"MJCF {mjcf_path} has no release showcase cameras "
            f"{RELEASE_SHOWCASE_CAMERAS}; found {sorted(mjcf_cameras)}"
        )

    if scenario in {"ppp_warehouse", "ppp"}:
        return list(_PPP_WAREHOUSE_CAMERAS)

    if scenario in {"dubins_race", "dubins"}:
        return list(_DUBINS_RACE_CAMERAS)

    raise ValueError(f"No showcase cameras found in MJCF: {mjcf_path}")


def showcase_output_name(scenario: str, camera: str) -> str:
    """Build the canonical showcase MP4 filename for a scenario/camera pair."""
    return f"{scenario}_{camera}.mp4"


def _ensure_fret_importable() -> None:
    """Add ``src/`` to ``sys.path`` so the renderer can call the planner."""
    src = _project_root() / "src"
    src_str = str(src)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)


def _scenario_config_path(scenario: str) -> Path:
    return _project_root() / "src/fret/config/scenarios" / f"{scenario}.yml"


def resolve_scenario_duration(
    scenario: str,
    *,
    default_s: float = 30.0,
) -> float:
    """Return the scenario YAML ``duration`` for showcase clip length."""
    _ensure_fret_importable()
    from fret.sitl_config import load_scenario_parameters

    try:
        params = load_scenario_parameters(_scenario_config_path(scenario))
    except (FileNotFoundError, OSError, ValueError):
        return default_s
    raw = params.get("duration", default_s)
    try:
        duration = float(raw)
    except (TypeError, ValueError):
        return default_s
    return duration if duration > 0.0 else default_s


def resolve_scenario_simulation_dt(
    scenario: str,
    *,
    default_s: float = 0.05,
) -> float:
    """Return scenario ``simulation_dt`` when present (Dubins race)."""
    _ensure_fret_importable()
    from fret.sitl_config import load_scenario_parameters

    try:
        params = load_scenario_parameters(_scenario_config_path(scenario))
    except (FileNotFoundError, OSError, ValueError):
        return default_s
    raw = params.get("simulation_dt", default_s)
    try:
        dt = float(raw)
    except (TypeError, ValueError):
        return default_s
    return dt if dt > 0.0 else default_s


def estimate_ppp_path_duration_s(
    waypoints: list[npt.NDArray[np.float64]],
    *,
    max_joint_velocity: npt.NDArray[np.float64] | None = None,
) -> float:
    """Estimate prismatic transit time using per-axis velocity limits."""
    if len(waypoints) < 2:
        return 0.0
    max_vel = (
        np.asarray(max_joint_velocity, dtype=np.float64)
        if max_joint_velocity is not None
        else np.array([3.0, 3.0, 1.5], dtype=np.float64)
    )
    total = 0.0
    for idx in range(len(waypoints) - 1):
        delta = np.abs(waypoints[idx + 1] - waypoints[idx])
        axis_times = delta / np.maximum(max_vel, 1e-6)
        total += float(np.max(axis_times))
    return total


def apply_realtime_video_speedup(
    video_path: Path,
    *,
    timing: ShowcaseTiming,
    label: str,
) -> None:
    """Accelerate a showcase MP4 so on-screen motion matches simulation time."""
    rtf = timing.real_time_factor
    print(
        f"[showcase] {label}: sim_time={timing.sim_time_s:.3f}s, "
        f"render_duration={timing.render_duration_s:.3f}s, "
        f"real_time_factor={rtf:.4f}"
    )
    if abs(rtf - 1.0) < 0.02:
        print(f"[showcase] {label}: already real-time (skip ffmpeg)")
        return

    tmp_path = video_path.with_name(f"{video_path.stem}_rtf{video_path.suffix}")
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        str(video_path),
        "-filter:v",
        f"setpts=PTS/{rtf:.8f}",
        "-an",
        str(tmp_path),
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "ffmpeg is required for real-time showcase post-processing. "
            "Install with: sudo apt install ffmpeg"
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"ffmpeg real-time speedup failed for {video_path}:\n{exc.stderr}"
        ) from exc

    tmp_path.replace(video_path)
    print(
        f"[showcase] {label}: wrote real-time video "
        f"({timing.sim_time_s:.3f}s playback)"
    )


def postprocess_showcase_results(
    results: list[RenderResult],
    *,
    enabled: bool = True,
) -> list[RenderResult]:
    """Apply ffmpeg speedup to each rendered clip when enabled."""
    if not enabled:
        return results
    for result in results:
        apply_realtime_video_speedup(
            result.path,
            timing=result.timing,
            label=f"{result.path.stem}",
        )
    return results


def plan_ppp_warehouse_path(
    scenario: str = "ppp_warehouse",
    *,
    collision_backend: str = "mujoco",
    planner_algorithm: str = "rrt_star",
    mjcf_path: Path | None = None,
) -> list[npt.NDArray[np.float64]]:
    """Plan a collision-free PPP warehouse transit path.

    Args:
        scenario: Scenario stem (``ppp_warehouse``).
        collision_backend: ``analytic`` or ``mujoco``.
        planner_algorithm: ARCO planner stem (``rrt_star`` or ``sst``).
        mjcf_path: Optional MJCF override for MuJoCo collision checks.

    Returns:
        Planner waypoints in joint space.

    Raises:
        RuntimeError: If planning does not succeed.
    """
    _ensure_fret_importable()
    from fret.interfaces import PlanningRequest, PlanningStatus
    from fret.control.grasp_magnet import parse_grasp_config
    from fret.planning.planner_node import PlannerNode
    from fret.planning.ppp_obstacles import load_preview_workspace_bounds
    from fret.scenario.ppp_warehouse_runner import _build_occupancy_adapter
    from fret.sitl_config import load_scenario_parameters

    scenario_path = _scenario_config_path(scenario)
    params = load_scenario_parameters(scenario_path)
    occ_adapter, box_occ = _build_occupancy_adapter(None)
    preview_bounds = load_preview_workspace_bounds(None)
    resolved_mjcf = mjcf_path or resolve_mjcf_path("ppp", scenario, None)
    resolved_planner = str(params.get("planner_algorithm", planner_algorithm))
    resolved_collision = str(
        params.get("collision_backend", collision_backend)
    )
    grasp_cfg = parse_grasp_config(dict(params.get("grasp", {})))
    plan_include_cargo = bool(params.get("plan_include_cargo", True))

    planner = PlannerNode(
        model="ppp",
        occupancy_adapter=occ_adapter,
        occupancy=box_occ,
        collision_backend=resolved_collision,  # type: ignore[arg-type]
        planner_algorithm=resolved_planner,  # type: ignore[arg-type]
        include_cargo=plan_include_cargo,
        grasp_config=grasp_cfg,
        scenario=str(params.get("scenario_id", scenario)),
        workspace_bounds=preview_bounds,
        mjcf_path=resolved_mjcf,
    )

    start = np.asarray(params["start_configuration"], dtype=np.float64)
    goal = np.asarray(params["goal_configuration"], dtype=np.float64)
    req = PlanningRequest(
        start_configuration=start.copy(),
        goal_configuration=goal.copy(),
        planning_timeout=float(params.get("planning_timeout", 30.0)),
        scenario_id=str(params.get("scenario_id", scenario)),
    )
    result = planner.plan(req)
    if result.status != PlanningStatus.SUCCESS or len(result.path) < 2:
        raise RuntimeError(
            "Showcase planning failed: "
            f"status={result.status}, error={result.error_code}"
        )
    return [np.asarray(q, dtype=np.float64) for q in result.path]


def build_showcase_waypoints(
    scenario: str = "ppp_warehouse",
    *,
    collision_backend: str = "mujoco",
    planner_algorithm: str = "rrt_star",
    mjcf_path: Path | None = None,
) -> list[npt.NDArray[np.float64]]:
    """Build a pick-transit-place path using the planning stack.

    Inserts explicit vertical pick, cruise, place, and depart segments so
    showcase videos show full Z-axis motion and cargo transport.
    """
    _ensure_fret_importable()
    from fret.control.grasp_magnet import parse_grasp_config
    from fret.sitl_config import load_scenario_parameters

    transit = plan_ppp_warehouse_path(
        scenario,
        collision_backend=collision_backend,
        planner_algorithm=planner_algorithm,
        mjcf_path=mjcf_path,
    )
    params = load_scenario_parameters(_scenario_config_path(scenario))
    grasp_cfg = parse_grasp_config(dict(params.get("grasp", {})))
    start, goal = transit[0], transit[-1]
    box_half_z = float(grasp_cfg.box_half_extent[2])
    pick_ee_z = box_half_z - _PPP_CARGO_EE_OFFSET_Z
    cruise_z = float(max(start[2], goal[2], 2.2))

    waypoints: list[npt.NDArray[np.float64]] = [
        start.copy(),
        np.array([start[0], start[1], pick_ee_z]),
        np.array([start[0], start[1], cruise_z]),
    ]
    for q in transit[1:-1]:
        waypoints.append(np.array([q[0], q[1], cruise_z]))
    waypoints.append(np.array([goal[0], goal[1], cruise_z]))
    waypoints.append(np.array([goal[0], goal[1], pick_ee_z]))
    waypoints.append(goal.copy())
    return waypoints


def pick_place_segment_durations(
    waypoints: list[npt.NDArray[np.float64]],
) -> list[float]:
    """Return per-segment durations with extra time on vertical pick/place."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")

    durations: list[float] = []
    for idx in range(len(waypoints) - 1):
        q0 = waypoints[idx]
        q1 = waypoints[idx + 1]
        delta = np.abs(q1 - q0)
        dz = float(delta[2])
        if dz > 0.2:
            descending = float(q1[2]) < float(q0[2])
            if descending and idx == 0:
                durations.append(_PPP_PICK_DESCEND_S)
            elif descending:
                durations.append(_PPP_PLACE_DESCEND_S)
            elif idx == 1:
                durations.append(_PPP_PICK_ASCEND_S)
            elif idx == len(waypoints) - 2:
                durations.append(_PPP_PLACE_ASCEND_S)
            else:
                durations.append(max(dz / 1.5, 1.0))
        else:
            durations.append(float(np.max(delta / np.array([3.0, 3.0, 1.5]))))
    return durations


def interpolate_segmented_waypoints(
    waypoints: list[npt.NDArray[np.float64]],
    segment_durations_s: list[float],
    fps: int,
) -> tuple[npt.NDArray[np.float64], float]:
    """Sample a trajectory with explicit per-segment timing."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")
    if len(segment_durations_s) != len(waypoints) - 1:
        raise ValueError("segment_durations_s must match waypoint segments")

    sim_time_s = float(sum(segment_durations_s))
    n_frames = max(2, int(round(sim_time_s * fps)))
    if n_frames < len(waypoints):
        n_frames = len(waypoints)

    frame_budgets = [
        max(1, int(round(duration / sim_time_s * n_frames)))
        for duration in segment_durations_s
    ]
    budget_sum = sum(frame_budgets)
    while budget_sum > n_frames:
        for idx in range(len(frame_budgets) - 1, -1, -1):
            if frame_budgets[idx] > 1 and budget_sum > n_frames:
                frame_budgets[idx] -= 1
                budget_sum -= 1
    while budget_sum < n_frames:
        frame_budgets[-2] += 1
        budget_sum += 1

    trajectory = np.zeros((n_frames, waypoints[0].shape[0]), dtype=np.float64)
    frame_idx = 0
    for seg_idx, seg_frames in enumerate(frame_budgets):
        q0 = waypoints[seg_idx]
        q1 = waypoints[seg_idx + 1]
        for local in range(seg_frames):
            if frame_idx >= n_frames:
                break
            alpha = 0.0 if seg_frames == 1 else local / (seg_frames - 1)
            smooth = alpha * alpha * (3.0 - 2.0 * alpha)
            trajectory[frame_idx] = (1.0 - smooth) * q0 + smooth * q1
            frame_idx += 1

    trajectory[-1] = waypoints[-1]
    return (
        np.clip(trajectory, _PPP_LIMITS[:, 0], _PPP_LIMITS[:, 1]),
        sim_time_s,
    )


def resolve_showcase_waypoints(
    scenario: str,
    mjcf_path: Path,
    *,
    collision_backend: str | None,
    planner_algorithm: str = "rrt_star",
    waypoints: list[npt.NDArray[np.float64]] | None,
) -> list[npt.NDArray[np.float64]]:
    """Return explicit, planned, or legacy showcase waypoints."""
    if waypoints is not None:
        return waypoints
    if collision_backend is not None:
        return build_showcase_waypoints(
            scenario,
            collision_backend=collision_backend,
            planner_algorithm=planner_algorithm,
            mjcf_path=mjcf_path,
        )
    return list(_PPP_WAREHOUSE_WAYPOINTS)


def build_pruned_dense_waypoints(
    path: list[npt.NDArray[np.float64]],
    *,
    scenario: str = "ppp_warehouse",
) -> list[npt.NDArray[np.float64]]:
    """Prune and densify a planner path for controller tracking."""
    _ensure_fret_importable()
    from fret.control.grasp_magnet import parse_grasp_config
    from fret.control.kinematics import Kinematics
    from fret.planning.cspace_checker import make_cspace_checker
    from fret.planning.planner_node import _CSpaceOccupancy
    from fret.planning.ppp_obstacles import (
        build_box_obstacle_occupancy,
        load_ppp_warehouse_preview_obstacles,
        load_preview_workspace_bounds,
    )
    from fret.planning.trajectory_generator import TrajectoryGenerator
    from fret.sitl_config import load_scenario_parameters

    params = load_scenario_parameters(_scenario_config_path(scenario))
    grasp_cfg = parse_grasp_config(dict(params.get("grasp", {})))
    plan_include_cargo = bool(params.get("plan_include_cargo", True))
    preview_bounds = load_preview_workspace_bounds(None)
    boxes = load_ppp_warehouse_preview_obstacles(None)

    kin = Kinematics("ppp")
    occ = build_box_obstacle_occupancy(boxes)
    checker = make_cspace_checker(
        kin,
        occ,
        include_cargo=plan_include_cargo,
        grasp_config=grasp_cfg,
        collision_backend="mujoco",
        scenario=scenario,
        workspace_bounds=preview_bounds,
        mjcf_path=resolve_mjcf_path("ppp", scenario, None),
    )
    traj_gen = TrajectoryGenerator(kin)
    traj_gen.set_collision_context(
        _CSpaceOccupancy(checker),
        np.full(kin.dof, 0.1, dtype=np.float64),
    )
    traj = traj_gen.process(path)
    return [np.asarray(pt.positions, dtype=np.float64) for pt in traj.points]


def _resample_joint_history(
    history: npt.NDArray[np.float64],
    n_frames: int,
) -> npt.NDArray[np.float64]:
    """Uniformly resample a joint-space execution log to ``n_frames`` poses."""
    if history.shape[0] == 0:
        raise ValueError("history must contain at least one sample")
    if history.shape[0] == 1:
        return np.repeat(history, n_frames, axis=0)
    indices = np.linspace(0, history.shape[0] - 1, n_frames)
    resampled = np.zeros((n_frames, history.shape[1]), dtype=np.float64)
    for i, idx in enumerate(indices):
        lo = int(np.floor(idx))
        hi = min(lo + 1, history.shape[0] - 1)
        alpha = float(idx - lo)
        resampled[i] = (1.0 - alpha) * history[lo] + alpha * history[hi]
    return np.clip(resampled, _PPP_LIMITS[:, 0], _PPP_LIMITS[:, 1])


def _assert_showcase_trajectory_moves(
    trajectory: npt.NDArray[np.float64],
) -> None:
    """Fail fast when a release clip would show a static gantry."""
    spans = trajectory.max(axis=0) - trajectory.min(axis=0)
    if float(spans[0]) < 5.0 or float(spans[1]) < 0.5:
        raise RuntimeError(
            "Showcase trajectory lacks horizontal transit "
            f"(joint spans X={spans[0]:.2f}, Y={spans[1]:.2f}, Z={spans[2]:.2f})"
        )


def build_showcase_trajectory(
    path_waypoints: list[npt.NDArray[np.float64]],
    *,
    scenario: str,
    duration_s: float | None,
    fps: int,
    collision_backend: str | None,
    use_tracking: bool,
    start: npt.NDArray[np.float64] | None = None,
) -> tuple[npt.NDArray[np.float64], float]:
    """Build joint samples for a showcase clip.

    Release renders use joint-space interpolation over the dense planned path
    (``--no-tracking``), matching the headless demo validated in cloud-agent
    setup.  Controller tracking remains available for executed-motion previews.

    Returns:
        Trajectory array ``(n_frames, 3)`` and simulated motion time [s].
    """
    if collision_backend is not None and use_tracking:
        dense = build_pruned_dense_waypoints(
            path_waypoints,
            scenario=scenario,
        )
        trajectory, sim_time_s = simulate_tracked_trajectory(
            dense,
            duration_s=duration_s,
            fps=fps,
            start=start if start is not None else path_waypoints[0],
        )
    else:
        interpolate_path = path_waypoints
        if collision_backend is not None:
            interpolate_path = build_pruned_dense_waypoints(
                path_waypoints,
                scenario=scenario,
            )
        if scenario in {"ppp_warehouse", "ppp"} and len(path_waypoints) >= 4:
            segment_durations = pick_place_segment_durations(path_waypoints)
            trajectory, sim_time_s = interpolate_segmented_waypoints(
                path_waypoints,
                segment_durations,
                fps,
            )
        else:
            sim_time_s = estimate_ppp_path_duration_s(interpolate_path)
            render_duration_s = (
                sim_time_s if duration_s is None else float(duration_s)
            )
            trajectory = interpolate_waypoints(
                interpolate_path,
                render_duration_s,
                fps,
            )
    _assert_showcase_trajectory_moves(trajectory)
    return trajectory, sim_time_s


def simulate_tracked_trajectory(
    waypoints: list[npt.NDArray[np.float64]],
    *,
    duration_s: float | None,
    fps: int,
    start: npt.NDArray[np.float64] | None = None,
) -> tuple[npt.NDArray[np.float64], float]:
    """Simulate PPP P-control tracking and sample frames for rendering.

    Follows the same per-axis velocity law as ``PPPControllerNode`` and
    ``PPPWarehouseRunner`` so release videos show executed motion rather
    than idealized joint interpolation.

    The full dense waypoint sequence is tracked to completion, then
    resampled to the requested frame count.  Sampling on simulation clock
    alone would stall the gantry in early vertical approach segments for
    the full clip duration.

    Args:
        waypoints: Dense joint references from pruning + interpolation.
        duration_s: Target clip duration in seconds.
        fps: Output frame rate.
        start: Optional initial joint configuration.

    Returns:
        Tuple of trajectory ``(n_frames, 3)`` and simulated motion time [s].
    """
    _ensure_fret_importable()
    from fret.control.controller_ppp import PPPControllerNode
    from fret.ros.mujoco_bridge import make_mujoco_bridge_core

    ctrl = PPPControllerNode(
        str(_project_root() / "src/fret/config/controllers/ppp.yml")
    )
    q0 = (
        np.asarray(start, dtype=np.float64)
        if start is not None
        else waypoints[0].copy()
    )
    bridge = make_mujoco_bridge_core(
        "ppp", "ppp_warehouse", initial_positions=q0
    )
    dt = 1.0 / float(ctrl.update_rate)
    convergence_m = 0.004
    max_inner_steps = 50

    history: list[npt.NDArray[np.float64]] = [bridge.get_positions().copy()]

    for q_ref in waypoints:
        for _ in range(max_inner_steps):
            q = bridge.get_positions()
            joint_error = q_ref - q
            q_dot = np.clip(
                ctrl._kp * joint_error,
                -ctrl._max_joint_velocity,
                ctrl._max_joint_velocity,
            )
            bridge.step(q_dot, dt)
            history.append(bridge.get_positions().copy())
            if float(np.linalg.norm(joint_error)) <= convergence_m:
                break

    q_ref = waypoints[-1]
    for _ in range(max_inner_steps):
        q = bridge.get_positions()
        joint_error = q_ref - q
        if float(np.linalg.norm(joint_error)) <= convergence_m:
            break
        q_dot = np.clip(
            ctrl._kp * joint_error,
            -ctrl._max_joint_velocity,
            ctrl._max_joint_velocity,
        )
        bridge.step(q_dot, dt)
        history.append(bridge.get_positions().copy())

    sim_time_s = max(0.0, (len(history) - 1) * dt)
    render_duration_s = (
        sim_time_s if duration_s is None else float(duration_s)
    )
    n_frames = max(2, int(round(render_duration_s * fps)))
    return (
        _resample_joint_history(
            np.asarray(history, dtype=np.float64), n_frames
        ),
        sim_time_s,
    )


def interpolate_waypoints(
    waypoints: list[npt.NDArray[np.float64]],
    duration_s: float,
    fps: int,
) -> npt.NDArray[np.float64]:
    """Sample a smooth joint-space trajectory through waypoints.

    Uses cubic ease-in-out segments between consecutive waypoints so the
    gantry accelerates and decelerates at each segment boundary.

    Args:
        waypoints: List of joint configurations, each shape ``(3,)``.
        duration_s: Total clip duration in seconds.
        fps: Output frame rate.

    Returns:
        Array of shape ``(n_frames, 3)`` with joint positions per frame.
    """
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")

    n_frames = max(2, int(round(duration_s * fps)))
    segment_count = len(waypoints) - 1
    trajectory = np.zeros((n_frames, 3), dtype=np.float64)

    for frame_idx in range(n_frames):
        global_t = frame_idx / (n_frames - 1)
        seg_float = global_t * segment_count
        seg_idx = min(int(seg_float), segment_count - 1)
        local_t = seg_float - seg_idx
        # Smoothstep ease-in-out within each segment.
        alpha = local_t * local_t * (3.0 - 2.0 * local_t)
        q0 = waypoints[seg_idx]
        q1 = waypoints[seg_idx + 1]
        trajectory[frame_idx] = (1.0 - alpha) * q0 + alpha * q1

    return np.clip(trajectory, _PPP_LIMITS[:, 0], _PPP_LIMITS[:, 1])


def _require_mujoco() -> tuple[object, object]:
    """Import MuJoCo and imageio, raising a clear error when missing."""
    try:
        import mujoco  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit(
            "MuJoCo is required for video rendering.\n"
            "Install with: pip install mujoco imageio imageio-ffmpeg"
        ) from exc
    except AttributeError as exc:
        if "eglQueryString" in str(exc):
            raise SystemExit(
                "MuJoCo EGL rendering requires system OpenGL/EGL libraries.\n"
                f"On Ubuntu 24.04: {_EGL_APT_HINT}"
            ) from exc
        raise
    try:
        import imageio.v3 as iio  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit(
            "imageio is required for MP4 export.\n"
            "Install with: pip install imageio imageio-ffmpeg"
        ) from exc
    return mujoco, iio


def _set_joint_position(
    mujoco: object,
    model: object,
    data: object,
    joint_name: str,
    value: float,
) -> None:
    """Write a scalar position into a joint's qpos slot."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint not found in MJCF: {joint_name}")
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr] = value


def _set_slide_joint(
    mujoco: object,
    model: object,
    data: object,
    joint_name: str,
    value: float,
) -> None:
    """Write a scalar position into a slide joint's qpos slot."""
    _set_joint_position(mujoco, model, data, joint_name, value)


def _resample_pose_history(
    history: npt.NDArray[np.float64],
    n_frames: int,
) -> npt.NDArray[np.float64]:
    """Uniformly resample SE(2) pose logs to ``n_frames`` samples."""
    if history.shape[0] == 0:
        raise ValueError("history must contain at least one sample")
    if history.shape[0] == 1:
        return np.repeat(history, n_frames, axis=0)
    indices = np.linspace(0, history.shape[0] - 1, n_frames)
    resampled = np.zeros((n_frames, history.shape[1]), dtype=np.float64)
    for i, idx in enumerate(indices):
        lo = int(np.floor(idx))
        hi = min(lo + 1, history.shape[0] - 1)
        alpha = float(idx - lo)
        resampled[i] = (1.0 - alpha) * history[lo] + alpha * history[hi]
    resampled[:, 2] = np.arctan2(
        np.sin(resampled[:, 2]),
        np.cos(resampled[:, 2]),
    )
    return np.clip(
        resampled,
        _DUBINS_LIMITS[:, 0],
        _DUBINS_LIMITS[:, 1],
    )


def simulate_dubins_race_poses(
    scenario: str = "dubins_race",
    *,
    duration_s: float | None,
    fps: int,
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], float]:
    """Run the SC-v11 race and resample agent poses for video export.

    Returns:
        RRT* poses, SST poses, and simulated race duration [s].
    """
    _ensure_fret_importable()
    from fret.scenario.dubins_race_runner import DubinsRaceRunner

    result = DubinsRaceRunner().run(record_poses=True)
    if not result.both_reached_goal:
        raise RuntimeError(
            "Dubins race simulation failed before both agents reached goal"
        )

    rrt_hist = np.asarray(result.rrt_pose_history, dtype=np.float64)
    sst_hist = np.asarray(result.sst_pose_history, dtype=np.float64)
    sim_dt = resolve_scenario_simulation_dt(scenario)
    if (
        result.rrt_time_to_goal_s is not None
        and result.sst_time_to_goal_s is not None
    ):
        sim_time_s = float(
            max(result.rrt_time_to_goal_s, result.sst_time_to_goal_s)
        )
    else:
        sim_time_s = float(max(0, len(rrt_hist) - 1)) * sim_dt

    render_duration_s = (
        sim_time_s if duration_s is None else float(duration_s)
    )
    n_frames = max(2, int(round(render_duration_s * fps)))
    return (
        _resample_pose_history(rrt_hist, n_frames),
        _resample_pose_history(sst_hist, n_frames),
        sim_time_s,
    )


def _assert_dubins_race_moves(
    rrt_poses: npt.NDArray[np.float64],
    sst_poses: npt.NDArray[np.float64],
) -> None:
    """Fail fast when a release clip would show static agents."""
    rrt_span = rrt_poses.max(axis=0) - rrt_poses.min(axis=0)
    sst_span = sst_poses.max(axis=0) - sst_poses.min(axis=0)
    if float(rrt_span[0]) < 5.0 or float(sst_span[0]) < 5.0:
        raise RuntimeError(
            "Dubins race clip lacks horizontal transit "
            f"(RRT* span={rrt_span[:2]}, SST span={sst_span[:2]})"
        )


def render_dubins_race_showcase_videos(
    mjcf_path: Path,
    output_dir: Path,
    *,
    scenario: str = "dubins_race",
    cameras: list[str] | None = None,
    output_names: dict[str, str] | None = None,
    duration_s: float | None = None,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    realtime_postprocess: bool = True,
) -> list[RenderResult]:
    """Render dual-agent Dubins race MP4s (V11-2 / V11-4)."""
    mujoco, _iio = _require_mujoco()
    camera_names = (
        cameras
        if cameras is not None
        else list_showcase_cameras(mjcf_path, scenario=scenario)
    )
    if not camera_names:
        raise ValueError("At least one showcase camera is required")

    rrt_poses, sst_poses, sim_time_s = simulate_dubins_race_poses(
        scenario,
        duration_s=duration_s,
        fps=fps,
    )
    _assert_dubins_race_moves(rrt_poses, sst_poses)
    render_duration_s = float(len(rrt_poses)) / float(fps)
    timing = ShowcaseTiming(
        sim_time_s=sim_time_s,
        render_duration_s=render_duration_s,
    )

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    for camera in camera_names:
        _assert_camera_exists(mujoco, model, camera)

    output_dir.mkdir(parents=True, exist_ok=True)
    names = output_names or {
        camera: showcase_output_name(scenario, camera)
        for camera in camera_names
    }
    paths = {camera: output_dir / names[camera] for camera in camera_names}
    writers = {
        camera: _open_video_writer(paths[camera], fps)
        for camera in camera_names
    }
    first_frames: dict[str, npt.NDArray[np.uint8]] = {}

    try:
        for rrt_q, sst_q in zip(rrt_poses, sst_poses, strict=True):
            for values, joint_names in (
                (rrt_q, _DUBINS_JOINT_NAMES[0]),
                (sst_q, _DUBINS_JOINT_NAMES[1]),
            ):
                for idx, joint_name in enumerate(joint_names):
                    _set_joint_position(
                        mujoco,
                        model,
                        data,
                        joint_name,
                        float(values[idx]),
                    )
            mujoco.mj_forward(model, data)
            for camera in camera_names:
                renderer.update_scene(data, camera=camera)
                frame = renderer.render()
                writers[camera].append_data(frame)
                if camera not in first_frames:
                    first_frames[camera] = frame.copy()
    finally:
        for writer in writers.values():
            writer.close()
        renderer.close()

    results: list[RenderResult] = []
    for camera in camera_names:
        frame = first_frames[camera]
        mean = _frame_mean(frame)
        if mean <= 1.0:
            raise RuntimeError(
                f"Camera {camera!r} render looks blank (frame mean={mean:.2f})"
            )
        results.append(
            RenderResult(
                camera=camera,
                path=paths[camera],
                frame_mean=mean,
                timing=timing,
            )
        )
    return postprocess_showcase_results(
        results,
        enabled=realtime_postprocess,
    )


def _set_geom_alpha(
    mujoco: object,
    model: object,
    geom_name: str,
    alpha: float,
) -> None:
    """Set geom RGBA alpha for show/hide toggles during pick-and-place."""
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0:
        raise ValueError(f"Geom not found in MJCF: {geom_name}")
    model.geom_rgba[geom_id][3] = float(np.clip(alpha, 0.0, 1.0))


def _set_mocap_position(
    mujoco: object,
    model: object,
    data: object,
    body_name: str,
    position: npt.NDArray[np.float64],
) -> None:
    """Place a mocap body used for floor cargo visuals."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"Mocap body not found in MJCF: {body_name}")
    mocap_id = model.body_mocapid[body_id]
    if int(mocap_id) < 0:
        raise ValueError(f"Body is not mocap-enabled: {body_name}")
    data.mocap_pos[mocap_id] = np.asarray(position, dtype=np.float64)


def _update_ppp_cargo_visuals(
    mujoco: object,
    model: object,
    data: object,
    *,
    grasp: object,
    box_anchor: npt.NDArray[np.float64],
    goal: npt.NDArray[np.float64],
    ee_position: npt.NDArray[np.float64],
    was_welded: bool,
) -> bool:
    """Drive floor vs welded cargo visuals from the magnetic grasp FSM."""
    from fret.control.grasp_magnet import GraspState

    grasp.update(ee_position, box_anchor, goal)
    if grasp.is_welded:
        _set_geom_alpha(mujoco, model, "cargo_box", 1.0)
        _set_geom_alpha(mujoco, model, "cargo_floor_box", 0.0)
        _set_mocap_position(
            mujoco,
            model,
            data,
            "cargo_floor",
            np.array([-20.0, -20.0, -5.0]),
        )
        return True

    floor_pos = (
        grasp.cargo_position
        if was_welded and grasp.state == GraspState.IDLE
        else box_anchor
    )
    _set_geom_alpha(mujoco, model, "cargo_box", 0.0)
    _set_geom_alpha(mujoco, model, "cargo_floor_box", 1.0)
    _set_mocap_position(mujoco, model, data, "cargo_floor", floor_pos)
    return was_welded


def _open_video_writer(path: Path, fps: int) -> object:
    """Open an incremental MP4 writer (low memory for multi-POV export)."""
    import imageio

    path.parent.mkdir(parents=True, exist_ok=True)
    return imageio.get_writer(
        str(path),
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
        macro_block_size=1,
    )


def _frame_mean(frame: npt.NDArray[np.uint8]) -> float:
    return float(frame.mean())


def _assert_camera_exists(mujoco: object, model: object, camera: str) -> None:
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
    if cam_id < 0:
        raise ValueError(f"Camera not found in MJCF: {camera}")


def render_video(
    mjcf_path: Path,
    output_path: Path,
    *,
    scenario: str = "ppp_warehouse",
    duration_s: float | None = None,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    camera: str = "overview",
    waypoints: list[npt.NDArray[np.float64]] | None = None,
    collision_backend: str | None = "mujoco",
    planner_algorithm: str = "rrt_star",
    use_tracking: bool = True,
    realtime_postprocess: bool = True,
) -> RenderResult:
    """Render a headless MuJoCo MP4 for the PPP warehouse preview.

    Args:
        mjcf_path: Path to the MJCF scene file.
        output_path: Destination ``.mp4`` path.
        duration_s: Clip length in seconds (default 30 for V10-6).
        fps: Frame rate.
        width: Frame width in pixels.
        height: Frame height in pixels.
        camera: Named MuJoCo camera in the MJCF.
        waypoints: Optional joint-space path; defaults to warehouse demo.

    Returns:
        Render metadata including the output path and first-frame mean.
    """
    results = render_showcase_videos(
        mjcf_path,
        output_path.parent,
        scenario=scenario,
        cameras=[camera],
        output_names={camera: output_path.name},
        duration_s=duration_s,
        fps=fps,
        width=width,
        height=height,
        waypoints=waypoints,
        collision_backend=collision_backend,
        planner_algorithm=planner_algorithm,
        use_tracking=use_tracking,
        realtime_postprocess=realtime_postprocess,
    )
    return results[0]


def render_showcase_videos(
    mjcf_path: Path,
    output_dir: Path,
    *,
    scenario: str = "ppp_warehouse",
    cameras: list[str] | None = None,
    output_names: dict[str, str] | None = None,
    duration_s: float | None = None,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    waypoints: list[npt.NDArray[np.float64]] | None = None,
    collision_backend: str | None = None,
    planner_algorithm: str = "rrt_star",
    use_tracking: bool = True,
    realtime_postprocess: bool = True,
) -> list[RenderResult]:
    """Render one MP4 per showcase camera in a single simulation pass.

    Each frame updates the robot once, then renders every requested POV.
    Frames stream directly to disk to keep memory bounded.

    Args:
        mjcf_path: Path to the MJCF scene file.
        output_dir: Directory for output MP4 files.
        scenario: Scenario stem used in default filenames.
        cameras: Camera names to export; defaults to all MJCF cameras.
        output_names: Optional per-camera filename overrides.
        duration_s: Clip length in seconds.
        fps: Frame rate.
        width: Frame width in pixels.
        height: Frame height in pixels.
        waypoints: Optional joint-space path; defaults to warehouse demo.

    Returns:
        One :class:`RenderResult` per exported camera.
    """
    if scenario in {"dubins_race", "dubins"}:
        return render_dubins_race_showcase_videos(
            mjcf_path,
            output_dir,
            scenario=scenario,
            cameras=cameras,
            output_names=output_names,
            duration_s=duration_s,
            fps=fps,
            width=width,
            height=height,
            realtime_postprocess=realtime_postprocess,
        )

    mujoco, _iio = _require_mujoco()
    camera_names = (
        cameras
        if cameras is not None
        else list_showcase_cameras(mjcf_path, scenario=scenario)
    )
    if not camera_names:
        raise ValueError("At least one showcase camera is required")

    path_waypoints = resolve_showcase_waypoints(
        scenario,
        mjcf_path,
        collision_backend=collision_backend,
        planner_algorithm=planner_algorithm,
        waypoints=waypoints,
    )
    trajectory, sim_time_s = build_showcase_trajectory(
        path_waypoints,
        scenario=scenario,
        duration_s=duration_s,
        fps=fps,
        collision_backend=collision_backend,
        use_tracking=use_tracking,
        start=path_waypoints[0],
    )
    render_duration_s = float(len(trajectory)) / float(fps)
    timing = ShowcaseTiming(
        sim_time_s=sim_time_s,
        render_duration_s=render_duration_s,
    )

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    for camera in camera_names:
        _assert_camera_exists(mujoco, model, camera)

    output_dir.mkdir(parents=True, exist_ok=True)
    names = output_names or {
        camera: showcase_output_name(scenario, camera)
        for camera in camera_names
    }
    paths = {camera: output_dir / names[camera] for camera in camera_names}
    writers = {
        camera: _open_video_writer(paths[camera], fps)
        for camera in camera_names
    }
    first_frames: dict[str, npt.NDArray[np.uint8]] = {}

    joint_names = ("joint_x", "joint_y", "joint_z")
    ppp_grasp = None
    box_anchor: npt.NDArray[np.float64] | None = None
    goal_position: npt.NDArray[np.float64] | None = None
    was_welded = False
    if scenario in {"ppp_warehouse", "ppp"}:
        _ensure_fret_importable()
        from fret.control.grasp_magnet import MagneticGraspFSM, parse_grasp_config
        from fret.sitl_config import load_scenario_parameters

        params = load_scenario_parameters(_scenario_config_path(scenario))
        grasp_cfg = parse_grasp_config(dict(params.get("grasp", {})))
        start = path_waypoints[0]
        goal_position = path_waypoints[-1].copy()
        box_anchor = np.array(
            [
                start[0],
                start[1],
                float(grasp_cfg.box_half_extent[2]),
            ],
            dtype=np.float64,
        )
        ppp_grasp = MagneticGraspFSM(grasp_cfg)
        ppp_grasp.begin_transport()

    try:
        for q in trajectory:
            for idx, name in enumerate(joint_names):
                _set_slide_joint(mujoco, model, data, name, float(q[idx]))
            if (
                ppp_grasp is not None
                and box_anchor is not None
                and goal_position is not None
            ):
                was_welded = _update_ppp_cargo_visuals(
                    mujoco,
                    model,
                    data,
                    grasp=ppp_grasp,
                    box_anchor=box_anchor,
                    goal=goal_position,
                    ee_position=q,
                    was_welded=was_welded,
                )
            mujoco.mj_forward(model, data)
            for camera in camera_names:
                renderer.update_scene(data, camera=camera)
                frame = renderer.render()
                writers[camera].append_data(frame)
                if camera not in first_frames:
                    first_frames[camera] = frame.copy()
    finally:
        for writer in writers.values():
            writer.close()
        renderer.close()

    results: list[RenderResult] = []
    for camera in camera_names:
        frame = first_frames[camera]
        mean = _frame_mean(frame)
        if mean <= 1.0:
            raise RuntimeError(
                f"Camera {camera!r} render looks blank (frame mean={mean:.2f})"
            )
        results.append(
            RenderResult(
                camera=camera,
                path=paths[camera],
                frame_mean=mean,
                timing=timing,
            )
        )

    return postprocess_showcase_results(
        results,
        enabled=realtime_postprocess,
    )


def write_showcase_timing_json(
    results: list[RenderResult],
    output_path: Path,
) -> None:
    """Persist per-clip timing metadata for release workflows."""
    import json

    payload = {
        "clips": [
            {
                "camera": result.camera,
                "file": result.path.name,
                "sim_time_s": result.timing.sim_time_s,
                "render_duration_s": result.timing.render_duration_s,
                "real_time_factor": result.timing.real_time_factor,
            }
            for result in results
        ]
    }
    output_path.write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description="Render headless MuJoCo MP4s for FRET showcase scenarios.",
    )
    parser.add_argument(
        "--model",
        default="ppp",
        help="Robot model name (default: ppp)",
    )
    parser.add_argument(
        "--scenario",
        default="ppp_warehouse",
        help="Scenario stem (default: ppp_warehouse)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output MP4 path for single-camera mode",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/tmp/fret_showcase"),
        help="Output directory for --all-cameras (default: /tmp/fret_showcase)",
    )
    parser.add_argument(
        "--mjcf",
        type=Path,
        default=None,
        help="Override MJCF path (default: resolved from model/scenario)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help=(
            "Optional clip stretch duration in seconds.  Omit to render the "
            "full simulated motion at real-time speed (default)."
        ),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate (default: 30)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Frame width (default: 1280)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Frame height (default: 720)",
    )
    parser.add_argument(
        "--camera",
        action="append",
        dest="cameras",
        metavar="NAME",
        help="MJCF camera name (repeatable; default: overview)",
    )
    parser.add_argument(
        "--all-cameras",
        action="store_true",
        help="Export every showcase camera defined in the MJCF",
    )
    parser.add_argument(
        "--collision-backend",
        choices=("analytic", "mujoco"),
        default="mujoco",
        help=(
            "Plan showcase motion with FRET collision checking "
            "(analytic or mujoco); omit for legacy hardcoded path"
        ),
    )
    parser.add_argument(
        "--planner-algorithm",
        choices=("rrt_star", "sst"),
        default="rrt_star",
        help="ARCO planner for showcase path planning (default: rrt_star)",
    )
    parser.add_argument(
        "--no-tracking",
        action="store_true",
        help="Use joint interpolation instead of controller tracking",
    )
    parser.add_argument(
        "--timing-json",
        type=Path,
        default=None,
        help="Write per-clip sim/render timing JSON (for release CI)",
    )
    parser.add_argument(
        "--no-realtime-postprocess",
        action="store_true",
        help="Skip ffmpeg speedup to match simulation time",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    mjcf_path = resolve_mjcf_path(args.model, args.scenario, args.mjcf)
    duration_s = args.duration

    use_tracking = not args.no_tracking
    realtime_postprocess = not args.no_realtime_postprocess

    if args.all_cameras:
        cameras = list_showcase_cameras(mjcf_path, scenario=args.scenario)
        results = render_showcase_videos(
            mjcf_path,
            args.output_dir,
            scenario=args.scenario,
            cameras=cameras,
            duration_s=duration_s,
            fps=args.fps,
            width=args.width,
            height=args.height,
            collision_backend=args.collision_backend,
            planner_algorithm=args.planner_algorithm,
            use_tracking=use_tracking,
            realtime_postprocess=realtime_postprocess,
        )
        for result in results:
            timing = result.timing
            print(
                f"Wrote {result.path} "
                f"(camera={result.camera}, mean={result.frame_mean:.1f}, "
                f"sim={timing.sim_time_s:.1f}s, "
                f"rtf={timing.real_time_factor:.3f})"
            )
        if args.timing_json is not None:
            write_showcase_timing_json(results, args.timing_json)
        return 0

    cameras = args.cameras or ["overview"]
    if len(cameras) == 1:
        output = args.output or Path(
            f"/tmp/{showcase_output_name(args.scenario, cameras[0])}"
        )
        result = render_video(
            mjcf_path,
            output,
            scenario=args.scenario,
            duration_s=duration_s,
            fps=args.fps,
            width=args.width,
            height=args.height,
            camera=cameras[0],
            collision_backend=args.collision_backend,
            planner_algorithm=args.planner_algorithm,
            use_tracking=use_tracking,
            realtime_postprocess=realtime_postprocess,
        )
        timing = result.timing
        print(
            f"Wrote {result.path} (sim={timing.sim_time_s:.1f}s, "
            f"rtf={timing.real_time_factor:.3f}, "
            f"mean={result.frame_mean:.1f})"
        )
        return 0

    results = render_showcase_videos(
        mjcf_path,
        args.output_dir,
        scenario=args.scenario,
        cameras=cameras,
        duration_s=duration_s,
        fps=args.fps,
        width=args.width,
        height=args.height,
        collision_backend=args.collision_backend,
        planner_algorithm=args.planner_algorithm,
        use_tracking=use_tracking,
        realtime_postprocess=realtime_postprocess,
    )
    for result in results:
        timing = result.timing
        print(
            f"Wrote {result.path} "
            f"(camera={result.camera}, mean={result.frame_mean:.1f}, "
            f"sim={timing.sim_time_s:.1f}s, "
            f"rtf={timing.real_time_factor:.3f})"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
