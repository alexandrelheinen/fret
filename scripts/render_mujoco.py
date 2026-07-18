#!/usr/bin/env python3
"""Headless MuJoCo video renderer for FRET showcase scenarios.

Renders MP4s of the Dubins race without requiring a ROS runtime.
Intended for README / article assets (releases.md V11).

Example::

    python3 scripts/render_mujoco.py --model dubins --scenario dubins_race \\
        --camera overview -o /tmp/v11.mp4 --fps 30 --width 1280 --height 720 \\
        --collision-backend mujoco --planner-algorithm rrt_star --full-duration
    ./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \\
        --output-dir /tmp/showcase --fps 30 --width 1280 --height 720 \\
        --collision-backend mujoco --planner-algorithm rrt_star --full-duration

Dependencies (not required for core FRET algorithms)::

    pip install mujoco imageio imageio-ffmpeg
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

# Release showcase exports: one oblique overview + one follow POV per scenario.
# Extra MJCF cameras remain available via explicit --camera flags.
RELEASE_SHOWCASE_CAMERAS: tuple[str, ...] = ("overview", "follow")

_DUBINS_RACE_CAMERAS: tuple[str, ...] = RELEASE_SHOWCASE_CAMERAS

# OM-X empty-cell / pick-place proof: top-down + oblique overview.
_OMX_RELEASE_CAMERAS: tuple[str, ...] = ("topdown", "overview")
_OMX_ARM_JOINTS: tuple[str, ...] = ("Joint1", "Joint2", "Joint3", "Joint4")
_OMX_REACH_SCENARIOS: frozenset[str] = frozenset(
    {"omx_reach", "omx_tabletop", "open_manipulator_x"}
)
_OMX_PICK_PLACE_SCENARIOS: frozenset[str] = frozenset(
    {"omx_pick_place", "pick_place"}
)
_OMX_DESK_CLUTTER_SCENARIOS: frozenset[str] = frozenset(
    {"omx_desk_clutter", "desk_clutter"}
)
_OMX_SCENARIOS: frozenset[str] = (
    _OMX_REACH_SCENARIOS
    | _OMX_PICK_PLACE_SCENARIOS
    | _OMX_DESK_CLUTTER_SCENARIOS
)
_OMX_MODELS: frozenset[str] = frozenset({"open_manipulator_x", "omx"})

# Freejoint base names in dubins_race.xml (TurtleBot3 agents).
_DUBINS_BASE_JOINTS: tuple[str, str, str] = (
    "rrt_base_joint",
    "sst_base_joint",
    "dummy_base_joint",
)
_DUBINS_AGENT_BASE_Z_M = 0.033

_DUBINS_LIMITS = np.array(
    [
        [0.0, 10.0],
        [0.0, 10.0],
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
    wall_sim_time_s: float | None = None

    @property
    def real_time_factor(self) -> float:
        """Playback speedup needed so video duration matches ``sim_time_s``."""
        if self.sim_time_s <= 0.0:
            return 1.0
        return self.render_duration_s / self.sim_time_s


def showcase_playback_timing(
    *,
    wall_sim_time_s: float,
    render_duration_s: float,
) -> ShowcaseTiming:
    """Build RTF timing for a physics or kinematic showcase clip.

    Physics SITL (V115-03) resamples the full ``qpos`` log into
    ``render_duration_s`` at fixed fps when wall-clock sim exceeds the scenario
    nominal duration.      That encode is already a time-compressed playback; using ``wall_sim_time_s``
    for ffmpeg ``setpts`` would stretch frames to ``n_frames / wall_sim_time_s``
    effective fps (slideshow on Dubins).
    """
    playback_sim_s = (
        render_duration_s
        if wall_sim_time_s > render_duration_s + 0.02
        else wall_sim_time_s
    )
    wall = wall_sim_time_s if wall_sim_time_s > playback_sim_s + 0.02 else None
    return ShowcaseTiming(
        sim_time_s=playback_sim_s,
        render_duration_s=render_duration_s,
        wall_sim_time_s=wall,
    )


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
        model: Robot model name (e.g. ``dubins``).
        scenario: Scenario stem (e.g. ``dubins_race``).
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

    if model == "dubins" and scenario in {"dubins_race", "dubins"}:
        candidate = _project_root() / "src/fret/mjcf/dubins_race.xml"
        if candidate.is_file():
            return candidate

    if model in _OMX_MODELS and scenario in _OMX_REACH_SCENARIOS:
        _ensure_fret_importable()
        from fret.sitl_config import mjcf_path as resolve_installed_mjcf

        return resolve_installed_mjcf("open_manipulator_x", "omx_reach")

    if model in _OMX_MODELS and scenario in _OMX_PICK_PLACE_SCENARIOS:
        _ensure_fret_importable()
        from fret.sitl_config import mjcf_path as resolve_installed_mjcf

        return resolve_installed_mjcf("open_manipulator_x", "omx_pick_place")

    if model in _OMX_MODELS and scenario in _OMX_DESK_CLUTTER_SCENARIOS:
        _ensure_fret_importable()
        from fret.sitl_config import mjcf_path as resolve_installed_mjcf

        return resolve_installed_mjcf("open_manipulator_x", "omx_desk_clutter")

    raise ValueError(
        f"Unsupported model/scenario combination: model={model!r}, "
        f"scenario={scenario!r}"
    )


def list_showcase_cameras(
    mjcf_path: Path,
    *,
    scenario: str = "dubins_race",
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
    preferred = (
        _OMX_RELEASE_CAMERAS
        if scenario in _OMX_SCENARIOS
        else RELEASE_SHOWCASE_CAMERAS
    )
    if mjcf_cameras:
        release = [cam for cam in preferred if cam in mjcf_cameras]
        if release:
            return release
        raise ValueError(
            f"MJCF {mjcf_path} has no release showcase cameras "
            f"{preferred}; found {sorted(mjcf_cameras)}"
        )

    if scenario in {"dubins_race", "dubins"}:
        return list(_DUBINS_RACE_CAMERAS)
    if scenario in _OMX_SCENARIOS:
        return list(_OMX_RELEASE_CAMERAS)

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


def resolve_physics_showcase_duration_s(
    scenario: str,
    sim_time_s: float,
    duration_s: float | None,
) -> float:
    """Cap physics showcase clip length to scenario nominal duration (V115-03).

    Physics SITL can run far longer than the kinematic showcase clip.  Release
    and dry-run renders subsample to ``min(sim_time_s, scenario.duration)`` so
    ``--physics-mode --full-duration`` stays within release job timeouts.
    """
    if duration_s is not None:
        return float(duration_s)
    nominal_s = resolve_scenario_duration(scenario)
    return min(float(sim_time_s), nominal_s)


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

    tmp_path = video_path.with_name(
        f"{video_path.stem}_rtf{video_path.suffix}"
    )
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


def _yaw_to_quat_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Planar yaw (about +z) to MuJoCo quaternion ``(w, x, y, z)``."""
    half = 0.5 * float(yaw)
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _set_freejoint_pose(
    mujoco: object,
    model: object,
    data: object,
    joint_name: str,
    pose: npt.NDArray[np.float64],
) -> None:
    """Write planar ``(x, y, yaw)`` into a freejoint qpos block."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint not found in MJCF: {joint_name}")
    adr = int(model.jnt_qposadr[joint_id])
    qw, qx, qy, qz = _yaw_to_quat_wxyz(float(pose[2]))
    data.qpos[adr] = float(pose[0])
    data.qpos[adr + 1] = float(pose[1])
    data.qpos[adr + 2] = _DUBINS_AGENT_BASE_Z_M
    data.qpos[adr + 3] = qw
    data.qpos[adr + 4] = qx
    data.qpos[adr + 5] = qy
    data.qpos[adr + 6] = qz


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
    physics_mode: bool = False,
) -> tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    float,
]:
    """Run the SC-v11 race and resample agent poses for video export.

    Returns:
        RRT* poses, SST poses, dummy poses, and simulated race duration [s].
    """
    _ensure_fret_importable()
    from fret.scenario.dubins_race_runner import DubinsRaceRunner
    from fret.scenario.planner_rng import SHOWCASE_PLANNER_RNG_SEED

    result = DubinsRaceRunner().run(
        record_poses=True,
        physics_mode=physics_mode,
        planner_rng_seed=(SHOWCASE_PLANNER_RNG_SEED if physics_mode else None),
    )
    if not result.both_reached_goal:
        raise RuntimeError(
            "Dubins race simulation failed before both agents reached goal "
            f"(race_duration_s={result.race_duration_s:.1f}, "
            f"max_cross_track_error_m={result.max_cross_track_error_m:.2f})"
        )
    if physics_mode and result.min_obstacle_clearance_m < 0.0:
        raise RuntimeError(
            "Dubins physics showcase export has obstacle penetration "
            f"(min_obstacle_clearance_m={result.min_obstacle_clearance_m:.3f})"
        )

    rrt_hist = np.asarray(result.rrt_pose_history, dtype=np.float64)
    sst_hist = np.asarray(result.sst_pose_history, dtype=np.float64)
    dummy_hist = np.asarray(result.dummy_pose_history, dtype=np.float64)
    sim_dt = resolve_scenario_simulation_dt(scenario)
    race_samples = max(
        1, int(round(float(result.race_duration_s) / sim_dt)) + 1
    )
    rrt_hist = rrt_hist[:race_samples]
    sst_hist = sst_hist[:race_samples]
    dummy_hist = dummy_hist[:race_samples]
    if (
        result.rrt_time_to_goal_s is not None
        and result.sst_time_to_goal_s is not None
    ):
        sim_time_s = float(
            max(result.rrt_time_to_goal_s, result.sst_time_to_goal_s)
        )
    else:
        sim_time_s = float(max(0, len(rrt_hist) - 1)) * sim_dt

    render_duration_s = resolve_physics_showcase_duration_s(
        scenario,
        sim_time_s,
        duration_s,
    )
    n_frames = max(2, int(round(render_duration_s * fps)))
    return (
        _resample_pose_history(rrt_hist, n_frames),
        _resample_pose_history(sst_hist, n_frames),
        _resample_pose_history(dummy_hist, n_frames),
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


_DUBINS_CAR_WIDTH_M: float = 0.72
_DUBINS_FOLLOW_FOVY_DEG: float = 50.0
_DUBINS_FOLLOW_FILL: float = 0.15
_DUBINS_FOLLOW_DISTANCE_SCALE: float = 2.0
# Static MJCF overview sits ~109 m from arena centre; chase the car midpoint at
# one quarter of that range for a tighter release export.
_DUBINS_OVERVIEW_STATIC_DISTANCE_M: float = 108.75
_DUBINS_OVERVIEW_DISTANCE_SCALE: float = 0.25
_DUBINS_OVERVIEW_DISTANCE_M: float = (
    _DUBINS_OVERVIEW_STATIC_DISTANCE_M * _DUBINS_OVERVIEW_DISTANCE_SCALE
)
_DUBINS_OVERVIEW_AZIMUTH_DEG: float = 145.0
_DUBINS_OVERVIEW_ELEVATION_DEG: float = -35.0
_DUBINS_OVERVIEW_LOOKAT_Z_M: float = 0.4


def _dubins_follow_distance(
    half_width: int,
    height: int,
    *,
    fovy_deg: float = _DUBINS_FOLLOW_FOVY_DEG,
    car_width: float = _DUBINS_CAR_WIDTH_M,
    fill: float = _DUBINS_FOLLOW_FILL,
) -> float:
    """Return tracking distance so the car spans ``fill`` of a half-panel."""
    half_fov_rad = math.radians(fovy_deg) / 2.0
    hfov_rad = 2.0 * math.atan(math.tan(half_fov_rad) * half_width / height)
    return (car_width / 2.0) / (fill * math.tan(hfov_rad / 2.0))


def _dubins_race_midpoint(
    rrt_q: npt.NDArray[np.float64],
    sst_q: npt.NDArray[np.float64],
) -> tuple[float, float, float]:
    """Return the XY midpoint between both race agents."""
    return (
        (float(rrt_q[0]) + float(sst_q[0])) / 2.0,
        (float(rrt_q[1]) + float(sst_q[1])) / 2.0,
        _DUBINS_OVERVIEW_LOOKAT_Z_M,
    )


def _make_dubins_overview_camera(
    mujoco: object,
    midpoint: tuple[float, float, float],
    *,
    distance: float = _DUBINS_OVERVIEW_DISTANCE_M,
    azimuth_deg: float = _DUBINS_OVERVIEW_AZIMUTH_DEG,
    elevation_deg: float = _DUBINS_OVERVIEW_ELEVATION_DEG,
) -> object:
    """Build an oblique overview camera locked on the dual-agent midpoint."""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[0] = midpoint[0]
    cam.lookat[1] = midpoint[1]
    cam.lookat[2] = midpoint[2]
    cam.distance = float(distance)
    cam.azimuth = float(azimuth_deg)
    cam.elevation = float(elevation_deg)
    return cam


def _make_dubins_tracking_camera(
    mujoco: object,
    model: object,
    body_name: str,
    yaw_rad: float,
    distance: float,
) -> object:
    """Build a third-person camera locked on a race car body."""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        body_name,
    )
    if int(cam.trackbodyid) < 0:
        raise ValueError(f"Body not found in MJCF: {body_name}")
    cam.distance = float(distance)
    cam.elevation = -18.0
    # MuJoCo tracking azimuth places the camera on the circle around the
    # target; match body heading so the chase view sits behind the vehicle.
    cam.azimuth = math.degrees(float(yaw_rad))
    return cam


def _apply_dubins_poses(
    mujoco: object,
    model: object,
    data: object,
    rrt_q: npt.NDArray[np.float64],
    sst_q: npt.NDArray[np.float64],
    dummy_q: npt.NDArray[np.float64],
) -> None:
    """Write RRT*, SST, and dummy SE(2) poses into freejoint qpos."""
    for values, joint_name in (
        (rrt_q, _DUBINS_BASE_JOINTS[0]),
        (sst_q, _DUBINS_BASE_JOINTS[1]),
        (dummy_q, _DUBINS_BASE_JOINTS[2]),
    ):
        _set_freejoint_pose(mujoco, model, data, joint_name, values)
    mujoco.mj_forward(model, data)


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
    physics_mode: bool = False,
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

    rrt_poses, sst_poses, dummy_poses, sim_time_s = simulate_dubins_race_poses(
        scenario,
        duration_s=duration_s,
        fps=fps,
        physics_mode=physics_mode,
    )
    _assert_dubins_race_moves(rrt_poses, sst_poses)
    render_duration_s = float(len(rrt_poses)) / float(fps)
    timing = showcase_playback_timing(
        wall_sim_time_s=sim_time_s,
        render_duration_s=render_duration_s,
    )

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)
    split_follow = "follow" in camera_names
    half_width = max(2, width // 2)
    follow_distance = (
        _dubins_follow_distance(half_width, height)
        * _DUBINS_FOLLOW_DISTANCE_SCALE
    )
    follow_renderer_left: object | None = None
    follow_renderer_right: object | None = None
    if split_follow:
        follow_renderer_left = mujoco.Renderer(
            model, height=height, width=half_width
        )
        follow_renderer_right = mujoco.Renderer(
            model, height=height, width=half_width
        )

    for camera in camera_names:
        if camera != "follow":
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
        for rrt_q, sst_q, dummy_q in zip(
            rrt_poses, sst_poses, dummy_poses, strict=True
        ):
            _apply_dubins_poses(mujoco, model, data, rrt_q, sst_q, dummy_q)
            for camera in camera_names:
                if camera == "follow":
                    assert follow_renderer_left is not None
                    assert follow_renderer_right is not None
                    cam_rrt = _make_dubins_tracking_camera(
                        mujoco,
                        model,
                        "car_rrt",
                        float(rrt_q[2]),
                        follow_distance,
                    )
                    cam_sst = _make_dubins_tracking_camera(
                        mujoco,
                        model,
                        "car_sst",
                        float(sst_q[2]),
                        follow_distance,
                    )
                    follow_renderer_left.update_scene(data, camera=cam_rrt)
                    follow_renderer_right.update_scene(data, camera=cam_sst)
                    frame = np.concatenate(
                        [
                            follow_renderer_left.render(),
                            follow_renderer_right.render(),
                        ],
                        axis=1,
                    )
                elif camera == "overview":
                    cam_overview = _make_dubins_overview_camera(
                        mujoco,
                        _dubins_race_midpoint(rrt_q, sst_q),
                    )
                    renderer.update_scene(data, camera=cam_overview)
                    frame = renderer.render()
                else:
                    renderer.update_scene(data, camera=camera)
                    frame = renderer.render()
                writers[camera].append_data(frame)
                if camera not in first_frames:
                    first_frames[camera] = frame.copy()
    finally:
        for writer in writers.values():
            writer.close()
        renderer.close()
        if follow_renderer_left is not None:
            follow_renderer_left.close()
        if follow_renderer_right is not None:
            follow_renderer_right.close()

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


def _load_omx_reach_configurations(
    scenario: str,
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """Return start/goal arm configurations from the OM-X scenario YAML."""
    _ensure_fret_importable()
    from fret.sitl_config import load_scenario_parameters

    params = load_scenario_parameters(_scenario_config_path(scenario))
    start = np.asarray(
        params["start_configuration"], dtype=np.float64
    ).reshape(4)
    goal = np.asarray(params["goal_configuration"], dtype=np.float64).reshape(
        4
    )
    return start, goal


def _omx_joint_qpos(
    mujoco: object,
    model: object,
    data: object,
) -> npt.NDArray[np.float64]:
    """Read Menagerie arm joint positions from ``data.qpos``."""
    q = np.zeros(4, dtype=np.float64)
    for i, name in enumerate(_OMX_ARM_JOINTS):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"OM-X joint not found in MJCF: {name}")
        q[i] = float(data.qpos[model.jnt_qposadr[jid]])
    return q


def simulate_omx_reach_poses(
    mjcf_path: Path,
    scenario: str = "omx_reach",
    *,
    duration_s: float | None,
    fps: int,
) -> tuple[npt.NDArray[np.float64], float]:
    """Drive OM-X position actuators start→goal; return per-frame joint poses.

    Uses MuJoCo ``mj_step`` with interpolated ``ctrl`` (empty-cell command-chain
    proof). Gripper stays open (ctrl[4] = 0).
    """
    mujoco, _iio = _require_mujoco()
    start, goal = _load_omx_reach_configurations(scenario)
    clip_s = (
        float(duration_s)
        if duration_s is not None
        else resolve_scenario_duration(scenario, default_s=8.0)
    )
    if clip_s <= 0.0:
        raise ValueError(f"OM-X showcase duration must be > 0 (got {clip_s})")

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    if model.nu < 5:
        raise ValueError(
            f"OM-X MJCF needs ≥5 actuators (arm+gripper); got nu={model.nu}"
        )

    n_frames = max(1, int(round(clip_s * fps)))
    dt = float(model.opt.timestep)
    settle_steps = max(1, int(round(0.5 / dt)))
    data.ctrl[:4] = start
    data.ctrl[4] = 0.0
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)

    poses = np.zeros((n_frames, 4), dtype=np.float64)
    steps_per_frame = max(1, int(round((1.0 / fps) / dt)))
    for frame_i in range(n_frames):
        alpha = frame_i / max(1, n_frames - 1)
        data.ctrl[:4] = (1.0 - alpha) * start + alpha * goal
        data.ctrl[4] = 0.0
        for _ in range(steps_per_frame):
            mujoco.mj_step(model, data)
        poses[frame_i] = _omx_joint_qpos(mujoco, model, data)

    travel = float(np.linalg.norm(poses[-1] - poses[0]))
    if travel < 0.05:
        raise RuntimeError(
            f"OM-X showcase barely moved (Δq={travel:.4f} rad); "
            "check actuators / start-goal"
        )
    sim_time_s = float(n_frames * steps_per_frame) * dt
    return poses, sim_time_s


def _apply_omx_joint_pose(
    mujoco: object,
    model: object,
    data: object,
    q: npt.NDArray[np.float64],
) -> None:
    """Teleport OM-X arm joints to ``q`` and forward kinematics."""
    for name, value in zip(_OMX_ARM_JOINTS, q, strict=True):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"OM-X joint not found in MJCF: {name}")
        data.qpos[model.jnt_qposadr[jid]] = float(value)
    mujoco.mj_forward(model, data)


def render_omx_reach_showcase_videos(
    mjcf_path: Path,
    output_dir: Path,
    *,
    scenario: str = "omx_reach",
    cameras: list[str] | None = None,
    output_names: dict[str, str] | None = None,
    duration_s: float | None = None,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    realtime_postprocess: bool = True,
) -> list[RenderResult]:
    """Render OM-X empty-tabletop A→B MP4s (SC-v13a free-space proof)."""
    mujoco, _iio = _require_mujoco()
    camera_names = (
        cameras
        if cameras is not None
        else list_showcase_cameras(mjcf_path, scenario=scenario)
    )
    if not camera_names:
        raise ValueError("At least one showcase camera is required")

    poses, sim_time_s = simulate_omx_reach_poses(
        mjcf_path,
        scenario,
        duration_s=duration_s,
        fps=fps,
    )
    render_duration_s = float(len(poses)) / float(fps)
    timing = showcase_playback_timing(
        wall_sim_time_s=sim_time_s,
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
        for q in poses:
            _apply_omx_joint_pose(mujoco, model, data, q)
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


def _apply_omx_pick_place_sample(
    mujoco: object,
    model: object,
    data: object,
    *,
    q_arm: npt.NDArray[np.float64],
    gripper: float,
    box_qpos: npt.NDArray[np.float64],
    box_qadr: int,
) -> None:
    """Teleport arm, gripper joint, and free box to a recorded sample."""
    _apply_omx_joint_pose(mujoco, model, data, q_arm)
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper")
    if gid >= 0:
        data.qpos[model.jnt_qposadr[gid]] = float(gripper)
    mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Gripper_mimic")
    if mid >= 0:
        data.qpos[model.jnt_qposadr[mid]] = float(gripper)
    data.qpos[box_qadr : box_qadr + 7] = np.asarray(box_qpos, dtype=np.float64)
    mujoco.mj_forward(model, data)


def render_omx_desk_clutter_showcase_videos(
    mjcf_path: Path,
    output_dir: Path,
    *,
    scenario: str = "omx_desk_clutter",
    cameras: list[str] | None = None,
    output_names: dict[str, str] | None = None,
    duration_s: float | None = None,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    realtime_postprocess: bool = True,
) -> list[RenderResult]:
    """Render OM-X desk-clutter MP4s (SC-v13c planned transfer around wall)."""
    mujoco, _iio = _require_mujoco()
    _ensure_fret_importable()
    from fret.control.pick_place_clutter_sim import simulate_pick_place_clutter
    from fret.control.pick_place_fsm import PickPlaceState

    camera_names = (
        cameras
        if cameras is not None
        else list_showcase_cameras(mjcf_path, scenario=scenario)
    )
    if not camera_names:
        raise ValueError("At least one showcase camera is required")

    clip_s = (
        float(duration_s)
        if duration_s is not None
        else resolve_scenario_duration(scenario, default_s=45.0)
    )
    model_probe = mujoco.MjModel.from_xml_path(str(mjcf_path))
    dt = float(model_probe.opt.timestep)
    record_every = max(1, int(round((1.0 / fps) / dt)))
    result = simulate_pick_place_clutter(
        duration_s=clip_s,
        joint_tol_rad=0.12,
        record_every_steps=record_every,
    )
    if result.state != PickPlaceState.DONE:
        raise RuntimeError(
            f"SC-v13c showcase FSM ended in {result.state.name}, expected DONE"
        )
    samples = result.samples
    if len(samples) < 2:
        raise RuntimeError("SC-v13c showcase recorded too few frames")

    pad = max(1, int(round(0.6 * fps)))
    samples = [samples[0]] * pad + list(samples) + [samples[-1]] * pad
    target_frames = max(len(samples), int(round(10.0 * fps)))
    if len(samples) < target_frames:
        idx = np.linspace(0, len(samples) - 1, target_frames)
        samples = [samples[int(round(i))] for i in idx]

    sim_time_s = float(len(samples)) / float(fps)
    render_duration_s = float(len(samples)) / float(fps)
    timing = showcase_playback_timing(
        wall_sim_time_s=sim_time_s,
        render_duration_s=render_duration_s,
    )

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    if box_jid < 0:
        raise ValueError("pick_box_joint missing from OM-X desk-clutter MJCF")
    box_qadr = int(model.jnt_qposadr[box_jid])
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
        for sample in samples:
            _apply_omx_pick_place_sample(
                mujoco,
                model,
                data,
                q_arm=sample.q_arm,
                gripper=sample.gripper,
                box_qpos=sample.box_qpos,
                box_qadr=box_qadr,
            )
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
        results.append(
            RenderResult(
                path=paths[camera],
                camera=camera,
                first_frame=first_frames[camera],
                timing=timing,
            )
        )
    return postprocess_showcase_results(
        results,
        enabled=realtime_postprocess,
    )


def render_omx_pick_place_showcase_videos(
    mjcf_path: Path,
    output_dir: Path,
    *,
    scenario: str = "omx_pick_place",
    cameras: list[str] | None = None,
    output_names: dict[str, str] | None = None,
    duration_s: float | None = None,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    realtime_postprocess: bool = True,
) -> list[RenderResult]:
    """Render OM-X pick-and-place MP4s (SC-v13b idle→pick→place→idle)."""
    mujoco, _iio = _require_mujoco()
    _ensure_fret_importable()
    from fret.control.pick_place_fsm import PickPlaceState
    from fret.control.pick_place_sim import simulate_pick_place

    camera_names = (
        cameras
        if cameras is not None
        else list_showcase_cameras(mjcf_path, scenario=scenario)
    )
    if not camera_names:
        raise ValueError("At least one showcase camera is required")

    clip_s = (
        float(duration_s)
        if duration_s is not None
        else resolve_scenario_duration(scenario, default_s=20.0)
    )
    model_probe = mujoco.MjModel.from_xml_path(str(mjcf_path))
    dt = float(model_probe.opt.timestep)
    record_every = max(1, int(round((1.0 / fps) / dt)))
    state, samples = simulate_pick_place(
        duration_s=clip_s,
        joint_tol_rad=0.12,
        record_every_steps=record_every,
    )
    if state != PickPlaceState.DONE:
        raise RuntimeError(
            f"SC-v13b showcase FSM ended in {state.name}, expected DONE"
        )
    if len(samples) < 2:
        raise RuntimeError("SC-v13b showcase recorded too few frames")

    # Pad idle at both ends and time-stretch so the clip is watchable.
    pad = max(1, int(round(0.6 * fps)))
    samples = [samples[0]] * pad + list(samples) + [samples[-1]] * pad
    target_frames = max(len(samples), int(round(8.0 * fps)))
    if len(samples) < target_frames:
        idx = np.linspace(0, len(samples) - 1, target_frames)
        samples = [samples[int(round(i))] for i in idx]

    sim_time_s = float(len(samples)) / float(fps)
    render_duration_s = float(len(samples)) / float(fps)
    timing = showcase_playback_timing(
        wall_sim_time_s=sim_time_s,
        render_duration_s=render_duration_s,
    )

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    box_jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "pick_box_joint"
    )
    if box_jid < 0:
        raise ValueError("pick_box_joint missing from OM-X pick-place MJCF")
    box_qadr = int(model.jnt_qposadr[box_jid])
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
        for sample in samples:
            _apply_omx_pick_place_sample(
                mujoco,
                model,
                data,
                q_arm=sample.q_arm,
                gripper=sample.gripper,
                box_qpos=sample.box_qpos,
                box_qadr=box_qadr,
            )
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


def render_video(
    mjcf_path: Path,
    output_path: Path,
    *,
    scenario: str = "dubins_race",
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
    physics_mode: bool = False,
) -> RenderResult:
    """Render a headless MuJoCo MP4 for a supported showcase scenario.

    Args:
        mjcf_path: Path to the MJCF scene file.
        output_path: Destination ``.mp4`` path.
        duration_s: Clip length in seconds.
        fps: Frame rate.
        width: Frame width in pixels.
        height: Frame height in pixels.
        camera: Named MuJoCo camera in the MJCF.
        waypoints: Unused (retained for call-site compatibility).

    Returns:
        Render metadata including the output path and first-frame mean.
    """
    _ = (waypoints, collision_backend, planner_algorithm, use_tracking)
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
        realtime_postprocess=realtime_postprocess,
        physics_mode=physics_mode,
    )
    return results[0]


def render_showcase_videos(
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
    waypoints: list[npt.NDArray[np.float64]] | None = None,
    collision_backend: str | None = None,
    planner_algorithm: str = "rrt_star",
    use_tracking: bool = True,
    realtime_postprocess: bool = True,
    physics_mode: bool = False,
) -> list[RenderResult]:
    """Render one MP4 per showcase camera in a single simulation pass.

    Supports Dubins race and OpenMANIPULATOR-X empty-tabletop reach.
    """
    _ = (waypoints, collision_backend, planner_algorithm, use_tracking)
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
            physics_mode=physics_mode,
        )
    if scenario in _OMX_REACH_SCENARIOS:
        _ = physics_mode  # OM-X showcase always steps position actuators.
        return render_omx_reach_showcase_videos(
            mjcf_path,
            output_dir,
            scenario="omx_reach",
            cameras=cameras,
            output_names=output_names,
            duration_s=duration_s,
            fps=fps,
            width=width,
            height=height,
            realtime_postprocess=realtime_postprocess,
        )
    if scenario in _OMX_PICK_PLACE_SCENARIOS:
        _ = physics_mode
        return render_omx_pick_place_showcase_videos(
            mjcf_path,
            output_dir,
            scenario="omx_pick_place",
            cameras=cameras,
            output_names=output_names,
            duration_s=duration_s,
            fps=fps,
            width=width,
            height=height,
            realtime_postprocess=realtime_postprocess,
        )
    if scenario in _OMX_DESK_CLUTTER_SCENARIOS:
        _ = physics_mode
        return render_omx_desk_clutter_showcase_videos(
            mjcf_path,
            output_dir,
            scenario="omx_desk_clutter",
            cameras=cameras,
            output_names=output_names,
            duration_s=duration_s,
            fps=fps,
            width=width,
            height=height,
            realtime_postprocess=realtime_postprocess,
        )
    raise ValueError(f"Unsupported showcase scenario: {scenario!r}")


def write_showcase_timing_json(
    results: list[RenderResult],
    output_path: Path,
) -> None:
    """Persist per-clip timing metadata for release workflows."""
    import json

    clips: list[dict[str, float | str]] = []
    for result in results:
        timing = result.timing
        clip: dict[str, float | str] = {
            "camera": result.camera,
            "file": result.path.name,
            "sim_time_s": timing.sim_time_s,
            "render_duration_s": timing.render_duration_s,
            "real_time_factor": timing.real_time_factor,
        }
        if timing.wall_sim_time_s is not None:
            clip["wall_sim_time_s"] = timing.wall_sim_time_s
        clips.append(clip)
    payload = {"clips": clips}
    output_path.write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
    )


def _die_missing(
    parser: argparse.ArgumentParser, missing: list[str], *, argv: list[str]
) -> None:
    """Print a concise error and usage when required CLI args are absent."""
    if not argv:
        print("missing arguments", file=sys.stderr)
    else:
        print(f"missing arguments: {', '.join(missing)}", file=sys.stderr)
    parser.print_help()
    raise SystemExit(2)


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description="Render headless MuJoCo MP4s for FRET showcase scenarios.",
    )
    parser.add_argument(
        "--model",
        required=True,
        help="Robot model name (e.g. dubins)",
    )
    parser.add_argument(
        "--scenario",
        required=True,
        help="Scenario stem (e.g. dubins_race)",
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
        default=None,
        help="Output directory for --all-cameras or multi --camera",
    )
    parser.add_argument(
        "--mjcf",
        type=Path,
        default=None,
        help="Override MJCF path (optional)",
    )
    duration_group = parser.add_mutually_exclusive_group(required=True)
    duration_group.add_argument(
        "--duration",
        type=float,
        help="Clip stretch duration in seconds",
    )
    duration_group.add_argument(
        "--full-duration",
        action="store_true",
        help="Render the full simulated motion at real-time speed",
    )
    parser.add_argument(
        "--fps",
        type=int,
        required=True,
        help="Frame rate",
    )
    parser.add_argument(
        "--width",
        type=int,
        required=True,
        help="Frame width in pixels",
    )
    parser.add_argument(
        "--height",
        type=int,
        required=True,
        help="Frame height in pixels",
    )
    parser.add_argument(
        "--camera",
        action="append",
        dest="cameras",
        metavar="NAME",
        help="MJCF camera name (repeatable; required for single-camera mode)",
    )
    parser.add_argument(
        "--all-cameras",
        action="store_true",
        help="Export every showcase camera defined in the MJCF",
    )
    parser.add_argument(
        "--collision-backend",
        choices=("analytic", "mujoco"),
        required=True,
        help="Plan showcase motion with FRET collision checking",
    )
    parser.add_argument(
        "--planner-algorithm",
        choices=("rrt_star", "sst"),
        required=True,
        help="ARCO planner for showcase path planning",
    )
    parser.add_argument(
        "--no-tracking",
        action="store_true",
        help="Use joint interpolation instead of controller tracking",
    )
    parser.add_argument(
        "--physics-mode",
        action="store_true",
        help="Drive MuJoCo via mj_step actuators (v1.2 physics SITL; opt-in)",
    )
    parser.add_argument(
        "--kinematic-mode",
        action="store_true",
        help="Legacy pose-teleport rendering (v1.0–v1.1 mirror path)",
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


def _validate_render_cli(
    parser: argparse.ArgumentParser, argv: list[str]
) -> argparse.Namespace:
    """Parse argv and fail with help when required flag combinations are missing."""
    if not argv:
        _die_missing(
            parser,
            [
                "--model",
                "--scenario",
                "--fps",
                "--width",
                "--height",
                "--collision-backend",
                "--planner-algorithm",
                "(--duration or --full-duration)",
                "(-o/--output and --camera) or (--all-cameras and --output-dir)",
            ],
            argv=argv,
        )

    args = parser.parse_args(argv)
    missing: list[str] = []

    if args.all_cameras:
        if args.output_dir is None:
            missing.append("--output-dir")
        if args.output is not None:
            parser.error("--output cannot be used with --all-cameras")
    elif args.cameras and len(args.cameras) > 1:
        if args.output_dir is None:
            missing.append("--output-dir")
    elif args.output is None:
        missing.append("-o/--output")
    elif not args.cameras:
        missing.append("--camera")

    if missing:
        _die_missing(parser, missing, argv=argv)

    return args


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = build_parser()
    cli_argv = list(sys.argv[1:] if argv is None else argv)
    args = _validate_render_cli(parser, cli_argv)
    mjcf_path = resolve_mjcf_path(args.model, args.scenario, args.mjcf)
    duration_s = None if args.full_duration else args.duration

    use_tracking = not args.no_tracking
    realtime_postprocess = not args.no_realtime_postprocess
    physics_mode = bool(args.physics_mode) and not args.kinematic_mode

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
            physics_mode=physics_mode,
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

    cameras = args.cameras
    assert cameras is not None
    if len(cameras) == 1:
        output = args.output
        assert output is not None
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
            physics_mode=physics_mode,
        )
        timing = result.timing
        print(
            f"Wrote {result.path} (sim={timing.sim_time_s:.1f}s, "
            f"rtf={timing.real_time_factor:.3f}, "
            f"mean={result.frame_mean:.1f})"
        )
        return 0

    assert args.output_dir is not None
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
        physics_mode=physics_mode,
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
