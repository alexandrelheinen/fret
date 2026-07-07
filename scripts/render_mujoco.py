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
        [0.0, 4.0],
        [0.0, 3.0],
    ],
    dtype=np.float64,
)

# Pick-and-place preview path in joint space (x, y, z) [m].
_PPP_WAREHOUSE_WAYPOINTS: list[npt.NDArray[np.float64]] = [
    np.array([2.0, 1.0, 2.4]),
    np.array([2.0, 1.0, 0.95]),
    np.array([2.0, 1.0, 2.2]),
    np.array([4.5, 1.0, 2.2]),
    np.array([6.0, 2.6, 2.2]),
    np.array([9.5, 3.2, 2.2]),
    np.array([10.5, 3.2, 0.95]),
    np.array([10.5, 3.2, 2.4]),
]

# Default release export order (must match MJCF camera names).
_PPP_WAREHOUSE_CAMERAS: tuple[str, ...] = (
    "overview",
    "aisle",
    "topdown",
    "follow",
    "pick",
)

_EGL_APT_HINT = (
    "sudo apt install libegl1 libegl-mesa0 libgles2 libgl1 "
    "libgl1-mesa-dri libosmesa6"
)


@dataclass(frozen=True)
class RenderResult:
    """One rendered showcase clip."""

    camera: str
    path: Path
    frame_mean: float


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
        Camera names in release-export order.
    """
    root = ET.parse(mjcf_path).getroot()
    cameras = [
        name
        for node in root.iter("camera")
        if (name := node.get("name")) is not None
    ]
    if cameras:
        return cameras

    if scenario in {"ppp_warehouse", "ppp"}:
        return list(_PPP_WAREHOUSE_CAMERAS)

    raise ValueError(f"No showcase cameras found in MJCF: {mjcf_path}")


def showcase_output_name(scenario: str, camera: str) -> str:
    """Build the canonical showcase MP4 filename for a scenario/camera pair."""
    return f"{scenario}_{camera}.mp4"


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


def _set_slide_joint(
    mujoco: object,
    model: object,
    data: object,
    joint_name: str,
    value: float,
) -> None:
    """Write a scalar position into a slide joint's qpos slot."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint not found in MJCF: {joint_name}")
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr] = value


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
    duration_s: float = 30.0,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    camera: str = "overview",
    waypoints: list[npt.NDArray[np.float64]] | None = None,
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
    )
    return results[0]


def render_showcase_videos(
    mjcf_path: Path,
    output_dir: Path,
    *,
    scenario: str = "ppp_warehouse",
    cameras: list[str] | None = None,
    output_names: dict[str, str] | None = None,
    duration_s: float = 30.0,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    waypoints: list[npt.NDArray[np.float64]] | None = None,
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
    mujoco, _iio = _require_mujoco()
    camera_names = cameras if cameras is not None else list_showcase_cameras(
        mjcf_path, scenario=scenario
    )
    if not camera_names:
        raise ValueError("At least one showcase camera is required")

    path_waypoints = (
        waypoints if waypoints is not None else _PPP_WAREHOUSE_WAYPOINTS
    )
    trajectory = interpolate_waypoints(path_waypoints, duration_s, fps)

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    for camera in camera_names:
        _assert_camera_exists(mujoco, model, camera)

    output_dir.mkdir(parents=True, exist_ok=True)
    names = output_names or {
        camera: showcase_output_name(scenario, camera) for camera in camera_names
    }
    paths = {
        camera: output_dir / names[camera] for camera in camera_names
    }
    writers = {
        camera: _open_video_writer(paths[camera], fps) for camera in camera_names
    }
    first_frames: dict[str, npt.NDArray[np.uint8]] = {}

    joint_names = ("joint_x", "joint_y", "joint_z")
    try:
        for q in trajectory:
            for idx, name in enumerate(joint_names):
                _set_slide_joint(mujoco, model, data, name, float(q[idx]))
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
        results.append(RenderResult(camera=camera, path=paths[camera], frame_mean=mean))

    return results


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
        default=30.0,
        help="Clip duration in seconds (default: 30)",
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
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    mjcf_path = resolve_mjcf_path(args.model, args.scenario, args.mjcf)

    if args.all_cameras:
        cameras = list_showcase_cameras(mjcf_path, scenario=args.scenario)
        results = render_showcase_videos(
            mjcf_path,
            args.output_dir,
            scenario=args.scenario,
            cameras=cameras,
            duration_s=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
        )
        for result in results:
            print(
                f"Wrote {result.path} "
                f"(camera={result.camera}, mean={result.frame_mean:.1f}, "
                f"{args.duration:.1f}s @ {args.fps} fps)"
            )
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
            duration_s=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
            camera=cameras[0],
        )
        print(
            f"Wrote {result.path} ({args.duration:.1f}s @ {args.fps} fps, "
            f"mean={result.frame_mean:.1f})"
        )
        return 0

    results = render_showcase_videos(
        mjcf_path,
        args.output_dir,
        scenario=args.scenario,
        cameras=cameras,
        duration_s=args.duration,
        fps=args.fps,
        width=args.width,
        height=args.height,
    )
    for result in results:
        print(
            f"Wrote {result.path} "
            f"(camera={result.camera}, mean={result.frame_mean:.1f})"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
