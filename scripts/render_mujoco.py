#!/usr/bin/env python3
"""Headless MuJoCo video renderer for FRET showcase scenarios.

Renders an MP4 of the PPP gantry traversing a warehouse preview scene
without requiring a ROS runtime.  Intended for README / article assets
(releases.md V10-6, T10-07).

Example::

    python3 scripts/render_mujoco.py --scenario ppp_warehouse -o /tmp/v10.mp4
    ./scripts/video.sh --model ppp --duration 30

Dependencies (not required for core FRET algorithms)::

    pip install mujoco imageio imageio-ffmpeg
"""

from __future__ import annotations

import argparse
import sys
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
    frames_per_segment = n_frames / segment_count
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


def render_video(
    mjcf_path: Path,
    output_path: Path,
    *,
    duration_s: float = 30.0,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    camera: str = "overview",
    waypoints: list[npt.NDArray[np.float64]] | None = None,
) -> Path:
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
        The output path (created parent directories as needed).
    """
    mujoco, iio = _require_mujoco()
    path_waypoints = (
        waypoints if waypoints is not None else _PPP_WAREHOUSE_WAYPOINTS
    )
    trajectory = interpolate_waypoints(path_waypoints, duration_s, fps)

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    frames: list[npt.NDArray[np.uint8]] = []

    joint_names = ("joint_x", "joint_y", "joint_z")
    for q in trajectory:
        for idx, name in enumerate(joint_names):
            _set_slide_joint(mujoco, model, data, name, float(q[idx]))
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        frames.append(renderer.render().copy())

    renderer.close()
    iio.imwrite(
        output_path,
        np.stack(frames, axis=0),
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
    )
    return output_path


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description="Render a headless MuJoCo MP4 for FRET showcase scenarios.",
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
        default=Path("/tmp/fret_ppp_warehouse.mp4"),
        help="Output MP4 path (default: /tmp/fret_ppp_warehouse.mp4)",
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
        default="overview",
        help="MJCF camera name (default: overview)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    mjcf_path = resolve_mjcf_path(args.model, args.scenario, args.mjcf)
    output = render_video(
        mjcf_path,
        args.output,
        duration_s=args.duration,
        fps=args.fps,
        width=args.width,
        height=args.height,
        camera=args.camera,
    )
    print(f"Wrote {output} ({args.duration:.1f}s @ {args.fps} fps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
