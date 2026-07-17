#!/usr/bin/env python3
"""Interactive MuJoCo 3D viewer for FRET showcase scenarios.

Opens a live MuJoCo window for the Dubins race scene.  No ROS runtime
required.

Example::

    ./scripts/view.sh --model dubins --scenario dubins_race \\
        --duration 30 --fps 60 --camera overview
    python3 scripts/view_mujoco.py --model dubins --scenario dubins_race \\
        --duration 30 --fps 60 --camera overview --dry-run

Dependencies::

    pip install mujoco
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import numpy.typing as npt

# Reuse MJCF resolution helpers from the headless renderer.
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from render_mujoco import (  # noqa: E402
    _DUBINS_JOINT_NAMES,
    _apply_dubins_poses,
    resolve_mjcf_path,
)


def _require_mujoco() -> object:
    """Import MuJoCo or exit with an install hint."""
    try:
        import mujoco  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit(
            "MuJoCo is required for the interactive viewer.\n"
            "Install with: pip install mujoco"
        ) from exc
    return mujoco


def _static_dubins_poses(
    duration_s: float,
    fps: int,
) -> tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
]:
    """Build a short linear Dubins demo for interactive viewing."""
    n_frames = max(2, int(round(duration_s * fps)))
    t = np.linspace(0.0, 1.0, n_frames)
    rrt = np.column_stack(
        [
            6.0 + 20.0 * t,
            np.full(n_frames, 6.0),
            np.zeros(n_frames),
        ]
    ).astype(np.float64)
    sst = np.column_stack(
        [
            6.0 + 18.0 * t,
            np.full(n_frames, 6.4),
            np.zeros(n_frames),
        ]
    ).astype(np.float64)
    dummy = np.column_stack(
        [
            6.0 + 16.0 * t,
            np.full(n_frames, 5.6),
            np.zeros(n_frames),
        ]
    ).astype(np.float64)
    return rrt, sst, dummy


def run_interactive_viewer(
    mjcf_path: Path,
    *,
    duration_s: float = 30.0,
    fps: int = 60,
    camera: str = "overview",
    loop: bool = True,
) -> None:
    """Open a passive MuJoCo viewer and animate Dubins race poses.

    Args:
        mjcf_path: Path to the MJCF scene file.
        duration_s: One animation cycle length [s].
        fps: Playback frame rate.
        camera: Named MJCF camera (used by the viewer default pose).
        loop: When True, restart the trajectory after each cycle.
    """
    mujoco = _require_mujoco()
    try:
        import mujoco.viewer  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit(
            "mujoco.viewer is unavailable in this MuJoCo build.\n"
            "Upgrade with: pip install -U mujoco"
        ) from exc

    rrt_poses, sst_poses, dummy_poses = _static_dubins_poses(duration_s, fps)
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    dt = 1.0 / fps

    frame_idx = 0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if cam_id >= 0:
            viewer.cam.fixedcamid = cam_id

        while viewer.is_running():
            step_start = time.monotonic()
            _apply_dubins_poses(
                mujoco,
                model,
                data,
                rrt_poses[frame_idx],
                sst_poses[frame_idx],
                dummy_poses[frame_idx],
            )
            viewer.sync()

            frame_idx += 1
            if frame_idx >= len(rrt_poses):
                if loop:
                    frame_idx = 0
                else:
                    break

            elapsed = time.monotonic() - step_start
            sleep_s = dt - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)


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
        description="Open an interactive MuJoCo 3D viewer for FRET scenarios.",
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
        "--mjcf",
        type=Path,
        default=None,
        help="Override MJCF path (optional)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        required=True,
        help="Animation cycle duration in seconds",
    )
    parser.add_argument(
        "--fps",
        type=int,
        required=True,
        help="Playback frame rate",
    )
    parser.add_argument(
        "--camera",
        required=True,
        help="MJCF camera name (e.g. overview, follow)",
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="Stop after one animation cycle instead of looping",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Load MJCF and build trajectory without opening a window (CI)",
    )
    return parser


def _validate_view_cli(
    parser: argparse.ArgumentParser, argv: list[str]
) -> argparse.Namespace:
    """Parse argv and fail with help when required flags are missing."""
    if not argv:
        _die_missing(
            parser,
            ["--model", "--scenario", "--duration", "--fps", "--camera"],
            argv=argv,
        )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = build_parser()
    cli_argv = list(sys.argv[1:] if argv is None else argv)
    args = _validate_view_cli(parser, cli_argv)
    mjcf_path = resolve_mjcf_path(args.model, args.scenario, args.mjcf)

    if args.dry_run:
        rrt_poses, _, _ = _static_dubins_poses(args.duration, args.fps)
        # Touch joint-name table so dry-run exercises Dubins helpers.
        assert len(_DUBINS_JOINT_NAMES) == 3
        print(
            f"dry-run ok: mjcf={mjcf_path.name} "
            f"frames={len(rrt_poses)} duration={args.duration}s"
        )
        return 0

    run_interactive_viewer(
        mjcf_path,
        duration_s=args.duration,
        fps=args.fps,
        camera=args.camera,
        loop=not args.no_loop,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
