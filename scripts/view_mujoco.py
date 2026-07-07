#!/usr/bin/env python3
"""Interactive MuJoCo 3D viewer for FRET showcase scenarios.

Opens a live MuJoCo window and animates the PPP gantry along the warehouse
preview path.  No ROS runtime required.

Example::

    ./scripts/view.sh
    python3 scripts/view_mujoco.py --scenario ppp_warehouse --duration 30

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

# Reuse MJCF resolution and trajectory helpers from the headless renderer.
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from render_mujoco import (  # noqa: E402
    _PPP_WAREHOUSE_WAYPOINTS,
    _set_slide_joint,
    interpolate_waypoints,
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


def run_interactive_viewer(
    mjcf_path: Path,
    *,
    duration_s: float = 30.0,
    fps: int = 60,
    camera: str = "overview",
    waypoints: list[npt.NDArray[np.float64]] | None = None,
    loop: bool = True,
) -> None:
    """Open a passive MuJoCo viewer and animate joint positions.

    Args:
        mjcf_path: Path to the MJCF scene file.
        duration_s: One animation cycle length [s].
        fps: Playback frame rate.
        camera: Named MJCF camera (used by the viewer default pose).
        waypoints: Optional joint-space path; defaults to warehouse demo.
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

    path_waypoints = (
        waypoints if waypoints is not None else _PPP_WAREHOUSE_WAYPOINTS
    )
    trajectory = interpolate_waypoints(path_waypoints, duration_s, fps)

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    joint_names = ("joint_x", "joint_y", "joint_z")
    dt = 1.0 / fps

    frame_idx = 0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if cam_id >= 0:
            viewer.cam.fixedcamid = cam_id

        while viewer.is_running():
            step_start = time.monotonic()
            q = trajectory[frame_idx]
            for idx, name in enumerate(joint_names):
                _set_slide_joint(mujoco, model, data, name, float(q[idx]))
            mujoco.mj_forward(model, data)
            viewer.sync()

            frame_idx += 1
            if frame_idx >= len(trajectory):
                if loop:
                    frame_idx = 0
                else:
                    break

            elapsed = time.monotonic() - step_start
            sleep_s = dt - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(
        description="Open an interactive MuJoCo 3D viewer for FRET scenarios.",
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
        "--mjcf",
        type=Path,
        default=None,
        help="Override MJCF path (default: resolved from model/scenario)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Animation cycle duration in seconds (default: 30)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=60,
        help="Playback frame rate (default: 60)",
    )
    parser.add_argument(
        "--camera",
        default="overview",
        help="MJCF camera name (default: overview)",
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


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    mjcf_path = resolve_mjcf_path(args.model, args.scenario, args.mjcf)

    if args.dry_run:
        trajectory = interpolate_waypoints(
            _PPP_WAREHOUSE_WAYPOINTS, args.duration, args.fps
        )
        print(
            f"dry-run ok: mjcf={mjcf_path.name} "
            f"frames={len(trajectory)} duration={args.duration}s"
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
