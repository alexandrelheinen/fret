#!/usr/bin/env python3
"""Standalone workspace-occupancy validation script.

Builds a 3-D voxel-grid occupancy map of the SCARA reachable workspace,
renders a matplotlib scatter plot (occupied = red, free = light-grey),
and optionally captures a Gazebo scene screenshot.

Modes
-----
Offline (default)
    Uses the obstacle geometry defined in ``src/fret/config/perception.yaml``
    to build a synthetic ``OccupancyUpdatePayload``.  No ROS or Gazebo required.

Live ROS session (``--live``)
    Subscribes to ``/obstacle_cloud`` (``sensor_msgs/PointCloud2``) for one
    message, then builds the occupancy map from the live data.

ROS bag replay (``--bag <path>``)
    Replays a ROS 2 bag to feed ``/obstacle_cloud``, then builds the map.

Outputs (written to ``--output-dir``, default: ``outputs/``)
---------
occupancy_<timestamp>.png
    3-D matplotlib scatter: occupied voxels in red, free voxels in light-grey.
gazebo_scene_<timestamp>.png  (optional)
    Screenshot of the Gazebo viewport captured via ``/gui/screenshot`` ROS
    service, or ``scrot`` / ``grim`` as fallback.  Skipped when Gazebo is
    not reachable.

Usage::

    python3 scripts/validate_occupancy.py
    python3 scripts/validate_occupancy.py --resolution 0.10
    python3 scripts/validate_occupancy.py --live
    python3 scripts/validate_occupancy.py --bag /path/to/bag
    python3 scripts/validate_occupancy.py --output-dir /tmp/occ_out
"""

from __future__ import annotations

import argparse
import datetime
import pathlib
import subprocess
import sys
import warnings

# Allow running without an installed package by adding src/ to the path.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "src"))

import numpy as np

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.workspace_occupancy import WorkspaceOccupancyBuilder

# ---------------------------------------------------------------------------
# Default obstacle geometry (mirrors src/fret/config/perception.yaml)
# ---------------------------------------------------------------------------

#: Box obstacles for the ARCO scenario world (side = 0.10 m).
_DEFAULT_OBSTACLES: list[dict[str, object]] = [
    {"type": "box", "pose": [0.30, 0.10, 0.05], "size": [0.10, 0.10, 0.10]},
    {"type": "box", "pose": [0.20, 0.30, 0.05], "size": [0.10, 0.10, 0.10]},
    {"type": "box", "pose": [-0.10, 0.25, 0.05], "size": [0.10, 0.10, 0.10]},
]

_N_SAMPLES_PER_FACE: int = 10  # surface sample density


# ---------------------------------------------------------------------------
# Synthetic payload builder
# ---------------------------------------------------------------------------


def _sample_box(pose: list[float], size: list[float], n: int) -> np.ndarray:
    """Return ``n`` uniformly distributed surface samples for a box.

    Args:
        pose: ``[cx, cy, cz]`` box centre in the world frame [m].
        size: ``[lx, ly, lz]`` box full-side lengths [m].
        n: Number of samples *per face*.

    Returns:
        Array of shape ``(6*n, 3)``.
    """
    cx, cy, cz = pose
    lx, ly, lz = size
    rng = np.random.default_rng(seed=42)

    pts = []
    # ±X faces
    for sx in [-0.5, 0.5]:
        ys = rng.uniform(-ly / 2, ly / 2, n) + cy
        zs = rng.uniform(-lz / 2, lz / 2, n) + cz
        xs = np.full(n, cx + sx * lx)
        pts.append(np.column_stack([xs, ys, zs]))
    # ±Y faces
    for sy in [-0.5, 0.5]:
        xs = rng.uniform(-lx / 2, lx / 2, n) + cx
        zs = rng.uniform(-lz / 2, lz / 2, n) + cz
        ys = np.full(n, cy + sy * ly)
        pts.append(np.column_stack([xs, ys, zs]))
    # ±Z faces
    for sz in [-0.5, 0.5]:
        xs = rng.uniform(-lx / 2, lx / 2, n) + cx
        ys = rng.uniform(-ly / 2, ly / 2, n) + cy
        zs = np.full(n, cz + sz * lz)
        pts.append(np.column_stack([xs, ys, zs]))
    return np.vstack(pts).astype(np.float64)


def _build_offline_payload() -> OccupancyUpdatePayload:
    """Return an obstacle payload built from the default scenario geometry."""
    parts = []
    for obs in _DEFAULT_OBSTACLES:
        if obs["type"] == "box":
            parts.append(
                _sample_box(
                    list(obs["pose"]),  # type: ignore[arg-type]
                    list(obs["size"]),  # type: ignore[arg-type]
                    _N_SAMPLES_PER_FACE,
                )
            )
    if parts:
        pts = np.vstack(parts).astype(np.float64)
    else:
        pts = np.empty((0, 3), dtype=np.float64)

    return OccupancyUpdatePayload(
        obstacle_points=pts,
        timestamp=0.0,
        frame_id="world",
    )


# ---------------------------------------------------------------------------
# Matplotlib visualisation
# ---------------------------------------------------------------------------


def _render_occupancy(
    builder: WorkspaceOccupancyBuilder,
    resolution: float,
    out_path: pathlib.Path,
) -> None:
    """Render a 3-D scatter plot of the occupancy map and save as PNG.

    Args:
        builder: Fully built ``WorkspaceOccupancyBuilder``.
        resolution: Voxel resolution in metres (for title label).
        out_path: Destination PNG path.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        warnings.warn(
            "matplotlib is not available — skipping occupancy plot.",
            stacklevel=2,
        )
        return

    occ = builder.occupied_centres()
    free = builder.free_centres()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    if len(free) > 0:
        ax.scatter(
            free[:, 0],
            free[:, 1],
            free[:, 2],
            c="lightgrey",
            s=30,
            alpha=0.4,
            label=f"Free ({len(free)} voxels)",
        )
    if len(occ) > 0:
        ax.scatter(
            occ[:, 0],
            occ[:, 1],
            occ[:, 2],
            c="red",
            s=60,
            alpha=0.9,
            label=f"Occupied ({len(occ)} voxels)",
        )

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")  # type: ignore[attr-defined]
    res_cm = int(round(resolution * 100))
    ax.set_title(
        f"FRET workspace occupancy — {res_cm} cm grid\n"
        f"({len(occ)} occupied / {len(free)} free / "
        f"{len(occ) + len(free)} total annular voxels)"
    )
    ax.legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"[validate_occupancy] Occupancy plot saved: {out_path}")


# ---------------------------------------------------------------------------
# Gazebo screenshot
# ---------------------------------------------------------------------------


def _capture_gazebo_screenshot(out_path: pathlib.Path) -> bool:
    """Try to capture a screenshot of the Gazebo viewport.

    Attempts (in order):
    1. ``ros2 service call /gui/screenshot`` (Gazebo Harmonic + gz-transport).
    2. ``scrot`` (Linux screenshot tool).
    3. ``grim`` (Wayland screenshot tool).

    Args:
        out_path: Destination PNG path.

    Returns:
        ``True`` if the screenshot was saved successfully, ``False`` otherwise.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Method 1 — ROS 2 /gui/screenshot service
    try:
        result = subprocess.run(
            [
                "ros2",
                "service",
                "call",
                "/gui/screenshot",
                "gz_msgs/Screenshot",
                f'{{"path": "{out_path}"}}',
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0 and out_path.exists():
            print(f"[validate_occupancy] Gazebo screenshot saved: {out_path}")
            return True
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Method 2 — scrot
    try:
        result = subprocess.run(
            ["scrot", str(out_path)],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0 and out_path.exists():
            print(
                f"[validate_occupancy] Gazebo screenshot (scrot) saved: "
                f"{out_path}"
            )
            return True
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Method 3 — grim (Wayland)
    try:
        result = subprocess.run(
            ["grim", str(out_path)],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0 and out_path.exists():
            print(
                f"[validate_occupancy] Gazebo screenshot (grim) saved: "
                f"{out_path}"
            )
            return True
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    warnings.warn(
        "Gazebo screenshot could not be captured "
        "(ROS 2 service, scrot, and grim all unavailable or failed). "
        "Skipping screenshot.",
        stacklevel=2,
    )
    return False


# ---------------------------------------------------------------------------
# Live ROS payload acquisition
# ---------------------------------------------------------------------------


def _acquire_live_payload(timeout_s: float = 30.0) -> OccupancyUpdatePayload:
    """Subscribe to /obstacle_cloud and return the first payload received.

    Args:
        timeout_s: Maximum wait time in seconds.

    Returns:
        ``OccupancyUpdatePayload`` built from the received cloud.

    Raises:
        RuntimeError: If no message is received within ``timeout_s``.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    from fret.scene.acquisition import SceneAcquisition

    rclpy.init()
    node = Node("validate_occupancy_node")
    acquisition = SceneAcquisition(node)

    deadline = node.get_clock().now().nanoseconds / 1e9 + timeout_s
    print(
        f"[validate_occupancy] Waiting for /obstacle_cloud (timeout={timeout_s}s)…"
    )

    while node.get_clock().now().nanoseconds / 1e9 < deadline:
        rclpy.spin_once(node, timeout_sec=1.0)
        try:
            payload = acquisition.get_latest_payload()
            print(
                f"[validate_occupancy] Received cloud with "
                f"{len(payload.obstacle_points)} points."
            )
            node.destroy_node()
            rclpy.shutdown()
            return payload
        except RuntimeError:
            continue

    node.destroy_node()
    rclpy.shutdown()
    raise RuntimeError(
        f"No /obstacle_cloud message received within {timeout_s} s. "
        "Ensure PerceptionBridgeNode is running."
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    """CLI entry point.

    Returns:
        Exit code: 0 on success, 1 on failure.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Validate FRET workspace occupancy map: "
            "build, visualise, and (optionally) capture Gazebo screenshot."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.20,
        metavar="M",
        help="Voxel edge length in metres (default: 0.20)",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs",
        metavar="DIR",
        help="Directory for output files (default: outputs/)",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Acquire obstacle cloud from a live ROS 2 session.",
    )
    parser.add_argument(
        "--bag",
        metavar="PATH",
        default=None,
        help="Path to a ROS 2 bag to replay (implies --live).",
    )
    parser.add_argument(
        "--screenshot",
        action="store_true",
        default=True,
        help="Attempt to capture a Gazebo screenshot (default: True).",
    )
    parser.add_argument(
        "--no-screenshot",
        dest="screenshot",
        action="store_false",
        help="Skip the Gazebo screenshot step.",
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # ------------------------------------------------------------------
    # 1 - Acquire obstacle payload
    # ------------------------------------------------------------------
    if args.bag or args.live:
        if args.bag:
            print(f"[validate_occupancy] Replaying bag: {args.bag}")
            bag_proc = subprocess.Popen(
                ["ros2", "bag", "play", args.bag],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        try:
            payload = _acquire_live_payload()
        finally:
            if args.bag:
                bag_proc.terminate()
    else:
        print(
            "[validate_occupancy] Offline mode — using default scenario obstacles."
        )
        payload = _build_offline_payload()

    print(
        f"[validate_occupancy] Obstacle points: {len(payload.obstacle_points)}"
    )

    # ------------------------------------------------------------------
    # 2 - Build workspace occupancy map
    # ------------------------------------------------------------------
    builder = WorkspaceOccupancyBuilder(resolution=args.resolution)
    builder.build(payload)

    occ = builder.occupied_centres()
    free = builder.free_centres()
    print(
        f"[validate_occupancy] Grid built: {len(occ)} occupied, "
        f"{len(free)} free annular voxels."
    )

    # ------------------------------------------------------------------
    # 3 - Render occupancy scatter plot
    # ------------------------------------------------------------------
    occ_png = output_dir / f"occupancy_{ts}.png"
    _render_occupancy(builder, args.resolution, occ_png)

    # ------------------------------------------------------------------
    # 4 - Gazebo screenshot (best-effort)
    # ------------------------------------------------------------------
    if args.screenshot:
        gz_png = output_dir / f"gazebo_scene_{ts}.png"
        _capture_gazebo_screenshot(gz_png)

    print("[validate_occupancy] Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
