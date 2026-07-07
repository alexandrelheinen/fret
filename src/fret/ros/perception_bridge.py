"""PerceptionBridgeNode: publishes a synthetic obstacle point cloud.

Reads obstacle definitions from ``perception.yaml``, samples surface points
uniformly for each obstacle, and publishes the merged cloud on
``/obstacle_cloud`` (``sensor_msgs/PointCloud2``) in the ``world`` frame.

Publishes:
    /obstacle_cloud  (``sensor_msgs/PointCloud2``, TRANSIENT_LOCAL)

Parameters (from ``perception.yaml`` via the launch file):
    update_hz (float, default 1.0): Publishing rate in Hz.
    surface_density (float, default 2000.0): Sampling density [points/m²].

Satisfies requirements FR-PER-01, FR-PER-02, FR-PER-03.
"""

from __future__ import annotations

import math
import os
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml

# ---------------------------------------------------------------------------
# Pure-Python sampling helpers (no ROS dependency)
# ---------------------------------------------------------------------------


def _rotation_matrix_rpy(
    roll: float, pitch: float, yaw: float
) -> npt.NDArray[np.float64]:
    """Build a 3×3 rotation matrix from roll-pitch-yaw (intrinsic XYZ).

    Args:
        roll: Rotation about X axis [rad].
        pitch: Rotation about Y axis [rad].
        yaw: Rotation about Z axis [rad].

    Returns:
        A (3, 3) rotation matrix.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _apply_pose(
    points_local: npt.NDArray[np.float64],
    pose: list[float],
) -> npt.NDArray[np.float64]:
    """Transform points from local frame to world frame using a 6-DOF pose.

    Args:
        points_local: Array of shape (N, 3) in the obstacle's local frame.
        pose: ``[x, y, z, roll, pitch, yaw]`` of the obstacle in world frame.

    Returns:
        Array of shape (N, 3) in the world frame.
    """
    tx, ty, tz, roll, pitch, yaw = pose
    R = _rotation_matrix_rpy(roll, pitch, yaw)
    t = np.array([tx, ty, tz], dtype=np.float64)
    return points_local @ R.T + t


def _sample_box_surface(
    size: list[float],
    pose: list[float],
    density: float,
) -> npt.NDArray[np.float64]:
    """Sample points uniformly on all six faces of an axis-aligned box.

    Args:
        size: ``[sx, sy, sz]`` dimensions of the box [m].
        pose: ``[x, y, z, roll, pitch, yaw]`` world-frame pose.
        density: Sampling density [points/m²].

    Returns:
        Array of shape (N, 3) with sampled points in world frame.
    """
    sx, sy, sz = float(size[0]), float(size[1]), float(size[2])
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

    # Each face: (area, sampler callable)
    all_pts: list[npt.NDArray[np.float64]] = []

    # ±X faces (area = sy * sz)
    for sign in (-1.0, 1.0):
        n = max(1, int(round(density * sy * sz)))
        u = np.random.uniform(-hy, hy, n)
        v = np.random.uniform(-hz, hz, n)
        pts = np.column_stack([np.full(n, sign * hx), u, v]).astype(np.float64)
        all_pts.append(pts)

    # ±Y faces (area = sx * sz)
    for sign in (-1.0, 1.0):
        n = max(1, int(round(density * sx * sz)))
        u = np.random.uniform(-hx, hx, n)
        v = np.random.uniform(-hz, hz, n)
        pts = np.column_stack([u, np.full(n, sign * hy), v]).astype(np.float64)
        all_pts.append(pts)

    # ±Z faces (area = sx * sy)
    for sign in (-1.0, 1.0):
        n = max(1, int(round(density * sx * sy)))
        u = np.random.uniform(-hx, hx, n)
        v = np.random.uniform(-hy, hy, n)
        pts = np.column_stack([u, v, np.full(n, sign * hz)]).astype(np.float64)
        all_pts.append(pts)

    local = np.vstack(all_pts)
    return _apply_pose(local, pose)


def _sample_cylinder_surface(
    radius: float,
    length: float,
    pose: list[float],
    density: float,
) -> npt.NDArray[np.float64]:
    """Sample points uniformly on a cylinder (lateral surface + 2 caps).

    The cylinder axis is aligned with the local Z axis, centred at origin.

    Args:
        radius: Cylinder radius [m].
        length: Cylinder length (height) [m].
        pose: ``[x, y, z, roll, pitch, yaw]`` world-frame pose.
        density: Sampling density [points/m²].

    Returns:
        Array of shape (N, 3) with sampled points in world frame.
    """
    half_len = length / 2.0
    all_pts: list[npt.NDArray[np.float64]] = []

    # Lateral surface: area = 2π r h
    lateral_area = 2.0 * math.pi * radius * length
    n_lat = max(1, int(round(density * lateral_area)))
    theta = np.random.uniform(0.0, 2.0 * math.pi, n_lat)
    z_lat = np.random.uniform(-half_len, half_len, n_lat)
    lat_pts = np.column_stack(
        [radius * np.cos(theta), radius * np.sin(theta), z_lat]
    ).astype(np.float64)
    all_pts.append(lat_pts)

    # Two circular caps: area = π r² each
    cap_area = math.pi * radius**2
    for sign in (-1.0, 1.0):
        n_cap = max(1, int(round(density * cap_area)))
        # Rejection sampling for uniform disk
        cap_pts_list: list[npt.NDArray[np.float64]] = []
        collected = 0
        while collected < n_cap:
            needed = n_cap - collected
            ux = np.random.uniform(-radius, radius, needed * 2)
            uy = np.random.uniform(-radius, radius, needed * 2)
            mask = ux**2 + uy**2 <= radius**2
            pts_accepted = np.column_stack(
                [ux[mask], uy[mask], np.full(mask.sum(), sign * half_len)]
            ).astype(np.float64)
            cap_pts_list.append(pts_accepted)
            collected += pts_accepted.shape[0]
        cap_all = np.vstack(cap_pts_list)[:n_cap]
        all_pts.append(cap_all)

    local = np.vstack(all_pts)
    return _apply_pose(local, pose)


def _load_config(config_path: str) -> dict[str, Any]:
    """Load and validate the perception YAML configuration.

    Args:
        config_path: Absolute or relative path to ``perception.yaml``.

    Returns:
        Parsed YAML as a nested dict.

    Raises:
        FileNotFoundError: If the config file cannot be found.
        KeyError: If the ``perception`` top-level key is missing.
    """
    with open(config_path, "r") as fh:
        raw: dict[str, Any] = yaml.safe_load(fh)
    if "perception" not in raw:
        raise KeyError(f"'perception' key missing in {config_path}")
    return dict(raw["perception"])


def _resolve_config_path() -> str:
    """Resolve the path to ``perception.yaml``.

    Tries ``ament_index`` first (installed ROS 2 package), then falls back
    to the source-tree relative path for testing.

    Returns:
        Absolute path to the config file.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("fret"), "config", "perception.yaml"
        )
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "..", "config", "perception.yaml")


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------


class PerceptionBridgeNode:
    """ROS 2 node that samples obstacle surfaces and publishes a point cloud.

    All ROS 2 imports are deferred to ``__init__`` so that the sampling helpers
    remain importable without a live ROS 2 installation (enabling unit tests).

    Args:
        config_path: Path to ``perception.yaml``.  When ``None``, resolved
            automatically via ``ament_index`` or the source-tree fallback.
    """

    def __init__(self, config_path: str | None = None) -> None:
        import rclpy
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy

        resolved = (
            config_path if config_path is not None else _resolve_config_path()
        )
        cfg = _load_config(resolved)

        self._update_hz: float = float(cfg.get("update_hz", 1.0))
        self._surface_density: float = float(
            cfg.get("surface_density", 2000.0)
        )
        self._obstacle_defs: list[dict[str, Any]] = cfg.get("obstacles", [])

        # Pre-sample all obstacle surfaces once (static obstacles).
        self._cloud_pts: npt.NDArray[np.float64] = self._build_cloud()

        class _Node(rclpy.node.Node):  # type: ignore[misc]
            pass

        self._node: rclpy.node.Node = _Node("perception_bridge_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        from sensor_msgs.msg import PointCloud2

        self._pub = self._node.create_publisher(
            PointCloud2, "/obstacle_cloud", qos
        )
        self._timer = self._node.create_timer(
            1.0 / self._update_hz, self._publish_cloud
        )
        self._node.get_logger().info(
            f"PerceptionBridgeNode ready: {self._cloud_pts.shape[0]} points, "
            f"{self._update_hz} Hz."
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_cloud(self) -> npt.NDArray[np.float64]:
        """Merge surface samples from all configured obstacles.

        Returns:
            Array of shape (N, 3) with all obstacle surface points in world frame.
            Returns an empty (0, 3) array when no obstacles are configured.
        """
        if not self._obstacle_defs:
            return np.empty((0, 3), dtype=np.float64)

        parts: list[npt.NDArray[np.float64]] = []
        for obs in self._obstacle_defs:
            pts = _sample_obstacle_static(obs, self._surface_density)
            parts.append(pts)

        return (
            np.vstack(parts) if parts else np.empty((0, 3), dtype=np.float64)
        )

    def _publish_cloud(self) -> None:
        """Timer callback: publish the pre-sampled obstacle cloud."""
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header

        header = Header()
        header.frame_id = "world"
        header.stamp = self._node.get_clock().now().to_msg()

        pts_list = (
            self._cloud_pts.tolist() if self._cloud_pts.shape[0] > 0 else []
        )
        cloud = create_cloud_xyz32(header, pts_list)
        self._pub.publish(cloud)

    # ------------------------------------------------------------------
    # Public lifecycle
    # ------------------------------------------------------------------

    def spin(self) -> None:
        """Spin the underlying ROS 2 node until shutdown."""
        try:
            self._rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()


# ---------------------------------------------------------------------------
# Module-level sampling functions (accessible without instantiating the node)
# ---------------------------------------------------------------------------


def _sample_obstacle_static(
    obs: dict[str, Any], density: float
) -> npt.NDArray[np.float64]:
    """Sample surface points for an obstacle without a node instance.

    Args:
        obs: Obstacle dict (same schema as ``_sample_obstacle``).
        density: Sampling density [points/m²].

    Returns:
        Array of shape (N, 3) in world frame.
    """
    pose: list[float] = [float(v) for v in obs["pose"]]
    obs_type: str = obs["type"]

    if obs_type == "box":
        size: list[float] = [float(v) for v in obs["size"]]
        return _sample_box_surface(size, pose, density)
    elif obs_type == "cylinder":
        radius = float(obs["radius"])
        length = float(obs["length"])
        return _sample_cylinder_surface(radius, length, pose, density)
    return np.empty((0, 3), dtype=np.float64)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``perception_bridge`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy

    rclpy.init(args=args)

    loader = rclpy.create_node("_perception_bridge_param_loader")
    loader.declare_parameter("config_path", "")
    config_path = (
        loader.get_parameter("config_path").get_parameter_value().string_value
    )
    loader.destroy_node()

    node = PerceptionBridgeNode(config_path=config_path or None)
    node.spin()
