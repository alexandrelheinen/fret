#!/usr/bin/env python3
"""Gazebo ground-truth to KD-tree occupancy bridge node.

This ROS 2 node connects Gazebo's entity pose ground truth to the pure-Python
:class:`~fret.perception.OccupancyAdapter` used by the ARCO planning pipeline.

**Pipeline overview**::

    Gazebo SceneBroadcaster
        │  /world/<world>/pose/info  (gz.msgs.Pose_V)
        ▼ (ros_gz_bridge → tf2_msgs/TFMessage → /tf)
    PerceptionBridgeNode
        │  TF lookup for each known obstacle
        ▼
    Uniform spatial sampling of obstacle bounding box
        │  TODO: replace uniform sampling with MCMC for unbiased coverage
        │        see https://alexandrelheinen.github.io/articles/2026-03-02-mcmc/
        ▼
    CloudFilter  (floor strip + range filter)
        ▼
    OccupancyAdapter.update()
        │
        ▼  /obstacle_cloud  (sensor_msgs/PointCloud2)
    ARCO planner node

**Obstacle configuration** is read from
``src/fret/config/perception.yaml`` at startup.  Each entry defines:
    - ``name``       : model name matching the Gazebo entity and TF frame id
    - ``dimensions`` : [dx, dy, dz] bounding box in metres (full extent)
    - ``samples``    : number of uniform random samples per update tick

**Usage**::

    ros2 run fret perception_bridge.py \\
        --ros-args -p config_path:=/path/to/perception.yaml
"""

from __future__ import annotations

import math
import os
import random
import sys

# Make the fret Python package importable from a colcon install tree.
# The environment hook (hooks/fret.sh) sets PYTHONPATH to include
# <install_prefix>/share, so "import fret" resolves to share/fret/.
# When running tests or from source, the src/ directory must be on
# PYTHONPATH instead.
_SHARE_PARENT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."
)
if _SHARE_PARENT not in sys.path:
    sys.path.insert(0, _SHARE_PARENT)

import rclpy  # noqa: E402  (ROS 2 import after path fixup)
import rclpy.node  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from tf2_ros import Buffer, LookupException, ExtrapolationException  # noqa: E402
from tf2_ros import ConnectivityException, TransformListener  # noqa: E402

from fret.perception.cloud_filter import CloudFilter  # noqa: E402
from fret.perception.occupancy_adapter import OccupancyAdapter  # noqa: E402

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_DEFAULT_CONFIG_PATH: str = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "config",
    "perception.yaml",
)
_PUBLISH_RATE_HZ: float = 1.0  # occupancy update frequency
_TF_TIMEOUT_SEC: float = 0.5  # TF lookup timeout per obstacle
_OBSTACLE_CLOUD_TOPIC: str = "/obstacle_cloud"
_WORLD_FRAME: str = "world"

# PointCloud2 field layout (XYZ, 32-bit float each)
_PC2_FIELDS: list[PointField] = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP: int = 12  # 3 × 4 bytes


class PerceptionBridgeNode(rclpy.node.Node):
    """ROS 2 node: Gazebo ground truth → OccupancyAdapter.

    Subscribes implicitly to ``/tf`` and ``/tf_static`` via :class:`tf2_ros.Buffer`.
    Publishes a :class:`sensor_msgs.msg.PointCloud2` on ``/obstacle_cloud``
    at :data:`_PUBLISH_RATE_HZ` Hz.
    """

    def __init__(self) -> None:
        super().__init__("perception_bridge")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("config_path", _DEFAULT_CONFIG_PATH)
        self.declare_parameter("inflation_radius", 0.05)
        self.declare_parameter("rng_seed", 42)

        config_path: str = (
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        inflation_radius: float = (
            self.get_parameter("inflation_radius")
            .get_parameter_value()
            .double_value
        )
        rng_seed: int = (
            self.get_parameter("rng_seed").get_parameter_value().integer_value
        )

        # ── Load obstacle definitions ──────────────────────────────────
        self._obstacles: list[dict] = self._load_config(config_path)
        self.get_logger().info(
            f"Loaded {len(self._obstacles)} obstacle(s) from {config_path}"
        )

        # ── Pure-Python pipeline components ───────────────────────────
        self._cloud_filter = CloudFilter(
            floor_z=0.0,
            floor_margin=0.01,
            min_range=0.01,
            max_range=5.0,
        )
        self._occupancy = OccupancyAdapter(inflation_radius=inflation_radius)
        self._rng = random.Random(rng_seed)

        # ── TF infrastructure ──────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Publisher ─────────────────────────────────────────────────
        self._cloud_pub = self.create_publisher(
            PointCloud2, _OBSTACLE_CLOUD_TOPIC, 10
        )

        # ── Periodic update timer ─────────────────────────────────────
        period = 1.0 / _PUBLISH_RATE_HZ
        self._timer = self.create_timer(period, self._update_occupancy)

        self.get_logger().info(
            f"PerceptionBridgeNode ready — publishing to {_OBSTACLE_CLOUD_TOPIC}"
            f" at {_PUBLISH_RATE_HZ} Hz"
        )

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _update_occupancy(self) -> None:
        """Sample each obstacle and refresh the OccupancyAdapter."""
        all_points: list[list[float]] = []

        for obstacle in self._obstacles:
            pts = self._sample_obstacle(obstacle)
            all_points.extend(pts)

        if not all_points:
            return

        # Pass through floor/range filter
        filtered = self._cloud_filter.apply(all_points)

        # Rebuild KD-tree occupancy model
        self._occupancy.update(filtered)

        # Publish PointCloud2 so other nodes (e.g. the planner) can
        # consume the same cloud without rebuilding it.
        msg = self._build_pc2(filtered)
        self._cloud_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _sample_obstacle(self, obstacle: dict) -> list[list[float]]:
        """Generate uniform random samples inside obstacle bounding box.

        Args:
            obstacle: Dict with keys ``name`` (str), ``dimensions``
                ([dx, dy, dz] in metres), and ``samples`` (int).

        Returns:
            List of ``[x, y, z]`` points in the world frame.  Returns
            an empty list when the TF lookup fails.

        Note:
            TODO: Replace uniform sampling with MCMC for unbiased spatial
            coverage. See
            https://alexandrelheinen.github.io/articles/2026-03-02-mcmc/
        """
        name: str = obstacle["name"]
        dx, dy, dz = obstacle["dimensions"]
        n_samples: int = obstacle.get("samples", 50)

        # Look up obstacle pose from TF (Gazebo SceneBroadcaster provides
        # entity poses on /tf after the ros_gz_bridge translates
        # /world/<world>/pose/info → tf2_msgs/TFMessage).
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                _WORLD_FRAME,
                name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=_TF_TIMEOUT_SEC),
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as exc:
            self.get_logger().debug(
                f"TF lookup failed for '{name}': {exc}"
            )
            return []

        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        oz = tf.transform.translation.z

        # Uniform random sampling inside the axis-aligned box.
        # TODO: Replace with MCMC sampling for better coverage.
        # See https://alexandrelheinen.github.io/articles/2026-03-02-mcmc/
        points: list[list[float]] = []
        for _ in range(n_samples):
            x = ox + self._rng.uniform(-dx / 2.0, dx / 2.0)
            y = oy + self._rng.uniform(-dy / 2.0, dy / 2.0)
            z = oz + self._rng.uniform(-dz / 2.0, dz / 2.0)
            points.append([x, y, z])

        return points

    @staticmethod
    def _build_pc2(points: list[list[float]]) -> PointCloud2:
        """Serialise a list of [x, y, z] triples to a PointCloud2 message.

        Args:
            points: List of [x, y, z] triples (metres).

        Returns:
            Populated :class:`sensor_msgs.msg.PointCloud2` message.
        """
        import struct

        msg = PointCloud2()
        msg.header.frame_id = _WORLD_FRAME
        msg.height = 1
        msg.width = len(points)
        msg.fields = _PC2_FIELDS
        msg.is_bigendian = False
        msg.point_step = _POINT_STEP
        msg.row_step = _POINT_STEP * len(points)
        msg.is_dense = True

        raw = bytearray(msg.row_step)
        for i, (x, y, z) in enumerate(points):
            offset = i * _POINT_STEP
            struct.pack_into("fff", raw, offset, float(x), float(y), float(z))
        msg.data = bytes(raw)
        return msg

    @staticmethod
    def _load_config(path: str) -> list[dict]:
        """Load and validate the perception YAML config.

        Args:
            path: Absolute path to the YAML configuration file.

        Returns:
            List of obstacle definition dicts.

        Raises:
            FileNotFoundError: If *path* does not exist.
            KeyError: If a required key is missing in an obstacle entry.
        """
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"Perception config not found: {path}"
            )
        with open(path, "r", encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)

        obstacles = cfg.get("perception", {}).get("obstacles", [])
        for obs in obstacles:
            if "name" not in obs or "dimensions" not in obs:
                raise KeyError(
                    f"Obstacle entry missing 'name' or 'dimensions': {obs}"
                )
        return obstacles


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    """Start the PerceptionBridgeNode."""
    rclpy.init(args=args)
    node = PerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
