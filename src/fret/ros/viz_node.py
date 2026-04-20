"""Trajectory visualisation node.

Publishes a ``visualization_msgs/MarkerArray`` on ``/fret/markers`` so that
RViz2 can show:

* A **green sphere** at the start end-effector position.
* A **red sphere** at the goal end-effector position.
* **Small cyan spheres** at every intermediate waypoint's EE position.
* A **yellow line strip** that grows in real time, tracing the actual
  end-effector path as the robot moves.

Subscribes:
    /joint_trajectory  (``trajectory_msgs/JointTrajectory``, TRANSIENT_LOCAL)
    /joint_states      (``sensor_msgs/JointState``)

Publishes:
    /fret/markers      (``visualization_msgs/MarkerArray``, TRANSIENT_LOCAL)
"""

from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt

# ---------------------------------------------------------------------------
# Marker IDs (stable across updates so RViz can overwrite correctly)
# ---------------------------------------------------------------------------

_ID_START = 0
_ID_GOAL = 1
_ID_WAYPOINTS = 2  # SPHERE_LIST for all intermediate waypoints
_ID_TRACE = 3  # LINE_STRIP growing with each joint_states update

_FRAME_ID = "base_link"


# ---------------------------------------------------------------------------
# Pure-Python core (unit-testable, no ROS import)
# ---------------------------------------------------------------------------


class VizNode:
    """Pure-Python trajectory visualisation logic.

    This class is intentionally decoupled from ROS so the marker-building
    logic can be unit-tested without a live ROS context.

    Args:
        model: Robot model name forwarded to ``Kinematics``.
    """

    def __init__(self, model: str = "scara") -> None:
        from fret.control.kinematics import Kinematics

        self._kin: Any = Kinematics(model)
        self._waypoint_positions: list[npt.NDArray[np.float64]] = []
        self._ee_trace: list[npt.NDArray[np.float64]] = []

    # ------------------------------------------------------------------
    # Trajectory ingestion
    # ------------------------------------------------------------------

    def set_trajectory(
        self, joint_waypoints: list[npt.NDArray[np.float64]]
    ) -> None:
        """Store FK-computed Cartesian positions for all waypoints.

        Resets the EE trace so a new trajectory starts a fresh line.

        Args:
            joint_waypoints: List of joint configurations, each shape ``(DOF,)``.
        """
        self._waypoint_positions = [
            self._kin.forward_kinematics(q)[:3, 3] for q in joint_waypoints
        ]
        self._ee_trace = []

    # ------------------------------------------------------------------
    # Live EE tracking
    # ------------------------------------------------------------------

    def append_ee(self, joint_positions: npt.NDArray[np.float64]) -> None:
        """Append the current FK end-effector position to the trace.

        Args:
            joint_positions: Current joint positions, shape ``(DOF,)``.
        """
        pos = self._kin.forward_kinematics(joint_positions)[:3, 3]
        self._ee_trace.append(pos.copy())

    # ------------------------------------------------------------------
    # Marker descriptors (pure dicts — no ROS types)
    # ------------------------------------------------------------------

    def build_marker_descriptors(
        self,
    ) -> list[dict[str, Any]]:
        """Return a list of marker descriptor dicts.

        Each dict has the keys ``id``, ``type``, ``points``, ``color``,
        ``scale`` so the ROS layer can construct
        ``visualization_msgs/Marker`` objects without knowing the FK logic.

        Returns:
            List of marker descriptor dicts.  Empty when no trajectory has
            been received yet.
        """
        if not self._waypoint_positions:
            return []

        descriptors: list[dict[str, Any]] = []

        start = self._waypoint_positions[0]
        goal = self._waypoint_positions[-1]

        # Start sphere (green)
        descriptors.append(
            {
                "id": _ID_START,
                "type": "sphere",
                "points": [start.tolist()],
                "color": (0.0, 0.9, 0.1, 1.0),  # RGBA
                "scale": (0.04, 0.04, 0.04),
            }
        )

        # Goal sphere (red)
        descriptors.append(
            {
                "id": _ID_GOAL,
                "type": "sphere",
                "points": [goal.tolist()],
                "color": (0.9, 0.1, 0.0, 1.0),
                "scale": (0.04, 0.04, 0.04),
            }
        )

        # Intermediate waypoints — SPHERE_LIST (cyan, smaller)
        intermediates = self._waypoint_positions[1:-1]
        if intermediates:
            descriptors.append(
                {
                    "id": _ID_WAYPOINTS,
                    "type": "sphere_list",
                    "points": [p.tolist() for p in intermediates],
                    "color": (0.0, 0.8, 0.9, 0.7),
                    "scale": (0.012, 0.012, 0.012),
                }
            )

        # EE trace — LINE_STRIP (yellow, grows over time)
        if len(self._ee_trace) >= 2:
            descriptors.append(
                {
                    "id": _ID_TRACE,
                    "type": "line_strip",
                    "points": [p.tolist() for p in self._ee_trace],
                    "color": (1.0, 0.9, 0.0, 1.0),
                    "scale": (0.006, 0.006, 0.006),
                }
            )

        return descriptors

    # ------------------------------------------------------------------
    # Accessors (for tests)
    # ------------------------------------------------------------------

    @property
    def waypoint_positions(self) -> list[npt.NDArray[np.float64]]:
        """Cartesian EE positions of the loaded trajectory waypoints."""
        return self._waypoint_positions

    @property
    def ee_trace(self) -> list[npt.NDArray[np.float64]]:
        """Accumulated EE trace positions."""
        return self._ee_trace


# ---------------------------------------------------------------------------
# ROS layer
# ---------------------------------------------------------------------------


class VizRosNode:  # pragma: no cover
    """ROS 2 wrapper that feeds ``VizNode`` from live topics.

    Args:
        model: Robot model name (forwarded to ``VizNode`` / ``Kinematics``).
    """

    def __init__(self, model: str = "scara") -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory
        from visualization_msgs.msg import Marker, MarkerArray

        self._Marker = Marker
        self._MarkerArray = MarkerArray

        from fret.control.kinematics import _JOINT_NAMES

        self._joint_names: list[str] = list(_JOINT_NAMES)

        self._core = VizNode(model)

        self._node: Node = rclpy.create_node(
            "viz_node",
            parameter_overrides=[],
        )

        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self._node.create_publisher(
            MarkerArray, "/fret/markers", transient_local_qos
        )

        self._node.create_subscription(
            JointTrajectory,
            "/joint_trajectory",
            self._on_trajectory,
            transient_local_qos,
        )

        self._node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )

        self._node.get_logger().info(
            f"VizNode ready (model={model}). Waiting for /joint_trajectory…"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_trajectory(self, msg: Any) -> None:
        """Parse trajectory message and pre-compute waypoint EE positions."""
        from fret.control.kinematics import _JOINT_NAMES

        joint_order = list(msg.joint_names)
        waypoints: list[npt.NDArray[np.float64]] = []

        for pt in msg.points:
            pos_raw = dict(zip(joint_order, pt.positions))
            q = np.array(
                [pos_raw.get(n, 0.0) for n in _JOINT_NAMES], dtype=np.float64
            )
            waypoints.append(q)

        self._core.set_trajectory(waypoints)
        self._node.get_logger().info(
            f"VizNode: loaded {len(waypoints)} waypoints from trajectory."
        )
        self._publish()

    def _on_joint_states(self, msg: Any) -> None:
        """Append current EE position to the trace and re-publish."""
        name_to_pos = dict(zip(msg.name, msg.position))
        q = np.array(
            [name_to_pos.get(n, 0.0) for n in self._joint_names],
            dtype=np.float64,
        )
        self._core.append_ee(q)
        self._publish()

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self) -> None:
        """Build and publish the full MarkerArray."""
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA, Header

        now = self._node.get_clock().now().to_msg()
        descriptors = self._core.build_marker_descriptors()

        markers = []
        for desc in descriptors:
            m = self._Marker()
            m.header.frame_id = _FRAME_ID
            m.header.stamp = now
            m.ns = "fret"
            m.id = desc["id"]
            m.action = self._Marker.ADD

            mtype = desc["type"]
            if mtype == "sphere":
                m.type = self._Marker.SPHERE
                p = desc["points"][0]
                m.pose.position.x = p[0]
                m.pose.position.y = p[1]
                m.pose.position.z = p[2]
                m.pose.orientation.w = 1.0
            elif mtype == "sphere_list":
                m.type = self._Marker.SPHERE_LIST
                m.pose.orientation.w = 1.0
                for p in desc["points"]:
                    pt = Point()
                    pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
                    m.points.append(pt)
            elif mtype == "line_strip":
                m.type = self._Marker.LINE_STRIP
                m.pose.orientation.w = 1.0
                for p in desc["points"]:
                    pt = Point()
                    pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
                    m.points.append(pt)

            sx, sy, sz = desc["scale"]
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz

            r, g, b, a = desc["color"]
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = float(a)

            # Keep markers visible indefinitely
            m.lifetime = Duration(sec=0, nanosec=0)

            markers.append(m)

        arr = self._MarkerArray()
        arr.markers = markers
        self._pub.publish(arr)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def spin(self) -> None:
        """Spin the node until shutdown."""
        import rclpy

        rclpy.spin(self._node)

    def destroy(self) -> None:
        """Destroy the underlying ROS node."""
        self._node.destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``viz_node`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy

    rclpy.init(args=args)
    node = VizRosNode()
    try:
        node.spin()
    finally:
        node.destroy()
        rclpy.shutdown()
