"""Arc trajectory injector for the ``arc`` scenario.

Reads the ``arc`` scenario configuration, generates ``n_waypoints`` Cartesian
positions along a circular arc in the horizontal plane (constant z), runs
analytical IK on each waypoint to produce a joint-space trajectory, and
publishes it once on ``/joint_trajectory``.

The resulting trajectory traces a circular arc in Cartesian space, which is
nonlinear in C-space (joint space) due to the SCARA kinematics.

Publishes:
    /joint_trajectory  (``trajectory_msgs/JointTrajectory``, TRANSIENT_LOCAL)

Parameters (from ``arc.yml`` via the launch file):
    arc_center_x (float): Arc centre X position [m].
    arc_center_y (float): Arc centre Y position [m].
    arc_radius (float): Arc radius [m].
    arc_start_deg (float): Start angle in degrees from positive X axis.
    arc_end_deg (float): End angle in degrees from positive X axis.
    z_height (float): Constant EE height above the base [m].
    duration (float): Total trajectory duration [s].
    n_waypoints (int): Number of waypoints (including start and end).
"""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt


def _generate_arc_trajectory(
    center_x: float,
    center_y: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    z_height: float,
    n_waypoints: int,
    duration: float,
) -> tuple[list[npt.NDArray[np.float64]], list[float]]:
    """Generate a joint trajectory that traces a circular arc in Cartesian space.

    Samples ``n_waypoints`` positions along a circular arc defined by its
    centre, radius, and angular range.  Runs analytical IK on each Cartesian
    position to obtain joint configurations.  Falls back to linear joint-space
    interpolation between the start and end IK solutions if IK fails at an
    intermediate waypoint.

    Args:
        center_x: X coordinate of the arc centre in the world frame [m].
        center_y: Y coordinate of the arc centre in the world frame [m].
        radius: Arc radius [m].
        start_deg: Arc start angle in degrees (measured from positive X axis).
        end_deg: Arc end angle in degrees.
        z_height: Constant EE height [m]; becomes the z component of each IK
            target.
        n_waypoints: Number of waypoints including start and end (>= 2).
        duration: Total trajectory duration [s].

    Returns:
        Tuple ``(joint_configs, timestamps_sec)`` where ``joint_configs`` is a
        list of ``(3,)`` arrays (SCARA RRP: [q1, q2, q3]) and
        ``timestamps_sec`` gives ``time_from_start`` in seconds.
    """
    from fret.control.kinematics import Kinematics

    kin = Kinematics("scara")

    angles = np.linspace(
        math.radians(start_deg), math.radians(end_deg), n_waypoints
    )

    # Compute IK for start and end as fallback endpoints
    def _cartesian_to_pose(
        x: float, y: float, z: float
    ) -> npt.NDArray[np.float64]:
        pose = np.eye(4, dtype=np.float64)
        pose[0, 3] = x
        pose[1, 3] = y
        pose[2, 3] = z
        return pose

    start_x = center_x + radius * math.cos(angles[0])
    start_y = center_y + radius * math.sin(angles[0])
    end_x = center_x + radius * math.cos(angles[-1])
    end_y = center_y + radius * math.sin(angles[-1])

    try:
        q_start = kin.inverse_kinematics(
            _cartesian_to_pose(start_x, start_y, z_height)
        )
    except RuntimeError:
        q_start = np.array([0.0, 0.0, 0.10], dtype=np.float64)

    try:
        q_end = kin.inverse_kinematics(
            _cartesian_to_pose(end_x, end_y, z_height)
        )
    except RuntimeError:
        q_end = np.array([0.5, 0.0, 0.10], dtype=np.float64)

    joint_configs: list[npt.NDArray[np.float64]] = []
    timestamps: list[float] = []

    for i, angle in enumerate(angles):
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        pose = _cartesian_to_pose(x, y, z_height)
        t = float(i) / (n_waypoints - 1) * duration

        try:
            q = kin.inverse_kinematics(pose)
        except RuntimeError:
            # Linear interpolation fallback
            alpha = float(i) / (n_waypoints - 1)
            q = q_start + alpha * (q_end - q_start)

        joint_configs.append(np.asarray(q, dtype=np.float64))
        timestamps.append(t)

    return joint_configs, timestamps


class ArcInjectorNode:
    """ROS 2 node that publishes an arc joint trajectory once.

    Reads scenario parameters from the ROS 2 parameter server (populated by
    the launch file from ``arc.yml``), generates the arc trajectory, and
    publishes it on ``/joint_trajectory`` with TRANSIENT_LOCAL durability so
    that late-joining subscribers (e.g. the controller node) receive it.

    Args:
        model: Robot model name (``"scara"`` only for this milestone).
    """

    def __init__(self, model: str = "scara") -> None:
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from trajectory_msgs.msg import JointTrajectory

        rclpy.node.Node.__init__(self, "arc_injector")

        self.declare_parameter("arc_center_x", 0.30)  # type: ignore[attr-defined]
        self.declare_parameter("arc_center_y", 0.00)  # type: ignore[attr-defined]
        self.declare_parameter("arc_radius", 0.15)  # type: ignore[attr-defined]
        self.declare_parameter("arc_start_deg", -45.0)  # type: ignore[attr-defined]
        self.declare_parameter("arc_end_deg", 45.0)  # type: ignore[attr-defined]
        self.declare_parameter("z_height", 0.138)  # type: ignore[attr-defined]
        self.declare_parameter("duration", 4.0)  # type: ignore[attr-defined]
        self.declare_parameter("n_waypoints", 200)  # type: ignore[attr-defined]

        def _float(name: str) -> float:
            return float(
                self.get_parameter(name).get_parameter_value().double_value  # type: ignore[attr-defined]
            )

        def _int(name: str) -> int:
            return int(
                self.get_parameter(name).get_parameter_value().integer_value  # type: ignore[attr-defined]
            )

        center_x = _float("arc_center_x")
        center_y = _float("arc_center_y")
        radius = _float("arc_radius")
        start_deg = _float("arc_start_deg")
        end_deg = _float("arc_end_deg")
        z_height = _float("z_height")
        duration = _float("duration")
        n_waypoints = _int("n_waypoints")

        joint_configs, timestamps = _generate_arc_trajectory(
            center_x=center_x,
            center_y=center_y,
            radius=radius,
            start_deg=start_deg,
            end_deg=end_deg,
            z_height=z_height,
            n_waypoints=n_waypoints,
            duration=duration,
        )

        # Build JointTrajectory message
        from fret.control.kinematics import _JOINT_NAMES

        msg = JointTrajectory()
        msg.joint_names = list(_JOINT_NAMES)

        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        for q, t in zip(joint_configs, timestamps):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(pt)

        # Publish with TRANSIENT_LOCAL so the controller receives it even if
        # it starts after the injector
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        pub = self.create_publisher(JointTrajectory, "/joint_trajectory", qos)  # type: ignore[attr-defined]
        pub.publish(msg)

        self.get_logger().info(  # type: ignore[attr-defined]
            f"ArcInjectorNode: published {len(joint_configs)}-waypoint arc "
            f"trajectory (center=({center_x:.2f}, {center_y:.2f}) m, "
            f"radius={radius:.2f} m, "
            f"angles=[{start_deg:.1f}°, {end_deg:.1f}°], "
            f"duration={duration:.1f} s)."
        )


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``arc_injector`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy
    import rclpy.node

    rclpy.init(args=args)

    class _ConcreteInjector(ArcInjectorNode, rclpy.node.Node):  # type: ignore[misc]
        def __init__(self) -> None:
            ArcInjectorNode.__init__(self)

    node = _ConcreteInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Standalone helper (used by simulation / testing scripts)
# ---------------------------------------------------------------------------


def generate_arc_trajectory(
    center_x: float = 0.30,
    center_y: float = 0.00,
    radius: float = 0.15,
    start_deg: float = -45.0,
    end_deg: float = 45.0,
    z_height: float = 0.138,
    n_waypoints: int = 200,
    duration: float = 4.0,
) -> tuple[list[npt.NDArray[np.float64]], list[float]]:
    """Generate an arc joint trajectory (importable without ROS).

    This wrapper is used by CI simulation scripts and unit tests to produce
    reference arc trajectory data without requiring a live ROS context.

    Args:
        center_x: Arc centre X [m] (default 0.30 m).
        center_y: Arc centre Y [m] (default 0.00 m).
        radius: Arc radius [m] (default 0.15 m).
        start_deg: Start angle in degrees (default -45°).
        end_deg: End angle in degrees (default +45°).
        z_height: Constant EE height [m] (default 0.138 m).
        n_waypoints: Number of waypoints (default 200).
        duration: Total duration in seconds (default 4.0 s).

    Returns:
        Tuple ``(joint_configs, timestamps_sec)``.
    """
    return _generate_arc_trajectory(
        center_x=center_x,
        center_y=center_y,
        radius=radius,
        start_deg=start_deg,
        end_deg=end_deg,
        z_height=z_height,
        n_waypoints=n_waypoints,
        duration=duration,
    )
