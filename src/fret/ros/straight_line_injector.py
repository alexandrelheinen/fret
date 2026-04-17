"""Straight-line trajectory injector for Milestone 1.

Reads the ``straight_line`` scenario configuration, generates 150 Cartesian
waypoints along the straight line from the start to the end end-effector
position, runs analytical IK on every waypoint to produce a joint-space
trajectory, and publishes it once on ``/joint_trajectory``.

The resulting trajectory is linear in Cartesian space but nonlinear in
C-space (joint space), illustrating the coupling of the SCARA kinematics.

Publishes:
    /joint_trajectory  (``trajectory_msgs/JointTrajectory``, TRANSIENT_LOCAL)
"""

from __future__ import annotations

import pathlib
from typing import Any

import numpy as np
import numpy.typing as npt


def _generate_straight_line_trajectory(
    start_q: npt.NDArray[np.float64],
    end_q: npt.NDArray[np.float64],
    n_waypoints: int,
    duration: float,
) -> tuple[list[npt.NDArray[np.float64]], list[float]]:
    """Generate a joint trajectory from a Cartesian straight line.

    Computes FK for the start and end joint configurations to obtain the
    corresponding Cartesian positions, linearly interpolates between them to
    form ``n_waypoints`` Cartesian waypoints, and runs analytical IK on each
    to produce joint configurations.  The result is nonlinear in joint space.

    Args:
        start_q: Start joint configuration, shape ``(3,)``.
        end_q: End joint configuration, shape ``(3,)``.
        n_waypoints: Number of waypoints including start and end.
        duration: Total trajectory duration in seconds.

    Returns:
        Tuple of (joint_configs, timestamps_sec) where joint_configs is a
        list of ``(3,)`` arrays and timestamps_sec gives the ``time_from_start``
        in seconds for each waypoint.
    """
    from fret.control.kinematics import Kinematics

    kin = Kinematics("scara")

    # FK to get start and end EE positions
    x_start = kin.forward_kinematics(start_q)[:3, 3]  # (3,)
    x_end = kin.forward_kinematics(end_q)[:3, 3]  # (3,)

    alphas = np.linspace(0.0, 1.0, n_waypoints)
    joint_configs: list[npt.NDArray[np.float64]] = []
    timestamps: list[float] = []

    for i, alpha in enumerate(alphas):
        # Cartesian straight-line interpolation
        x_target = x_start + alpha * (x_end - x_start)

        # Build 4×4 homogeneous pose for IK
        pose = np.eye(4, dtype=np.float64)
        pose[:3, 3] = x_target

        try:
            q = kin.inverse_kinematics(pose)
        except RuntimeError:
            # Fall back to linear joint interpolation if IK fails
            q = start_q + alpha * (end_q - start_q)

        joint_configs.append(q)
        timestamps.append(float(i) / (n_waypoints - 1) * duration)

    return joint_configs, timestamps


class StraightLineInjectorNode:
    """ROS 2 node that publishes a straight-line joint trajectory once.

    Reads scenario parameters from the ROS 2 parameter server (populated by
    the launch file from ``straight_line.yml``), generates the trajectory, and
    publishes it on ``/joint_trajectory`` with TRANSIENT_LOCAL durability so
    that late-joining subscribers (e.g. the controller node) receive it.

    Args:
        model: Robot model name (``"scara"`` only for Phase 1).
    """

    def __init__(self, model: str = "scara") -> None:
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from trajectory_msgs.msg import JointTrajectory

        rclpy.node.Node.__init__(self, "straight_line_injector")  # type: ignore[call-arg]

        self.declare_parameter("start_configuration", [0.0, 0.0, 0.10])  # type: ignore[attr-defined]
        self.declare_parameter("end_configuration", [0.785, 0.0, 0.10])  # type: ignore[attr-defined]
        self.declare_parameter("duration", 3.0)  # type: ignore[attr-defined]
        self.declare_parameter("n_waypoints", 150)  # type: ignore[attr-defined]

        start_q = np.array(
            self.get_parameter("start_configuration").get_parameter_value().double_array_value,  # type: ignore[attr-defined]
            dtype=np.float64,
        )
        end_q = np.array(
            self.get_parameter("end_configuration").get_parameter_value().double_array_value,  # type: ignore[attr-defined]
            dtype=np.float64,
        )
        duration = float(
            self.get_parameter("duration").get_parameter_value().double_value  # type: ignore[attr-defined]
        )
        n_waypoints = int(
            self.get_parameter("n_waypoints").get_parameter_value().integer_value  # type: ignore[attr-defined]
        )

        joint_configs, timestamps = _generate_straight_line_trajectory(
            start_q=start_q,
            end_q=end_q,
            n_waypoints=n_waypoints,
            duration=duration,
        )

        # Build JointTrajectory message
        from fret.control.kinematics import _JOINT_NAMES

        msg = JointTrajectory()
        msg.joint_names = list(_JOINT_NAMES)

        from trajectory_msgs.msg import JointTrajectoryPoint  # type: ignore[import-untyped]
        from builtin_interfaces.msg import Duration  # type: ignore[import-untyped]

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
            f"StraightLineInjectorNode: published {len(joint_configs)}-waypoint "
            f"trajectory (duration={duration:.1f} s)."
        )


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``straight_line_injector`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy
    import rclpy.node

    rclpy.init(args=args)

    class _ConcreteInjector(StraightLineInjectorNode, rclpy.node.Node):
        def __init__(self) -> None:
            StraightLineInjectorNode.__init__(self)

    node = _ConcreteInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Standalone helper (used by scripts/simulate_straight_line.py)
# ---------------------------------------------------------------------------


def generate_trajectory(
    start_q: npt.NDArray[np.float64] | None = None,
    end_q: npt.NDArray[np.float64] | None = None,
    n_waypoints: int = 150,
    duration: float = 3.0,
) -> tuple[list[npt.NDArray[np.float64]], list[float]]:
    """Generate a straight-line joint trajectory (importable without ROS).

    This wrapper is used by the CI simulation script to produce reference
    trajectory data without requiring a live ROS context.

    Args:
        start_q: Start joint configuration, shape ``(3,)``; defaults to
            ``[0.0, 0.0, 0.10]``.
        end_q: End joint configuration, shape ``(3,)``; defaults to
            ``[0.785, 0.0, 0.10]``.
        n_waypoints: Number of waypoints.
        duration: Total duration in seconds.

    Returns:
        Tuple of (joint_configs, timestamps_sec).
    """
    _start = np.array([0.0, 0.40, 0.10], dtype=np.float64) if start_q is None else np.asarray(start_q, dtype=np.float64)
    _end = np.array([0.785, -0.40, 0.10], dtype=np.float64) if end_q is None else np.asarray(end_q, dtype=np.float64)
    return _generate_straight_line_trajectory(_start, _end, n_waypoints, duration)
