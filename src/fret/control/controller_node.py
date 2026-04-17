"""Jacobian-based trajectory tracking controller.

Level 3 (``ControllerNode``) implements the pure-Python state machine,
Jacobian-based tracking, and fault detection so the logic can be unit-tested
without a live ROS context.

Level 4 (``ControllerRosNode``) subclasses ``rclpy.node.Node`` and wires in
the 50 Hz timer, ``/joint_trajectory`` subscription, ``/joint_states``
subscription (via ``StateEstimator``), and ``/joint_commands`` publisher.

FSM states: ``IDLE → TRACKING → HALTED`` — see docs/interfaces.md.

Level 4 topics:
    /joint_trajectory  (``trajectory_msgs/JointTrajectory``, Reliable)  input
    /joint_commands    (``std_msgs/Float64MultiArray``, Best Effort)  output
    /controller_fault  (``std_msgs/Bool``)  True when EE error > threshold

Satisfies requirements FR-CTL-01 through FR-CTL-06.
"""

from __future__ import annotations

import enum
import pathlib
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from fret.control.kinematics import Kinematics

# ---------------------------------------------------------------------------
# Controller FSM
# ---------------------------------------------------------------------------


class _NodeState(enum.IntEnum):
    """Internal FSM state for the controller."""

    IDLE = 0
    TRACKING = 1
    HALTED = 2


# ---------------------------------------------------------------------------
# Default configuration values (mirrors config/controllers/jacobian.yml)
# ---------------------------------------------------------------------------

_SCARA_DOF: int = 3
_DEFAULT_FAULT_THRESHOLD: float = 0.020  # [m]
_DEFAULT_DAMPING: float = 0.01
_DEFAULT_MAX_VEL: float = 1.57  # [rad/s]
_DEFAULT_RATE: float = 50.0  # [Hz]
_DEFAULT_KP: float = 20.0  # Cartesian proportional gain


class ControllerNode:
    """Level 3 logic core for Jacobian-based 50 Hz trajectory tracking.

    This class encapsulates the pure-Python state machine and algorithmic
    methods.  It is intentionally decoupled from the ROS 2 Node base class
    so that the logic can be unit-tested without a live ROS context.  The
    Level 4 implementation (``ControllerRosNode``) subclasses
    ``rclpy.node.Node`` and wires in timers, subscriptions, and publishers.

    Args:
        model: Robot model name.  Accepts ``"scara"``; reserved for future
            models.
        config_path: Path to the controller YAML configuration file
            (e.g. ``config/controllers/jacobian.yml``) or a directory.  If
            the file cannot be found or parsed, default values are used
            silently.
    """

    def __init__(self, model: str, config_path: str) -> None:
        self._model = model
        self._state: _NodeState = _NodeState.IDLE
        self._fault_threshold: float = _DEFAULT_FAULT_THRESHOLD
        self._damping: float = _DEFAULT_DAMPING
        self._max_joint_velocity: float = _DEFAULT_MAX_VEL
        self._update_rate: float = _DEFAULT_RATE
        self._kp: float = _DEFAULT_KP
        self._dof: int = _SCARA_DOF
        self._current_command: npt.NDArray[np.float64] = np.zeros(
            self._dof, dtype=np.float64
        )
        self._trajectory: list[npt.NDArray[np.float64]] | None = None
        self._trajectory_index: int = 0
        self._load_config(config_path)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def _load_config(self, config_path: str) -> None:
        """Load controller parameters from a YAML file if one is found.

        Silently falls back to built-in defaults on any error.

        Args:
            config_path: Path to the YAML file or a directory.  If a
                directory is given, ``jacobian.yml`` is tried inside it.
        """
        import yaml  # type: ignore[import-untyped]

        path = pathlib.Path(config_path)
        if path.is_dir():
            path = path / "jacobian.yml"
        if not path.is_file():
            return
        try:
            with path.open() as fh:
                data: Any = yaml.safe_load(fh)
            if not isinstance(data, dict):
                return
            # ROS 2 parameter YAML layout: /**:  ros__parameters:  key: val
            for section in data.values():
                if not isinstance(section, dict):
                    continue
                params: dict[str, Any] = section.get("ros__parameters", {})
                self._fault_threshold = float(
                    params.get("fault_threshold", self._fault_threshold)
                )
                self._damping = float(
                    params.get("damping_factor", self._damping)
                )
                self._max_joint_velocity = float(
                    params.get("max_joint_velocity", self._max_joint_velocity)
                )
                self._update_rate = float(
                    params.get("update_rate", self._update_rate)
                )
                self._kp = float(params.get("kp", self._kp))
                break
        except (yaml.YAMLError, OSError, ValueError, KeyError) as exc:
            # Silently fall back to defaults; config errors are non-fatal
            _ = exc

    # ------------------------------------------------------------------
    # Trajectory management
    # ------------------------------------------------------------------

    def set_trajectory(
        self, trajectory: list[npt.NDArray[np.float64]]
    ) -> None:
        """Store a joint trajectory and transition to ``TRACKING``.

        Args:
            trajectory: Ordered list of joint configurations, each shape
                ``(DOF,)``.  Must contain at least two waypoints.

        Raises:
            ValueError: If ``trajectory`` has fewer than two waypoints.
        """
        if len(trajectory) < 2:
            raise ValueError(
                "Trajectory must contain at least 2 waypoints, "
                f"got {len(trajectory)}"
            )
        self._trajectory = [np.asarray(q, dtype=np.float64) for q in trajectory]
        self._trajectory_index = 0
        self._state = _NodeState.TRACKING

    def has_trajectory(self) -> bool:
        """Return ``True`` when a trajectory has been loaded."""
        return self._trajectory is not None

    def is_trajectory_complete(self) -> bool:
        """Return ``True`` when all trajectory waypoints have been consumed."""
        if self._trajectory is None:
            return False
        return self._trajectory_index >= len(self._trajectory)

    # ------------------------------------------------------------------
    # Jacobian tracking (pure-Python, unit-testable without ROS)
    # ------------------------------------------------------------------

    def compute_jacobian_command(
        self,
        kinematics: Kinematics,
        current_positions: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """Compute the joint velocity command for the current trajectory step.

        Uses a damped Jacobian pseudo-inverse with a proportional Cartesian
        error term (FR-CTL-02, FR-CTL-03).  The current trajectory waypoint
        is advanced after the command is computed.

        Args:
            kinematics: Kinematics engine used to compute FK and Jacobian.
            current_positions: Current joint positions, shape ``(DOF,)``.

        Returns:
            Joint velocity command array, shape ``(DOF,)`` (rad/s or m/s).
            Returns zeros when the controller is not in ``TRACKING`` state or
            the trajectory is complete.
        """
        if (
            self._state != _NodeState.TRACKING
            or self._trajectory is None
            or self._trajectory_index >= len(self._trajectory)
        ):
            return np.zeros(self._dof, dtype=np.float64)

        q_ref = self._trajectory[self._trajectory_index]

        # FK for Cartesian positions (position part of 4×4 homogeneous matrix)
        x_ref = kinematics.forward_kinematics(q_ref)[:3, 3]
        x_cur = kinematics.forward_kinematics(current_positions)[:3, 3]

        cart_error = x_ref - x_cur  # (3,)
        error_m = float(np.linalg.norm(cart_error))

        # Fault detection (FR-CTL-04)
        if self._check_fault(error_m):
            self._enter_halted()
            return self._current_command.copy()

        # Jacobian (6×3) — use only position rows (0:3)
        j_full = kinematics.jacobian(current_positions)
        j_pos = j_full[:3, :]  # (3, 3)

        # Damped pseudo-inverse: J† = Jᵀ·(J·Jᵀ + λ²I)⁻¹
        lam2 = self._damping**2
        j_jt = j_pos @ j_pos.T  # (3, 3)
        j_pinv = j_pos.T @ np.linalg.inv(j_jt + lam2 * np.eye(3))  # (3, 3)

        q_dot = j_pinv @ (self._kp * cart_error)  # (3,)

        # Clamp to maximum joint velocity (FR-CTL-03)
        q_dot = np.clip(q_dot, -self._max_joint_velocity, self._max_joint_velocity)

        self._current_command = q_dot
        self._trajectory_index += 1
        return q_dot.copy()

    def get_ee_error_m(
        self,
        kinematics: Kinematics,
        current_positions: npt.NDArray[np.float64],
    ) -> float:
        """Return the Cartesian EE position error against the current waypoint.

        Args:
            kinematics: Kinematics engine.
            current_positions: Current joint positions, shape ``(DOF,)``.

        Returns:
            Euclidean EE position error in metres.  Returns 0.0 when no
            trajectory is active or it is already complete.
        """
        if (
            self._trajectory is None
            or self._trajectory_index >= len(self._trajectory)
        ):
            return 0.0
        q_ref = self._trajectory[self._trajectory_index]
        x_ref = kinematics.forward_kinematics(q_ref)[:3, 3]
        x_cur = kinematics.forward_kinematics(current_positions)[:3, 3]
        return float(np.linalg.norm(x_ref - x_cur))

    # ------------------------------------------------------------------
    # State machine helpers (unit-testable)
    # ------------------------------------------------------------------

    def _check_fault(self, error_m: float) -> bool:
        """Return ``True`` if the EE tracking error exceeds the threshold.

        Args:
            error_m: End-effector position error in metres.

        Returns:
            ``True`` when ``error_m > fault_threshold`` (FR-CTL-04).
        """
        return error_m > self._fault_threshold

    def _enter_halted(self) -> None:
        """Transition to the ``HALTED`` state and zero all joint commands."""
        self._state = _NodeState.HALTED
        self._current_command = np.zeros(self._dof, dtype=np.float64)

    def _get_current_command(self) -> npt.NDArray[np.float64]:
        """Return a copy of the current joint velocity command.

        Returns:
            Joint velocity command array, shape ``(DOF,)``.
        """
        return self._current_command.copy()


# ---------------------------------------------------------------------------
# Level 4 — ROS 2 node wiring
# ---------------------------------------------------------------------------


class ControllerRosNode:
    """Level 4 ROS 2 node: wires ControllerNode into the ROS graph.

    Subclasses ``rclpy.node.Node``.  Subscriptions, publishers, and the
    50 Hz timer are registered in ``__init__``.  The Jacobian tracking loop
    runs inside the timer callback.

    Topics:
        /joint_trajectory  (``trajectory_msgs/JointTrajectory``) — input
        /joint_states      (``sensor_msgs/JointState``)           — input
        /joint_commands    (``std_msgs/Float64MultiArray``)        — output
        /controller_fault  (``std_msgs/Bool``)                    — output

    Args:
        model: Robot model name (``"scara"`` only).
        config_path: Path to ``jacobian.yml`` (or its parent directory).
    """

    def __init__(self, model: str, config_path: str) -> None:
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, Float64MultiArray
        from trajectory_msgs.msg import JointTrajectory

        from fret.control.kinematics import Kinematics
        from fret.control.state_estimator import StateEstimator

        # Initialise ROS 2 node (must be the first super() call)
        rclpy.node.Node.__init__(self, "controller_node")  # type: ignore[call-arg]

        self._logic = ControllerNode(model=model, config_path=config_path)
        self._kinematics = Kinematics(model=model)
        self._state_estimator = StateEstimator(
            node=self,  # type: ignore[arg-type]
            kinematics=self._kinematics,
        )

        # Reliable QoS for trajectory (guaranteed delivery)
        traj_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(  # type: ignore[attr-defined]
            JointTrajectory,
            "/joint_trajectory",
            self._trajectory_callback,
            traj_qos,
        )

        # Best-effort QoS for commands (high-frequency, loss-tolerant)
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._cmd_pub = self.create_publisher(  # type: ignore[attr-defined]
            Float64MultiArray,
            "/joint_commands",
            cmd_qos,
        )
        self._fault_pub = self.create_publisher(  # type: ignore[attr-defined]
            Bool,
            "/controller_fault",
            1,
        )

        dt = 1.0 / self._logic._update_rate
        self.create_timer(dt, self._timer_callback)  # type: ignore[attr-defined]

        self.get_logger().info(  # type: ignore[attr-defined]
            f"ControllerRosNode ready (model={model}, "
            f"rate={self._logic._update_rate} Hz)"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _trajectory_callback(self, msg: object) -> None:
        """Receive a ``JointTrajectory`` and load it into the logic core.

        Args:
            msg: A ``trajectory_msgs/JointTrajectory`` message.
        """
        import numpy as np

        from trajectory_msgs.msg import JointTrajectory  # type: ignore[import-untyped]

        traj_msg: JointTrajectory = msg  # type: ignore[assignment]
        if not traj_msg.points:
            self.get_logger().warning(  # type: ignore[attr-defined]
                "Received empty JointTrajectory — ignoring."
            )
            return

        waypoints = [
            np.array(list(pt.positions), dtype=np.float64)
            for pt in traj_msg.points
        ]
        try:
            self._logic.set_trajectory(waypoints)
            self.get_logger().info(  # type: ignore[attr-defined]
                f"Trajectory loaded: {len(waypoints)} waypoints."
            )
        except ValueError as exc:
            self.get_logger().error(  # type: ignore[attr-defined]
                f"Invalid trajectory: {exc}"
            )

    def _timer_callback(self) -> None:
        """50 Hz control loop: read state, compute command, publish."""
        from std_msgs.msg import Bool, Float64MultiArray

        try:
            robot_state = self._state_estimator.get_current_state()
        except RuntimeError:
            # No joint state received yet — wait silently
            return

        q_current = robot_state.joint_positions

        if not self._logic.has_trajectory():
            return

        q_dot = self._logic.compute_jacobian_command(
            self._kinematics, q_current
        )

        # Publish joint velocity command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = list(q_dot)
        self._cmd_pub.publish(cmd_msg)

        # Publish fault flag
        is_halted = self._logic._state == _NodeState.HALTED
        fault_msg = Bool()
        fault_msg.data = is_halted
        self._fault_pub.publish(fault_msg)


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``controller_node`` executable.

    Initialises ``rclpy``, spins ``ControllerRosNode`` until shutdown, then
    cleans up.

    Args:
        args: Optional command-line argument list (forwarded to
            ``rclpy.init``).
    """
    import rclpy
    import rclpy.node

    from ament_index_python.packages import get_package_share_directory  # type: ignore[import-untyped]

    rclpy.init(args=args)

    pkg_share = get_package_share_directory("fret")
    config_path = str(pathlib.Path(pkg_share) / "config" / "controllers")

    # ControllerRosNode inherits Node at runtime; create via composition
    node = _make_controller_ros_node(config_path=config_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _make_controller_ros_node(
    model: str = "scara",
    config_path: str = "",
) -> Any:
    """Construct a ``ControllerRosNode`` that also inherits ``rclpy.node.Node``.

    The inheritance is deferred until runtime so that the module can be
    imported and unit-tested without an active ROS context.

    Args:
        model: Robot model name.
        config_path: Path to the controller YAML config directory.

    Returns:
        A live ROS 2 node instance.
    """
    import rclpy.node

    class _ConcreteControllerNode(ControllerRosNode, rclpy.node.Node):
        """Concrete class combining ControllerRosNode with rclpy.node.Node."""

        def __init__(self) -> None:
            ControllerRosNode.__init__(
                self, model=model, config_path=config_path
            )

    return _ConcreteControllerNode()
