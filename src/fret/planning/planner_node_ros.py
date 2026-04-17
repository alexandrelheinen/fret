"""ROS 2 node entry point for the planner (Level 4 — auto-trigger).

This module provides the ``planner_node`` executable entry point.  On
startup the node:

1. Reads the goal configuration, planning timeout, and scenario ID from the
   ROS 2 parameter server (populated by the launch file from the scenario
   YAML).
2. Subscribes to ``/joint_states`` to obtain the current robot configuration
   as the planning start point.
3. Once the first joint state has been received, automatically triggers the
   planning pipeline via :class:`fret.planning.planner_node.PlannerNode`
   (Level 3).
4. Publishes the resulting ``trajectory_msgs/JointTrajectory`` on
   ``/joint_trajectory`` with ``TRANSIENT_LOCAL`` durability so that
   late-joining controller nodes still receive it.

Publishes:
    /joint_trajectory  (``trajectory_msgs/JointTrajectory``, TRANSIENT_LOCAL)

Subscribes:
    /joint_states  (``sensor_msgs/JointState``, BEST_EFFORT)

Parameters:
    model (str, default: ``"scara"``)
        Robot model name forwarded to :class:`fret.planning.planner_node.PlannerNode`.
    scenario_id (str, default: ``"static_reach"``)
        Human-readable identifier used in logs and planning feedback.
    goal_configuration (float[], default: ``[0.3272, 0.4712, 0.05]``)
        Goal joint configuration (SCARA RRP: [rad, rad, m]).
    planning_timeout (float, default: ``10.0``)
        Maximum planning time in seconds (FR-PLN-01).
    start_configuration (float[], default: ``[]``)
        Optional override for the start configuration.  When empty the node
        uses the first ``/joint_states`` message it receives.

Satisfies requirements FR-PLN-01 through FR-PLN-07 at the ROS level.
"""

from __future__ import annotations

# Nanoseconds per second — used when splitting a float timestamp into
# (sec, nanosec) fields required by builtin_interfaces/Duration.
_NS_PER_SEC: int = 1_000_000_000


class PlannerRosNode:
    """Level 4 ROS 2 node that auto-triggers ARCO planning at startup.

    Subclass ``rclpy.node.Node`` at the call site (see :func:`main`) so that
    the class body itself is free of ROS imports and can be unit-tested with a
    minimal mock.

    Args:
        model: Robot model name (e.g. ``"scara"``).
    """

    def __init__(self, model: str = "scara") -> None:
        import rclpy.node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory

        rclpy.node.Node.__init__(self, "planner_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("model", model)  # type: ignore[attr-defined]
        self.declare_parameter("scenario_id", "static_reach")  # type: ignore[attr-defined]
        self.declare_parameter(  # type: ignore[attr-defined]
            "goal_configuration", [0.3272, 0.4712, 0.05]
        )
        self.declare_parameter("planning_timeout", 10.0)  # type: ignore[attr-defined]
        self.declare_parameter("start_configuration", [0.0, 0.0, 0.0])  # type: ignore[attr-defined]

        self._model = str(
            self.get_parameter("model").get_parameter_value().string_value  # type: ignore[attr-defined]
        )
        self._scenario_id = str(
            self.get_parameter("scenario_id").get_parameter_value().string_value  # type: ignore[attr-defined]
        )
        self._goal_cfg = list(
            self.get_parameter(  # type: ignore[attr-defined]
                "goal_configuration"
            )
            .get_parameter_value()
            .double_array_value
        )
        self._planning_timeout = float(
            self.get_parameter(  # type: ignore[attr-defined]
                "planning_timeout"
            )
            .get_parameter_value()
            .double_value
        )
        self._start_cfg_override = list(
            self.get_parameter(  # type: ignore[attr-defined]
                "start_configuration"
            )
            .get_parameter_value()
            .double_array_value
        )

        # ------------------------------------------------------------------
        # Publisher — TRANSIENT_LOCAL so late-joining controller receives it
        # ------------------------------------------------------------------
        tl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._traj_pub = self.create_publisher(  # type: ignore[attr-defined]
            JointTrajectory, "/joint_trajectory", tl_qos
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._start_cfg: list[float] | None = (
            self._start_cfg_override
            if len(self._start_cfg_override) > 0
            else None
        )
        self._planned: bool = False

        # ------------------------------------------------------------------
        # If start configuration is overridden, plan immediately; otherwise
        # wait for the first /joint_states message.
        # ------------------------------------------------------------------
        if self._start_cfg is not None:
            self._trigger_planning()
        else:
            be_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=1,
            )
            self._js_sub = self.create_subscription(  # type: ignore[attr-defined]
                JointState,
                "/joint_states",
                self._joint_states_callback,
                be_qos,
            )
            self.get_logger().info(  # type: ignore[attr-defined]
                "PlannerNode: waiting for first /joint_states message …"
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _joint_states_callback(self, msg: object) -> None:
        """Handle the first /joint_states message to obtain the start config.

        Args:
            msg: ``sensor_msgs/JointState`` message.
        """
        if self._planned:
            return

        import numpy as np

        positions = list(
            getattr(msg, "position", [])
        )  # type: ignore[arg-type]
        if len(positions) < 3:
            return

        self._start_cfg = list(positions[:3])
        self.get_logger().info(  # type: ignore[attr-defined]
            f"PlannerNode: start configuration received: "
            f"{[round(v, 4) for v in self._start_cfg]}"
        )
        self._trigger_planning()

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _trigger_planning(self) -> None:
        """Run the planning pipeline and publish the resulting trajectory.

        Uses :class:`fret.planning.planner_node.PlannerNode` (Level 3) as
        the collision-free path search engine and
        :class:`fret.planning.trajectory_generator.TrajectoryGenerator` to
        post-process the raw path into a timed ``JointTrajectory``.
        """
        import numpy as np

        from fret.interfaces import PlanningRequest, PlanningStatus
        from fret.planning.planner_node import PlannerNode
        from fret.planning.trajectory_generator import TrajectoryGenerator
        from fret.scene.occupancy_adapter import OccupancyAdapter

        self._planned = True

        assert self._start_cfg is not None  # guaranteed by callers
        start = np.array(self._start_cfg, dtype=np.float64)
        goal = np.array(self._goal_cfg, dtype=np.float64)

        self.get_logger().info(  # type: ignore[attr-defined]
            f"PlannerNode: planning {self._scenario_id!r} "
            f"start={start.tolist()} goal={goal.tolist()} "
            f"timeout={self._planning_timeout} s"
        )

        occ_adapter = OccupancyAdapter()
        core = PlannerNode(model=self._model, occupancy_adapter=occ_adapter)
        request = PlanningRequest(
            start_configuration=start,
            goal_configuration=goal,
            planning_timeout=self._planning_timeout,
            scenario_id=self._scenario_id,
        )

        result = core.plan(request)

        if result.status != PlanningStatus.SUCCESS:
            self.get_logger().error(  # type: ignore[attr-defined]
                f"PlannerNode: planning FAILED — "
                f"status={result.status.name} "
                f"error={result.error_code.name} "
                f"duration={result.planning_duration:.2f} s"
            )
            return

        self.get_logger().info(  # type: ignore[attr-defined]
            f"PlannerNode: planning SUCCESS in "
            f"{result.planning_duration:.2f} s — "
            f"{len(result.path)} waypoints"
        )

        # Post-process raw path → timed trajectory
        from fret.control.kinematics import Kinematics

        kin = Kinematics(self._model)
        traj_gen = TrajectoryGenerator(kin)
        timed = traj_gen.process(result.path)

        self._publish_trajectory(timed)

    def _publish_trajectory(self, timed_traj: object) -> None:
        """Convert a post-processed trajectory to a ROS message and publish.

        Args:
            timed_traj: Object with ``joint_names`` and ``points`` attributes
                (duck-type compatible with ``trajectory_msgs/JointTrajectory``
                and with :class:`fret.planning.trajectory_generator._JointTrajectory`).
        """
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        msg = JointTrajectory()
        msg.joint_names = list(getattr(timed_traj, "joint_names", []))

        for pt in getattr(timed_traj, "points", []):
            ros_pt = JointTrajectoryPoint()
            ros_pt.positions = list(getattr(pt, "positions", []))
            ros_pt.velocities = list(getattr(pt, "velocities", []))
            t = float(getattr(pt, "time_from_start", 0.0))
            sec = int(t)
            nanosec = int((t - sec) * _NS_PER_SEC)
            ros_pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(ros_pt)

        self._traj_pub.publish(msg)  # type: ignore[attr-defined]
        self.get_logger().info(  # type: ignore[attr-defined]
            f"PlannerNode: published JointTrajectory with "
            f"{len(msg.points)} waypoints on /joint_trajectory."
        )


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``planner_node`` executable.

    Args:
        args: Optional command-line argument list forwarded to ``rclpy.init``.
    """
    import rclpy
    import rclpy.node

    class _ConcretePlannerNode(PlannerRosNode, rclpy.node.Node):  # type: ignore[misc]
        def __init__(self) -> None:
            PlannerRosNode.__init__(self)

    rclpy.init(args=args)
    node = _ConcretePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
