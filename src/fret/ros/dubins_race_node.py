"""ROS 2 node for the SC-v11 Dubins dual-agent warehouse race (v1.1).

Plans RRT* and SST paths at startup, then steps both ARCO tracking loops
while mirroring poses into ``dubins_race.xml`` via
:class:`~fret.ros.mujoco_bridge.DubinsRaceBridgeCore`.

Publishes:
    /joint_states  (``sensor_msgs/JointState``) — six joints for both agents

Parameters:
    scenario (str, default: ``dubins_race``)
        Scenario YAML stem under ``config/scenarios/``.
"""

from __future__ import annotations

import pathlib

import numpy as np
import yaml

_RRT_JOINTS: tuple[str, ...] = (
    "rrt_joint_x",
    "rrt_joint_y",
    "rrt_joint_yaw",
)
_SST_JOINTS: tuple[str, ...] = (
    "sst_joint_x",
    "sst_joint_y",
    "sst_joint_yaw",
)
_ALL_JOINTS: tuple[str, ...] = _RRT_JOINTS + _SST_JOINTS


def _controller_update_rate() -> float:
    from fret.sitl_config import controller_config_path

    ctrl_path = controller_config_path("dubins")
    with ctrl_path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if isinstance(data, dict):
        for section in data.values():
            if isinstance(section, dict):
                params = section.get("ros__parameters", {})
                if isinstance(params, dict):
                    return float(params.get("update_rate", 20.0))
    return 20.0


class DubinsRaceRosNode:  # pragma: no cover
    """Level-4 ROS node that runs the Dubins warehouse race loop."""

    def __init__(self) -> None:
        import rclpy.node
        from sensor_msgs.msg import JointState

        rclpy.node.Node.__init__(self, "dubins_race_node")

        self.declare_parameter("scenario", "dubins_race")  # type: ignore[attr-defined]
        scenario_stem = str(
            self.get_parameter("scenario")  # type: ignore[attr-defined]
            .get_parameter_value()
            .string_value
        )

        from fret.ros.mujoco_bridge import make_dubins_race_bridge_core
        from fret.scenario.dubins_race_runner import DubinsRaceRunner
        from fret.sitl_config import (
            controller_config_path,
            load_scenario_parameters,
            resolve_package_file,
            scenario_config_path,
        )

        scenario_path = scenario_config_path(scenario_stem)
        params = load_scenario_parameters(scenario_path)
        race_timeout = float(params.get("race_timeout", 90.0))
        dt = float(params.get("simulation_dt", 0.05))
        self._max_steps = int(race_timeout / dt)

        obstacle_rel = str(
            params.get("obstacle_file", "worlds/dubins_race_obstacles.yml")
        )
        obstacle_path = resolve_package_file(
            "config", *pathlib.Path(obstacle_rel).parts
        )

        self._runner = DubinsRaceRunner(
            scenario_path=scenario_path,
            obstacle_path=obstacle_path,
            controller_config_path=controller_config_path("dubins"),
            sync_mujoco=False,
        )
        self._rrt_plan, self._sst_plan, session = (
            self._runner.prepare_simulation()
        )
        if session is None:
            msg = (
                "Dubins race planning failed: "
                f"rrt={self._rrt_plan.path_found}, "
                f"sst={self._sst_plan.path_found}"
            )
            self.get_logger().error(msg)  # type: ignore[attr-defined]
            raise RuntimeError(msg)

        self._session = session
        self._bridge = make_dubins_race_bridge_core(
            initial_rrt=np.array(
                session.rrt_vehicle.pose,
                dtype=np.float64,
            ),
            initial_sst=np.array(
                session.sst_vehicle.pose,
                dtype=np.float64,
            ),
        )
        self._step_count = 0
        self._finished_logged = False

        update_rate = _controller_update_rate()
        self._state_pub = self.create_publisher(  # type: ignore[attr-defined]
            JointState,
            "/joint_states",
            10,
        )
        self._timer = self.create_timer(  # type: ignore[attr-defined]
            1.0 / update_rate,
            self._timer_callback,
        )
        self.get_logger().info(  # type: ignore[attr-defined]
            "DubinsRaceRosNode ready: "
            f"scenario={scenario_stem}, "
            f"mjcf={self._bridge.mjcf_path.name}, "
            f"rate={update_rate:.1f} Hz, "
            f"mujoco_runtime={self._bridge.has_mujoco_runtime}"
        )
        self._publish_state()

    def _publish_state(self) -> None:
        from sensor_msgs.msg import JointState

        rrt = self._bridge.get_rrt_pose()
        sst = self._bridge.get_sst_pose()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # type: ignore[attr-defined]
        msg.header.frame_id = "world"
        msg.name = list(_ALL_JOINTS)
        msg.position = [
            float(rrt[0]),
            float(rrt[1]),
            float(rrt[2]),
            float(sst[0]),
            float(sst[1]),
            float(sst[2]),
        ]
        self._state_pub.publish(msg)

    def _timer_callback(self) -> None:
        if self._session.finished:
            if not self._finished_logged:
                result = self._session.to_result(
                    self._rrt_plan,
                    self._sst_plan,
                    race_duration_s=self._step_count * self._session.dt,
                )
                self.get_logger().info(  # type: ignore[attr-defined]
                    "Dubins race finished: "
                    f"winner={result.winner}, "
                    f"rrt={result.rrt_time_to_goal_s:.2f}s, "
                    f"sst={result.sst_time_to_goal_s:.2f}s"
                )
                self._finished_logged = True
            return

        if self._step_count >= self._max_steps:
            self.get_logger().warning(  # type: ignore[attr-defined]
                "Dubins race reached step limit before both agents finished"
            )
            self._timer.cancel()
            return

        self._session.step()
        self._bridge.set_rrt_pose(self._session.rrt_vehicle.pose)
        self._bridge.set_sst_pose(self._session.sst_vehicle.pose)
        self._publish_state()
        self._step_count += 1


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``dubins_race_node`` executable."""
    import rclpy
    import rclpy.node

    class _ConcreteDubinsRaceNode(DubinsRaceRosNode, rclpy.node.Node):  # type: ignore[misc]
        def __init__(self) -> None:
            DubinsRaceRosNode.__init__(self)

    rclpy.init(args=args)
    node: _ConcreteDubinsRaceNode | None = None
    try:
        node = _ConcreteDubinsRaceNode()
        rclpy.spin(node)
    except Exception:
        if node is not None:
            node.get_logger().exception(
                "Dubins race node failed during startup"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["DubinsRaceRosNode", "main"]
