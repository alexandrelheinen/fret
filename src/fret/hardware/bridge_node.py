"""ROS 2 shell around :class:`fret.hardware.bridge_core.BridgeCore`.

Topic contract matches ``MuJoCoBridgeNode`` (FR-HW-25):

* ``/joint_commands`` — ``std_msgs/Float64MultiArray``, BEST_EFFORT
* ``/joint_states`` — ``sensor_msgs/JointState``, ≥ 50 Hz
* ``/controller_fault`` — ``std_msgs/Bool`` (FR-HW-46)

Services (``std_srvs/Trigger``): ``~/enable``, ``~/disable``, ``~/clear``,
``~/stop`` (category-2 latched hold; never an emergency stop).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from fret.hardware.bridge_core import BridgeCore, BridgeState
from fret.hardware.config import (
    BridgeConfig,
    default_bridge_config_path,
    default_example_calibration_path,
    load_bridge_config,
)
from fret.hardware.sd_notify import WatchdogNotifier, watchdog_sec_from_env
from fret.hardware.sts.errors import StsError, TorqueEnableRefusedError
from fret.hardware.sts.fake_device import FakeStsDevice
from fret.hardware.sts.mock_bus import make_so101_mock
from fret.hardware.sts.replay import ReplayTransport
from fret.hardware.sts.transport import (
    InMemoryTransport,
    SerialTransport,
    port_is_mock_safe,
)


def _attach_mock_transact(transport: InMemoryTransport, bus: Any) -> None:
    from fret.hardware.sts import protocol as proto
    from fret.hardware.sts.errors import (
        StsChecksumError,
        StsIdMismatchError,
        StsLengthError,
        StsTimeoutError,
    )

    def _transact(
        request: bytes,
        *,
        expected_id: int | None,
        timeout_s: float | None = None,  # noqa: ARG001
        expect_replies: int = 1,
    ) -> list[Any]:
        reply_bytes = bus.handle_request(request)
        if not reply_bytes and expect_replies > 0:
            raise StsTimeoutError("mock returned no reply")
        replies: list[Any] = []
        buf = bytearray(reply_bytes)
        while len(replies) < expect_replies and buf:
            exp = expected_id if expect_replies == 1 else None
            reply, consumed = proto.parse_reply(bytes(buf), expected_id=exp)
            replies.append(reply)
            del buf[:consumed]
        if len(replies) < expect_replies:
            raise StsTimeoutError(
                f"mock replies {len(replies)}/{expect_replies}"
            )
        return replies

    transport.transact = _transact  # type: ignore[attr-defined]


def open_transport(
    port: str,
    baud_rate: int,
    *,
    serial_factory: object | None = None,
) -> Any:
    """Open a transport for ``port`` (mock: / replay: / serial / pts)."""
    if port.startswith("mock:"):
        bus = make_so101_mock()
        transport = InMemoryTransport(bus, port_name=port)
        _attach_mock_transact(transport, bus)
        return transport
    if port.startswith("replay:"):
        path = port[len("replay:") :]
        return ReplayTransport(path, port_name=port)
    return SerialTransport(port, baud_rate, serial_factory=serial_factory)


class BridgeNode:
    """ROS 2 bridge node between the ROS graph and the Feetech bus.

    Subscribes:
        /joint_commands  (``std_msgs/Float64MultiArray``, BEST_EFFORT)
        /controller_fault (``std_msgs/Bool``)

    Publishes:
        /joint_states    (``sensor_msgs/JointState``, default 50 Hz)

    Services:
        ~/enable, ~/disable, ~/clear, ~/stop (``std_srvs/Trigger``)
    """

    def __init__(
        self,
        port: str,
        baud_rate: int = 1_000_000,
        *,
        config: BridgeConfig | None = None,
        config_path: str | None = None,
        calibration_path: str | None = None,
        transport: Any | None = None,
        enable_ros: bool = True,
        auto_connect: bool = False,
        auto_enable: bool = False,
    ) -> None:
        if not port:
            raise ValueError("port must be a non-empty string")
        if baud_rate <= 0:
            raise ValueError(f"baud_rate must be positive, got {baud_rate}")

        self.port = port
        self.baud_rate = baud_rate
        if config is None:
            cfg_path = config_path or str(default_bridge_config_path())
            cal_path = calibration_path or str(
                default_example_calibration_path()
            )
            config = load_bridge_config(cfg_path, calibration_path=cal_path)
        if baud_rate != config.baud_rate:
            config = BridgeConfig(
                robot=config.robot,
                baud_rate=baud_rate,
                update_rate_hz=config.update_rate_hz,
                cycle_budget_ms=config.cycle_budget_ms,
                overrun_warn_consecutive=config.overrun_warn_consecutive,
                connect_range_margin_rad=config.connect_range_margin_rad,
                reply_timeout_s=config.reply_timeout_s,
                reconnect_backoff_s=config.reconnect_backoff_s,
                max_joint_velocity=config.max_joint_velocity,
                command_timeout_s=config.command_timeout_s,
                max_dt_s=config.max_dt_s,
                velocity_source=config.velocity_source,
                joint_names=config.joint_names,
                servo_ids=config.servo_ids,
                expected_model_number=config.expected_model_number,
                calibration=config.calibration,
                safety=config.safety,
            )
        self._config = config
        self._owned_fake: FakeStsDevice | None = None
        if transport is None:
            transport = open_transport(port, baud_rate)
        self._transport = transport
        self._core = BridgeCore(transport, config)
        self._node: Any | None = None
        self._rclpy: Any | None = None
        self._state_pub: Any | None = None
        self._watchdog: WatchdogNotifier | None = None
        watchdog_sec = watchdog_sec_from_env()
        if watchdog_sec is not None:
            self._watchdog = WatchdogNotifier(watchdog_sec)

        if enable_ros:
            self._setup_ros()

        if auto_connect:
            self.connect()
            if auto_enable:
                try:
                    self.enable_torque()
                except TorqueEnableRefusedError as exc:
                    if self._node is not None:
                        self._node.get_logger().error(str(exc))

    @property
    def core(self) -> BridgeCore:
        return self._core

    @property
    def state(self) -> BridgeState:
        return self._core.state

    def connect(self) -> None:
        self._core.connect()

    def enable_torque(self) -> None:
        self._core.enable_torque()

    def disable_torque(self) -> None:
        self._core.disable_torque()

    def clear(self) -> None:
        self._core.clear()

    def stop(self) -> None:
        """Category-2 latched hold. Not an emergency stop."""
        self._core.stop()

    def _setup_ros(self) -> None:
        import rclpy
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64MultiArray
        from std_srvs.srv import Trigger

        self._rclpy = rclpy

        class _Node(rclpy.node.Node):  # type: ignore[misc]
            pass

        self._node = _Node("bridge_node")
        self._node.declare_parameter("port", self.port)
        self._node.declare_parameter("baud_rate", float(self.baud_rate))
        self._node.declare_parameter(
            "update_rate", float(self._config.update_rate_hz)
        )
        self._node.declare_parameter("auto_enable", False)

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._node.create_subscription(
            Float64MultiArray,
            "/joint_commands",
            self._command_callback,
            cmd_qos,
        )
        self._node.create_subscription(
            Bool,
            "/controller_fault",
            self._controller_fault_callback,
            10,
        )
        self._state_pub = self._node.create_publisher(
            JointState, "/joint_states", 10
        )
        self._node.create_service(Trigger, "~/enable", self._srv_enable)
        self._node.create_service(Trigger, "~/disable", self._srv_disable)
        self._node.create_service(Trigger, "~/clear", self._srv_clear)
        self._node.create_service(Trigger, "~/stop", self._srv_stop)
        rate = float(
            self._node.get_parameter("update_rate")
            .get_parameter_value()
            .double_value
        )
        self._node.create_timer(1.0 / rate, self._timer_callback)
        self._node.get_logger().info(
            f"BridgeNode ready: port={self.port}, baud={self.baud_rate}, "
            f"rate={rate} Hz, mock_safe={port_is_mock_safe(self.port)}"
        )

    def _srv_enable(self, _request: Any, response: Any) -> Any:
        try:
            if self._core.state == BridgeState.DISCONNECTED:
                self._core.connect()
            if self._core.state == BridgeState.ENABLED:
                self._core.renew_enable_lease()
                response.success = True
                response.message = "lease renewed"
                return response
            self._core.enable_torque()
            response.success = True
            response.message = "enabled"
        except StsError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _srv_disable(self, _request: Any, response: Any) -> Any:
        self._core.disable_torque()
        response.success = True
        response.message = "disabled; arm goes limp"
        return response

    def _srv_clear(self, _request: Any, response: Any) -> Any:
        before = self._core.state
        self._core.clear()
        if (
            before is BridgeState.FAULT
            and self._core.state is BridgeState.FAULT
        ):
            response.success = False
            response.message = (
                f"clear refused; fault persists "
                f"({self._core.diagnostics.fault_reason})"
            )
        else:
            response.success = True
            response.message = "cleared"
        return response

    def _srv_stop(self, _request: Any, response: Any) -> Any:
        self._core.stop()
        response.success = True
        response.message = "stop latched (category 2 hold)"
        return response

    def _controller_fault_callback(self, msg: Any) -> None:
        self._core.set_controller_fault(bool(getattr(msg, "data", False)))

    def _command_callback(self, msg: Any) -> None:
        data = list(getattr(msg, "data", []))
        dof = self._config.dof
        if len(data) < dof:
            if self._node is not None:
                self._node.get_logger().warning(
                    f"Received /joint_commands with {len(data)} values; "
                    f"expected {dof}."
                )
            return
        self._core.command_velocity(np.asarray(data[:dof], dtype=np.float64))
        if self._core.state == BridgeState.ENABLED:
            self._core.renew_enable_lease()

    def _timer_callback(self) -> None:
        from sensor_msgs.msg import JointState

        if self._core.state == BridgeState.DISCONNECTED:
            try:
                self._core.connect()
            except StsError as exc:
                if self._node is not None:
                    self._node.get_logger().warning(f"connect failed: {exc}")
                if self._watchdog is not None:
                    self._watchdog.mark_stall()
                return
        try:
            self._core.cycle()
        except StsError as exc:
            if self._node is not None:
                self._node.get_logger().warning(f"cycle fault: {exc}")
            if self._watchdog is not None:
                self._watchdog.mark_stall()
            return
        if self._watchdog is not None:
            self._watchdog.clear_stall()
            self._watchdog.maybe_notify()
        if self._state_pub is None or self._node is None:
            return
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.name = list(self._core.joint_names)
        msg.position = self._core.positions.tolist()
        msg.velocity = self._core.velocities.tolist()
        self._state_pub.publish(msg)

    def spin(self) -> None:
        """Spin the underlying ROS 2 node until shutdown."""
        if self._rclpy is None or self._node is None:
            raise RuntimeError("ROS was not enabled for this BridgeNode")
        try:
            if self._core.state == BridgeState.DISCONNECTED:
                self._core.connect()
            self._rclpy.spin(self._node)
        finally:
            self._core.disconnect()
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``bridge_node`` executable."""
    import argparse

    import rclpy

    parser = argparse.ArgumentParser(description="FRET SO-101 hardware bridge")
    parser.add_argument("--port", default="mock:so101")
    parser.add_argument("--baud-rate", type=int, default=1_000_000)
    parser.add_argument("--calibration", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument(
        "--auto-enable",
        action="store_true",
        help="Enable torque after connect",
    )
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = BridgeNode(
        port=parsed.port,
        baud_rate=parsed.baud_rate,
        config_path=parsed.config,
        calibration_path=parsed.calibration,
        auto_connect=True,
        auto_enable=parsed.auto_enable,
    )
    node.spin()
