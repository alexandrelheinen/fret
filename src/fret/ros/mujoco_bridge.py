"""MuJoCo backend adapter for FRET simulator I/O (v1.0).

Level-3 ``MuJoCoBridgeCore`` integrates joint velocity commands against an
MJCF scene without requiring a live ROS context.  When the optional
``mujoco`` package is installed the core also keeps an ``MjModel`` /
``MjData`` pair in sync for forward kinematics and future rendering hooks.

Level-4 ``MuJoCoBridgeNode`` subscribes to ``/joint_commands`` and publishes
``/joint_states`` at the configured rate (default 50 Hz).

Satisfies requirements FR-SIM-01 and FR-SIM-02.
"""

from __future__ import annotations

import os
import pathlib
from typing import Any

import numpy as np
import numpy.typing as npt

from fret.control.kinematics_ppp import PPPKinematics

# MJCF preview scale (1:5) limits for ppp_warehouse.xml.
_PPP_MJCF_LIMITS: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 12.0],
        [0.0, 4.0],
        [0.0, 3.0],
    ],
    dtype=np.float64,
)

_DEFAULT_UPDATE_RATE_HZ: float = 50.0


def _project_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def resolve_mjcf_path(
    model: str,
    scenario: str,
    mjcf_override: str | pathlib.Path | None = None,
) -> pathlib.Path:
    """Return the MJCF file path for a model/scenario pair.

    Args:
        model: Robot model name (e.g. ``ppp``).
        scenario: Scenario stem (e.g. ``ppp_warehouse``).
        mjcf_override: Optional explicit MJCF path.

    Returns:
        Resolved path to an existing MJCF file.

    Raises:
        FileNotFoundError: If an override path does not exist.
        ValueError: If the model/scenario combination is unsupported.
    """
    if mjcf_override is not None:
        path = pathlib.Path(mjcf_override)
        if not path.is_file():
            raise FileNotFoundError(f"MJCF not found: {path}")
        return path

    if model == "ppp" and scenario in {"ppp_warehouse", "ppp"}:
        candidate = _project_root() / "mjcf" / "ppp_warehouse.xml"
        if candidate.is_file():
            return candidate

    raise ValueError(
        f"Unsupported model/scenario combination: model={model!r}, "
        f"scenario={scenario!r}"
    )


def integrate_joint_velocities(
    positions: npt.NDArray[np.float64],
    velocities: npt.NDArray[np.float64],
    dt: float,
    limits: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """Integrate joint velocities and clip to per-axis limits.

    Args:
        positions: Current joint positions, shape ``(DOF,)``.
        velocities: Commanded joint velocities [m/s or rad/s], shape ``(DOF,)``.
        dt: Integration timestep [s].
        limits: Joint limits array, shape ``(DOF, 2)`` with ``[lower, upper]``.

    Returns:
        Updated joint positions, shape ``(DOF,)``.
    """
    if positions.shape != velocities.shape:
        raise ValueError("positions and velocities must have the same shape")
    if limits.shape != (positions.shape[0], 2):
        raise ValueError(
            f"limits must have shape ({positions.shape[0]}, 2), got {limits.shape}"
        )
    updated = positions + velocities * dt
    return np.clip(updated, limits[:, 0], limits[:, 1]).astype(np.float64)


def make_mujoco_bridge_core(
    model: str,
    scenario: str,
    *,
    mjcf_path: str | pathlib.Path | None = None,
    initial_positions: npt.NDArray[np.float64] | None = None,
) -> MuJoCoBridgeCore:
    """Build a model-appropriate MuJoCo bridge core (FR-SYS-01).

    Args:
        model: Robot model name (``ppp`` supported in v1.0).
        scenario: Scenario stem used for MJCF resolution.
        mjcf_path: Optional explicit MJCF path override.
        initial_positions: Optional initial joint configuration.

    Returns:
        Configured ``MuJoCoBridgeCore`` instance.

    Raises:
        ValueError: If ``model`` is not recognised.
    """
    if model != "ppp":
        raise ValueError(f"Unknown MuJoCo bridge model: {model!r}")

    kin = PPPKinematics()
    resolved = resolve_mjcf_path(model, scenario, mjcf_path)
    q0 = (
        np.zeros(kin.dof, dtype=np.float64)
        if initial_positions is None
        else np.asarray(initial_positions, dtype=np.float64)
    )
    if q0.shape != (kin.dof,):
        raise ValueError(f"initial_positions must have shape ({kin.dof},)")

    return MuJoCoBridgeCore(
        mjcf_path=resolved,
        joint_names=kin.joint_names,
        limits=_PPP_MJCF_LIMITS,
        initial_positions=q0,
    )


class MuJoCoBridgeCore:
    """Level-3 MuJoCo joint I/O core.

      Integrates velocity commands in joint space and optionally mirrors the
    state into a MuJoCo model when the package is available.

      Args:
          mjcf_path: Path to the MJCF scene file.
          joint_names: Ordered joint names matching command/state vectors.
          limits: Joint limits array, shape ``(DOF, 2)``.
          initial_positions: Starting joint configuration.
    """

    def __init__(
        self,
        mjcf_path: str | pathlib.Path,
        joint_names: list[str],
        limits: npt.NDArray[np.float64],
        initial_positions: npt.NDArray[np.float64],
    ) -> None:
        self._mjcf_path = pathlib.Path(mjcf_path)
        self._joint_names = list(joint_names)
        self._limits = np.asarray(limits, dtype=np.float64)
        self._positions = np.clip(
            np.asarray(initial_positions, dtype=np.float64),
            self._limits[:, 0],
            self._limits[:, 1],
        )
        self._velocities = np.zeros_like(self._positions)
        self._mujoco: Any | None = None
        self._model: Any | None = None
        self._data: Any | None = None
        self._qpos_adrs: list[int] = []
        self._load_mujoco_optional()
        self._write_mujoco_state()

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names."""
        return list(self._joint_names)

    @property
    def limits(self) -> npt.NDArray[np.float64]:
        """Joint limits array, shape ``(DOF, 2)``."""
        return self._limits.copy()

    @property
    def mjcf_path(self) -> pathlib.Path:
        """Loaded MJCF scene path."""
        return self._mjcf_path

    @property
    def has_mujoco_runtime(self) -> bool:
        """Return ``True`` when the optional MuJoCo package is active."""
        return self._model is not None

    def get_positions(self) -> npt.NDArray[np.float64]:
        """Return a copy of the current joint positions."""
        return self._positions.copy()

    def get_velocities(self) -> npt.NDArray[np.float64]:
        """Return a copy of the most recent joint velocities."""
        return self._velocities.copy()

    def set_positions(self, positions: npt.NDArray[np.float64]) -> None:
        """Set joint positions directly (clipped to limits).

        Args:
            positions: Joint configuration, shape ``(DOF,)``.
        """
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != self._positions.shape:
            raise ValueError(
                f"Expected shape {self._positions.shape}, got {q.shape}"
            )
        self._positions = np.clip(q, self._limits[:, 0], self._limits[:, 1])
        self._write_mujoco_state()

    def step(
        self,
        velocities: npt.NDArray[np.float64],
        dt: float,
    ) -> npt.NDArray[np.float64]:
        """Integrate one control timestep from a velocity command.

        Args:
            velocities: Commanded joint velocities, shape ``(DOF,)``.
            dt: Integration period [s].

        Returns:
            Updated joint positions after integration.
        """
        v = np.asarray(velocities, dtype=np.float64)
        if v.shape != self._positions.shape:
            raise ValueError(
                f"Expected velocity shape {self._positions.shape}, got {v.shape}"
            )
        self._velocities = v.copy()
        self._positions = integrate_joint_velocities(
            self._positions,
            self._velocities,
            dt,
            self._limits,
        )
        self._write_mujoco_state()
        return self._positions.copy()

    def _load_mujoco_optional(self) -> None:
        try:
            import mujoco
        except ImportError:
            return

        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_path(str(self._mjcf_path))
        self._data = mujoco.MjData(self._model)
        adrs: list[int] = []
        for name in self._joint_names:
            joint_id = mujoco.mj_name2id(
                self._model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            if joint_id < 0:
                raise ValueError(f"Joint not found in MJCF: {name}")
            adrs.append(int(self._model.jnt_qposadr[joint_id]))
        self._qpos_adrs = adrs

    def _write_mujoco_state(self) -> None:
        if self._model is None or self._data is None or self._mujoco is None:
            return
        for idx, adr in enumerate(self._qpos_adrs):
            self._data.qpos[adr] = float(self._positions[idx])
        self._mujoco.mj_forward(self._model, self._data)


def _resolve_config_path(config_path: str | None) -> str:
    if config_path is not None:
        return config_path
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("fret")
        return os.path.join(share, "config", "simulation", "mujoco.yml")
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "..", "config", "simulation", "mujoco.yml")


def _load_bridge_config(config_path: str) -> dict[str, Any]:
    import yaml

    with open(config_path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        return {}
    for section in data.values():
        if isinstance(section, dict):
            params = section.get("ros__parameters", {})
            if isinstance(params, dict):
                return params
    return {}


class MuJoCoBridgeNode:
    """ROS 2 node bridging MuJoCo joint I/O to the FRET graph.

    Subscribes:
        /joint_commands  (``std_msgs/Float64MultiArray``, BEST_EFFORT)

    Publishes:
        /joint_states    (``sensor_msgs/JointState``, default 50 Hz)

    Args:
        config_path: Optional path to ``mujoco.yml``.
        model: Robot model override.
        scenario: Scenario stem override.
        mjcf_path: Optional MJCF path override.
    """

    def __init__(
        self,
        config_path: str | None = None,
        *,
        model: str | None = None,
        scenario: str | None = None,
        mjcf_path: str | None = None,
    ) -> None:
        import rclpy
        import rclpy.node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._rclpy = rclpy

        class _Node(rclpy.node.Node):  # type: ignore[misc]
            pass

        self._node: rclpy.node.Node = _Node("mujoco_bridge_node")

        resolved = _resolve_config_path(config_path)
        cfg = _load_bridge_config(resolved)

        self._node.declare_parameter(
            "model", model or str(cfg.get("model", "ppp"))
        )
        self._node.declare_parameter(
            "scenario", scenario or str(cfg.get("scenario", "ppp_warehouse"))
        )
        self._node.declare_parameter(
            "update_rate",
            float(cfg.get("update_rate", _DEFAULT_UPDATE_RATE_HZ)),
        )
        self._node.declare_parameter(
            "initial_joint_positions",
            list(cfg.get("initial_joint_positions", [0.0, 0.0, 0.0])),
        )

        model_name = str(
            self._node.get_parameter("model")
            .get_parameter_value()
            .string_value
        )
        scenario_name = str(
            self._node.get_parameter("scenario")
            .get_parameter_value()
            .string_value
        )
        self._update_rate = float(
            self._node.get_parameter("update_rate")
            .get_parameter_value()
            .double_value
        )
        initial = (
            self._node.get_parameter("initial_joint_positions")
            .get_parameter_value()
            .double_array_value
        )
        q0 = np.asarray(initial, dtype=np.float64)

        self._core = make_mujoco_bridge_core(
            model_name,
            scenario_name,
            mjcf_path=mjcf_path,
            initial_positions=q0,
        )
        self._latest_cmd = np.zeros(
            self._core.get_positions().shape, dtype=np.float64
        )

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

        self._state_pub = self._node.create_publisher(
            JointState, "/joint_states", 10
        )
        self._timer = self._node.create_timer(
            1.0 / self._update_rate,
            self._timer_callback,
        )
        self._node.get_logger().info(
            "MuJoCoBridgeNode ready: "
            f"model={model_name}, scenario={scenario_name}, "
            f"mjcf={self._core.mjcf_path.name}, "
            f"rate={self._update_rate} Hz, "
            f"mujoco_runtime={self._core.has_mujoco_runtime}"
        )

    def _command_callback(self, msg: Any) -> None:
        data = list(getattr(msg, "data", []))
        dof = len(self._latest_cmd)
        if len(data) < dof:
            self._node.get_logger().warning(
                f"Received /joint_commands with {len(data)} values; expected {dof}."
            )
            return
        self._latest_cmd = np.asarray(data[:dof], dtype=np.float64)

    def _timer_callback(self) -> None:
        from sensor_msgs.msg import JointState

        dt = 1.0 / self._update_rate
        positions = self._core.step(self._latest_cmd, dt)
        velocities = self._core.get_velocities()

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.name = self._core.joint_names
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        self._state_pub.publish(msg)

    def spin(self) -> None:
        """Spin the underlying ROS 2 node until shutdown."""
        try:
            self._rclpy.spin(self._node)
        finally:
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """Entry point for the ``mujoco_bridge`` executable."""
    import rclpy

    rclpy.init(args=args)
    node = MuJoCoBridgeNode()
    node.spin()


__all__ = [
    "MuJoCoBridgeCore",
    "MuJoCoBridgeNode",
    "integrate_joint_velocities",
    "make_mujoco_bridge_core",
    "resolve_mjcf_path",
]
