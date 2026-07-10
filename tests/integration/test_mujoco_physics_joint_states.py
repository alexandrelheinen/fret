"""V12-4 / FR-SIM-07: /joint_states provenance under physics_mode."""

from __future__ import annotations

import pathlib
import shutil
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from fret.ros.mujoco_bridge import MuJoCoBridgeNode, make_mujoco_bridge_core

_SIM_CONFIG_DIR = (
    pathlib.Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "config"
    / "simulation"
)


def _mujoco_available() -> bool:
    return make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime


def _physics_bridge_config(tmp_path: pathlib.Path) -> pathlib.Path:
    """Copy bridge YAML pair with ``physics_mode: true`` for SITL tests."""
    for name in ("mujoco.yml", "mujoco_physics.yml"):
        shutil.copy(_SIM_CONFIG_DIR / name, tmp_path / name)
    cfg_path = tmp_path / "mujoco.yml"
    text = cfg_path.read_text(encoding="utf-8").replace(
        "physics_mode: false",
        "physics_mode: true",
    )
    cfg_path.write_text(text, encoding="utf-8")
    return cfg_path


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_physics_joint_states_match_simulated_qpos(
    ros_context: None,
    test_node: Node,
    tmp_path: pathlib.Path,
) -> None:
    """V12-4: bridge publishes mj_step qpos/qvel — no open-loop pose injection."""
    del ros_context

    cfg_path = _physics_bridge_config(tmp_path)
    bridge = MuJoCoBridgeNode(config_path=str(cfg_path))
    assert bridge._core.physics_mode is True

    received: list[JointState] = []

    def _on_joint_state(msg: JointState) -> None:
        received.append(msg)

    test_node.create_subscription(
        JointState,
        "/joint_states",
        _on_joint_state,
        10,
    )
    cmd_pub = test_node.create_publisher(Float64MultiArray, "/joint_commands", 10)

    core = bridge._core
    q0 = core.get_positions().copy()
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline and len(received) < 5:
        cmd = Float64MultiArray()
        cmd.data = [0.5, 0.0, 0.0]
        cmd_pub.publish(cmd)
        rclpy.spin_once(bridge._node, timeout_sec=0.05)
        rclpy.spin_once(test_node, timeout_sec=0.05)

    bridge._node.destroy_node()

    assert len(received) >= 2, "expected /joint_states from physics bridge"
    stamps = [
        float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9
        for msg in received
    ]
    assert stamps[-1] >= stamps[0], "joint_states stamps must be monotonic"

    final_positions = np.asarray(received[-1].position, dtype=np.float64)
    assert (
        final_positions[0] > q0[0]
    ), "physics step must advance joint X from actuator commands"
    assert received[-1].velocity, "physics bridge must publish joint velocities"

    with pytest.raises(RuntimeError, match="set_positions"):
        core.set_positions(np.zeros(3))
