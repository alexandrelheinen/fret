"""FR-HW-25: topic names/types/QoS parity with MuJoCoBridgeNode."""

from __future__ import annotations

import inspect

from fret.hardware import bridge_node as hw_bridge
from fret.ros import mujoco_bridge as mj_bridge


def test_topic_contract_documented_parity_fr_hw_25() -> None:
    hw_doc = hw_bridge.BridgeNode.__doc__ or ""
    mj_doc = mj_bridge.MuJoCoBridgeNode.__doc__ or ""
    for topic in ("/joint_commands", "/joint_states"):
        assert topic in hw_doc
        assert topic in mj_doc
    assert "Float64MultiArray" in hw_doc
    assert "JointState" in hw_doc
    assert "BEST_EFFORT" in hw_doc


def test_setup_ros_subscribes_same_topics_fr_hw_25() -> None:
    src = inspect.getsource(hw_bridge.BridgeNode._setup_ros)
    assert '"/joint_commands"' in src
    assert '"/joint_states"' in src
    assert "BEST_EFFORT" in src
    assert "Float64MultiArray" in src
    assert "JointState" in src
