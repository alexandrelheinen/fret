"""Tests for fret.hardware.BridgeNode.

Acceptance criteria (FR-HW-20..25, FR-SYS-18):
  - Construction with a port and baud rate succeeds.
  - Receives velocity commands on /joint_commands and drives the bus.
  - Publishes feedback on /joint_states.
"""

from __future__ import annotations

from fret.hardware.bridge_node import BridgeNode


def test_construction() -> None:
    """BridgeNode must accept port and baud_rate arguments."""
    node = BridgeNode(
        port="mock:so101",
        baud_rate=1_000_000,
        enable_ros=False,
        auto_connect=False,
    )
    assert node.port == "mock:so101"
    assert node.baud_rate == 1_000_000


def test_construction_with_different_baud() -> None:
    node = BridgeNode(
        port="mock:test",
        baud_rate=500_000,
        enable_ros=False,
        auto_connect=False,
    )
    assert node.baud_rate == 500_000


def test_identity_safety_supervisor_seam() -> None:
    """PR4 seam: IdentitySafetySupervisor passes goals through."""
    import numpy as np

    from fret.hardware.bridge_core import IdentitySafetySupervisor

    safety = IdentitySafetySupervisor()
    goals = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    measured = np.zeros(6)
    out = safety.filter_goals(goals, measured, 0.0)
    assert np.allclose(out, goals)
