"""Tests for fret.hardware.BridgeNode.

Acceptance criteria (FR-HW-01 through FR-HW-03):
  - Construction with a port and baud rate succeeds.
  - Receives a JointTrajectory on /joint_commands and serialises it to bytes.
  - Reads encoder feedback and re-publishes on /joint_states.
"""

from __future__ import annotations

import pytest

from fret.hardware.bridge_node import BridgeNode


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction() -> None:
    """BridgeNode must accept port and baud_rate arguments."""
    BridgeNode(port="/dev/ttyUSB0", baud_rate=115200)


@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_construction_with_different_baud() -> None:
    BridgeNode(port="/dev/ttyACM0", baud_rate=9600)
