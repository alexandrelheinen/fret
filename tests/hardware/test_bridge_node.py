"""Tests for fret.hardware.BridgeNode.

Acceptance criteria (FR-HW-01 through FR-HW-03, superseded in v2.0,
pending redefinition of the v2.x hardware target; see
docs/hardware/README.md):
  - Construction with a port and baud rate succeeds.
  - Receives a JointTrajectory on /joint_commands and serialises it to bytes.
  - Reads feedback and re-publishes on /joint_states.

Both tests below are ``xfail(strict=True)`` because BridgeNode is an
intentional stub (raises NotImplementedError); remove the marker in the
same commit that lands the v2.0 Level 4 implementation.
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
