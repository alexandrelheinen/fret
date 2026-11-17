"""FR-SYS-17: hardware launch description + pty fake reaches CONNECTED_DISABLED."""

from __future__ import annotations

import time

import pytest

from fret.hardware.bridge_core import BridgeState
from fret.hardware.bridge_node import BridgeNode
from fret.hardware.sts.fake_device import FakeStsDevice


def test_hardware_launch_description_builds_fr_sys_17() -> None:
    from fret.launch.hardware import generate_launch_description

    ld = generate_launch_description()
    assert ld is not None
    assert len(ld.entities) >= 1


def test_bridge_on_pty_reaches_connected_disabled_fr_sys_17() -> None:
    with FakeStsDevice() as device:
        time.sleep(0.05)
        node = BridgeNode(
            port=device.port_name,
            baud_rate=1_000_000,
            enable_ros=False,
            auto_connect=False,
        )
        node.connect()
        assert node.state is BridgeState.CONNECTED_DISABLED
        # Publish path: cycle produces joint states at ≥ 50 Hz intent.
        t0 = time.monotonic()
        for i in range(10):
            node.core.cycle(t0 + i * 0.02)
        assert node.core.positions.shape == (6,)
        node.core.disconnect()
