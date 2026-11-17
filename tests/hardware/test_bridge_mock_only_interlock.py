"""FR-SYS-18: refuse torque enable on a real serial port until PR4."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from fret.hardware.bridge_core import BridgeCore, BridgeState
from fret.hardware.config import (
    default_bridge_config_path,
    default_example_calibration_path,
    load_bridge_config,
)
from fret.hardware.sts.errors import TorqueEnableRefusedError
from fret.hardware.sts.transport import port_is_mock_safe


def test_mock_port_is_safe() -> None:
    assert port_is_mock_safe("mock:so101")
    assert port_is_mock_safe("replay:/tmp/t.jsonl")
    assert port_is_mock_safe("/dev/pts/3")
    assert not port_is_mock_safe("/dev/ttyUSB0")
    assert not port_is_mock_safe("/dev/ttyACM0")


def test_enable_torque_refused_on_real_port_fr_sys_18() -> None:
    cfg = load_bridge_config(
        default_bridge_config_path(),
        calibration_path=default_example_calibration_path(),
    )
    transport = MagicMock()
    transport.port_name = "/dev/ttyUSB0"
    transport.is_mock_safe = False

    # Pretend connect already succeeded.
    core = BridgeCore(transport, cfg)
    core._state = BridgeState.CONNECTED_DISABLED
    with pytest.raises(TorqueEnableRefusedError, match="FR-SYS-18"):
        core.enable_torque()
    assert core.state is BridgeState.CONNECTED_DISABLED


def test_enable_torque_allowed_on_mock() -> None:
    from fret.hardware.bridge_core import make_bridge_core_for_mock

    core, _ = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque()
    assert core.state is BridgeState.ENABLED
