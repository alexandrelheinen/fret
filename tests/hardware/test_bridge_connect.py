"""FR-HW-20: connect reads absolute positions and validates servos."""

from __future__ import annotations

import pytest

from fret.hardware.bridge_core import BridgeState, make_bridge_core_for_mock
from fret.hardware.config import (
    default_bridge_config_path,
    default_example_calibration_path,
    load_bridge_config,
)
from fret.hardware.sts.errors import StsError
from fret.hardware.sts.mock_bus import make_so101_mock
from fret.hardware.sts.registers import STS3215_MODEL_NUMBER


def test_connect_reads_positions_and_reaches_connected_disabled_fr_hw_20() -> (
    None
):
    core, _bus = make_bridge_core_for_mock()
    assert core.state is BridgeState.DISCONNECTED
    core.connect()
    assert core.state is BridgeState.CONNECTED_DISABLED
    assert core.positions.shape == (6,)
    assert abs(float(core.positions[0])) < 1e-6


def test_connect_refuses_unexpected_model_fr_hw_20() -> None:
    bus = make_so101_mock()
    bus.servo(1).model_number = 999
    bus.servo(1).registers[3] = 999 & 0xFF
    bus.servo(1).registers[4] = (999 >> 8) & 0xFF
    cfg = load_bridge_config(
        default_bridge_config_path(),
        calibration_path=default_example_calibration_path(),
    )
    core, _ = make_bridge_core_for_mock(cfg, bus=bus)
    with pytest.raises(StsError, match="unexpected model"):
        core.connect()
    assert core.state is BridgeState.DISCONNECTED
    assert STS3215_MODEL_NUMBER == 777


def test_connect_refuses_missing_servo_fr_hw_20() -> None:
    bus = make_so101_mock()
    bus._servos.pop(3)
    core, _ = make_bridge_core_for_mock(bus=bus)
    with pytest.raises(StsError):
        core.connect()


def test_protection_registers_verified_fr_hw_43() -> None:
    from fret.hardware.sts.registers import (
        MAX_TEMPERATURE_LIMIT,
        MAX_TORQUE_LIMIT,
        PROTECTION_CURRENT,
    )

    core, bus = make_bridge_core_for_mock()
    core.connect()
    safety = core._config.safety
    for sid in range(1, 7):
        servo = bus.servo(sid)
        torque = servo._read_u16(MAX_TORQUE_LIMIT[0])
        current = servo._read_u16(PROTECTION_CURRENT[0])
        temp = servo.registers.get(MAX_TEMPERATURE_LIMIT[0], -1)
        assert torque == safety.torque_limit
        assert current == safety.protection_current
        assert temp == safety.max_temperature


def test_protection_mismatch_is_connect_error_fr_hw_43() -> None:
    bus = make_so101_mock()
    core, _ = make_bridge_core_for_mock(bus=bus)

    # Sabotage readback: after first write, freeze torque limit to wrong value.
    original_write = bus.servo(1).write_bytes

    def _bad_write(address: int, data: bytes) -> None:
        original_write(address, data)
        from fret.hardware.sts.registers import MAX_TORQUE_LIMIT

        if address == MAX_TORQUE_LIMIT[0]:
            bus.servo(1)._write_u16(MAX_TORQUE_LIMIT[0], 1)

    bus.servo(1).write_bytes = _bad_write  # type: ignore[method-assign]
    with pytest.raises(StsError, match="torque_limit mismatch"):
        core.connect()
