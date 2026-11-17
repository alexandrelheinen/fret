"""FR-HW-24: serial transport through a pty fake device."""

from __future__ import annotations

import time

import pytest

from fret.hardware.sts import protocol as proto
from fret.hardware.sts.errors import StsTimeoutError
from fret.hardware.sts.fake_device import FakeStsDevice
from fret.hardware.sts.registers import PRESENT_POSITION
from fret.hardware.sts.transport import SerialTransport


def test_pyserial_round_trip_through_pty_fr_hw_24() -> None:
    with FakeStsDevice() as device:
        transport = SerialTransport(device.port_name, 1_000_000)
        try:
            # Allow the reader thread to start.
            time.sleep(0.05)
            request = proto.build_ping(1)
            replies = transport.transact(request, expected_id=1, timeout_s=1.0)
            assert len(replies) == 1
            assert replies[0].servo_id == 1

            request = proto.build_read(
                1, PRESENT_POSITION[0], PRESENT_POSITION[1]
            )
            replies = transport.transact(request, expected_id=1, timeout_s=1.0)
            assert len(replies[0].params) == 2
        finally:
            transport.close()


def test_transact_timeout_on_missing_servo() -> None:
    with FakeStsDevice() as device:
        device.bus._servos.pop(9, None)
        transport = SerialTransport(device.port_name, 1_000_000)
        try:
            time.sleep(0.05)
            with pytest.raises(StsTimeoutError):
                transport.transact(
                    proto.build_ping(9),
                    expected_id=9,
                    timeout_s=0.2,
                )
        finally:
            transport.close()
