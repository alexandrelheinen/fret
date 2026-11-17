"""Golden STS framing vectors (FR-HW-45 integrity).

Register addresses and packet format cited from LeRobot Feetech tables
(Apache-2.0), commit ``e99c55af``:

https://github.com/huggingface/lerobot/blob/e99c55af/src/lerobot/motors/feetech/tables.py

Feetech STS/SMS memory table:
http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0
"""

from __future__ import annotations

import pytest

from fret.hardware.sts import protocol as proto
from fret.hardware.sts.errors import (
    StsChecksumError,
    StsIdMismatchError,
    StsLengthError,
)
from fret.hardware.sts.registers import (
    GOAL_POSITION,
    PRESENT_POSITION,
    STS3215_MODEL_NUMBER,
    TORQUE_ENABLE,
)


def test_ping_packet_golden_id_1() -> None:
    """PING id=1 → FF FF 01 02 01 FB."""
    assert proto.build_ping(1) == bytes.fromhex("FF FF 01 02 01 FB")


def test_ping_reply_golden_ok() -> None:
    """Status reply id=1, no error → FF FF 01 02 00 FC."""
    raw = bytes.fromhex("FF FF 01 02 00 FC")
    reply, consumed = proto.parse_reply(raw, expected_id=1)
    assert consumed == 6
    assert reply.servo_id == 1
    assert reply.error == 0
    assert reply.params == b""
    assert reply.ok


def test_read_present_position_golden() -> None:
    """READ Present_Position (addr 56, size 2) from id=1."""
    pkt = proto.build_read(1, PRESENT_POSITION[0], PRESENT_POSITION[1])
    assert pkt == bytes.fromhex("FF FF 01 04 02 38 02 BE")


def test_write_torque_enable_golden() -> None:
    """WRITE Torque_Enable=1 to id=1."""
    pkt = proto.build_write(1, TORQUE_ENABLE[0], b"\x01")
    assert pkt == bytes.fromhex("FF FF 01 04 03 28 01 CE")


def test_sync_write_goal_position_two_servos() -> None:
    """SYNC WRITE Goal_Position for ids 1 and 2 (mid ticks 2048)."""
    mid = proto.pack_u16_le(2048)
    pkt = proto.build_sync_write(
        GOAL_POSITION[0], GOAL_POSITION[1], [(1, mid), (2, mid)]
    )
    # FF FF FE LEN 83 2A 02 01 00 08 02 00 08 CHK
    assert pkt[:3] == bytes.fromhex("FF FF FE")
    assert pkt[4] == proto.INST_SYNC_WRITE
    assert pkt[5] == GOAL_POSITION[0]
    assert pkt[6] == GOAL_POSITION[1]


def test_sync_read_build_contains_ids() -> None:
    pkt = proto.build_sync_read(PRESENT_POSITION[0], 2, [1, 2, 3])
    assert pkt[4] == proto.INST_SYNC_READ
    assert list(pkt[7:10]) == [1, 2, 3]


def test_checksum_rejects_corrupt_reply_fr_hw_45() -> None:
    """FR-HW-45: bad checksum is rejected, never returned."""
    raw = bytearray.fromhex("FF FF 01 02 00 FC")
    raw[-1] ^= 0xFF
    with pytest.raises(StsChecksumError):
        proto.parse_reply(bytes(raw), expected_id=1)


def test_wrong_id_rejected_fr_hw_45() -> None:
    """FR-HW-45: wrong id is rejected."""
    raw = bytes.fromhex("FF FF 02 02 00 FB")
    with pytest.raises(StsIdMismatchError):
        proto.parse_reply(raw, expected_id=1)


def test_truncated_packet_raises_length() -> None:
    with pytest.raises(StsLengthError):
        proto.parse_reply(bytes.fromhex("FF FF 01 04 02"), expected_id=1)


@pytest.mark.parametrize(
    ("value", "sign_bit", "raw"),
    [
        (0, 15, 0),
        (100, 15, 100),
        (-100, 15, 100 | (1 << 15)),
        (-1, 10, 1 | (1 << 10)),
        (500, 10, 500),
    ],
)
def test_sign_magnitude_round_trip(
    value: int, sign_bit: int, raw: int
) -> None:
    assert proto.encode_sign_magnitude(value, sign_bit) == raw
    assert proto.decode_sign_magnitude(raw, sign_bit) == value


def test_sts3215_model_number_from_cited_table() -> None:
    assert STS3215_MODEL_NUMBER == 777


def test_ticks_radians_mid_is_zero() -> None:
    assert proto.ticks_to_radians(2048) == pytest.approx(0.0)
    assert proto.radians_to_ticks(0.0) == 2048


def test_error_byte_surfaced_in_reply() -> None:
    # error bit overload = 0x20 (example non-zero status)
    body = bytes((1, 2, 0x20))
    raw = proto.HEADER + body + bytes((proto.checksum(body),))
    reply, _ = proto.parse_reply(raw, expected_id=1)
    assert reply.error == 0x20
    assert not reply.ok
