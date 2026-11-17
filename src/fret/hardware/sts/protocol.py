"""Feetech STS packet framing and sign-magnitude codecs.

Packet layout (same family as Dynamixel Protocol 1 / Feetech SMS_STS):

``FF FF ID LENGTH INSTRUCTION PARAMS… CHECKSUM``

Checksum: ``~(sum(bytes from ID through last param)) & 0xFF``.

Register addresses and encoding bits: see :mod:`fret.hardware.sts.registers`
(LeRobot Feetech tables, Apache-2.0, commit ``e99c55af``).
"""

from __future__ import annotations

from dataclasses import dataclass

from fret.hardware.sts.errors import (
    StsChecksumError,
    StsIdMismatchError,
    StsLengthError,
    StsProtocolError,
)

HEADER: bytes = b"\xff\xff"
BROADCAST_ID: int = 0xFE

INST_PING: int = 0x01
INST_READ: int = 0x02
INST_WRITE: int = 0x03
INST_SYNC_WRITE: int = 0x83
INST_SYNC_READ: int = 0x82


def checksum(body: bytes) -> int:
    """Compute the Feetech checksum over ``ID..last_param`` (no header)."""
    return (~sum(body)) & 0xFF


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    """Encode a signed integer as Feetech sign-magnitude.

    Negative values set ``sign_bit`` and store ``abs(value)`` in the lower
    bits. Raises ``ValueError`` if ``abs(value)`` does not fit.
    """
    if sign_bit < 1 or sign_bit > 15:
        raise ValueError(f"sign_bit out of range: {sign_bit}")
    mask = (1 << sign_bit) - 1
    if value < 0:
        magnitude = -value
        if magnitude > mask:
            raise ValueError(f"magnitude {magnitude} exceeds {mask}")
        return magnitude | (1 << sign_bit)
    if value > mask:
        raise ValueError(f"value {value} exceeds {mask}")
    return value


def decode_sign_magnitude(raw: int, sign_bit: int) -> int:
    """Decode a Feetech sign-magnitude field to a signed Python int."""
    if sign_bit < 1 or sign_bit > 15:
        raise ValueError(f"sign_bit out of range: {sign_bit}")
    mask = (1 << sign_bit) - 1
    magnitude = raw & mask
    if raw & (1 << sign_bit):
        return -magnitude
    return magnitude


def pack_u8(value: int) -> bytes:
    if not 0 <= value <= 0xFF:
        raise ValueError(f"u8 out of range: {value}")
    return bytes((value,))


def pack_u16_le(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"u16 out of range: {value}")
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def unpack_u16_le(data: bytes) -> int:
    if len(data) < 2:
        raise StsLengthError("need 2 bytes for u16")
    return data[0] | (data[1] << 8)


def build_packet(
    servo_id: int, instruction: int, params: bytes = b""
) -> bytes:
    """Build a complete request packet including header and checksum."""
    if not 0 <= servo_id <= 0xFE:
        raise ValueError(f"servo id out of range: {servo_id}")
    if not 0 <= instruction <= 0xFF:
        raise ValueError(f"instruction out of range: {instruction}")
    length = 1 + len(params) + 1  # instruction + params + checksum
    body = bytes((servo_id, length, instruction)) + params
    return HEADER + body + bytes((checksum(body),))


def build_ping(servo_id: int) -> bytes:
    return build_packet(servo_id, INST_PING)


def build_read(servo_id: int, address: int, size: int) -> bytes:
    return build_packet(servo_id, INST_READ, pack_u8(address) + pack_u8(size))


def build_write(servo_id: int, address: int, data: bytes) -> bytes:
    return build_packet(servo_id, INST_WRITE, pack_u8(address) + data)


def build_sync_read(address: int, size: int, servo_ids: list[int]) -> bytes:
    if not servo_ids:
        raise ValueError("sync_read requires at least one servo id")
    params = pack_u8(address) + pack_u8(size) + bytes(servo_ids)
    return build_packet(BROADCAST_ID, INST_SYNC_READ, params)


def build_sync_write(
    address: int, size: int, writes: list[tuple[int, bytes]]
) -> bytes:
    if not writes:
        raise ValueError("sync_write requires at least one write")
    chunks: list[bytes] = [pack_u8(address), pack_u8(size)]
    for servo_id, data in writes:
        if len(data) != size:
            raise ValueError(
                f"sync_write data length {len(data)} != size {size}"
            )
        chunks.append(bytes((servo_id,)) + data)
    return build_packet(BROADCAST_ID, INST_SYNC_WRITE, b"".join(chunks))


@dataclass(frozen=True, slots=True)
class StsReply:
    """Decoded status packet from a servo."""

    servo_id: int
    error: int
    params: bytes

    @property
    def ok(self) -> bool:
        return self.error == 0


def parse_reply(
    raw: bytes, *, expected_id: int | None = None
) -> tuple[StsReply, int]:
    """Parse one status packet from ``raw``.

    Returns ``(reply, bytes_consumed)``. Raises typed ``Sts*Error`` on
    integrity failures; never returns a reply that failed checksum / id /
    length checks.
    """
    if len(raw) < 6:
        raise StsLengthError(f"packet too short: {len(raw)} bytes")
    if raw[0] != 0xFF or raw[1] != 0xFF:
        raise StsProtocolError("missing FF FF header")
    servo_id = raw[2]
    length = raw[3]
    # length = error + params + checksum
    if length < 2:
        raise StsLengthError(f"invalid length field: {length}")
    total = 4 + length  # header(2)+id+len + (error+params+chk)
    # Actually: bytes after length field = length bytes (error, params..., chk)
    # total packet = 4 + length  (FF FF ID LEN + length bytes)
    if len(raw) < total:
        raise StsLengthError(
            f"truncated packet: need {total}, have {len(raw)}"
        )
    frame = raw[2:total]  # id, length, error, params..., checksum
    body, chk = frame[:-1], frame[-1]
    if checksum(body) != chk:
        raise StsChecksumError(
            f"checksum mismatch: got 0x{chk:02X}, "
            f"expected 0x{checksum(body):02X}"
        )
    error = body[2]
    params = body[3:]
    if expected_id is not None and servo_id != expected_id:
        raise StsIdMismatchError(f"expected id {expected_id}, got {servo_id}")
    # length field must equal 1 (error) + len(params) + 1 (checksum)
    if length != 1 + len(params) + 1:
        raise StsLengthError(
            f"length field {length} inconsistent with params "
            f"({len(params)} bytes)"
        )
    return StsReply(servo_id=servo_id, error=error, params=params), total


def ticks_to_radians(
    ticks: int, *, mid: int = 2048, sign: float = 1.0, resolution: int = 4096
) -> float:
    """Map encoder ticks to radians with zero at ``mid``."""
    import math

    return float(sign) * (ticks - mid) * (2.0 * math.pi) / float(resolution)


def radians_to_ticks(
    radians: float,
    *,
    mid: int = 2048,
    sign: float = 1.0,
    resolution: int = 4096,
) -> int:
    """Map radians to encoder ticks with zero at ``mid``."""
    import math

    ticks = mid + int(
        round(float(sign) * radians * resolution / (2.0 * math.pi))
    )
    return max(0, min(resolution - 1, ticks))
