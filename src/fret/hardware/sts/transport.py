"""Serial transport for the Feetech STS bus (pyserial)."""

from __future__ import annotations

import time
from typing import Any, Protocol, cast

from fret.hardware.sts.errors import (
    StsChecksumError,
    StsIdMismatchError,
    StsLengthError,
    StsPortError,
    StsProtocolError,
    StsTimeoutError,
)
from fret.hardware.sts.protocol import StsReply, parse_reply


class _SerialLike(Protocol):
    def write(self, data: bytes) -> int: ...

    def read(self, size: int = 1) -> bytes: ...

    def close(self) -> None: ...


class BusTransport(Protocol):
    """Minimal byte transport used by the bridge and CLI."""

    def write(self, data: bytes) -> None: ...

    def read(self, size: int, timeout_s: float) -> bytes: ...

    def close(self) -> None: ...

    @property
    def port_name(self) -> str: ...

    @property
    def is_mock_safe(self) -> bool:
        """True when FR-SYS-18 allows torque enable on this transport."""
        ...


def port_is_mock_safe(port: str) -> bool:
    """FR-SYS-18: mock / replay / pty paths may enable torque in v2.1.0."""
    if port.startswith("mock:") or port.startswith("replay:"):
        return True
    # Linux pts slaves and common pty paths from FakeStsDevice.
    if port.startswith("/dev/pts/"):
        return True
    if port.startswith("pty:") or port.startswith("fake:"):
        return True
    return False


class SerialTransport:
    """pyserial-backed half-duplex transport with reply timeouts."""

    def __init__(
        self,
        port: str,
        baud_rate: int = 1_000_000,
        *,
        default_timeout_s: float = 0.05,
        serial_factory: object | None = None,
    ) -> None:
        if baud_rate <= 0:
            raise ValueError(f"baud_rate must be positive, got {baud_rate}")
        self._port_name = port
        self._baud_rate = baud_rate
        self._default_timeout_s = default_timeout_s
        self._serial_factory = serial_factory
        self._ser: _SerialLike | None = None
        self._open()

    def _open(self) -> None:
        try:
            if self._serial_factory is not None:
                factory = self._serial_factory
                assert callable(factory)
                self._ser = cast(
                    _SerialLike,
                    factory(
                        self._port_name,
                        self._baud_rate,
                        timeout=0,
                    ),
                )
            else:
                import serial

                self._ser = cast(
                    _SerialLike,
                    serial.Serial(
                        port=self._port_name,
                        baudrate=self._baud_rate,
                        timeout=0,
                        write_timeout=self._default_timeout_s,
                    ),
                )
        except Exception as exc:  # noqa: BLE001 — boundary to typed error
            raise StsPortError(
                f"cannot open {self._port_name}: {exc}"
            ) from exc

    @property
    def port_name(self) -> str:
        return self._port_name

    @property
    def is_mock_safe(self) -> bool:
        return port_is_mock_safe(self._port_name)

    def write(self, data: bytes) -> None:
        ser = self._require_open()
        try:
            written = ser.write(data)
            if written != len(data):
                raise StsPortError(
                    f"short write on {self._port_name}: "
                    f"{written}/{len(data)}"
                )
        except StsPortError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise StsPortError(
                f"write failed on {self._port_name}: {exc}"
            ) from exc

    def read(self, size: int, timeout_s: float) -> bytes:
        if size <= 0:
            return b""
        ser = self._require_open()
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        try:
            while len(buf) < size:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                chunk = ser.read(size - len(buf))
                if chunk:
                    buf.extend(chunk)
                else:
                    time.sleep(min(0.001, remaining))
        except Exception as exc:  # noqa: BLE001
            raise StsPortError(
                f"read failed on {self._port_name}: {exc}"
            ) from exc
        return bytes(buf)

    def transact(
        self,
        request: bytes,
        *,
        expected_id: int | None,
        timeout_s: float | None = None,
        expect_replies: int = 1,
    ) -> list[StsReply]:
        """Write ``request`` and parse ``expect_replies`` status packets."""
        timeout = self._default_timeout_s if timeout_s is None else timeout_s
        self.write(request)
        replies: list[StsReply] = []
        buf = bytearray()
        deadline = time.monotonic() + timeout * max(1, expect_replies)
        while len(replies) < expect_replies:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise StsTimeoutError(
                    f"timeout waiting for reply "
                    f"({len(replies)}/{expect_replies}) on {self._port_name}"
                )
            chunk = self.read(64, min(0.01, remaining))
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                try:
                    exp = expected_id if expect_replies == 1 else None
                    reply, consumed = parse_reply(bytes(buf), expected_id=exp)
                except StsLengthError:
                    break
                except (
                    StsChecksumError,
                    StsIdMismatchError,
                    StsProtocolError,
                ):
                    # FR-HW-45: never use a corrupt frame; surface to caller.
                    raise
                else:
                    replies.append(reply)
                    del buf[:consumed]
                    if len(replies) >= expect_replies:
                        break
        return replies

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def _require_open(self) -> _SerialLike:
        if self._ser is None:
            raise StsPortError(f"port {self._port_name} is closed")
        return self._ser


class InMemoryTransport:
    """Loopback transport used by unit tests (paired with MockStsBus)."""

    def __init__(self, bus: object, *, port_name: str = "mock:inproc") -> None:
        self._bus = bus
        self._port_name = port_name
        self._rx = bytearray()

    @property
    def port_name(self) -> str:
        return self._port_name

    @property
    def is_mock_safe(self) -> bool:
        return True

    def write(self, data: bytes) -> None:
        handle = getattr(self._bus, "handle_request")
        reply = handle(data)
        if reply:
            self._rx.extend(reply)

    def read(self, size: int, timeout_s: float) -> bytes:  # noqa: ARG002
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def close(self) -> None:
        self._rx.clear()
