"""Pty-backed fake STS serial device (exercises real pyserial)."""

from __future__ import annotations

import os
import select
import threading
from typing import Callable

from fret.hardware.sts.mock_bus import MockStsBus, make_so101_mock


class FakeStsDevice:
    """Run ``MockStsBus`` behind a pty slave path usable as ``port``.

    The slave path (e.g. ``/dev/pts/N``) is mock-safe under FR-SYS-18.
    """

    def __init__(
        self,
        bus: MockStsBus | None = None,
        *,
        on_traffic: Callable[[bytes, bytes], None] | None = None,
    ) -> None:
        self._bus = bus if bus is not None else make_so101_mock()
        if on_traffic is not None:
            self._bus.add_listener(on_traffic)
        self._master_fd, self._slave_fd = os.openpty()
        self._slave_name = os.ttyname(self._slave_fd)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._rx = bytearray()

    @property
    def port_name(self) -> str:
        return self._slave_name

    @property
    def bus(self) -> MockStsBus:
        return self._bus

    def start(self) -> str:
        """Start the background reader; return the slave port path."""
        if self._thread is not None:
            return self._slave_name
        self._thread = threading.Thread(
            target=self._loop, name="fake-sts-device", daemon=True
        )
        self._thread.start()
        return self._slave_name

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for fd in (self._master_fd, self._slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def __enter__(self) -> FakeStsDevice:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def _loop(self) -> None:
        while not self._stop.is_set():
            ready, _, _ = select.select([self._master_fd], [], [], 0.05)
            if not ready:
                continue
            try:
                chunk = os.read(self._master_fd, 256)
            except OSError:
                break
            if not chunk:
                break
            self._rx.extend(chunk)
            self._drain()

    def _drain(self) -> None:
        while len(self._rx) >= 6:
            if self._rx[0] != 0xFF or self._rx[1] != 0xFF:
                del self._rx[0]
                continue
            length = self._rx[3]
            total = 4 + length
            if len(self._rx) < total:
                return
            request = bytes(self._rx[:total])
            del self._rx[:total]
            reply = self._bus.handle_request(request)
            if reply:
                try:
                    os.write(self._master_fd, reply)
                except OSError:
                    return
