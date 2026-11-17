"""systemd ``sd_notify`` watchdog keep-alive helper (FR-HW-64).

The bridge cycle thread calls :meth:`WatchdogNotifier.maybe_notify` each
cycle. Notifications fire at half of ``WatchdogSec`` while the cycle is
healthy and stop when the cycle stalls, so systemd restarts the unit
(disabled, per FR-HW-57).
"""

from __future__ import annotations

import os
import socket
from typing import Callable


class WatchdogNotifier:
    """Notify systemd watchdog at half the configured interval."""

    def __init__(
        self,
        watchdog_sec: float,
        *,
        clock: Callable[[], float] | None = None,
        notify: Callable[[str], None] | None = None,
    ) -> None:
        if watchdog_sec <= 0:
            raise ValueError("watchdog_sec must be positive")
        self._interval_s = watchdog_sec / 2.0
        self._clock = clock or (lambda: __import__("time").monotonic())
        self._notify = notify or _sd_notify
        self._last_notify_s: float | None = None
        self._stalled = False
        self._notify_count = 0

    @property
    def notify_count(self) -> int:
        return self._notify_count

    @property
    def stalled(self) -> bool:
        return self._stalled

    def mark_stall(self) -> None:
        """Stop notifying so systemd's watchdog can fire."""
        self._stalled = True

    def clear_stall(self) -> None:
        self._stalled = False

    def maybe_notify(self, now_s: float | None = None) -> bool:
        """Send ``WATCHDOG=1`` when due. Returns True if a notify was sent."""
        if self._stalled:
            return False
        now = self._clock() if now_s is None else now_s
        if (
            self._last_notify_s is not None
            and (now - self._last_notify_s) < self._interval_s
        ):
            return False
        self._notify("WATCHDOG=1")
        self._last_notify_s = now
        self._notify_count += 1
        return True


def _sd_notify(state: str) -> None:
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return
    if addr[0] == "@":
        addr = "\0" + addr[1:]
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.connect(addr)
        sock.sendall(state.encode("utf-8"))
    finally:
        sock.close()


def watchdog_sec_from_env(
    env: dict[str, str] | None = None,
) -> float | None:
    """Parse ``WATCHDOG_USEC`` from the process environment, if set."""
    mapping = env if env is not None else os.environ
    raw = mapping.get("WATCHDOG_USEC")
    if not raw:
        return None
    usec = float(raw)
    if usec <= 0:
        return None
    return usec / 1_000_000.0
