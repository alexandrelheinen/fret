"""FR-HW-64: watchdog notify at half interval; stops on stall."""

from __future__ import annotations

from fret.hardware.sd_notify import WatchdogNotifier, watchdog_sec_from_env


def test_notifies_at_half_interval() -> None:
    sent: list[str] = []
    clock = {"t": 0.0}

    def now() -> float:
        return clock["t"]

    notifier = WatchdogNotifier(
        4.0, clock=now, notify=lambda s: sent.append(s)
    )
    assert notifier.maybe_notify() is True
    assert sent == ["WATCHDOG=1"]
    clock["t"] = 1.5
    assert notifier.maybe_notify() is False
    clock["t"] = 2.0
    assert notifier.maybe_notify() is True
    assert len(sent) == 2


def test_stops_on_stall() -> None:
    sent: list[str] = []
    notifier = WatchdogNotifier(
        2.0, clock=lambda: 10.0, notify=lambda s: sent.append(s)
    )
    notifier.mark_stall()
    assert notifier.maybe_notify() is False
    assert sent == []


def test_watchdog_usec_env() -> None:
    assert watchdog_sec_from_env({"WATCHDOG_USEC": "5000000"}) == 5.0
    assert watchdog_sec_from_env({}) is None
