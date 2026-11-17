"""FR-HW-45 integrity: bad replies rejected and counted."""

from __future__ import annotations

import pytest

from fret.hardware.bridge_core import make_bridge_core_for_mock
from fret.hardware.sts.errors import (
    StsChecksumError,
    StsIdMismatchError,
    StsTimeoutError,
)
from fret.hardware.sts.mock_bus import FaultConfig, FaultKind


def test_corrupt_checksum_rejected_and_counted_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(kind=FaultKind.CORRUPT_CHECKSUM, remaining=1))
    with pytest.raises(StsChecksumError):
        core.cycle(0.0)
    assert core.diagnostics.integrity_rejects >= 1


def test_wrong_id_rejected_and_counted_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(kind=FaultKind.WRONG_ID, remaining=1))
    with pytest.raises(
        (StsIdMismatchError, StsTimeoutError, StsChecksumError)
    ):
        core.cycle(0.0)
    assert (
        core.diagnostics.integrity_rejects + core.diagnostics.timeouts
    ) >= 1


def test_dropped_reply_counts_timeout_fr_hw_45() -> None:
    core, bus = make_bridge_core_for_mock()
    core.connect()
    bus.inject_fault(FaultConfig(kind=FaultKind.DROP_REPLY, remaining=6))
    with pytest.raises(StsTimeoutError):
        core.cycle(0.0)
    assert core.diagnostics.timeouts >= 1
