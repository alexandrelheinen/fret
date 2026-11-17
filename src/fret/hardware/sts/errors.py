"""Typed errors for the STS protocol and bridge (C2 boundary)."""

from __future__ import annotations


class StsError(Exception):
    """Base class for Feetech STS bus errors."""


class StsChecksumError(StsError):
    """Reply checksum did not match."""


class StsIdMismatchError(StsError):
    """Reply servo id did not match the request."""


class StsLengthError(StsError):
    """Packet length field inconsistent with payload."""


class StsTimeoutError(StsError):
    """No complete reply within the configured timeout."""


class StsProtocolError(StsError):
    """Malformed header or instruction."""


class StsPortError(StsError):
    """Serial port disappeared or could not be opened."""


class TorqueEnableRefusedError(StsError):
    """Torque enable refused without an active SafetySupervisor (FR-SYS-19)."""
