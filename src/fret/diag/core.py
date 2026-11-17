"""Shared diagnostics result types and exit codes (FR-HW-65)."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class CheckStatus(str, Enum):
    """Pass / fail / skip for a single diagnostic check."""

    PASS = "pass"
    FAIL = "fail"
    SKIP = "skip"


@dataclass(frozen=True)
class CheckResult:
    """One named check outcome with optional details."""

    name: str
    status: CheckStatus
    message: str
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["status"] = self.status.value
        return payload


def exit_code_for(results: list[CheckResult]) -> int:
    """Map results to process exit code: 0 pass, 1 fail, 2 skip/needs operator."""
    if any(r.status is CheckStatus.FAIL for r in results):
        return 1
    if results and all(r.status is CheckStatus.SKIP for r in results):
        return 2
    if any(r.status is CheckStatus.SKIP for r in results) and not any(
        r.status is CheckStatus.PASS for r in results
    ):
        return 2
    return 0


# Read-only commands run by ``fret-diag all`` (motion excluded: FR-HW-66).
READ_ONLY_ORDER: tuple[str, ...] = (
    "pi",
    "usb",
    "bus",
    "servos",
    "calibration",
    "estop",
    "camera",
    "vision",
    "loop",
    "load",
)

MOTION_COMMANDS: frozenset[str] = frozenset({"joints", "gripper", "e2e"})
ALL_COMMANDS: tuple[str, ...] = READ_ONLY_ORDER + (
    "joints",
    "gripper",
    "e2e",
    "all",
)
