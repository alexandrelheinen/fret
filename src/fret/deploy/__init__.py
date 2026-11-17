"""Deploy helpers for Raspberry Pi 5 release switching (FR-HW-56)."""

from fret.deploy.release_switch import (
    ReleaseSwitchError,
    activate_release,
    previous_release,
    rollback_release,
)

__all__ = [
    "ReleaseSwitchError",
    "activate_release",
    "previous_release",
    "rollback_release",
]
