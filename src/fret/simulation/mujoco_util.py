"""Shared MuJoCo availability helpers for unit sandboxes."""

from __future__ import annotations


def mujoco_available() -> bool:
    """Return ``True`` when the optional ``mujoco`` package imports."""
    try:
        import mujoco
    except ImportError:  # pragma: no cover
        return False
    return mujoco is not None
