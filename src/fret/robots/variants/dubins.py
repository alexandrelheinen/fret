"""Dubins mobile-robot variant registration (FR-SYS-08)."""

from __future__ import annotations

from fret.control.kinematics_dubins import DubinsKinematics
from fret.robots.registry import register_kinematics


@register_kinematics("dubins")
def _build_dubins_kinematics() -> DubinsKinematics:
    return DubinsKinematics()
