"""Unit tests for Dubins non-holonomic wheel velocity projection."""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.control.dubins_wheel_model import (
    max_body_lateral_speed_m_s,
    project_planar_velocity_forward_only,
)


def test_project_velocity_forward_only_zeros_lateral() -> None:
    heading = math.pi / 4.0
    vx, vy = project_planar_velocity_forward_only(1.0, 1.0, heading)
    forward = vx * math.cos(heading) + vy * math.sin(heading)
    lateral = -vx * math.sin(heading) + vy * math.cos(heading)
    assert forward == pytest.approx(math.sqrt(2.0))
    assert lateral == pytest.approx(0.0, abs=1e-12)


def test_project_velocity_pure_lateral_becomes_zero() -> None:
    vx, vy = project_planar_velocity_forward_only(0.0, 2.5, 0.0)
    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(0.0)


def test_max_lateral_speed_detects_sideways_motion() -> None:
    poses = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    assert max_body_lateral_speed_m_s(poses, dt=0.05) == pytest.approx(10.0)


def test_max_lateral_speed_forward_motion_is_low() -> None:
    poses = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    assert max_body_lateral_speed_m_s(poses, dt=0.05) == pytest.approx(0.0, abs=1e-12)
