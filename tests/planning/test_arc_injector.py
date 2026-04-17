"""Tests for fret.ros.arc_injector.

Covers the standalone ``_generate_arc_trajectory`` function and the
``generate_arc_trajectory`` public wrapper that can be used without ROS.

Acceptance criteria:
    - Output list lengths match ``n_waypoints``.
    - Timestamps are monotonically increasing from 0 to ``duration``.
    - Each joint configuration has exactly 3 elements (SCARA RRP).
    - Joint values are within SCARA joint limits.
    - Arc starts near the expected Cartesian start point (FK check).
    - Arc ends near the expected Cartesian end point (FK check).
    - Default parameters do not raise.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from fret.ros.arc_injector import generate_arc_trajectory

# SCARA joint limits (mirrors kinematics.py constants)
_Q1_LIM = math.radians(132.0)
_Q2_LIM = math.radians(150.0)
_Q3_MAX = 0.20  # m


class TestGenerateArcTrajectory:
    """Unit tests for generate_arc_trajectory (no ROS required)."""

    def test_output_length_matches_n_waypoints(self) -> None:
        """Joint configs and timestamps must have exactly n_waypoints entries."""
        configs, times = generate_arc_trajectory(n_waypoints=50, duration=2.0)
        assert len(configs) == 50
        assert len(times) == 50

    def test_timestamps_start_at_zero(self) -> None:
        """First timestamp must be 0.0."""
        _, times = generate_arc_trajectory(n_waypoints=20, duration=3.0)
        assert times[0] == pytest.approx(0.0, abs=1e-9)

    def test_timestamps_end_at_duration(self) -> None:
        """Last timestamp must equal ``duration``."""
        duration = 4.5
        _, times = generate_arc_trajectory(n_waypoints=20, duration=duration)
        assert times[-1] == pytest.approx(duration, rel=1e-6)

    def test_timestamps_monotonically_increasing(self) -> None:
        """Timestamps must be strictly increasing."""
        _, times = generate_arc_trajectory(n_waypoints=30, duration=3.0)
        diffs = [times[i + 1] - times[i] for i in range(len(times) - 1)]
        assert all(d > 0 for d in diffs)

    def test_joint_configs_have_three_elements(self) -> None:
        """Each joint configuration must have exactly 3 elements (SCARA RRP)."""
        configs, _ = generate_arc_trajectory(n_waypoints=10, duration=1.0)
        for q in configs:
            assert len(q) == 3

    def test_joint_values_within_limits(self) -> None:
        """Joint values must be within SCARA joint limits."""
        configs, _ = generate_arc_trajectory(n_waypoints=100, duration=4.0)
        for q in configs:
            assert (
                -_Q1_LIM - 1e-6 <= q[0] <= _Q1_LIM + 1e-6
            ), f"q1={q[0]:.4f} out of limits ±{math.degrees(_Q1_LIM):.1f}°"
            assert (
                -_Q2_LIM - 1e-6 <= q[1] <= _Q2_LIM + 1e-6
            ), f"q2={q[1]:.4f} out of limits ±{math.degrees(_Q2_LIM):.1f}°"
            assert (
                -1e-6 <= q[2] <= _Q3_MAX + 1e-6
            ), f"q3={q[2]:.4f} out of limits [0, {_Q3_MAX}] m"

    def test_arc_start_near_expected_cartesian(self) -> None:
        """FK of the first waypoint must be near the arc start point."""
        from fret.control.kinematics import Kinematics

        center_x, center_y = 0.30, 0.00
        radius = 0.15
        start_deg = -45.0
        z_height = 0.138

        configs, _ = generate_arc_trajectory(
            center_x=center_x,
            center_y=center_y,
            radius=radius,
            start_deg=start_deg,
            end_deg=45.0,
            z_height=z_height,
            n_waypoints=20,
            duration=2.0,
        )

        kin = Kinematics("scara")
        ee_pos = kin.forward_kinematics(configs[0])[:3, 3]

        expected_x = center_x + radius * math.cos(math.radians(start_deg))
        expected_y = center_y + radius * math.sin(math.radians(start_deg))

        assert ee_pos[0] == pytest.approx(
            expected_x, abs=5e-3
        ), f"EE X at arc start: got {ee_pos[0]:.4f}, expected {expected_x:.4f}"
        assert ee_pos[1] == pytest.approx(
            expected_y, abs=5e-3
        ), f"EE Y at arc start: got {ee_pos[1]:.4f}, expected {expected_y:.4f}"

    def test_arc_end_near_expected_cartesian(self) -> None:
        """FK of the last waypoint must be near the arc end point."""
        from fret.control.kinematics import Kinematics

        center_x, center_y = 0.30, 0.00
        radius = 0.15
        end_deg = 45.0
        z_height = 0.138

        configs, _ = generate_arc_trajectory(
            center_x=center_x,
            center_y=center_y,
            radius=radius,
            start_deg=-45.0,
            end_deg=end_deg,
            z_height=z_height,
            n_waypoints=20,
            duration=2.0,
        )

        kin = Kinematics("scara")
        ee_pos = kin.forward_kinematics(configs[-1])[:3, 3]

        expected_x = center_x + radius * math.cos(math.radians(end_deg))
        expected_y = center_y + radius * math.sin(math.radians(end_deg))

        assert ee_pos[0] == pytest.approx(
            expected_x, abs=5e-3
        ), f"EE X at arc end: got {ee_pos[0]:.4f}, expected {expected_x:.4f}"
        assert ee_pos[1] == pytest.approx(
            expected_y, abs=5e-3
        ), f"EE Y at arc end: got {ee_pos[1]:.4f}, expected {expected_y:.4f}"

    def test_default_parameters_do_not_raise(self) -> None:
        """generate_arc_trajectory() with default args must not raise."""
        configs, times = generate_arc_trajectory()
        assert len(configs) == 200
        assert len(times) == 200

    def test_minimum_n_waypoints(self) -> None:
        """n_waypoints=2 (minimum) must return start and end."""
        configs, times = generate_arc_trajectory(n_waypoints=2, duration=1.0)
        assert len(configs) == 2
        assert times[0] == pytest.approx(0.0, abs=1e-9)
        assert times[1] == pytest.approx(1.0, abs=1e-9)

    def test_full_circle_does_not_raise(self) -> None:
        """A full 360° arc must produce valid output."""
        configs, times = generate_arc_trajectory(
            start_deg=0.0,
            end_deg=360.0,
            n_waypoints=50,
            duration=5.0,
        )
        assert len(configs) == 50
