"""Unit tests for the post-hoc collision monitor.

Replaces the removed pre-emptive occupancy-based motion block
(``_physics_forward_blocked``): an agent's control loop should keep
tracking its planned path at full speed and only stop after a real
impact, detected either from an actual MuJoCo contact force or, as a
fallback, a sudden loss of realized speed between ticks.
"""

from __future__ import annotations

import pytest

from fret.scenario.dubins_race_runner import _CollisionMonitor


def test_real_contact_force_latches_collided() -> None:
    """A contact force above threshold marks the agent collided immediately."""
    monitor = _CollisionMonitor()
    collided = monitor.update(
        prev_pose=(1.0, 1.0, 0.0),
        new_pose=(1.01, 1.0, 0.0),
        contact_force_n=5.0,
        dt=0.05,
    )
    assert collided is True
    assert monitor.collided is True
    assert monitor.peak_force_n == pytest.approx(5.0)


def test_small_contact_force_does_not_latch() -> None:
    """A negligible contact force (e.g. sensor noise) must not stop the agent."""
    monitor = _CollisionMonitor()
    collided = monitor.update(
        prev_pose=(1.0, 1.0, 0.0),
        new_pose=(1.018, 1.0, 0.0),
        contact_force_n=0.05,
        dt=0.05,
    )
    assert collided is False
    assert monitor.collided is False
    # Peak force is still tracked for diagnostics even without latching.
    assert monitor.peak_force_n == pytest.approx(0.05)


def test_normal_startup_acceleration_does_not_latch() -> None:
    """Accelerating from a stop toward cruise_speed is normal, not a crash.

    Regression test: an early version of this monitor used ``abs(delta
    speed)`` and falsely latched on the very first tick, when realized
    speed jumps from 0 to a large fraction of cruise_speed within one
    control tick.
    """
    monitor = _CollisionMonitor()
    # From standstill to 0.36 m/s (dubins.yml cruise_speed) in one 0.05 s
    # tick — a ~7 m/s^2 realized acceleration, well above the collision
    # threshold, but it is a *gain* in speed, not a loss.
    collided = monitor.update(
        prev_pose=(1.0, 1.0, 0.0),
        new_pose=(1.018, 1.0, 0.0),
        contact_force_n=0.0,
        dt=0.05,
    )
    assert collided is False
    assert monitor.collided is False


def test_sudden_deceleration_fallback_latches_without_contact_force() -> None:
    """A speed-loss spike with zero measured contact force still latches.

    Covers a fast bounce that leaves no contact at the tick boundary where
    forces are sampled.
    """
    monitor = _CollisionMonitor()
    # Warm up to a steady cruising speed first.
    monitor.update(
        prev_pose=(1.0, 1.0, 0.0),
        new_pose=(1.018, 1.0, 0.0),
        contact_force_n=0.0,
        dt=0.05,
    )
    assert monitor.collided is False

    # Next tick: realized speed collapses to near zero without a sampled
    # contact force (e.g. bounced clear between force samples).
    collided = monitor.update(
        prev_pose=(1.018, 1.0, 0.0),
        new_pose=(1.0185, 1.0, 0.0),
        contact_force_n=0.0,
        dt=0.05,
    )
    assert collided is True
    assert monitor.collided is True


def test_gradual_heading_gate_slowdown_does_not_latch() -> None:
    """A multi-tick cornering slowdown must stay under the decel threshold."""
    monitor = _CollisionMonitor()
    speeds = [0.36, 0.30, 0.24, 0.18, 0.14]
    x = 1.0
    collided = False
    for speed in speeds:
        new_x = x + speed * 0.05
        collided = monitor.update(
            prev_pose=(x, 1.0, 0.0),
            new_pose=(new_x, 1.0, 0.0),
            contact_force_n=0.0,
            dt=0.05,
        )
        x = new_x
    assert collided is False
    assert monitor.collided is False


def test_collision_flag_is_sticky() -> None:
    """Once collided, later zero-force/zero-decel ticks must not clear it."""
    monitor = _CollisionMonitor()
    monitor.update(
        prev_pose=(1.0, 1.0, 0.0),
        new_pose=(1.0, 1.0, 0.0),
        contact_force_n=10.0,
        dt=0.05,
    )
    assert monitor.collided is True

    collided = monitor.update(
        prev_pose=(1.0, 1.0, 0.0),
        new_pose=(1.0, 1.0, 0.0),
        contact_force_n=0.0,
        dt=0.05,
    )
    assert collided is True
    assert monitor.collided is True
