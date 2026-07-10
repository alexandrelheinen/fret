"""PPP physics SITL smoke tests (v1.1.2 / V112)."""

from __future__ import annotations

import numpy as np
import pytest

from fret.ros.mujoco_bridge import (
    _load_merged_bridge_config,
    _resolve_config_path,
    make_mujoco_bridge_core,
    physics_config_from_bridge_yaml,
)


def _mujoco_available() -> bool:
    return make_mujoco_bridge_core("ppp", "ppp_warehouse").has_mujoco_runtime


def _make_physics_bridge(start: np.ndarray) -> object:
    bridge = make_mujoco_bridge_core(
        "ppp",
        "ppp_warehouse",
        initial_positions=start.copy(),
    )
    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg["physics_mode"] = True
    bridge.configure_physics(
        physics_config_from_bridge_yaml(cfg, "ppp", physics_mode=True)
    )
    return bridge


def _hold_altitude(
    bridge: object,
    cruise_z: float,
    *,
    dt: float,
    steps: int,
    kp_z: float = 15.0,
    max_vz: float = 2.0,
) -> None:
    """Apply vertical velocity hold toward ``cruise_z`` for ``steps`` ticks."""
    for _ in range(steps):
        q = bridge.get_positions()  # type: ignore[attr-defined]
        vz = float(np.clip(kp_z * (cruise_z - q[2]), -max_vz, max_vz))
        bridge.step(np.array([0.0, 0.0, vz], dtype=np.float64), dt=dt)  # type: ignore[attr-defined]


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_ppp_physics_step_moves_x_without_constraint_lock() -> None:
    """V112-01/02: velocity actuators move joint_x without constraint cancellation."""
    start = np.array([2.0, 1.0, 2.4], dtype=np.float64)
    bridge = _make_physics_bridge(start)
    q0 = bridge.get_positions().copy()
    dt = 0.02
    for _ in range(250):
        bridge.step(np.array([1.0, 0.0, 0.0], dtype=np.float64), dt=dt)
    delta_x = float(bridge.get_positions()[0] - q0[0])
    assert delta_x >= 1.0


@pytest.mark.skipif(
    not _mujoco_available(), reason="mujoco package not installed"
)
def test_ppp_physics_two_meter_x_transit_at_cruise_z_under_30s() -> None:
    """V112-03: 2 m X transit at cruise altitude completes within 30 s simulated time."""
    start = np.array([2.0, 1.0, 2.4], dtype=np.float64)
    target_x = 4.0
    cruise_z = 2.4
    bridge = _make_physics_bridge(start)
    dt = 0.02
    kp_z = 50.0
    max_vx = 0.25
    max_vz = 5.0

    _hold_altitude(bridge, cruise_z, dt=dt, steps=100, kp_z=kp_z, max_vz=max_vz)

    steps = 50
    max_steps = int(30.0 / dt)
    while float(bridge.get_positions()[0]) < target_x - 0.05 and steps < max_steps:
        q = bridge.get_positions()
        vz = float(np.clip(kp_z * (cruise_z - q[2]), -max_vz, max_vz))
        bridge.step(np.array([max_vx, 0.0, vz], dtype=np.float64), dt=dt)
        steps += 1

    final = bridge.get_positions()
    sim_time_s = float(steps) * dt
    assert float(final[0]) >= target_x - 0.1
    assert float(final[2]) >= cruise_z - 0.15
    assert sim_time_s < 30.0
