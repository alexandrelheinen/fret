"""FR-HW-69: SafetySupervisor in the SO-101 MuJoCo position-actuator IO."""

from __future__ import annotations

import os

import numpy as np
import pytest

pytest.importorskip("mujoco")

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import mujoco

from fret.hardware.safety import SafetyConfig
from fret.mjcf.so101 import ensure_so101_tabletop_mjcf
from fret.simulation.position_actuator_io import PositionActuatorIO


_ARM = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def _io() -> tuple[PositionActuatorIO, mujoco.MjModel, mujoco.MjData]:
    path = ensure_so101_tabletop_mjcf()
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    act_ids = []
    for name in _ARM:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise AssertionError(f"missing actuator {name}")
        act_ids.append(aid)
    limits = np.array([[-2.5, 2.5]] * 6, dtype=np.float64)
    io = PositionActuatorIO(
        model,
        data,
        actuator_ids=act_ids,
        joint_limits_rad=limits,
        max_joint_velocity=2.0,
        safety_config=SafetyConfig(
            watchdog_s=0.1,
            enable_lease_s=0.5,
            limit_margin_rad=0.03,
            max_joint_velocity=2.0,
            dt_s=0.02,
        ),
        clock=lambda: float(data.time),
        mj=mujoco,
    )
    return io, model, data


def test_sim_soft_start_and_watchdog_fr_hw_69() -> None:
    io, _model, data = _io()
    goals = io.step(float(data.time))
    assert goals.shape == (6,)
    data.time += 0.2
    io.step(float(data.time))
    assert io.safety.diagnostics.watchdog is True


def test_sim_limit_clamp_fr_hw_69() -> None:
    io, model, data = _io()
    io.renew_lease(float(data.time))
    io.command_velocity(np.full(6, 10.0), float(data.time))
    for _ in range(50):
        data.time += 0.02
        io.renew_lease(float(data.time))
        io.command_velocity(np.full(6, 10.0), float(data.time))
        goals = io.step(float(data.time))
        mujoco.mj_step(model, data)
        assert float(np.max(np.abs(goals))) <= 2.5 + 1e-6


def test_sim_lease_expiry_holds_fr_hw_69() -> None:
    io, _model, data = _io()
    io.command_velocity(np.zeros(6), float(data.time))
    io.step(float(data.time))
    data.time += 0.6
    io.command_velocity(np.ones(6) * 0.1, float(data.time))
    io.step(float(data.time))
    assert io.safety.diagnostics.lease_expired is True
