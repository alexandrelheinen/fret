"""Unit tests for ARCO JointSpaceMPC OMX helpers."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.joint_mpc import (
    JointPathMPCTracker,
    build_omx_joint_mpc,
    path_arc_lengths,
    path_pos_at_arc,
)

_CTRL_DT = 0.02


def test_build_omx_joint_mpc_matches_control_period_dt() -> None:
    """ARCO ≥ v0.3.7 requires step(dt) == config.dt; default is 50 Hz."""
    mpc = build_omx_joint_mpc()
    assert abs(mpc.config.dt - _CTRL_DT) < 1e-9
    # Preserve ARCO's default physical horizon (12 × 0.05 s = 0.6 s).
    assert mpc.config.horizon_step_count == 30
    assert abs(mpc.config.horizon_step_count * mpc.config.dt - 0.6) < 1e-9


def test_build_omx_joint_mpc_tracks_target() -> None:
    mpc = build_omx_joint_mpc()
    q0 = np.zeros(4, dtype=np.float64)
    target = np.array([0.4, -0.3, 0.2, 0.1], dtype=np.float64)
    mpc.reset(q0)
    q = q0.copy()
    for _ in range(100):
        q = np.asarray(mpc.step(target, _CTRL_DT), dtype=np.float64)
    assert float(np.linalg.norm(q - target)) < 0.05
    assert mpc.last_solver_success is True


def test_joint_mpc_rejects_dt_mismatch() -> None:
    """Mismatched plant/model dt must raise (ARCO zigzag guard)."""
    mpc = build_omx_joint_mpc()
    mpc.reset(np.zeros(4, dtype=np.float64))
    with pytest.raises(ValueError, match="must equal config.dt"):
        mpc.step(np.zeros(4, dtype=np.float64), 0.05)


def test_path_carrot_tracker_reaches_goal() -> None:
    path = [
        np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([0.2, -0.1, 0.1, 0.0], dtype=np.float64),
        np.array([0.5, -0.3, 0.2, 0.1], dtype=np.float64),
    ]
    arcs = path_arc_lengths(path)
    assert arcs[-1] > 0.0
    mid = path_pos_at_arc(path, arcs, arcs[-1] * 0.5)
    assert mid.shape == (4,)

    tracker = JointPathMPCTracker(
        path,
        build_omx_joint_mpc(),
        race_speed=1.2,
        max_carrot_lag=0.35,
        goal_tol=0.12,
    )
    tracker.reset(path[0])
    for _ in range(200):
        tracker.step(_CTRL_DT)
        if tracker.complete:
            break
    assert tracker.complete is True
    assert float(np.linalg.norm(tracker.q - path[-1])) < 0.15
