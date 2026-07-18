"""Unit tests for ARCO JointSpaceMPC OMX helpers."""

from __future__ import annotations

import numpy as np

from fret.control.joint_mpc import (
    JointPathMPCTracker,
    build_omx_joint_mpc,
    path_arc_lengths,
    path_pos_at_arc,
)


def test_build_omx_joint_mpc_tracks_target() -> None:
    mpc = build_omx_joint_mpc()
    q0 = np.zeros(4, dtype=np.float64)
    target = np.array([0.4, -0.3, 0.2, 0.1], dtype=np.float64)
    mpc.reset(q0)
    q = q0.copy()
    for _ in range(50):
        q = np.asarray(mpc.step(target, 0.05), dtype=np.float64)
    assert float(np.linalg.norm(q - target)) < 0.05
    assert mpc.last_solver_success is True


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
    for _ in range(80):
        tracker.step(0.05)
        if tracker.complete:
            break
    assert tracker.complete is True
    assert float(np.linalg.norm(tracker.q - path[-1])) < 0.15
