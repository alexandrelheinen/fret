"""C-space barrier occupancy for JointSpaceMPC soft obstacle costs."""

from __future__ import annotations

import numpy as np
import pytest

from fret.control.cspace_mpc_occupancy import (
    build_cspace_barrier_occupancy,
    build_wall_cspace_barrier_occupancy,
    fit_clearance_to_waypoints,
    sample_colliding_configurations,
)
from fret.control.joint_mpc import build_omx_joint_mpc
from fret.control.kinematics import Kinematics
from fret.mjcf.omx import ensure_omx_wall_maze_mjcf

mujoco = pytest.importorskip("mujoco")


def test_sample_colliding_configurations_finds_hits() -> None:
    limits = np.array([[-1.0, 1.0], [-1.0, 1.0]], dtype=np.float64)

    def is_occupied(q: np.ndarray) -> bool:
        return float(np.linalg.norm(q)) < 0.35

    hits = sample_colliding_configurations(
        is_occupied,
        limits,
        n_samples=4000,
        rng=np.random.default_rng(0),
    )
    assert hits.shape[1] == 2
    assert hits.shape[0] >= 20
    assert all(is_occupied(q) for q in hits)


def test_empty_collision_cloud_still_builds_kdtree() -> None:
    occ = build_cspace_barrier_occupancy(
        np.zeros((0, 4), dtype=np.float64),
        clearance=0.2,
        joint_limits=[(-1, 1)] * 4,
    )
    dist, _ = occ.nearest_obstacle(np.zeros(4, dtype=np.float64))
    assert dist > 10.0


def test_joint_mpc_barrier_keeps_clearance_from_wall_cspace() -> None:
    """Control-space MPC with occupancy must not dive into wall configs."""
    kin = Kinematics("open_manipulator_x")
    xml = ensure_omx_wall_maze_mjcf()
    occ = build_wall_cspace_barrier_occupancy(
        mjcf_path=str(xml),
        joint_limits=kin.joint_limits,
        clearance=0.20,
        n_samples=10000,
        rng=np.random.default_rng(3),
    )
    # Seed a start outside the barrier and a target deep in the collision
    # cloud — without barriers the carrot tracker collapses onto the target.
    pts = np.asarray(occ.points, dtype=np.float64)
    # Drop the far dummy if present.
    pts = pts[np.linalg.norm(pts, axis=1) < 50.0]
    assert pts.shape[0] >= 30
    target = pts[0].copy()
    rng = np.random.default_rng(5)
    start = None
    for _ in range(8000):
        cand = target + rng.normal(0.0, 0.35, size=4)
        lo = np.asarray([lim[0] for lim in kin.joint_limits], dtype=np.float64)
        hi = np.asarray([lim[1] for lim in kin.joint_limits], dtype=np.float64)
        cand = np.clip(cand, lo, hi)
        if (
            not occ.is_occupied(cand)
            and float(np.linalg.norm(cand - target)) > 0.35
        ):
            start = cand
            break
    assert start is not None

    # ARCO ≥ v0.3.7: step dt must equal config.dt (FRET default 0.02 s).
    ctrl_dt = 0.02
    n_steps = 113  # ≈ 2.25 s — same horizon as the former 45 × 0.05 s loop
    mpc_open = build_omx_joint_mpc()
    mpc_open.reset(start)
    q_open = start.copy()
    min_open = 1.0e9
    for _ in range(n_steps):
        q_open = np.asarray(mpc_open.step(target, ctrl_dt), dtype=np.float64)
        d, _ = occ.nearest_obstacle(q_open)
        min_open = min(min_open, float(d))

    mpc_safe = build_omx_joint_mpc(occupancy=occ)
    mpc_safe.reset(start)
    q_safe = start.copy()
    min_safe = 1.0e9
    for _ in range(n_steps):
        q_safe = np.asarray(mpc_safe.step(target, ctrl_dt), dtype=np.float64)
        d, _ = occ.nearest_obstacle(q_safe)
        min_safe = min(min_safe, float(d))

    assert min_open < 0.05, "sanity: open MPC should enter the collision cloud"
    assert min_safe > min_open + 0.05
    assert min_safe > 0.12


def test_fit_clearance_keeps_a_barrier_the_waypoints_already_clear() -> None:
    occ = build_cspace_barrier_occupancy(
        np.array([[1.0, 1.0]], dtype=np.float64),
        clearance=0.2,
        joint_limits=[(-2, 2)] * 2,
    )
    fitted = fit_clearance_to_waypoints(occ, [np.zeros(2, dtype=np.float64)])
    assert fitted is occ
    assert float(fitted.clearance) == 0.2


def test_fit_clearance_shrinks_until_a_commanded_goal_is_reachable() -> None:
    """A goal inside the barrier stalls the arm one clearance short.

    From ARCO v0.5.0 the obstacle barrier is a half-space with a penalized
    slack whose marginal cost holds at the barrier weight however deep the
    arm is (deviations A-30 and A-33), so the joint-space MPC stops at the
    boundary instead of buying through it as the quartic penalty allowed.
    """
    goal = np.array([0.85, 0.0], dtype=np.float64)
    occ = build_cspace_barrier_occupancy(
        np.array([[1.0, 0.0]], dtype=np.float64),
        clearance=0.4,
        joint_limits=[(-2, 2)] * 2,
    )
    assert occ.is_occupied(goal) is True

    fitted = fit_clearance_to_waypoints(
        occ, [np.zeros(2, dtype=np.float64), goal]
    )

    assert fitted.is_occupied(goal) is False
    assert float(fitted.clearance) == pytest.approx(0.135)
    assert np.allclose(np.asarray(fitted.points), np.asarray(occ.points))


def test_fit_clearance_rejects_a_waypoint_on_an_obstacle() -> None:
    occ = build_cspace_barrier_occupancy(
        np.array([[0.5, 0.5]], dtype=np.float64),
        clearance=0.3,
        joint_limits=[(-2, 2)] * 2,
    )
    with pytest.raises(ValueError, match="collides"):
        fit_clearance_to_waypoints(
            occ, [np.array([0.5, 0.5], dtype=np.float64)]
        )
