"""C-space occupancy clouds for ARCO ``JointSpaceMPC`` soft barriers.

ARCO's joint-space MPC expects an occupancy map in **configuration space**
(``nearest_obstacle(q)`` returns a nearby colliding joint vector).  FRET's
planning stack uses world-frame point clouds + FK sampling; this module
bridges the two by sampling colliding joint configurations and wrapping them
in ``KDTreeOccupancy`` for the controller.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np
import numpy.typing as npt

try:
    from arco.mapping import KDTreeOccupancy
except ImportError:  # pragma: no cover
    KDTreeOccupancy = None


def sample_colliding_configurations(
    is_occupied: Callable[[npt.NDArray[np.float64]], bool],
    joint_limits: npt.NDArray[np.float64] | list[tuple[float, float]],
    *,
    n_samples: int = 12000,
    rng: np.random.Generator | None = None,
) -> npt.NDArray[np.float64]:
    """Return joint configs where ``is_occupied(q)`` is True.

    Args:
        is_occupied: Predicate on a joint vector.
        joint_limits: Per-joint ``(lo, hi)`` bounds, shape ``(dof, 2)``.
        n_samples: Uniform random samples drawn inside the joint box.
        rng: Optional NumPy generator (deterministic when seeded).

    Returns:
        Array of shape ``(M, dof)`` with ``M ≥ 0`` colliding samples.
    """
    limits = np.asarray(joint_limits, dtype=np.float64)
    if limits.ndim != 2 or limits.shape[1] != 2:
        raise ValueError(f"joint_limits must be (dof, 2), got {limits.shape}")
    dof = int(limits.shape[0])
    gen = rng if rng is not None else np.random.default_rng()
    qs = np.column_stack(
        [
            gen.uniform(
                float(limits[i, 0]), float(limits[i, 1]), int(n_samples)
            )
            for i in range(dof)
        ]
    )
    hits: list[npt.NDArray[np.float64]] = []
    for q in qs:
        if is_occupied(np.asarray(q, dtype=np.float64)):
            hits.append(np.asarray(q, dtype=np.float64).copy())
    if not hits:
        return np.zeros((0, dof), dtype=np.float64)
    return np.asarray(hits, dtype=np.float64)


def build_cspace_barrier_occupancy(
    collision_configs: npt.NDArray[np.float64],
    *,
    clearance: float = 0.28,
    joint_limits: (
        npt.NDArray[np.float64] | list[tuple[float, float]] | None
    ) = None,
    rng: np.random.Generator | None = None,
) -> Any:
    """Build a joint-space ``KDTreeOccupancy`` for MPC obstacle barriers.

    When ``collision_configs`` is empty, inserts a dummy point far outside the
    joint box so the KDTree constructor stays valid and barriers stay inert.

    Args:
        collision_configs: Colliding joint samples, shape ``(M, dof)``.
        clearance: Soft-barrier influence radius in joint-space units (rad).
        joint_limits: Optional bounds used to place the empty-cloud dummy.
        rng: Unused; retained for call-site symmetry with samplers.

    Returns:
        ARCO ``KDTreeOccupancy`` over configuration space.
    """
    _ = rng
    if KDTreeOccupancy is None:  # pragma: no cover
        raise ImportError("arco.mapping.KDTreeOccupancy is required")
    pts = np.asarray(collision_configs, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    if pts.shape[0] == 0:
        if joint_limits is None:
            dummy = np.full((1, 4), 1.0e3, dtype=np.float64)
        else:
            limits = np.asarray(joint_limits, dtype=np.float64)
            dof = int(limits.shape[0])
            dummy = np.full((1, dof), 1.0e3, dtype=np.float64)
        pts = dummy
    return KDTreeOccupancy(pts, clearance=float(clearance))


def mujoco_wall_occupied_predicate(
    mjcf_path: str,
    *,
    joint_names: tuple[str, ...] = ("Joint1", "Joint2", "Joint3", "Joint4"),
    wall_prefix: str | tuple[str, ...] = "transfer_wall",
) -> Callable[[npt.NDArray[np.float64]], bool]:
    """Return ``is_occupied(q)`` using MuJoCo contacts with wall geoms."""
    import mujoco as mj

    prefixes = (
        (wall_prefix,) if isinstance(wall_prefix, str) else tuple(wall_prefix)
    )
    model = mj.MjModel.from_xml_path(str(mjcf_path))
    data = mj.MjData(model)
    qadrs = [
        int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)])
        for n in joint_names
    ]
    box_jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pick_box_joint")
    if box_jid >= 0:
        qid = int(model.jnt_qposadr[box_jid])
        data.qpos[qid : qid + 3] = [0.5, 0.5, 0.5]

    def is_occupied(q: npt.NDArray[np.float64]) -> bool:
        qq = np.asarray(q, dtype=np.float64)
        for i, adr in enumerate(qadrs):
            data.qpos[adr] = float(qq[i])
        data.qvel[:] = 0.0
        mj.mj_forward(model, data)
        for ci in range(data.ncon):
            c = data.contact[ci]
            g1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom1) or ""
            g2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom2) or ""
            if any(
                g1.startswith(prefix) or g2.startswith(prefix)
                for prefix in prefixes
            ):
                return True
        return False

    return is_occupied


def build_wall_cspace_barrier_occupancy(
    *,
    mjcf_path: str,
    joint_limits: npt.NDArray[np.float64] | list[tuple[float, float]],
    joint_names: tuple[str, ...] = ("Joint1", "Joint2", "Joint3", "Joint4"),
    clearance: float = 0.28,
    n_samples: int = 12000,
    rng: np.random.Generator | None = None,
) -> Any:
    """Sample MuJoCo wall contacts and build a C-space barrier occupancy."""
    pred = mujoco_wall_occupied_predicate(mjcf_path, joint_names=joint_names)
    hits = sample_colliding_configurations(
        pred, joint_limits, n_samples=n_samples, rng=rng
    )
    return build_cspace_barrier_occupancy(
        hits, clearance=clearance, joint_limits=joint_limits, rng=rng
    )
