"""ARCO JointSpaceMPC helpers for OpenMANIPULATOR-X pick-and-place.

Replaces proportional / Jacobian waypoint tracking with the CasADi carrot
NMPC from ARCO v0.3.2 (``build_joint_tracker(..., tracker="mpc")``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt
from arco.control.mpc import JointSpaceMPC, JointSpaceMPCConfig
from arco.simulator.sim.tracking import build_joint_tracker

if TYPE_CHECKING:
    from arco.mapping.occupancy import Occupancy

_OMX_DOF = 4
_DEFAULT_MAX_VEL_RAD_S = 1.57
_DEFAULT_MAX_ACC_RAD_S2 = 3.0


def build_joint_mpc(
    dof: int = _OMX_DOF,
    *,
    occupancy: Occupancy | None = None,
    max_vel: float = _DEFAULT_MAX_VEL_RAD_S,
    max_acc: float = _DEFAULT_MAX_ACC_RAD_S2,
    mpc_cfg: JointSpaceMPCConfig | None = None,
) -> JointSpaceMPC:
    """Build a joint-space MPC tracker for ``dof`` arm joints."""
    max_vel_vec = np.full(int(dof), float(max_vel), dtype=np.float64)
    max_acc_vec = np.full(int(dof), float(max_acc), dtype=np.float64)
    tracker = build_joint_tracker(
        max_vel=max_vel_vec,
        max_acc=max_acc_vec,
        occupancy=occupancy,
        tracker="mpc",
        mpc_cfg=mpc_cfg,
    )
    assert isinstance(tracker, JointSpaceMPC)
    return tracker


def build_omx_joint_mpc(
    *,
    occupancy: Occupancy | None = None,
    max_vel: float = _DEFAULT_MAX_VEL_RAD_S,
    max_acc: float = _DEFAULT_MAX_ACC_RAD_S2,
    mpc_cfg: JointSpaceMPCConfig | None = None,
) -> JointSpaceMPC:
    """Build a 4-DOF joint-space MPC tracker for OMX."""
    return build_joint_mpc(
        _OMX_DOF,
        occupancy=occupancy,
        max_vel=max_vel,
        max_acc=max_acc,
        mpc_cfg=mpc_cfg,
    )


def path_arc_lengths(
    path: list[npt.NDArray[np.float64]],
) -> list[float]:
    """Return cumulative arc lengths along a joint-space path."""
    if not path:
        return [0.0]
    arcs = [0.0]
    for i in range(1, len(path)):
        step = float(np.linalg.norm(path[i] - path[i - 1]))
        arcs.append(arcs[-1] + step)
    return arcs


def path_pos_at_arc(
    path: list[npt.NDArray[np.float64]],
    arcs: list[float],
    dist: float,
) -> npt.NDArray[np.float64]:
    """Interpolate a joint configuration at arc length ``dist``."""
    if not path:
        raise ValueError("path must be non-empty")
    if len(path) == 1 or arcs[-1] <= 1e-12:
        return np.asarray(path[0], dtype=np.float64).copy()
    d = float(np.clip(dist, 0.0, arcs[-1]))
    for i in range(1, len(arcs)):
        if d <= arcs[i]:
            seg = arcs[i] - arcs[i - 1]
            if seg <= 1e-12:
                return np.asarray(path[i], dtype=np.float64).copy()
            alpha = (d - arcs[i - 1]) / seg
            return (1.0 - alpha) * path[i - 1] + alpha * path[i]
    return np.asarray(path[-1], dtype=np.float64).copy()


class JointPathMPCTracker:
    """Carrot-on-a-path wrapper around :class:`JointSpaceMPC`.

    Mirrors the ARCO PPP/RRP race loop: advance a carrot along the dense
    path when lag is small, then ``mpc.step(carrot, dt)``.
    """

    def __init__(
        self,
        path: list[npt.NDArray[np.float64]],
        mpc: JointSpaceMPC,
        *,
        race_speed: float = 0.8,
        max_carrot_lag: float = 0.25,
        goal_tol: float = 0.12,
    ) -> None:
        if len(path) < 2:
            raise ValueError("path must contain at least 2 waypoints")
        self._path = [np.asarray(q, dtype=np.float64) for q in path]
        self._arcs = path_arc_lengths(self._path)
        self._mpc = mpc
        self._race_speed = float(race_speed)
        self._max_carrot_lag = float(max_carrot_lag)
        self._goal_tol = float(goal_tol)
        self._carrot_dist = 0.0
        self._carrot = self._path[0].copy()
        self._complete = False

    @property
    def complete(self) -> bool:
        """True when the tracker has reached the final path waypoint."""
        return self._complete

    @property
    def mpc(self) -> JointSpaceMPC:
        """Underlying joint-space MPC instance."""
        return self._mpc

    @property
    def q(self) -> npt.NDArray[np.float64]:
        """Current MPC configuration estimate."""
        return np.asarray(self._mpc.q, dtype=np.float64)

    def reset(self, q0: npt.NDArray[np.float64]) -> None:
        """Reset MPC state and carrot to the start of the path."""
        self._mpc.reset(np.asarray(q0, dtype=np.float64))
        self._carrot_dist = 0.0
        self._carrot = self._path[0].copy()
        self._complete = False

    def sync_from_measurement(self, q_meas: npt.NDArray[np.float64]) -> None:
        """Align the MPC state with a measured joint configuration."""
        sync_mpc_state_from_measurement(self._mpc, q_meas)

    def step(self, dt: float) -> npt.NDArray[np.float64]:
        """Advance carrot + MPC; return the new configuration command."""
        if self._complete:
            return self._path[-1].copy()
        lag = float(np.linalg.norm(self._mpc.q - self._carrot))
        if lag < self._max_carrot_lag:
            self._carrot_dist = min(
                self._carrot_dist + self._race_speed * float(dt),
                self._arcs[-1],
            )
        self._carrot = path_pos_at_arc(
            self._path, self._arcs, self._carrot_dist
        )
        q = np.asarray(
            self._mpc.step(self._carrot, float(dt)), dtype=np.float64
        )
        if float(np.linalg.norm(q - self._path[-1])) < self._goal_tol:
            self._complete = True
            return self._path[-1].copy()
        return q


def sync_mpc_state_from_measurement(
    mpc: JointSpaceMPC, q_meas: npt.NDArray[np.float64]
) -> None:
    """Pull the MPC configuration toward the measured joint state.

    MuJoCo position actuators may lag the command; keeping the NLP state
    near the measured ``q`` avoids large warm-start errors.
    """
    mpc.q = np.asarray(q_meas, dtype=np.float64).copy()
