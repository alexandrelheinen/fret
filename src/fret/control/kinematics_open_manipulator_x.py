"""OpenMANIPULATOR-X kinematics via Menagerie MuJoCo FK (v1.2.3).

FK and Jacobian are evaluated on the bundled Menagerie model. IK is a
damped least-squares numerical solver seeded from the current configuration
(sufficient for empty-cell A→B demos; refine later for clutter).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import numpy.typing as npt

_JOINT_NAMES: list[str] = ["Joint1", "Joint2", "Joint3", "Joint4"]
_EE_BODY = "link5"
_MAX_IK_ITERS = 80
_IK_TOL_M = 1e-3
_IK_DAMPING = 1e-2


def _menagerie_omx_xml() -> Path:
    """Return path to Menagerie ``open_manipulator_x.xml``."""
    return (
        Path(__file__).resolve().parents[3]
        / "third_party"
        / "robotis_mujoco_menagerie"
        / "robotis_open_manipulator_x"
        / "open_manipulator_x.xml"
    )


class OpenManipulatorXKinematics:
    """4-DOF OpenMANIPULATOR-X kinematics backed by MuJoCo."""

    def __init__(self) -> None:
        try:
            import mujoco as mj
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "mujoco is required for OpenMANIPULATOR-X kinematics "
                "(pip install -e '.[sim]')"
            ) from exc

        xml = _menagerie_omx_xml()
        if not xml.is_file():
            raise FileNotFoundError(
                f"Menagerie OM-X MJCF missing: {xml}. "
                "Run: git submodule update --init --recursive"
            )
        self._mj = mj
        self._model = mj.MjModel.from_xml_path(str(xml))
        self._data = mj.MjData(self._model)
        self._qpos_adrs = [
            int(
                self._model.jnt_qposadr[
                    self._mj.mj_name2id(
                        self._model, self._mj.mjtObj.mjOBJ_JOINT, name
                    )
                ]
            )
            for name in _JOINT_NAMES
        ]
        self._body_id = int(
            self._mj.mj_name2id(
                self._model, self._mj.mjtObj.mjOBJ_BODY, _EE_BODY
            )
        )
        limits = []
        for name in _JOINT_NAMES:
            jid = self._mj.mj_name2id(
                self._model, self._mj.mjtObj.mjOBJ_JOINT, name
            )
            limits.append(
                [
                    float(self._model.jnt_range[jid, 0]),
                    float(self._model.jnt_range[jid, 1]),
                ]
            )
        self._limits = np.asarray(limits, dtype=np.float64)

    @property
    def dof(self) -> int:
        """Arm degrees of freedom (gripper excluded)."""
        return 4

    @property
    def joint_names(self) -> list[str]:
        """Menagerie arm joint names."""
        return list(_JOINT_NAMES)

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """Joint limits ``(4, 2)`` lower/upper [rad]."""
        return self._limits.copy()

    def _set_q(self, q: npt.NDArray[np.float64]) -> None:
        q = np.asarray(q, dtype=np.float64).reshape(4)
        for adr, val in zip(self._qpos_adrs, q):
            self._data.qpos[adr] = float(val)
        # Keep gripper fingers closed-open mid for FK.
        self._mj.mj_forward(self._model, self._data)

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return 4×4 EE pose of ``link5`` in the world frame."""
        self._set_q(joint_positions)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self._data.xmat[self._body_id].reshape(3, 3)
        T[:3, 3] = self._data.xpos[self._body_id]
        return T

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return geometric Jacobian ``(6, 4)`` at ``link5``."""
        self._set_q(joint_positions)
        jacp = np.zeros((3, self._model.nv), dtype=np.float64)
        jacr = np.zeros((3, self._model.nv), dtype=np.float64)
        self._mj.mj_jacBody(self._model, self._data, jacp, jacr, self._body_id)
        # Map nv columns → the four arm hinge DOFs (qvel adr == qpos for hinges).
        cols = []
        for name in _JOINT_NAMES:
            jid = self._mj.mj_name2id(
                self._model, self._mj.mjtObj.mjOBJ_JOINT, name
            )
            cols.append(int(self._model.jnt_dofadr[jid]))
        J = np.vstack([jacp[:, cols], jacr[:, cols]])
        return J.astype(np.float64)

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]:
        """Numerical position IK (damped least squares) for EE translation."""
        target = np.asarray(ee_pose, dtype=np.float64)
        if target.shape == (4, 4):
            target_xyz = target[:3, 3]
        else:
            target_xyz = target.reshape(-1)[:3]

        q = (
            np.asarray(seed, dtype=np.float64).reshape(4)
            if seed is not None
            else np.zeros(4, dtype=np.float64)
        )
        for _ in range(_MAX_IK_ITERS):
            T = self.forward_kinematics(q)
            err = target_xyz - T[:3, 3]
            if float(np.linalg.norm(err)) < _IK_TOL_M:
                break
            J = self.jacobian(q)[:3, :]
            JJt = J @ J.T + _IK_DAMPING * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, err)
            q = np.clip(q + dq, self._limits[:, 0], self._limits[:, 1])
        return q.astype(np.float64)
