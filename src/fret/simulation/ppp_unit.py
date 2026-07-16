"""Minimal PPP (Cartesian) MuJoCo physics sandbox (FR-SIM-11).

Loads ``mjcf/ppp_unit.xml`` and exposes open-loop velocity stepping for
unit tests. Independent of the warehouse bridge and ROS SITL stack.
"""

from __future__ import annotations

import pathlib

import numpy as np
import numpy.typing as npt

_JOINT_NAMES: tuple[str, ...] = ("joint_x", "joint_y", "joint_z")
_ACTUATOR_NAMES: tuple[str, ...] = (
    "act_joint_x",
    "act_joint_y",
    "act_joint_z",
)
_DEFAULT_LIMITS: npt.NDArray[np.float64] = np.array(
    [[0.0, 4.0], [0.0, 4.0], [0.0, 3.0]],
    dtype=np.float64,
)
_DEFAULT_CONTROL_DT_S: float = 0.02


def ppp_unit_mjcf_path() -> pathlib.Path:
    """Return the path to ``ppp_unit.xml``."""
    return (
        pathlib.Path(__file__).resolve().parents[1] / "mjcf" / "ppp_unit.xml"
    )


class PPPUnitRobot:
    """Three-prismatic Cartesian robot driven by MuJoCo velocity actuators.

    Args:
        mjcf_path: Optional MJCF override (defaults to ``ppp_unit.xml``).
        initial_positions: Starting ``(x, y, z)`` [m].
    """

    def __init__(
        self,
        *,
        mjcf_path: str | pathlib.Path | None = None,
        initial_positions: npt.NDArray[np.float64] | None = None,
    ) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "PPPUnitRobot requires the optional mujoco package"
            ) from exc

        path = (
            pathlib.Path(mjcf_path)
            if mjcf_path is not None
            else ppp_unit_mjcf_path()
        )
        if not path.is_file():
            raise FileNotFoundError(f"PPP unit MJCF not found: {path}")

        self._mujoco = mujoco
        self._mjcf_path = path
        self._model = mujoco.MjModel.from_xml_path(str(path))
        self._data = mujoco.MjData(self._model)
        self._limits = _DEFAULT_LIMITS.copy()
        self._qpos_adrs, self._qvel_adrs = self._bind_joints()
        self._actuator_ids = self._bind_actuators()

        q0 = (
            np.array([1.0, 1.0, 1.5], dtype=np.float64)
            if initial_positions is None
            else np.asarray(initial_positions, dtype=np.float64)
        )
        if q0.shape != (3,):
            raise ValueError("initial_positions must have shape (3,)")
        self.set_positions(q0)

    @property
    def mjcf_path(self) -> pathlib.Path:
        """Loaded MJCF path."""
        return self._mjcf_path

    @property
    def timestep_s(self) -> float:
        """MuJoCo integrator timestep [s]."""
        return float(self._model.opt.timestep)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Ordered prismatic joint names."""
        return _JOINT_NAMES

    def get_positions(self) -> npt.NDArray[np.float64]:
        """Return current joint positions ``(x, y, z)`` [m]."""
        return np.array(
            [float(self._data.qpos[adr]) for adr in self._qpos_adrs],
            dtype=np.float64,
        )

    def get_velocities(self) -> npt.NDArray[np.float64]:
        """Return current joint velocities [m/s]."""
        return np.array(
            [float(self._data.qvel[adr]) for adr in self._qvel_adrs],
            dtype=np.float64,
        )

    def set_positions(self, positions: npt.NDArray[np.float64]) -> None:
        """Teleport joints (unit-test setup only; not used during stepping)."""
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != (3,):
            raise ValueError("positions must have shape (3,)")
        q = np.clip(q, self._limits[:, 0], self._limits[:, 1])
        for adr, value in zip(self._qpos_adrs, q, strict=True):
            self._data.qpos[adr] = float(value)
        self._data.qvel[:] = 0.0
        self._mujoco.mj_forward(self._model, self._data)

    def step_velocity(
        self,
        velocities: npt.NDArray[np.float64],
        *,
        dt: float = _DEFAULT_CONTROL_DT_S,
    ) -> npt.NDArray[np.float64]:
        """Apply Cartesian velocity commands and advance physics by ``dt``.

        Args:
            velocities: Target ``(vx, vy, vz)`` [m/s].
            dt: Control period [s]; must be a positive multiple of the MJCF
                timestep (substeps = ``round(dt / timestep)``).

        Returns:
            Joint positions after the step.
        """
        v = np.asarray(velocities, dtype=np.float64).reshape(3)
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        step_count = max(1, int(round(dt / self.timestep_s)))
        for idx, act_id in enumerate(self._actuator_ids):
            self._data.ctrl[act_id] = float(v[idx])
        for _ in range(step_count):
            self._mujoco.mj_step(self._model, self._data)
        return self.get_positions()

    def _bind_joints(self) -> tuple[list[int], list[int]]:
        qpos_adrs: list[int] = []
        qvel_adrs: list[int] = []
        for name in _JOINT_NAMES:
            joint_id = self._mujoco.mj_name2id(
                self._model,
                self._mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            if joint_id < 0:
                raise ValueError(f"Joint not found in PPP unit MJCF: {name}")
            qpos_adrs.append(int(self._model.jnt_qposadr[joint_id]))
            qvel_adrs.append(int(self._model.jnt_dofadr[joint_id]))
        return qpos_adrs, qvel_adrs

    def _bind_actuators(self) -> list[int]:
        actuator_ids: list[int] = []
        for name in _ACTUATOR_NAMES:
            act_id = self._mujoco.mj_name2id(
                self._model,
                self._mujoco.mjtObj.mjOBJ_ACTUATOR,
                name,
            )
            if act_id < 0:
                raise ValueError(
                    f"Actuator not found in PPP unit MJCF: {name}"
                )
            actuator_ids.append(int(act_id))
        return actuator_ids


def mujoco_available() -> bool:
    """Return ``True`` when the optional ``mujoco`` package imports."""
    try:
        import mujoco
    except ImportError:  # pragma: no cover
        return False
    return mujoco is not None
