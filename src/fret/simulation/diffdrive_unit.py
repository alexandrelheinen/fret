"""Minimal differential-drive MuJoCo physics sandbox (FR-SIM-11).

Loads ``mjcf/diffdrive_unit.xml``: freejoint chassis + two hinge wheels.
Body velocity ``(v, ω)`` maps to left/right wheel angular rates. Non-holonomic
constraints come from wheel–floor contact only — this module must never call
``enforce_slide_yaw_nonholonomic_qvel``.
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import numpy.typing as npt

# Geometry must match diffdrive_unit.xml.
_WHEEL_RADIUS_M: float = 0.05
_TRACK_WIDTH_M: float = 0.18  # distance between wheel centres
_WHEEL_JOINT_NAMES: tuple[str, str] = ("wheel_l", "wheel_r")
_ACTUATOR_NAMES: tuple[str, str] = ("act_wheel_l", "act_wheel_r")
_DEFAULT_CONTROL_DT_S: float = 0.02


def diffdrive_unit_mjcf_path() -> pathlib.Path:
    """Return the path to ``diffdrive_unit.xml``."""
    return (
        pathlib.Path(__file__).resolve().parents[1]
        / "mjcf"
        / "diffdrive_unit.xml"
    )


def body_velocity_to_wheel_rates(
    linear_m_s: float,
    yaw_rate_rad_s: float,
    *,
    wheel_radius_m: float = _WHEEL_RADIUS_M,
    track_width_m: float = _TRACK_WIDTH_M,
) -> tuple[float, float]:
    """Map body ``(v, ω)`` to ``(ω_left, ω_right)`` [rad/s]."""
    if wheel_radius_m <= 0.0:
        raise ValueError("wheel_radius_m must be positive")
    half = 0.5 * track_width_m
    omega_l = (linear_m_s - yaw_rate_rad_s * half) / wheel_radius_m
    omega_r = (linear_m_s + yaw_rate_rad_s * half) / wheel_radius_m
    return float(omega_l), float(omega_r)


class DiffDriveUnitRobot:
    """Two-wheel differential-drive robot under pure MuJoCo physics.

    Args:
        mjcf_path: Optional MJCF override (defaults to ``diffdrive_unit.xml``).
        settle_s: Seconds of zero command after load so the chassis rests.
    """

    # Explicit policy flag for FR-SIM-11 audits / tests.
    USES_NONHOLONOMIC_QVEL_HACK: bool = False

    def __init__(
        self,
        *,
        mjcf_path: str | pathlib.Path | None = None,
        settle_s: float = 0.5,
    ) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "DiffDriveUnitRobot requires the optional mujoco package"
            ) from exc

        path = (
            pathlib.Path(mjcf_path)
            if mjcf_path is not None
            else diffdrive_unit_mjcf_path()
        )
        if not path.is_file():
            raise FileNotFoundError(f"Diff-drive unit MJCF not found: {path}")

        self._mujoco = mujoco
        self._mjcf_path = path
        self._model = mujoco.MjModel.from_xml_path(str(path))
        self._data = mujoco.MjData(self._model)
        self._actuator_ids = self._bind_actuators()
        self._wheel_radius_m = _WHEEL_RADIUS_M
        self._track_width_m = _TRACK_WIDTH_M
        self._mujoco.mj_forward(self._model, self._data)
        if settle_s > 0.0:
            self.step_body_velocity(0.0, 0.0, dt=settle_s)

    @property
    def mjcf_path(self) -> pathlib.Path:
        """Loaded MJCF path."""
        return self._mjcf_path

    @property
    def timestep_s(self) -> float:
        """MuJoCo integrator timestep [s]."""
        return float(self._model.opt.timestep)

    @property
    def wheel_radius_m(self) -> float:
        """Driven wheel radius [m]."""
        return self._wheel_radius_m

    @property
    def track_width_m(self) -> float:
        """Distance between wheel centres [m]."""
        return self._track_width_m

    def get_pose(self) -> tuple[float, float, float]:
        """Return planar pose ``(x, y, yaw)`` from the freejoint."""
        x = float(self._data.qpos[0])
        y = float(self._data.qpos[1])
        qw, qx, qy, qz = (float(v) for v in self._data.qpos[3:7])
        sin_y = 2.0 * (qw * qz + qx * qy)
        cos_y = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(sin_y, cos_y)
        return x, y, yaw

    def step_wheel_rates(
        self,
        omega_left_rad_s: float,
        omega_right_rad_s: float,
        *,
        dt: float = _DEFAULT_CONTROL_DT_S,
    ) -> tuple[float, float, float]:
        """Command wheel angular rates and advance physics by ``dt``."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        step_count = max(1, int(round(dt / self.timestep_s)))
        self._data.ctrl[self._actuator_ids[0]] = float(omega_left_rad_s)
        self._data.ctrl[self._actuator_ids[1]] = float(omega_right_rad_s)
        for _ in range(step_count):
            # Pure mj_step — no qvel projection / nonholonomic hack.
            self._mujoco.mj_step(self._model, self._data)
        return self.get_pose()

    def step_body_velocity(
        self,
        linear_m_s: float,
        yaw_rate_rad_s: float,
        *,
        dt: float = _DEFAULT_CONTROL_DT_S,
    ) -> tuple[float, float, float]:
        """Command body ``(v, ω)``, map to wheels, and advance physics."""
        omega_l, omega_r = body_velocity_to_wheel_rates(
            linear_m_s,
            yaw_rate_rad_s,
            wheel_radius_m=self._wheel_radius_m,
            track_width_m=self._track_width_m,
        )
        return self.step_wheel_rates(omega_l, omega_r, dt=dt)

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
                    f"Actuator not found in diff-drive unit MJCF: {name}"
                )
            actuator_ids.append(int(act_id))
        # Ensure wheel hinge joints exist (geometry contract).
        for name in _WHEEL_JOINT_NAMES:
            joint_id = self._mujoco.mj_name2id(
                self._model,
                self._mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            if joint_id < 0:
                raise ValueError(
                    f"Wheel joint not found in diff-drive unit MJCF: {name}"
                )
        return actuator_ids
