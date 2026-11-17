"""Structural protocols implemented by per-model robot variants (FR-SYS-08).

Each protocol is a creational seam: :class:`fret.robots.registry.RobotRegistry`
resolves a concrete implementation for the active robot model so that no
module outside ``fret.robots`` / ``config/robots`` needs to compare a model
name to a string literal.
"""

from __future__ import annotations

import pathlib
from typing import TYPE_CHECKING, Protocol

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    pass


class KinematicsBackend(Protocol):
    """Structural protocol shared by per-model kinematics engines.

    Moved here (public) from the private ``_KinematicsBackend`` that lived
    in ``control/kinematics.py`` before PR1.
    """

    @property
    def dof(self) -> int: ...

    @property
    def joint_names(self) -> list[str]: ...

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]: ...

    def forward_kinematics(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]: ...

    def inverse_kinematics(
        self,
        ee_pose: npt.NDArray[np.float64],
        seed: npt.NDArray[np.float64] | None = None,
    ) -> npt.NDArray[np.float64]: ...

    def jacobian(
        self, joint_positions: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]: ...


class ControllerFactory(Protocol):
    """Builds a Level-3 controller logic core for one robot model."""

    def __call__(self, model: str, config_path: str) -> object: ...


class TransferPlannerStrategy(Protocol):
    """Per-model arm transfer-path planning strategy.

    Replaces the ``robot_model == "omy"`` branch in
    ``control/pick_place_planning.py``. Not yet consumed by all call sites
    as of PR1 (tracked follow-up; see docs/decisions.md).
    """

    def plan(
        self,
        start: npt.NDArray[np.float64],
        goal: npt.NDArray[np.float64],
        *,
        scenario_path: str | pathlib.Path,
        seed_offset: int,
        prefer_detour: bool | None,
    ) -> tuple[list[npt.NDArray[np.float64]], bool]: ...


class SimIOAdapter(Protocol):
    """Per-model MuJoCo scenario/MJCF resolution adapter."""

    def mjcf_path(self, scenario: str) -> pathlib.Path: ...


class RobotIO(Protocol):
    """Hardware-agnostic seam the pick-and-place FSM drives through
    (FR-SYS-11). The MuJoCo implementation lives in
    ``fret.simulation.mujoco_robot_io``.
    """

    def read_joint_state(self) -> npt.NDArray[np.float64]: ...

    def send_joint_velocity(
        self, joint_velocity: npt.NDArray[np.float64]
    ) -> None: ...

    def set_gripper(self, value: float) -> None: ...

    def now(self) -> float: ...
