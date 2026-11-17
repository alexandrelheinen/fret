"""Robot profile schema: load and validate ``config/robots/<model>.yml``.

A profile is the single source of truth for a robot model's static shape
(joint names/limits, which config files it uses, which kind of transfer
planning and vision defaults apply). Per-model *behavior* (kinematics,
controller, transfer-planner strategy, simulation I/O, vision defaults) is
resolved through :mod:`fret.robots.registry` using the keys declared here
(FR-SYS-06, FR-SYS-07).
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass, field
from typing import Any

import yaml

_REQUIRED_STR_FIELDS: tuple[str, ...] = (
    "name",
    "kind",
    "kinematics",
    "controller_config",
    "planning_config",
)
_VALID_KINDS: frozenset[str] = frozenset({"mobile", "arm"})


class RobotProfileError(ValueError):
    """Raised when a robot profile file is missing, ill-typed, or invalid.

    Always names the offending field and the source file (FR-SYS-07).
    """


@dataclass(frozen=True)
class RobotProfile:
    """Static description of one robot model, loaded from one YAML file.

    Attributes:
        key: Canonical registry key (the YAML file stem, e.g.
            ``"open_manipulator_x"``).
        name: Human-readable display name.
        kind: ``"mobile"`` or ``"arm"``.
        joint_names: Joint names in kinematic-chain order.
        joint_limits: ``(DOF, 2)`` array-like of ``[lower, upper]`` pairs.
        kinematics: Registry key a ``KinematicsBackend`` factory is
            registered under for this model (``register_kinematics``).
        controller_config: Package-relative controller YAML path.
        planning_config: Package-relative planning YAML path.
        aliases: Additional accepted keys for this model (FR-SYS-06).
        mjcf: Package-relative default MJCF path, when static.
        vision_config: Package-relative default vision YAML, when used.
        grasp_modes: Supported grasp modes (``arm`` only).
        transfer_planner: Registry key a ``TransferPlannerStrategy`` is
            registered under, when this model needs one.
        uses_vision: Whether the pick-and-place pipeline runs vision for
            this model.
        source_file: Absolute path to the YAML this profile was loaded
            from (for error messages).
    """

    key: str
    name: str
    kind: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...]
    kinematics: str
    controller_config: str
    planning_config: str
    aliases: tuple[str, ...] = field(default_factory=tuple)
    mjcf: str | None = None
    vision_config: str | None = None
    grasp_modes: tuple[str, ...] = field(default_factory=tuple)
    transfer_planner: str | None = None
    uses_vision: bool = False
    source_file: str = ""

    @property
    def dof(self) -> int:
        """Number of joints declared for this model."""
        return len(self.joint_names)


def _require(data: dict[str, Any], field_name: str, path: pathlib.Path) -> Any:
    if field_name not in data:
        raise RobotProfileError(
            f"Robot profile missing required field {field_name!r} in {path}"
        )
    return data[field_name]


def _require_type(
    value: Any, expected: type | tuple[type, ...], field_name: str, path: pathlib.Path
) -> Any:
    if not isinstance(value, expected):
        raise RobotProfileError(
            f"Robot profile field {field_name!r} in {path} must be "
            f"{expected}, got {type(value).__name__}"
        )
    return value


def load_robot_profile(path: str | pathlib.Path) -> RobotProfile:
    """Load and validate one robot profile YAML (FR-SYS-07).

    Args:
        path: Path to ``config/robots/<model>.yml``.

    Returns:
        A validated, frozen :class:`RobotProfile`. ``key`` is the file
        stem (canonical registry key).

    Raises:
        FileNotFoundError: If ``path`` does not exist.
        RobotProfileError: If a required field is missing or ill-typed,
            or if ``mjcf``/``vision_config``/``controller_config``/
            ``planning_config`` name a file that does not exist.
    """
    file_path = pathlib.Path(path)
    if not file_path.is_file():
        raise FileNotFoundError(f"Robot profile not found: {file_path}")

    with file_path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise RobotProfileError(f"Invalid YAML root (expected mapping): {file_path}")

    for field_name in _REQUIRED_STR_FIELDS:
        _require_type(
            _require(data, field_name, file_path), str, field_name, file_path
        )

    kind = str(data["kind"])
    if kind not in _VALID_KINDS:
        raise RobotProfileError(
            f"Robot profile field 'kind' in {file_path} must be one of "
            f"{sorted(_VALID_KINDS)}, got {kind!r}"
        )

    joint_names_raw = _require(data, "joint_names", file_path)
    _require_type(joint_names_raw, list, "joint_names", file_path)
    joint_names = tuple(str(n) for n in joint_names_raw)
    if not joint_names:
        raise RobotProfileError(
            f"Robot profile field 'joint_names' in {file_path} must not be empty"
        )

    joint_limits_raw = _require(data, "joint_limits", file_path)
    _require_type(joint_limits_raw, list, "joint_limits", file_path)
    if len(joint_limits_raw) != len(joint_names):
        raise RobotProfileError(
            f"Robot profile 'joint_limits' in {file_path} must have "
            f"{len(joint_names)} entries (one per joint_names), got "
            f"{len(joint_limits_raw)}"
        )
    joint_limits: list[tuple[float, float]] = []
    for i, pair in enumerate(joint_limits_raw):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise RobotProfileError(
                f"Robot profile 'joint_limits[{i}]' in {file_path} must be "
                "a 2-element [lower, upper] pair"
            )
        joint_limits.append((float(pair[0]), float(pair[1])))

    aliases_raw = data.get("aliases", [])
    _require_type(aliases_raw, list, "aliases", file_path)
    aliases = tuple(str(a) for a in aliases_raw)

    grasp_modes_raw = data.get("grasp_modes", [])
    _require_type(grasp_modes_raw, list, "grasp_modes", file_path)
    grasp_modes = tuple(str(g) for g in grasp_modes_raw)
    if kind == "mobile" and grasp_modes:
        raise RobotProfileError(
            f"Robot profile {file_path} has kind 'mobile' but declares "
            "grasp_modes; mobile robots have no grasp modes"
        )

    mjcf = data.get("mjcf")
    if mjcf is not None:
        _require_type(mjcf, str, "mjcf", file_path)
    vision_config = data.get("vision_config")
    if vision_config is not None:
        _require_type(vision_config, str, "vision_config", file_path)
    transfer_planner = data.get("transfer_planner")
    if transfer_planner is not None:
        _require_type(transfer_planner, str, "transfer_planner", file_path)
    uses_vision = bool(data.get("uses_vision", False))

    return RobotProfile(
        key=file_path.stem,
        name=str(data["name"]),
        kind=kind,
        joint_names=joint_names,
        joint_limits=tuple(joint_limits),
        kinematics=str(data["kinematics"]),
        controller_config=str(data["controller_config"]),
        planning_config=str(data["planning_config"]),
        aliases=aliases,
        mjcf=mjcf,
        vision_config=vision_config,
        grasp_modes=grasp_modes,
        transfer_planner=transfer_planner,
        uses_vision=uses_vision,
        source_file=str(file_path),
    )
