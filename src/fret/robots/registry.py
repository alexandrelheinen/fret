"""Robot model registry: profile lookup and per-model variant tables.

The registry is the single place that turns a robot model name (from a
scenario YAML, a launch argument, or a CLI flag) into behavior: a
:class:`fret.robots.profile.RobotProfile`, a kinematics backend, a
controller factory, and (when applicable) a transfer-planner strategy or a
simulation I/O adapter (FR-SYS-06, FR-SYS-08).

Two registries built from the same files are equivalent: profiles are
always re-read from disk, never cached as mutable global state (edge case:
"two launches in one process build the registry twice").
"""

from __future__ import annotations

import pathlib
from typing import Callable, TypeVar

from fret.robots.profile import RobotProfile, load_robot_profile
from fret.robots.protocols import (
    ControllerFactory,
    KinematicsBackend,
    SimIOAdapter,
    TransferPlannerStrategy,
)

T = TypeVar("T")


class UnknownRobotModelError(ValueError):
    """Raised when a model key is not a known profile key or alias (FR-SYS-06)."""

    def __init__(self, model: str, known_keys: list[str]) -> None:
        self.model = model
        self.known_keys = known_keys
        joined = ", ".join(known_keys)
        super().__init__(
            f"Unknown robot model {model!r}. Known model keys: {joined}"
        )


class RobotRegistrationError(ValueError):
    """Raised on alias collisions or duplicate variant registration."""


class RobotVariantNotRegisteredError(ValueError):
    """Raised when a profile names a variant key with no registered factory."""


def _default_profiles_dir() -> pathlib.Path:
    """Return ``config/robots`` from the package share or source tree."""
    from fret.sitl_config import resolve_package_file

    # Any file under config/robots/ anchors the directory; dubins.yml is
    # always present (FR-SYS-07 required profile).
    return resolve_package_file("config", "robots", "dubins.yml").parent


class RobotRegistry:
    """Resolves robot models to profiles and per-model variant factories.

    Args:
        profiles_dir: Directory containing ``*.yml`` robot profiles.
            Defaults to the installed/source ``config/robots`` directory.
        kinematics_factories: Mapping of registry key -> zero-arg factory
            returning a :class:`KinematicsBackend`. Defaults to the module
            -level table populated by :func:`register_kinematics`.
        controller_factories: Mapping of registry key -> :class:`ControllerFactory`.
        transfer_planner_factories: Mapping of registry key -> zero-arg
            factory returning a :class:`TransferPlannerStrategy`.
        sim_io_factories: Mapping of registry key -> zero-arg factory
            returning a :class:`SimIOAdapter`.

    Raises:
        RobotRegistrationError: If two profiles declare the same alias, or
            an alias collides with another profile's canonical key.
    """

    def __init__(
        self,
        profiles_dir: str | pathlib.Path | None = None,
        *,
        kinematics_factories: (
            dict[str, Callable[[], KinematicsBackend]] | None
        ) = None,
        controller_factories: dict[str, ControllerFactory] | None = None,
        transfer_planner_factories: (
            dict[str, Callable[[], TransferPlannerStrategy]] | None
        ) = None,
        sim_io_factories: dict[str, Callable[[], SimIOAdapter]] | None = None,
    ) -> None:
        directory = (
            pathlib.Path(profiles_dir)
            if profiles_dir is not None
            else _default_profiles_dir()
        )
        self._profiles: dict[str, RobotProfile] = {}
        self._alias_to_key: dict[str, str] = {}
        self._load_profiles(directory)

        self._kinematics_factories = (
            dict(kinematics_factories)
            if kinematics_factories is not None
            else dict(_KINEMATICS_FACTORIES)
        )
        self._controller_factories = (
            dict(controller_factories)
            if controller_factories is not None
            else dict(_CONTROLLER_FACTORIES)
        )
        self._transfer_planner_factories = (
            dict(transfer_planner_factories)
            if transfer_planner_factories is not None
            else dict(_TRANSFER_PLANNER_FACTORIES)
        )
        self._sim_io_factories = (
            dict(sim_io_factories)
            if sim_io_factories is not None
            else dict(_SIM_IO_FACTORIES)
        )

    def _load_profiles(self, directory: pathlib.Path) -> None:
        if not directory.is_dir():
            raise FileNotFoundError(
                f"Robot profiles directory not found: {directory}"
            )
        for path in sorted(directory.glob("*.yml")):
            profile = load_robot_profile(path)
            if profile.key in self._profiles:
                raise RobotRegistrationError(
                    f"Duplicate robot profile key {profile.key!r}: "
                    f"{self._profiles[profile.key].source_file} and {path}"
                )
            self._profiles[profile.key] = profile
            for alias in (profile.key, *profile.aliases):
                if alias in self._alias_to_key and alias != profile.key:
                    other_key = self._alias_to_key[alias]
                    other_file = self._profiles[other_key].source_file
                    raise RobotRegistrationError(
                        f"Alias {alias!r} declared by {path} collides with "
                        f"model {other_key!r} from {other_file}"
                    )
                if alias in self._profiles and alias != profile.key:
                    raise RobotRegistrationError(
                        f"Alias {alias!r} declared by {path} collides with "
                        f"the canonical key of another profile "
                        f"({self._profiles[alias].source_file})"
                    )
                self._alias_to_key[alias] = profile.key

    # ------------------------------------------------------------------
    # Profile lookup (FR-SYS-06)
    # ------------------------------------------------------------------

    def known_keys(self) -> list[str]:
        """Return every canonical key and alias known to this registry."""
        return sorted(self._alias_to_key)

    def resolve_key(self, model: str) -> str:
        """Return the canonical profile key for ``model`` (key or alias).

        Raises:
            UnknownRobotModelError: When ``model`` is not a known key or
                alias. Keys are matched exactly (no case folding, no
                trimming).
        """
        key = self._alias_to_key.get(model)
        if key is None:
            raise UnknownRobotModelError(model, self.known_keys())
        return key

    def get_profile(self, model: str) -> RobotProfile:
        """Return the validated :class:`RobotProfile` for ``model``."""
        return self._profiles[self.resolve_key(model)]

    # ------------------------------------------------------------------
    # Variant resolution (FR-SYS-08)
    # ------------------------------------------------------------------

    def get_kinematics(self, model: str) -> KinematicsBackend:
        """Return a fresh kinematics backend instance for ``model``."""
        profile = self.get_profile(model)
        factory = self._kinematics_factories.get(profile.kinematics)
        if factory is None:
            raise RobotVariantNotRegisteredError(
                f"No kinematics backend registered for key "
                f"{profile.kinematics!r} (model {model!r}, {profile.source_file})"
            )
        return factory()

    def get_controller_factory(self, model: str) -> ControllerFactory:
        """Return the registered controller factory for ``model``."""
        profile = self.get_profile(model)
        factory = self._controller_factories.get(profile.kinematics)
        if factory is None:
            raise RobotVariantNotRegisteredError(
                f"No controller factory registered for key "
                f"{profile.kinematics!r} (model {model!r}, {profile.source_file})"
            )
        return factory

    def get_transfer_planner(
        self, model: str
    ) -> TransferPlannerStrategy | None:
        """Return the registered transfer-planner strategy for ``model``.

        Returns ``None`` when the profile does not declare
        ``transfer_planner`` (not every model needs one).
        """
        profile = self.get_profile(model)
        if profile.transfer_planner is None:
            return None
        factory = self._transfer_planner_factories.get(
            profile.transfer_planner
        )
        if factory is None:
            raise RobotVariantNotRegisteredError(
                f"No transfer-planner strategy registered for key "
                f"{profile.transfer_planner!r} (model {model!r}, "
                f"{profile.source_file})"
            )
        return factory()

    def get_sim_io(self, model: str) -> SimIOAdapter | None:
        """Return the registered simulation I/O adapter for ``model``, if any."""
        profile = self.get_profile(model)
        factory = self._sim_io_factories.get(profile.kinematics)
        if factory is None:
            return None
        return factory()


# ---------------------------------------------------------------------------
# Module-level variant tables populated by decorators in
# ``fret.robots.variants.*`` at import time. A fresh ``RobotRegistry()``
# captures a snapshot of these tables; it never mutates them.
# ---------------------------------------------------------------------------

_KINEMATICS_FACTORIES: dict[str, Callable[[], KinematicsBackend]] = {}
_CONTROLLER_FACTORIES: dict[str, ControllerFactory] = {}
_TRANSFER_PLANNER_FACTORIES: dict[
    str, Callable[[], TransferPlannerStrategy]
] = {}
_SIM_IO_FACTORIES: dict[str, Callable[[], SimIOAdapter]] = {}


def register_kinematics(
    key: str,
) -> Callable[
    [Callable[[], KinematicsBackend]], Callable[[], KinematicsBackend]
]:
    """Decorator: register a zero-arg ``KinematicsBackend`` factory under ``key``.

    Raises:
        RobotRegistrationError: If ``key`` is already registered.
    """

    def decorator(
        factory: Callable[[], KinematicsBackend],
    ) -> Callable[[], KinematicsBackend]:
        if key in _KINEMATICS_FACTORIES:
            raise RobotRegistrationError(
                f"Kinematics backend already registered for key {key!r}"
            )
        _KINEMATICS_FACTORIES[key] = factory
        return factory

    return decorator


def register_controller(
    key: str,
) -> Callable[[ControllerFactory], ControllerFactory]:
    """Decorator: register a :class:`ControllerFactory` under ``key``.

    Raises:
        RobotRegistrationError: If ``key`` is already registered.
    """

    def decorator(factory: ControllerFactory) -> ControllerFactory:
        if key in _CONTROLLER_FACTORIES:
            raise RobotRegistrationError(
                f"Controller factory already registered for key {key!r}"
            )
        _CONTROLLER_FACTORIES[key] = factory
        return factory

    return decorator


def register_transfer_planner(
    key: str,
) -> Callable[
    [Callable[[], TransferPlannerStrategy]],
    Callable[[], TransferPlannerStrategy],
]:
    """Decorator: register a zero-arg ``TransferPlannerStrategy`` factory."""

    def decorator(
        factory: Callable[[], TransferPlannerStrategy],
    ) -> Callable[[], TransferPlannerStrategy]:
        if key in _TRANSFER_PLANNER_FACTORIES:
            raise RobotRegistrationError(
                f"Transfer-planner strategy already registered for key {key!r}"
            )
        _TRANSFER_PLANNER_FACTORIES[key] = factory
        return factory

    return decorator


def register_sim_io(
    key: str,
) -> Callable[[Callable[[], SimIOAdapter]], Callable[[], SimIOAdapter]]:
    """Decorator: register a zero-arg ``SimIOAdapter`` factory under ``key``."""

    def decorator(
        factory: Callable[[], SimIOAdapter],
    ) -> Callable[[], SimIOAdapter]:
        if key in _SIM_IO_FACTORIES:
            raise RobotRegistrationError(
                f"Sim I/O adapter already registered for key {key!r}"
            )
        _SIM_IO_FACTORIES[key] = factory
        return factory

    return decorator


_default_registry: RobotRegistry | None = None


def get_default_registry() -> RobotRegistry:
    """Return the process-wide default registry (lazy singleton).

    Built from ``config/robots`` the first time it is requested. Call
    :func:`reset_default_registry` in tests that mutate module state.
    """
    global _default_registry
    if _default_registry is None:
        import fret.robots.variants  # noqa: F401  (registers built-in variants)

        _default_registry = RobotRegistry()
    return _default_registry


def reset_default_registry() -> None:
    """Drop the cached default registry so the next call rebuilds it."""
    global _default_registry
    _default_registry = None
