"""Unit tests for MuJoCo physics contact logging (T12-05)."""

from __future__ import annotations

import json
import pathlib

import numpy as np
import pytest

from fret.ros.mujoco_bridge import (
    _load_merged_bridge_config,
    _resolve_config_path,
    contact_log_config_from_bridge_yaml,
    make_dubins_race_bridge_core,
    physics_config_from_bridge_yaml,
)
from fret.ros.mujoco_physics_log import (
    ContactLogConfig,
    PhysicsContactLogger,
    _counts_as_penetration,
    agent_obstacle_contact_forces,
)


def test_contact_log_config_from_yaml() -> None:
    """Merged mujoco config should resolve default artifact paths."""
    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    log_cfg = contact_log_config_from_bridge_yaml(
        cfg,
        "dubins_race",
        physics_mode=True,
        enabled=True,
    )
    assert log_cfg.enabled is True
    assert log_cfg.log_path.name == "contacts.jsonl"
    assert log_cfg.metrics_path.name == "metrics.json"


@pytest.mark.skipif(
    not make_dubins_race_bridge_core().has_mujoco_runtime,
    reason="mujoco package not installed",
)
def test_physics_contact_logger_writes_jsonl(tmp_path: pathlib.Path) -> None:
    """Physics ticks with contacts should append JSONL and metrics."""
    cfg = _load_merged_bridge_config(_resolve_config_path(None))
    cfg = dict(cfg)
    cfg["physics_mode"] = True
    physics = physics_config_from_bridge_yaml(cfg, "dubins", physics_mode=True)
    core = make_dubins_race_bridge_core(
        initial_rrt=np.array([6.0, 6.0, 0.0]),
        initial_sst=np.array([6.0, 6.4, 0.0]),
        physics_config=physics,
    )
    log_path = tmp_path / "contacts.jsonl"
    metrics_path = tmp_path / "metrics.json"
    core.configure_contact_logging(
        ContactLogConfig(
            enabled=True,
            log_path=log_path,
            metrics_path=metrics_path,
            scenario_id="dubins_race",
            physics_mode=True,
        )
    )
    core.step_physics(np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    written = core.finalize_physics_metrics(max_tracking_error_m=0.005)
    assert written == metrics_path
    assert metrics_path.is_file()
    payload = json.loads(metrics_path.read_text(encoding="utf-8"))
    assert payload["scenario_id"] == "dubins_race"
    assert payload["physics_mode"] is True
    assert payload["max_tracking_error_m"] == pytest.approx(0.005)


def test_penetration_violation_requires_consecutive_ticks() -> None:
    """Single-tick interpenetration should not increment violation count."""
    logger = PhysicsContactLogger(
        ContactLogConfig(
            enabled=False,
            log_path=pathlib.Path("/tmp/unused.jsonl"),
            metrics_path=pathlib.Path("/tmp/unused_metrics.json"),
            scenario_id="dubins_race",
            physics_mode=True,
        )
    )

    class _Contact:
        def __init__(self, dist: float, geom1: int, geom2: int) -> None:
            self.dist = dist
            self.geom1 = geom1
            self.geom2 = geom2
            self.pos = np.zeros(3, dtype=np.float64)

    class _Data:
        def __init__(self, contacts: list[_Contact]) -> None:
            self.time = 0.0
            self.contact = contacts
            self.ncon = len(contacts)

    class _Model:
        pass

    class _Mujoco:
        mjtObj = type("mjtObj", (), {"mjOBJ_GEOM": 5})()

        @staticmethod
        def mj_id2name(_model: object, _obj: int, geom_id: int) -> str:
            return "rrt_collision" if geom_id == 0 else "str_000_col"

        @staticmethod
        def mj_contactForce(
            _model: object, _data: object, _idx: int, _force: np.ndarray
        ) -> None:
            return

    tick = _Data([_Contact(-0.002, 0, 1)])
    logger.record_tick(_Model(), tick, _Mujoco())
    assert logger.metrics.penetration_violations == 0

    tick.time = 0.05
    logger.record_tick(_Model(), tick, _Mujoco())
    assert logger.metrics.penetration_violations == 1


def test_counts_as_penetration_ignores_floor_contact() -> None:
    """Floor contacts must not count as obstacle penetration."""
    assert _counts_as_penetration("rrt_collision", "floor", -0.01) is False
    assert (
        _counts_as_penetration("rrt_collision", "str_000_col", -0.002) is True
    )


class _FakeContact:
    def __init__(self, geom1: int, geom2: int) -> None:
        self.geom1 = geom1
        self.geom2 = geom2
        self.dist = -0.001
        self.pos = np.zeros(3, dtype=np.float64)


class _FakeData:
    def __init__(self, contacts: list[_FakeContact]) -> None:
        self.time = 0.0
        self.contact = contacts
        self.ncon = len(contacts)


class _FakeModel:
    pass


def _make_fake_mujoco(force_norm: float) -> object:
    class _FakeMujoco:
        mjtObj = type("mjtObj", (), {"mjOBJ_GEOM": 5})()
        _names = {0: "rrt_collision", 1: "str_000_col", 2: "floor"}

        @classmethod
        def mj_id2name(cls, _model: object, _obj: int, geom_id: int) -> str:
            return cls._names[geom_id]

        @staticmethod
        def mj_contactForce(
            _model: object, _data: object, _idx: int, force: np.ndarray
        ) -> None:
            force[0] = force_norm

    return _FakeMujoco()


def test_agent_obstacle_contact_forces_detects_agent_obstacle_contact() -> (
    None
):
    """Real collision monitor primary signal: agent-vs-obstacle force."""
    data = _FakeData([_FakeContact(geom1=0, geom2=1)])
    forces = agent_obstacle_contact_forces(
        _FakeModel(), data, _make_fake_mujoco(2.5), ("rrt_collision",)
    )
    assert forces["rrt_collision"] == pytest.approx(2.5)


def test_agent_obstacle_contact_forces_ignores_floor_contact() -> None:
    """Agent-vs-floor contact must not register as an obstacle collision."""
    data = _FakeData([_FakeContact(geom1=0, geom2=2)])
    forces = agent_obstacle_contact_forces(
        _FakeModel(), data, _make_fake_mujoco(9.0), ("rrt_collision",)
    )
    assert forces["rrt_collision"] == 0.0


def test_agent_obstacle_contact_forces_no_contacts_returns_zero() -> None:
    """With ``ncon == 0`` every tracked agent geom reports zero force."""
    forces = agent_obstacle_contact_forces(
        _FakeModel(),
        _FakeData([]),
        _make_fake_mujoco(0.0),
        ("rrt_collision", "sst_collision"),
    )
    assert forces == {"rrt_collision": 0.0, "sst_collision": 0.0}
