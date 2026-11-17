"""FR-SYS-19: enable on a non-mock port only with an active supervisor.

FR-SYS-18 (v2.1.0 mock-only interlock) is removed in the same commit that
lands the safety suite green.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from fret.hardware.bridge_core import (
    BridgeCore,
    BridgeState,
    IdentitySafetySupervisor,
)
from fret.hardware.config import (
    default_bridge_config_path,
    default_example_calibration_path,
    load_bridge_config,
)
from fret.hardware.safety import SafetyConfig, SafetySupervisor
from fret.hardware.sts.errors import TorqueEnableRefusedError
from fret.hardware.sts.transport import port_is_mock_safe


def test_mock_port_is_safe() -> None:
    assert port_is_mock_safe("mock:so101")
    assert port_is_mock_safe("replay:/tmp/t.jsonl")
    assert port_is_mock_safe("/dev/pts/3")
    assert not port_is_mock_safe("/dev/ttyUSB0")
    assert not port_is_mock_safe("/dev/ttyACM0")


def test_enable_refused_on_real_port_without_active_supervisor_fr_sys_19() -> (
    None
):
    cfg = load_bridge_config(
        default_bridge_config_path(),
        calibration_path=default_example_calibration_path(),
    )
    transport = MagicMock()
    transport.port_name = "/dev/ttyUSB0"
    transport.is_mock_safe = False

    core = BridgeCore(transport, cfg, safety=IdentitySafetySupervisor())
    core._state = BridgeState.CONNECTED_DISABLED
    with pytest.raises(TorqueEnableRefusedError, match="FR-SYS-19"):
        core.enable_torque()
    assert core.state is BridgeState.CONNECTED_DISABLED


def test_enable_allowed_on_real_port_with_active_supervisor_fr_sys_19() -> (
    None
):
    cfg = load_bridge_config(
        default_bridge_config_path(),
        calibration_path=default_example_calibration_path(),
    )
    transport = MagicMock()
    transport.port_name = "/dev/ttyUSB0"
    transport.is_mock_safe = False
    written: list[bytes] = []

    def _write(data: bytes) -> None:
        written.append(data)

    def _transact(
        request: bytes,
        *,
        expected_id: int | None = None,
        timeout_s: float | None = None,
        expect_replies: int = 1,
    ) -> list[object]:
        # Minimal status replies so enable torque writes succeed.
        from fret.hardware.sts import protocol as proto

        class _R:
            def __init__(self, sid: int) -> None:
                self.servo_id = sid
                self.error = 0
                self.params = b""

        if expect_replies <= 0:
            return []
        sid = expected_id if expected_id is not None else 1
        return [_R(sid) for _ in range(expect_replies)]

    transport.write = _write
    transport.transact = _transact

    safety = SafetySupervisor(
        cfg.joint_limits_rad(),
        config=SafetyConfig(limit_margin_rad=0.0),
    )
    core = BridgeCore(transport, cfg, safety=safety)
    core._state = BridgeState.CONNECTED_DISABLED
    core._positions = np.zeros(cfg.dof, dtype=np.float64)
    core._integrator = None
    # Soft-start needs integrator optional; enable creates writes.
    from fret.control.velocity_to_position import VelocityToPositionIntegrator

    core._integrator = VelocityToPositionIntegrator(
        joint_limits=cfg.joint_limits_rad(),
        max_joint_velocity=cfg.max_joint_velocity,
        command_timeout_s=cfg.command_timeout_s,
        max_dt_s=cfg.max_dt_s,
    )
    core._integrator.start(core._positions)
    core.enable_torque(0.0)
    assert core.state is BridgeState.ENABLED
    assert safety.active is True


def test_enable_torque_allowed_on_mock() -> None:
    from fret.hardware.bridge_core import make_bridge_core_for_mock

    core, _ = make_bridge_core_for_mock()
    core.connect()
    core.enable_torque()
    assert core.state is BridgeState.ENABLED
