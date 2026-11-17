"""ROS-free SO-101 bridge core (C2).

State machine: DISCONNECTED → CONNECTED_DISABLED → ENABLED.
PR4 plugs a real ``SafetySupervisor`` between integrator output and SYNC
WRITE; PR3 leaves that seam as an identity pass-through.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Callable, Protocol

import numpy as np
import numpy.typing as npt

from fret.control.velocity_to_position import VelocityToPositionIntegrator
from fret.hardware.config import BridgeConfig, JointCalibration
from fret.hardware.sts import protocol as proto
from fret.hardware.sts.errors import (
    StsChecksumError,
    StsError,
    StsIdMismatchError,
    StsLengthError,
    StsPortError,
    StsTimeoutError,
    TorqueEnableRefusedError,
)
from fret.hardware.sts.registers import (
    GOAL_POSITION,
    MODEL_NUMBER,
    PRESENT_POSITION,
    PRESENT_VELOCITY,
    TORQUE_ENABLE,
)
from fret.hardware.sts.transport import port_is_mock_safe


class BridgeState(Enum):
    DISCONNECTED = auto()
    CONNECTED_DISABLED = auto()
    ENABLED = auto()


class SafetySupervisor(Protocol):
    """PR4 seam: filter position goals before SYNC WRITE."""

    def filter_goals(
        self,
        goals_rad: npt.NDArray[np.float64],
        measured_rad: npt.NDArray[np.float64],
        now_s: float,
    ) -> npt.NDArray[np.float64]: ...


class IdentitySafetySupervisor:
    """PR3 placeholder — passes goals through unchanged."""

    def filter_goals(
        self,
        goals_rad: npt.NDArray[np.float64],
        measured_rad: npt.NDArray[np.float64],  # noqa: ARG002
        now_s: float,  # noqa: ARG002
    ) -> npt.NDArray[np.float64]:
        return goals_rad


@dataclass
class BridgeDiagnostics:
    integrity_rejects: int = 0
    timeouts: int = 0
    bad_commands: int = 0
    cycle_overruns: int = 0
    consecutive_overruns: int = 0
    overrun_warnings: int = 0
    servo_error_counts: dict[int, int] = field(default_factory=dict)
    velocity_source: str = "present_speed"
    last_cycle_ms: float = 0.0


class BridgeCore:
    """Cycle the Feetech bus without ROS."""

    def __init__(
        self,
        transport: Any,
        config: BridgeConfig,
        *,
        safety: SafetySupervisor | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self._transport = transport
        self._config = config
        self._safety: SafetySupervisor = (
            safety if safety is not None else IdentitySafetySupervisor()
        )
        self._clock = clock or time.monotonic
        self._state = BridgeState.DISCONNECTED
        self._diag = BridgeDiagnostics(velocity_source=config.velocity_source)
        self._positions = np.zeros(config.dof, dtype=np.float64)
        self._velocities = np.zeros(config.dof, dtype=np.float64)
        self._cal_by_id = {c.servo_id: c for c in config.calibration}
        self._integrator: VelocityToPositionIntegrator | None = None
        self._last_positions: npt.NDArray[np.float64] | None = None
        self._last_read_time: float | None = None
        self._transact = getattr(transport, "transact", None)

    @property
    def state(self) -> BridgeState:
        return self._state

    @property
    def diagnostics(self) -> BridgeDiagnostics:
        return self._diag

    @property
    def positions(self) -> npt.NDArray[np.float64]:
        return self._positions.copy()

    @property
    def velocities(self) -> npt.NDArray[np.float64]:
        return self._velocities.copy()

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._config.joint_names

    @property
    def port_name(self) -> str:
        return str(getattr(self._transport, "port_name", ""))

    def connect(self) -> None:
        """FR-HW-20: read all servos; refuse CONNECTED if any check fails."""
        if self._state != BridgeState.DISCONNECTED:
            return
        positions: list[float] = []
        for sid in self._config.servo_ids:
            model = self._read_u16(sid, MODEL_NUMBER[0])
            if model != self._config.expected_model_number:
                raise StsError(
                    f"servo {sid}: unexpected model {model}, "
                    f"expected {self._config.expected_model_number}"
                )
            ticks = self._read_present_position(sid)
            rad = self._ticks_to_rad(sid, ticks)
            cal = self._cal_by_id.get(sid)
            if cal is not None:
                margin = self._config.connect_range_margin_rad
                lo = proto.ticks_to_radians(
                    cal.range_min, mid=cal.zero, sign=cal.drive_sign
                )
                hi = proto.ticks_to_radians(
                    cal.range_max, mid=cal.zero, sign=cal.drive_sign
                )
                lo, hi = min(lo, hi), max(lo, hi)
                if rad < lo - margin or rad > hi + margin:
                    raise StsError(
                        f"servo {sid}: position {rad:.4f} rad outside "
                        f"calibrated range [{lo:.4f}, {hi:.4f}] "
                        f"± {margin:.4f}"
                    )
            positions.append(rad)
        self._positions = np.asarray(positions, dtype=np.float64)
        self._velocities = np.zeros_like(self._positions)
        self._integrator = VelocityToPositionIntegrator(
            joint_limits=self._config.joint_limits_rad(),
            max_joint_velocity=self._config.max_joint_velocity,
            command_timeout_s=self._config.command_timeout_s,
            max_dt_s=self._config.max_dt_s,
        )
        self._integrator.start(self._positions)
        self._state = BridgeState.CONNECTED_DISABLED
        self._last_positions = self._positions.copy()
        self._last_read_time = self._clock()

    def enable_torque(self) -> None:
        """Enter ENABLED after FR-SYS-18 mock-only check."""
        if self._state == BridgeState.DISCONNECTED:
            raise StsError("cannot enable torque while DISCONNECTED")
        if not self._torque_allowed():
            raise TorqueEnableRefusedError(
                "FR-SYS-18: torque enable refused on real serial port "
                f"{self.port_name!r}; safety layer (PR4) is required. "
                "Use mock:, replay:, or a /dev/pts/ pty fake device."
            )
        for sid in self._config.servo_ids:
            self._write_u8(sid, TORQUE_ENABLE[0], 1)
        self._state = BridgeState.ENABLED

    def disable_torque(self) -> None:
        if self._state == BridgeState.DISCONNECTED:
            return
        try:
            for sid in self._config.servo_ids:
                self._write_u8(sid, TORQUE_ENABLE[0], 0)
        except StsError:
            pass
        if self._state == BridgeState.ENABLED:
            self._state = BridgeState.CONNECTED_DISABLED

    def command_velocity(
        self, velocity: npt.NDArray[np.float64], now_s: float | None = None
    ) -> None:
        if self._integrator is None:
            self._diag.bad_commands += 1
            return
        v = np.asarray(velocity, dtype=np.float64)
        if v.shape != (self._config.dof,) or not np.all(np.isfinite(v)):
            self._diag.bad_commands += 1
            return
        self._integrator.command_velocity(
            v, self._clock() if now_s is None else now_s
        )

    def cycle(self, now_s: float | None = None) -> None:
        """One control cycle: SYNC READ, integrate, optional SYNC WRITE."""
        t0 = self._clock() if now_s is None else now_s
        if self._state == BridgeState.DISCONNECTED:
            return
        try:
            self._sync_read(t0)
            if (
                self._state == BridgeState.ENABLED
                and self._integrator is not None
            ):
                goals = self._integrator.step(t0)
                filtered = self._safety.filter_goals(
                    goals, self._positions, t0
                )
                self._sync_write_goals(filtered)
        except (StsChecksumError, StsIdMismatchError, StsLengthError):
            # Counted in _do_transact.
            raise
        except StsTimeoutError:
            # Counted in _do_transact.
            raise
        except StsPortError:
            self._state = BridgeState.DISCONNECTED
            raise
        finally:
            elapsed_ms = (self._clock() - t0) * 1000.0
            self._diag.last_cycle_ms = elapsed_ms
            if elapsed_ms > self._config.cycle_budget_ms:
                self._diag.cycle_overruns += 1
                self._diag.consecutive_overruns += 1
                if (
                    self._diag.consecutive_overruns
                    >= self._config.overrun_warn_consecutive
                ):
                    self._diag.overrun_warnings += 1
            else:
                self._diag.consecutive_overruns = 0

    def disconnect(self) -> None:
        self.disable_torque()
        self._state = BridgeState.DISCONNECTED
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001
            pass

    def _torque_allowed(self) -> bool:
        if bool(getattr(self._transport, "is_mock_safe", False)):
            return True
        return port_is_mock_safe(self.port_name)

    def _cal(self, servo_id: int) -> JointCalibration:
        cal = self._cal_by_id.get(servo_id)
        if cal is not None:
            return cal
        name = (
            self._config.joint_names[self._config.servo_ids.index(servo_id)]
            if servo_id in self._config.servo_ids
            else f"servo_{servo_id}"
        )
        return JointCalibration(
            name=name,
            servo_id=servo_id,
            drive_sign=1.0,
            range_min=0,
            range_max=4095,
            zero=2048,
        )

    def _ticks_to_rad(self, servo_id: int, ticks: int) -> float:
        cal = self._cal(servo_id)
        return proto.ticks_to_radians(ticks, mid=cal.zero, sign=cal.drive_sign)

    def _rad_to_ticks(self, servo_id: int, radians: float) -> int:
        cal = self._cal(servo_id)
        return proto.radians_to_ticks(
            radians, mid=cal.zero, sign=cal.drive_sign
        )

    def _sync_read(self, now_s: float) -> None:
        # Present position (2) + present velocity (2) starting at 56.
        addr = PRESENT_POSITION[0]
        size = 4
        request = proto.build_sync_read(
            addr, size, list(self._config.servo_ids)
        )
        replies = self._do_transact(
            request,
            expected_id=None,
            expect_replies=len(self._config.servo_ids),
        )
        by_id = {r.servo_id: r for r in replies}
        positions = np.zeros(self._config.dof, dtype=np.float64)
        velocities = np.zeros(self._config.dof, dtype=np.float64)
        for i, sid in enumerate(self._config.servo_ids):
            reply = by_id.get(sid)
            if reply is None:
                self._diag.timeouts += 1
                raise StsTimeoutError(f"missing sync-read reply for id {sid}")
            if reply.error:
                self._diag.servo_error_counts[sid] = (
                    self._diag.servo_error_counts.get(sid, 0) + 1
                )
            if len(reply.params) < 4:
                raise StsLengthError(
                    f"servo {sid}: short present state ({len(reply.params)})"
                )
            ticks = proto.unpack_u16_le(reply.params[0:2])
            raw_vel = proto.unpack_u16_le(reply.params[2:4])
            positions[i] = self._ticks_to_rad(sid, ticks)
            if self._config.velocity_source == "present_speed":
                signed = proto.decode_sign_magnitude(raw_vel, 15)
                # Feetech present speed unit ≈ 0.0146 rpm * sign; map coarsely
                # to rad/s via ticks/s at 50 Hz scale for the mock.
                velocities[i] = (
                    float(self._cal(sid).drive_sign)
                    * signed
                    * (2.0 * math.pi)
                    / 4096.0
                )
                self._diag.velocity_source = "present_speed"
            else:
                velocities[i] = 0.0
        if self._config.velocity_source != "present_speed":
            if (
                self._last_positions is not None
                and self._last_read_time is not None
            ):
                dt = max(now_s - self._last_read_time, 1e-6)
                velocities = (positions - self._last_positions) / dt
            self._diag.velocity_source = "finite_difference"
        self._positions = positions
        self._velocities = velocities
        self._last_positions = positions.copy()
        self._last_read_time = now_s

    def _sync_write_goals(self, goals_rad: npt.NDArray[np.float64]) -> None:
        writes: list[tuple[int, bytes]] = []
        for i, sid in enumerate(self._config.servo_ids):
            ticks = self._rad_to_ticks(sid, float(goals_rad[i]))
            writes.append((sid, proto.pack_u16_le(ticks)))
        request = proto.build_sync_write(
            GOAL_POSITION[0], GOAL_POSITION[1], writes
        )
        self._transport.write(request)

    def _read_present_position(self, servo_id: int) -> int:
        request = proto.build_read(
            servo_id, PRESENT_POSITION[0], PRESENT_POSITION[1]
        )
        replies = self._do_transact(request, expected_id=servo_id)
        return proto.unpack_u16_le(replies[0].params)

    def _read_u16(self, servo_id: int, address: int) -> int:
        request = proto.build_read(servo_id, address, 2)
        replies = self._do_transact(request, expected_id=servo_id)
        return proto.unpack_u16_le(replies[0].params)

    def _write_u8(self, servo_id: int, address: int, value: int) -> None:
        request = proto.build_write(servo_id, address, bytes((value,)))
        self._do_transact(request, expected_id=servo_id)

    def _do_transact(
        self,
        request: bytes,
        *,
        expected_id: int | None,
        expect_replies: int = 1,
    ) -> list[Any]:
        timeout = self._config.reply_timeout_s
        if self._transact is not None:
            try:
                result = self._transact(
                    request,
                    expected_id=expected_id,
                    timeout_s=timeout,
                    expect_replies=expect_replies,
                )
                return list(result)
            except (StsChecksumError, StsIdMismatchError, StsLengthError):
                self._diag.integrity_rejects += 1
                raise
            except StsTimeoutError:
                self._diag.timeouts += 1
                raise
        # Fallback for transports without transact().
        self._transport.write(request)
        replies: list[Any] = []
        buf = bytearray()
        deadline = time.monotonic() + timeout * max(1, expect_replies)
        while len(replies) < expect_replies:
            if time.monotonic() > deadline:
                self._diag.timeouts += 1
                raise StsTimeoutError("reply timeout")
            chunk = self._transport.read(64, 0.01)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                try:
                    exp = expected_id if expect_replies == 1 else None
                    reply, consumed = proto.parse_reply(
                        bytes(buf), expected_id=exp
                    )
                except StsLengthError:
                    break
                except (StsChecksumError, StsIdMismatchError):
                    self._diag.integrity_rejects += 1
                    raise
                else:
                    replies.append(reply)
                    del buf[:consumed]
                    if len(replies) >= expect_replies:
                        break
        return replies


def make_bridge_core_for_mock(
    config: BridgeConfig | None = None,
    *,
    bus: Any | None = None,
) -> tuple[BridgeCore, Any]:
    """Factory used by unit tests: in-process mock + BridgeCore."""
    from fret.hardware.config import (
        default_bridge_config_path,
        default_example_calibration_path,
        load_bridge_config,
    )
    from fret.hardware.sts.mock_bus import make_so101_mock
    from fret.hardware.sts.transport import InMemoryTransport

    if config is None:
        config = load_bridge_config(
            default_bridge_config_path(),
            calibration_path=default_example_calibration_path(),
        )
    mock = bus if bus is not None else make_so101_mock()
    transport = InMemoryTransport(mock)

    # Attach a simple transact helper.
    def _transact(
        request: bytes,
        *,
        expected_id: int | None,
        timeout_s: float | None = None,  # noqa: ARG001
        expect_replies: int = 1,
    ) -> list[Any]:
        reply_bytes = mock.handle_request(request)
        if not reply_bytes and expect_replies > 0:
            raise StsTimeoutError("mock returned no reply")
        replies: list[Any] = []
        buf = bytearray(reply_bytes)
        while len(replies) < expect_replies and buf:
            exp = expected_id if expect_replies == 1 else None
            try:
                reply, consumed = proto.parse_reply(
                    bytes(buf), expected_id=exp
                )
            except (StsChecksumError, StsIdMismatchError, StsLengthError):
                raise
            replies.append(reply)
            del buf[:consumed]
        if len(replies) < expect_replies:
            raise StsTimeoutError(
                f"mock replies {len(replies)}/{expect_replies}"
            )
        return replies

    transport.transact = _transact  # type: ignore[attr-defined]
    return BridgeCore(transport, config), mock
