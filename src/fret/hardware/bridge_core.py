"""ROS-free SO-101 bridge core (C2).

State machine:
``DISCONNECTED → CONNECTED_DISABLED → ENABLED → FAULT``.
``FAULT → (clear) → CONNECTED_DISABLED``. Nothing reaches ``ENABLED``
except an explicit enable with a fresh lease.

``SafetySupervisor`` filters integrator output before SYNC WRITE.
Fault policy (D9): hold position; torque off only on shutdown or
explicit disable.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Callable, Protocol

import numpy as np
import numpy.typing as npt

from fret.control.velocity_to_position import VelocityToPositionIntegrator
from fret.hardware.config import BridgeConfig, JointCalibration
from fret.hardware.safety import SafetyConfig
from fret.hardware.safety import SafetySupervisor as ConcreteSafetySupervisor
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
    MAX_TEMPERATURE_LIMIT,
    MAX_TORQUE_LIMIT,
    MODEL_NUMBER,
    PRESENT_POSITION,
    PRESENT_VELOCITY,
    PROTECTION_CURRENT,
    TORQUE_ENABLE,
)
from fret.hardware.sts.transport import port_is_mock_safe

_LOG = logging.getLogger(__name__)

# Servo status bits (Feetech STS status register).
_STATUS_OVERHEAT = 0x04
_STATUS_OVERLOAD = 0x20
_STATUS_LOW_VOLTAGE = 0x01


class BridgeState(Enum):
    DISCONNECTED = auto()
    CONNECTED_DISABLED = auto()
    ENABLED = auto()
    FAULT = auto()


class SafetySupervisor(Protocol):
    """Filter position goals before SYNC WRITE (PR3 seam / PR4 concrete)."""

    @property
    def active(self) -> bool: ...

    def filter_goals(
        self,
        goals_rad: npt.NDArray[np.float64],
        measured_rad: npt.NDArray[np.float64],
        now_s: float,
    ) -> npt.NDArray[np.float64]: ...


class IdentitySafetySupervisor:
    """PR3 placeholder — passes goals through; not an active supervisor."""

    @property
    def active(self) -> bool:
        return False

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
    consecutive_failed_cycles: int = 0
    fault_reason: str = ""
    clamp_events: int = 0
    watchdog: bool = False
    lease_expired: bool = False
    stop_latched: bool = False


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
        self._clock = clock or time.monotonic
        if safety is None:
            self._safety: SafetySupervisor = ConcreteSafetySupervisor(
                config.joint_limits_rad(),
                config=config.safety,
                calibrated_limits_rad=config.joint_limits_rad(),
            )
        else:
            self._safety = safety
        self._state = BridgeState.DISCONNECTED
        self._diag = BridgeDiagnostics(velocity_source=config.velocity_source)
        self._positions = np.zeros(config.dof, dtype=np.float64)
        self._velocities = np.zeros(config.dof, dtype=np.float64)
        self._cal_by_id = {c.servo_id: c for c in config.calibration}
        self._integrator: VelocityToPositionIntegrator | None = None
        self._last_positions: npt.NDArray[np.float64] | None = None
        self._last_read_time: float | None = None
        self._transact = getattr(transport, "transact", None)
        self._hold_goals: npt.NDArray[np.float64] | None = None
        self._power_present = True
        self._shutdown_logged = False
        self._last_servo_status: dict[int, int] = {}

    @property
    def state(self) -> BridgeState:
        return self._state

    @property
    def diagnostics(self) -> BridgeDiagnostics:
        return self._diag

    @property
    def safety(self) -> SafetySupervisor:
        return self._safety

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
        """FR-HW-20 / FR-HW-43: validate servos and protection registers."""
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
            self._verify_protection_registers(sid)
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
        self._diag.consecutive_failed_cycles = 0
        self._diag.fault_reason = ""
        self._power_present = True

    def enable_torque(self, now_s: float | None = None) -> None:
        """Soft-start enable with a fresh lease (FR-HW-44, FR-HW-47)."""
        t = self._clock() if now_s is None else now_s
        if self._state == BridgeState.DISCONNECTED:
            raise StsError("cannot enable torque while DISCONNECTED")
        if self._state == BridgeState.FAULT:
            raise StsError(
                f"cannot enable while FAULT ({self._diag.fault_reason}); "
                "call clear() first"
            )
        if not self._supervisor_allows_enable():
            raise TorqueEnableRefusedError(
                "FR-SYS-19: torque enable refused on real serial port "
                f"{self.port_name!r} without an active SafetySupervisor. "
                "Inject fret.hardware.safety.SafetySupervisor."
            )
        concrete = self._concrete_safety()
        if concrete is not None:
            reason = concrete.check_enable_pose(self._positions)
            if reason is not None:
                raise StsError(f"enable refused: {reason}")
            goals = concrete.soft_start_goals(self._positions)
            self._sync_write_goals(goals)
            concrete.renew_lease(t)
            concrete.note_command(t)
            concrete.set_enabled(True)
            concrete.clear_stop()
            concrete.set_controller_fault(False)
        for sid in self._config.servo_ids:
            self._write_u8(sid, TORQUE_ENABLE[0], 1)
        if self._integrator is not None:
            self._integrator.start(self._positions)
        self._hold_goals = self._positions.copy()
        self._state = BridgeState.ENABLED
        self._diag.fault_reason = ""

    def disable_torque(self, *, shutdown: bool = False) -> None:
        """Torque off; arm goes limp (support by hand first)."""
        if self._state == BridgeState.DISCONNECTED:
            return
        try:
            for sid in self._config.servo_ids:
                self._write_u8(sid, TORQUE_ENABLE[0], 0)
        except StsError:
            pass
        concrete = self._concrete_safety()
        if concrete is not None:
            concrete.set_enabled(False)
        if shutdown and not self._shutdown_logged:
            _LOG.warning(
                "bridge shutdown: torque disabled; arm goes limp "
                "(support the arm by hand)"
            )
            self._shutdown_logged = True
        if self._state in {BridgeState.ENABLED, BridgeState.FAULT}:
            self._state = BridgeState.CONNECTED_DISABLED

    def clear(self) -> None:
        """Clear latched stop; FAULT → CONNECTED_DISABLED if cause is gone.

        Does not enable motion. Refused while bus power is still absent.
        """
        concrete = self._concrete_safety()
        if concrete is not None:
            concrete.clear_stop()
            self._diag.stop_latched = False
        if self._state != BridgeState.FAULT:
            return
        if not self._power_present:
            self._diag.fault_reason = "power_loss"
            return
        if concrete is not None:
            concrete.set_controller_fault(False)
            concrete.set_enabled(False)
        self._diag.consecutive_failed_cycles = 0
        self._diag.fault_reason = ""
        self._state = BridgeState.CONNECTED_DISABLED

    def stop(self) -> None:
        """Latched category-2 hold. Not an emergency stop (FR-HW-70)."""
        concrete = self._concrete_safety()
        if concrete is not None:
            concrete.latch_stop()
        self._hold_goals = self._positions.copy()
        self._diag.stop_latched = True

    def set_controller_fault(self, active: bool) -> None:
        concrete = self._concrete_safety()
        if concrete is not None:
            concrete.set_controller_fault(active)
        if active and self._state == BridgeState.ENABLED:
            self._enter_fault("controller_fault")

    def command_velocity(
        self, velocity: npt.NDArray[np.float64], now_s: float | None = None
    ) -> None:
        if self._integrator is None:
            self._diag.bad_commands += 1
            return
        if self._state in {BridgeState.FAULT, BridgeState.DISCONNECTED}:
            self._diag.bad_commands += 1
            return
        concrete = self._concrete_safety()
        if concrete is not None and concrete.stop_latched:
            self._diag.bad_commands += 1
            return
        if concrete is not None and concrete.controller_fault:
            self._diag.bad_commands += 1
            return
        v = np.asarray(velocity, dtype=np.float64)
        if v.shape != (self._config.dof,) or not np.all(np.isfinite(v)):
            self._diag.bad_commands += 1
            return
        t = self._clock() if now_s is None else now_s
        self._integrator.command_velocity(v, t)
        if concrete is not None:
            concrete.note_command(t)

    def renew_enable_lease(self, now_s: float | None = None) -> None:
        """Re-assert the enable lease (FR-HW-47)."""
        if self._state != BridgeState.ENABLED:
            return
        concrete = self._concrete_safety()
        if concrete is not None:
            concrete.renew_lease(self._clock() if now_s is None else now_s)

    def cycle(self, now_s: float | None = None) -> None:
        """One control cycle: SYNC READ, integrate, optional SYNC WRITE."""
        t0 = self._clock() if now_s is None else now_s
        if self._state == BridgeState.DISCONNECTED:
            return
        try:
            self._sync_read(t0)
            self._power_present = True
            self._observe_servo_faults()
            if self._state == BridgeState.FAULT:
                # Hold last pose; do not integrate (FR-HW-46).
                if self._hold_goals is not None:
                    self._sync_write_goals(self._hold_goals)
                self._diag.consecutive_failed_cycles = 0
                return
            if (
                self._state == BridgeState.ENABLED
                and self._integrator is not None
            ):
                goals = self._integrator.step(t0)
                filtered = self._safety.filter_goals(
                    goals, self._positions, t0
                )
                self._sync_write_goals(filtered)
                self._hold_goals = filtered.copy()
                self._copy_safety_diag()
            self._diag.consecutive_failed_cycles = 0
        except (StsChecksumError, StsIdMismatchError, StsLengthError):
            self._note_failed_cycle("integrity")
            raise
        except StsTimeoutError:
            self._note_failed_cycle("timeout")
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

    def note_power_loss(self) -> None:
        """External signal that bus power disappeared (mock power-cut)."""
        self._power_present = False
        self._note_failed_cycle("power_loss")

    def disconnect(self) -> None:
        self.disable_torque(shutdown=True)
        self._state = BridgeState.DISCONNECTED
        try:
            self._transport.close()
        except Exception:  # noqa: BLE001
            pass

    def _supervisor_allows_enable(self) -> bool:
        if bool(getattr(self._safety, "active", False)):
            return True
        if bool(getattr(self._transport, "is_mock_safe", False)):
            return True
        return port_is_mock_safe(self.port_name)

    def _concrete_safety(self) -> ConcreteSafetySupervisor | None:
        if isinstance(self._safety, ConcreteSafetySupervisor):
            return self._safety
        return None

    def _copy_safety_diag(self) -> None:
        concrete = self._concrete_safety()
        if concrete is None:
            return
        d = concrete.diagnostics
        self._diag.clamp_events = d.clamp_events
        self._diag.watchdog = d.watchdog
        self._diag.lease_expired = d.lease_expired
        self._diag.stop_latched = d.stop_latched

    def _enter_fault(self, reason: str) -> None:
        self._state = BridgeState.FAULT
        self._diag.fault_reason = reason
        self._hold_goals = self._positions.copy()
        concrete = self._concrete_safety()
        if concrete is not None:
            concrete.set_enabled(False)

    def _note_failed_cycle(self, reason: str) -> None:
        self._diag.consecutive_failed_cycles += 1
        limit = self._config.safety.fault_failed_cycles
        if reason in {"power_loss", "timeout", "low_voltage"}:
            if reason == "power_loss":
                self._power_present = False
            # Bus silence / brown-out: FAULT within N cycles (FR-HW-40).
            if (
                self._state
                in {
                    BridgeState.ENABLED,
                    BridgeState.CONNECTED_DISABLED,
                }
                and self._diag.consecutive_failed_cycles >= limit
            ):
                self._power_present = False
                self._enter_fault("power_loss")
            return
        if (
            self._state == BridgeState.ENABLED
            and self._diag.consecutive_failed_cycles >= limit
        ):
            self._enter_fault(reason)

    def _observe_servo_faults(self) -> None:
        for sid, count in list(self._diag.servo_error_counts.items()):
            if count <= 0:
                continue
            # Immediate FAULT on overload / overheat status bits last seen.
            # The status is carried on the last sync-read error field; we
            # also check sticky mock status via a best-effort read helper.
            status = self._last_servo_status.get(sid, 0)
            if status & _STATUS_OVERLOAD:
                self._enter_fault(f"overload:servo_{sid}")
                return
            if status & _STATUS_OVERHEAT:
                self._enter_fault(f"overheat:servo_{sid}")
                return
            if status & _STATUS_LOW_VOLTAGE:
                self._note_failed_cycle("low_voltage")

    def _verify_protection_registers(self, servo_id: int) -> None:
        """FR-HW-43: write and read back protection thresholds."""
        safety: SafetyConfig = self._config.safety
        self._write_u16(servo_id, MAX_TORQUE_LIMIT[0], safety.torque_limit)
        self._write_u16(
            servo_id, PROTECTION_CURRENT[0], safety.protection_current
        )
        self._write_u8(
            servo_id, MAX_TEMPERATURE_LIMIT[0], safety.max_temperature
        )
        got_torque = self._read_u16(servo_id, MAX_TORQUE_LIMIT[0])
        got_current = self._read_u16(servo_id, PROTECTION_CURRENT[0])
        got_temp = self._read_u8(servo_id, MAX_TEMPERATURE_LIMIT[0])
        if got_torque != safety.torque_limit:
            raise StsError(
                f"servo {servo_id}: torque_limit mismatch "
                f"{got_torque} != {safety.torque_limit}"
            )
        if got_current != safety.protection_current:
            raise StsError(
                f"servo {servo_id}: protection_current mismatch "
                f"{got_current} != {safety.protection_current}"
            )
        if got_temp != safety.max_temperature:
            raise StsError(
                f"servo {servo_id}: max_temperature mismatch "
                f"{got_temp} != {safety.max_temperature}"
            )

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
            self._last_servo_status[sid] = int(reply.error)
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

    def _read_u8(self, servo_id: int, address: int) -> int:
        request = proto.build_read(servo_id, address, 1)
        replies = self._do_transact(request, expected_id=servo_id)
        return int(replies[0].params[0])

    def _write_u8(self, servo_id: int, address: int, value: int) -> None:
        request = proto.build_write(servo_id, address, bytes((value & 0xFF,)))
        self._do_transact(request, expected_id=servo_id)

    def _write_u16(self, servo_id: int, address: int, value: int) -> None:
        request = proto.build_write(
            servo_id, address, proto.pack_u16_le(value)
        )
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
    safety: SafetySupervisor | None = None,
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
    return BridgeCore(transport, config, safety=safety), mock
