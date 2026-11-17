"""In-process STS3215 bus mock with fault injection (FR-HW-71)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable

from fret.hardware.sts import protocol as proto
from fret.hardware.sts.registers import (
    GOAL_POSITION,
)
from fret.hardware.sts.registers import ID as REG_ID
from fret.hardware.sts.registers import (
    MODEL_NUMBER,
    PRESENT_LOAD,
    PRESENT_POSITION,
    PRESENT_TEMPERATURE,
    PRESENT_VELOCITY,
    PRESENT_VOLTAGE,
    STATUS,
    STS3215_MID_TICKS,
    STS3215_MODEL_NUMBER,
    STS3215_RESOLUTION,
    TORQUE_ENABLE,
)


class FaultKind(Enum):
    """Faults the mock can inject (each must have a bridge reaction test)."""

    LATENCY = auto()
    DROP_REPLY = auto()
    CORRUPT_CHECKSUM = auto()
    WRONG_ID = auto()
    MISSING_SERVO = auto()
    LOW_VOLTAGE = auto()
    OVERLOAD = auto()
    OVERHEAT = auto()
    CABLE_PULL = auto()


@dataclass
class FaultConfig:
    kind: FaultKind
    servo_id: int | None = None
    latency_s: float = 0.0
    remaining: int = 1  # how many matching replies to affect


@dataclass
class MockServo:
    servo_id: int
    model_number: int = STS3215_MODEL_NUMBER
    present_position: int = STS3215_MID_TICKS
    goal_position: int = STS3215_MID_TICKS
    present_velocity: int = 0
    torque_enable: int = 0
    voltage: int = 120  # 0.1 V units → 12.0 V
    temperature: int = 35
    load: int = 0
    status: int = 0
    registers: dict[int, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.registers.setdefault(REG_ID[0], self.servo_id)
        self.registers.setdefault(MODEL_NUMBER[0], self.model_number & 0xFF)
        self.registers.setdefault(
            MODEL_NUMBER[0] + 1, (self.model_number >> 8) & 0xFF
        )
        self.registers.setdefault(TORQUE_ENABLE[0], self.torque_enable)
        self._write_u16(PRESENT_POSITION[0], self.present_position)
        self._write_u16(GOAL_POSITION[0], self.goal_position)
        self._write_u16(PRESENT_VELOCITY[0], 0)
        self.registers.setdefault(PRESENT_VOLTAGE[0], self.voltage)
        self.registers.setdefault(PRESENT_TEMPERATURE[0], self.temperature)
        self._write_u16(PRESENT_LOAD[0], self.load)
        self.registers.setdefault(STATUS[0], self.status)

    def _write_u16(self, addr: int, value: int) -> None:
        self.registers[addr] = value & 0xFF
        self.registers[addr + 1] = (value >> 8) & 0xFF

    def _read_u16(self, addr: int) -> int:
        return self.registers.get(addr, 0) | (
            self.registers.get(addr + 1, 0) << 8
        )

    def read_bytes(self, address: int, size: int) -> bytes:
        self._sync_regs_out()
        return bytes(self.registers.get(address + i, 0) for i in range(size))

    def write_bytes(self, address: int, data: bytes) -> None:
        for i, byte in enumerate(data):
            self.registers[address + i] = byte
        if address == TORQUE_ENABLE[0] and data:
            self.torque_enable = data[0]
        if address == GOAL_POSITION[0] and len(data) >= 2:
            self.goal_position = data[0] | (data[1] << 8)
        if address == REG_ID[0] and data:
            self.servo_id = data[0]
            self.registers[REG_ID[0]] = data[0]
        self._sync_regs_out()

    def _sync_regs_out(self) -> None:
        self._write_u16(PRESENT_POSITION[0], self.present_position)
        self._write_u16(GOAL_POSITION[0], self.goal_position)
        encoded_vel = proto.encode_sign_magnitude(self.present_velocity, 15)
        self._write_u16(PRESENT_VELOCITY[0], encoded_vel)
        self.registers[PRESENT_VOLTAGE[0]] = self.voltage
        self.registers[PRESENT_TEMPERATURE[0]] = self.temperature
        self._write_u16(
            PRESENT_LOAD[0],
            proto.encode_sign_magnitude(self.load, 10) & 0xFFFF,
        )
        self.registers[STATUS[0]] = self.status
        self.registers[TORQUE_ENABLE[0]] = self.torque_enable


class MockStsBus:
    """Register-level model of N STS3215 servos with first-order motion."""

    def __init__(
        self,
        servo_ids: list[int] | None = None,
        *,
        max_ticks_per_s: float = 2048.0,
        clock: Callable[[], float] | None = None,
        reply_latency_s: float = 0.0,
    ) -> None:
        ids = servo_ids or [1, 2, 3, 4, 5, 6]
        self._servos: dict[int, MockServo] = {
            i: MockServo(servo_id=i) for i in ids
        }
        self._max_ticks_per_s = max_ticks_per_s
        self._clock = clock or time.monotonic
        self._last_tick = self._clock()
        self._reply_latency_s = reply_latency_s
        self._faults: list[FaultConfig] = []
        self._cable_pulled = False
        self._listeners: list[Callable[[bytes, bytes], None]] = []

    @property
    def cable_pulled(self) -> bool:
        return self._cable_pulled

    def add_listener(self, cb: Callable[[bytes, bytes], None]) -> None:
        self._listeners.append(cb)

    def inject_fault(self, fault: FaultConfig) -> None:
        if fault.kind == FaultKind.CABLE_PULL:
            self._cable_pulled = True
        if (
            fault.kind == FaultKind.MISSING_SERVO
            and fault.servo_id is not None
        ):
            self._servos.pop(fault.servo_id, None)
        if fault.kind == FaultKind.LOW_VOLTAGE and fault.servo_id is not None:
            servo = self._servos.get(fault.servo_id)
            if servo is not None:
                servo.voltage = 50  # 5.0 V
                servo.status |= 0x01
        if fault.kind == FaultKind.OVERLOAD and fault.servo_id is not None:
            servo = self._servos.get(fault.servo_id)
            if servo is not None:
                servo.status |= 0x20
                servo.load = 900
        if fault.kind == FaultKind.OVERHEAT and fault.servo_id is not None:
            servo = self._servos.get(fault.servo_id)
            if servo is not None:
                servo.temperature = 85
                servo.status |= 0x04
        self._faults.append(fault)

    def clear_faults(self) -> None:
        self._faults.clear()
        self._cable_pulled = False

    def servo(self, servo_id: int) -> MockServo:
        return self._servos[servo_id]

    def tick(self, now: float | None = None) -> None:
        """Advance first-order motion toward each enabled goal."""
        t = self._clock() if now is None else now
        dt = max(0.0, t - self._last_tick)
        self._last_tick = t
        max_delta = self._max_ticks_per_s * dt
        for servo in self._servos.values():
            if not servo.torque_enable:
                servo.present_velocity = 0
                continue
            delta = servo.goal_position - servo.present_position
            step = max(-max_delta, min(max_delta, float(delta)))
            servo.present_position = int(round(servo.present_position + step))
            servo.present_position = max(
                0, min(STS3215_RESOLUTION - 1, servo.present_position)
            )
            servo.present_velocity = int(round(step / dt)) if dt > 0 else 0
            servo._sync_regs_out()

    def handle_request(self, request: bytes) -> bytes:
        """Parse one request packet and return concatenated replies."""
        if self._cable_pulled:
            return b""
        self.tick()
        if len(request) < 6 or request[0] != 0xFF or request[1] != 0xFF:
            return b""
        servo_id = request[2]
        length = request[3]
        total = 4 + length
        if len(request) < total:
            return b""
        instruction = request[4]
        params = request[5 : total - 1]
        replies = self._dispatch(servo_id, instruction, params)
        for req_listener in self._listeners:
            req_listener(request[:total], replies)
        if self._reply_latency_s > 0:
            time.sleep(self._reply_latency_s)
        return replies

    def _dispatch(
        self, servo_id: int, instruction: int, params: bytes
    ) -> bytes:
        if instruction == proto.INST_PING:
            if servo_id not in self._servos and servo_id != proto.BROADCAST_ID:
                return b""
            if servo_id == proto.BROADCAST_ID:
                out = bytearray()
                for sid in self._servos:
                    out.extend(self._reply_for(sid, b""))
                return bytes(out)
            return self._reply_for(servo_id, b"")
        if instruction == proto.INST_READ:
            if len(params) < 2:
                return b""
            addr, size = params[0], params[1]
            servo = self._servos.get(servo_id)
            if servo is None:
                return b""
            return self._reply_for(servo_id, servo.read_bytes(addr, size))
        if instruction == proto.INST_WRITE:
            if len(params) < 1:
                return b""
            addr, data = params[0], params[1:]
            servo = self._servos.get(servo_id)
            if servo is None:
                return b""
            servo.write_bytes(addr, data)
            # Re-key the map when the servo ID register changes (set-id).
            if addr == REG_ID[0] and data:
                new_id = int(data[0])
                if new_id != servo_id:
                    self._servos.pop(servo_id, None)
                    self._servos[new_id] = servo
                    return self._reply_for(new_id, b"")
            return self._reply_for(servo_id, b"")
        if instruction == proto.INST_SYNC_READ:
            if len(params) < 2:
                return b""
            addr, size = params[0], params[1]
            ids = list(params[2:])
            out = bytearray()
            for sid in ids:
                servo = self._servos.get(sid)
                if servo is None:
                    continue
                out.extend(self._reply_for(sid, servo.read_bytes(addr, size)))
            return bytes(out)
        if instruction == proto.INST_SYNC_WRITE:
            if len(params) < 2:
                return b""
            addr, size = params[0], params[1]
            payload = params[2:]
            i = 0
            while i + 1 + size <= len(payload):
                sid = payload[i]
                data = payload[i + 1 : i + 1 + size]
                servo = self._servos.get(sid)
                if servo is not None:
                    servo.write_bytes(addr, data)
                i += 1 + size
            return b""  # broadcast, no reply
        return b""

    def _reply_for(self, servo_id: int, params: bytes) -> bytes:
        fault = self._consume_fault(servo_id)
        if fault is not None:
            if fault.kind == FaultKind.DROP_REPLY:
                return b""
            if fault.kind == FaultKind.LATENCY and fault.latency_s > 0:
                time.sleep(fault.latency_s)
            if fault.kind == FaultKind.WRONG_ID:
                servo_id = (servo_id % 252) + 1
            if fault.kind == FaultKind.CORRUPT_CHECKSUM:
                packet = self._make_status(servo_id, params)
                mutable = bytearray(packet)
                mutable[-1] ^= 0xFF
                return bytes(mutable)
        servo = self._servos.get(servo_id)
        error = 0 if servo is None else servo.status
        return self._make_status(servo_id, params, error=error)

    def _make_status(
        self, servo_id: int, params: bytes, *, error: int = 0
    ) -> bytes:
        length = 1 + len(params) + 1
        body = bytes((servo_id, length, error)) + params
        return proto.HEADER + body + bytes((proto.checksum(body),))

    def _consume_fault(self, servo_id: int) -> FaultConfig | None:
        for i, fault in enumerate(self._faults):
            if fault.kind in {
                FaultKind.MISSING_SERVO,
                FaultKind.CABLE_PULL,
                FaultKind.LOW_VOLTAGE,
                FaultKind.OVERLOAD,
                FaultKind.OVERHEAT,
            }:
                # sticky / already applied
                continue
            if fault.servo_id is not None and fault.servo_id != servo_id:
                continue
            if fault.remaining <= 0:
                continue
            fault.remaining -= 1
            if fault.remaining <= 0:
                self._faults.pop(i)
            return fault
        return None


def make_so101_mock(*, mid: int = STS3215_MID_TICKS) -> MockStsBus:
    """Six-servo SO-101 follower mock centered at mid-range."""
    bus = MockStsBus([1, 2, 3, 4, 5, 6])
    for servo in bus._servos.values():
        servo.present_position = mid
        servo.goal_position = mid
        servo._sync_regs_out()
    return bus
