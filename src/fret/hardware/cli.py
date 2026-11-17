"""``fret-hw`` servo tooling CLI (FR-HW-26, FR-HW-27, FR-HW-68)."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Sequence

import yaml

from fret.hardware.config import JointCalibration
from fret.hardware.sts import protocol as proto
from fret.hardware.sts.errors import StsError, StsTimeoutError
from fret.hardware.sts.fake_device import FakeStsDevice
from fret.hardware.sts.mock_bus import MockStsBus, make_so101_mock
from fret.hardware.sts.registers import (
    BAUD_RATE,
    BAUD_RATE_TABLE,
)
from fret.hardware.sts.registers import ID as REG_ID
from fret.hardware.sts.registers import (
    PRESENT_POSITION,
    TORQUE_ENABLE,
)
from fret.hardware.sts.replay import (
    RecordingTransport,
    ReplayTransport,
    TraceRecorder,
)
from fret.hardware.sts.transport import InMemoryTransport, SerialTransport


def _open_bus(port: str, baud: int) -> _Client:
    if port.startswith("mock:"):
        bus = make_so101_mock()
        return _Client(InMemoryTransport(bus, port_name=port), bus=bus)
    return _Client(SerialTransport(port, baud))


class _Client:
    def __init__(self, transport: Any, bus: MockStsBus | None = None) -> None:
        self.transport = transport
        self.bus = bus

    def close(self) -> None:
        self.transport.close()

    def ping(self, servo_id: int) -> bool:
        request = proto.build_ping(servo_id)
        try:
            replies = self._transact(request, expected_id=servo_id)
            return bool(replies) and replies[0].ok
        except StsError:
            return False

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        request = proto.build_read(servo_id, address, size)
        replies = self._transact(request, expected_id=servo_id)
        return bytes(replies[0].params)

    def write(self, servo_id: int, address: int, data: bytes) -> None:
        request = proto.build_write(servo_id, address, data)
        # ID changes reply under the new id; do not require expected_id.
        expected = None if address == REG_ID[0] else servo_id
        self._transact(request, expected_id=expected)

    def scan(self, ids: range | list[int]) -> list[int]:
        found: list[int] = []
        for sid in ids:
            if self.ping(sid):
                found.append(sid)
        return found

    def _transact(
        self,
        request: bytes,
        *,
        expected_id: int | None,
        expect_replies: int = 1,
    ) -> list[Any]:
        if self.bus is not None:
            raw = self.bus.handle_request(request)
            if not raw:
                raise StsTimeoutError("no reply")
            replies: list[Any] = []
            buf = bytearray(raw)
            while len(replies) < expect_replies and buf:
                reply, n = proto.parse_reply(
                    bytes(buf), expected_id=expected_id
                )
                replies.append(reply)
                del buf[:n]
            return replies
        if hasattr(self.transport, "transact"):
            result = self.transport.transact(
                request,
                expected_id=expected_id,
                expect_replies=expect_replies,
            )
            return list(result)
        self.transport.write(request)
        raise StsTimeoutError("transport has no transact")


def cmd_scan(args: argparse.Namespace) -> int:
    client = _open_bus(args.port, args.baud)
    try:
        found = client.scan(range(args.start, args.end + 1))
        print(",".join(str(i) for i in found) if found else "(none)")
        return 0
    finally:
        client.close()


def cmd_ping(args: argparse.Namespace) -> int:
    client = _open_bus(args.port, args.baud)
    try:
        ok = client.ping(args.id)
        print("ok" if ok else "fail")
        return 0 if ok else 1
    finally:
        client.close()


def cmd_read(args: argparse.Namespace) -> int:
    client = _open_bus(args.port, args.baud)
    try:
        data = client.read(args.id, args.address, args.size)
        if args.size == 2:
            print(proto.unpack_u16_le(data))
        else:
            print(data.hex())
        return 0
    finally:
        client.close()


def cmd_watch(args: argparse.Namespace) -> int:
    client = _open_bus(args.port, args.baud)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            data = client.read(args.id, PRESENT_POSITION[0], 2)
            print(proto.unpack_u16_le(data))
            time.sleep(args.period)
        return 0
    finally:
        client.close()


def cmd_set_id(args: argparse.Namespace) -> int:
    """FR-HW-27: refuse when more than one servo answers; PING-verify."""
    client = _open_bus(args.port, args.baud)
    try:
        present = client.scan(range(0, 253))
        if len(present) != 1:
            print(
                f"error: set-id requires exactly one servo on the bus, "
                f"found {len(present)}: {present}",
                file=sys.stderr,
            )
            return 2
        old_id = present[0]
        client.write(old_id, REG_ID[0], bytes((args.new_id,)))
        if not client.ping(args.new_id):
            print(
                f"error: PING of new id {args.new_id} failed",
                file=sys.stderr,
            )
            return 1
        print(f"{old_id} -> {args.new_id}")
        return 0
    finally:
        client.close()


def cmd_set_baud(args: argparse.Namespace) -> int:
    if args.baud_value not in BAUD_RATE_TABLE:
        print(
            f"error: unsupported baud {args.baud_value}; "
            f"known: {sorted(BAUD_RATE_TABLE)}",
            file=sys.stderr,
        )
        return 2
    client = _open_bus(args.port, args.baud)
    try:
        code = BAUD_RATE_TABLE[args.baud_value]
        client.write(args.id, BAUD_RATE[0], bytes((code,)))
        print(f"id {args.id} baud -> {args.baud_value} (code {code})")
        return 0
    finally:
        client.close()


def cmd_torque_off(args: argparse.Namespace) -> int:
    client = _open_bus(args.port, args.baud)
    try:
        ids = args.ids or client.scan(range(1, 7))
        for sid in ids:
            client.write(sid, TORQUE_ENABLE[0], b"\x00")
            print(f"id {sid} torque off")
        return 0
    finally:
        client.close()


def cmd_calibrate(args: argparse.Namespace) -> int:
    """FR-HW-26: record per-joint range with torque off; write YAML."""
    client = _open_bus(args.port, args.baud)
    try:
        joints: list[dict[str, Any]] = []
        for name, sid in zip(args.names, args.ids, strict=True):
            client.write(sid, TORQUE_ENABLE[0], b"\x00")
            samples: list[int] = []
            print(f"Move joint {name} (id {sid}); sampling {args.samples}…")
            for _ in range(args.samples):
                data = client.read(sid, PRESENT_POSITION[0], 2)
                samples.append(proto.unpack_u16_le(data))
                time.sleep(args.period)
            lo, hi = min(samples), max(samples)
            zero = (lo + hi) // 2
            joints.append(
                {
                    "name": name,
                    "id": sid,
                    "drive_sign": 1,
                    "range_min": lo,
                    "range_max": hi,
                    "zero": zero,
                }
            )
        out = {"robot": "so101", "joints": joints}
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(out, sort_keys=False), encoding="utf-8")
        print(f"wrote {path}")
        return 0
    finally:
        client.close()


def cmd_import_lerobot_calibration(args: argparse.Namespace) -> int:
    """FR-HW-26: convert LeRobot calibration JSON to FRET YAML."""
    raw = json.loads(Path(args.input).read_text(encoding="utf-8"))
    joints: list[dict[str, Any]] = []
    # LeRobot format variants: dict of joint -> {id, range_min, range_max, …}
    items = raw.items() if isinstance(raw, dict) else []
    if "motors" in raw:
        items = raw["motors"].items()
    for name, entry in items:
        if not isinstance(entry, dict):
            continue
        lo = int(entry.get("range_min", entry.get("homing_offset", 0)) or 0)
        hi = int(entry.get("range_max", 4095) or 4095)
        zero = int(
            entry.get("homing_offset", (lo + hi) // 2) or ((lo + hi) // 2)
        )
        if "range_min" not in entry and "homing_offset" in entry:
            # homing_offset alone is not a range; keep full travel defaults.
            lo, hi, zero = 0, 4095, int(entry["homing_offset"])
        joint_id = entry.get("id", entry.get("motor_id", 0))
        joints.append(
            {
                "name": str(name),
                "id": int(joint_id if joint_id is not None else 0),
                "drive_sign": (
                    int(entry.get("drive_mode", 0) or 0) * -2 + 1
                    if "drive_mode" in entry
                    else int(entry.get("drive_sign", 1) or 1)
                ),
                "range_min": lo,
                "range_max": hi,
                "zero": zero,
            }
        )
    out = {"robot": "so101", "joints": joints}
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(out, sort_keys=False), encoding="utf-8")
    print(f"wrote {path}")
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    bus = make_so101_mock()
    recorder = TraceRecorder(args.output)

    def on_traffic(req: bytes, rep: bytes) -> None:
        recorder.record("tx", req)
        if rep:
            recorder.record("rx", rep)

    bus.add_listener(on_traffic)
    client = _Client(InMemoryTransport(bus, port_name="mock:record"), bus=bus)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            for sid in range(1, 7):
                client.read(sid, PRESENT_POSITION[0], 2)
            time.sleep(0.02)
        print(f"wrote {args.output}")
        return 0
    finally:
        client.close()
        recorder.close()


def cmd_replay(args: argparse.Namespace) -> int:
    transport = ReplayTransport(args.input)
    # Smoke: replay should yield at least one readable event without looping.
    try:
        # Consume by writing the first tx if present — otherwise just load.
        from fret.hardware.sts.replay import load_trace

        events = load_trace(args.input)
        print(f"loaded {len(events)} events from {args.input}")
        transport.close()
        return 0
    except StsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


def cmd_fake_bus(args: argparse.Namespace) -> int:
    ids = [int(x) for x in args.servos.split(",") if x.strip()]
    bus = MockStsBus(ids)
    device = FakeStsDevice(bus)
    port = device.start()
    print(port)
    try:
        if args.seconds <= 0:
            while True:
                time.sleep(1.0)
        else:
            time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        device.stop()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="fret-hw")
    parser.add_argument("--port", default="mock:so101")
    parser.add_argument("--baud", type=int, default=1_000_000)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("scan")
    p.add_argument("--start", type=int, default=1)
    p.add_argument("--end", type=int, default=10)
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("ping")
    p.add_argument("--id", type=int, required=True)
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser("read")
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--size", type=int, default=2)
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("watch")
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--seconds", type=float, default=1.0)
    p.add_argument("--period", type=float, default=0.05)
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("set-id")
    p.add_argument("--new-id", type=int, required=True)
    p.set_defaults(func=cmd_set_id)

    p = sub.add_parser("set-baud")
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--baud-value", type=int, required=True)
    p.set_defaults(func=cmd_set_baud)

    p = sub.add_parser("torque")
    torque_sub = p.add_subparsers(dest="torque_cmd", required=True)
    p_off = torque_sub.add_parser("off")
    p_off.add_argument("--ids", type=int, nargs="*")
    p_off.set_defaults(func=cmd_torque_off)

    p = sub.add_parser("calibrate")
    p.add_argument(
        "--names",
        nargs="+",
        default=[
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ],
    )
    p.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    p.add_argument("--samples", type=int, default=5)
    p.add_argument("--period", type=float, default=0.01)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("import-lerobot-calibration")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_import_lerobot_calibration)

    p = sub.add_parser("record")
    p.add_argument("--output", required=True)
    p.add_argument("--seconds", type=float, default=0.2)
    p.set_defaults(func=cmd_record)

    p = sub.add_parser("replay")
    p.add_argument("--input", required=True)
    p.set_defaults(func=cmd_replay)

    p = sub.add_parser("fake-bus")
    p.add_argument("--servos", default="1,2,3,4,5,6")
    p.add_argument("--seconds", type=float, default=0.0)
    p.set_defaults(func=cmd_fake_bus)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    return int(args.func(args))


# Re-export for tests that construct calibrations.
__all__ = ["JointCalibration", "build_parser", "main"]
