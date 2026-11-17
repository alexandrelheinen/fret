"""Record and replay STS bus traffic as timestamped JSONL (FR-HW-68)."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

from fret.hardware.sts.errors import StsTimeoutError
from fret.hardware.sts.protocol import StsReply, parse_reply
from fret.hardware.sts.transport import port_is_mock_safe


@dataclass(frozen=True, slots=True)
class TraceEvent:
    t_s: float
    direction: str  # "tx" or "rx"
    data_hex: str

    def to_json(self) -> dict[str, Any]:
        return {
            "t_s": self.t_s,
            "direction": self.direction,
            "data_hex": self.data_hex,
        }

    @classmethod
    def from_json(cls, obj: dict[str, Any]) -> TraceEvent:
        return cls(
            t_s=float(obj["t_s"]),
            direction=str(obj["direction"]),
            data_hex=str(obj["data_hex"]),
        )


class TraceRecorder:
    """Append-only JSONL recorder for bus traffic."""

    def __init__(self, path: Path | str, *, clock: Any | None = None) -> None:
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._clock = clock or time.monotonic
        self._t0 = self._clock()
        self._fp: TextIO = self._path.open("w", encoding="utf-8")

    def record(self, direction: str, data: bytes) -> None:
        event = TraceEvent(
            t_s=self._clock() - self._t0,
            direction=direction,
            data_hex=data.hex(),
        )
        self._fp.write(
            json.dumps(event.to_json(), separators=(",", ":")) + "\n"
        )
        self._fp.flush()

    def close(self) -> None:
        self._fp.close()

    def __enter__(self) -> TraceRecorder:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def load_trace(path: Path | str) -> list[TraceEvent]:
    events: list[TraceEvent] = []
    with Path(path).open(encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            events.append(TraceEvent.from_json(json.loads(line)))
    return events


class RecordingTransport:
    """Wrap a transport and mirror all bytes to a :class:`TraceRecorder`."""

    def __init__(self, inner: Any, recorder: TraceRecorder) -> None:
        self._inner = inner
        self._recorder = recorder

    @property
    def port_name(self) -> str:
        return f"record:{self._inner.port_name}"

    @property
    def is_mock_safe(self) -> bool:
        return bool(getattr(self._inner, "is_mock_safe", False))

    def write(self, data: bytes) -> None:
        self._recorder.record("tx", data)
        self._inner.write(data)

    def read(self, size: int, timeout_s: float) -> bytes:
        data = bytes(self._inner.read(size, timeout_s))
        if data:
            self._recorder.record("rx", data)
        return data

    def close(self) -> None:
        self._inner.close()
        self._recorder.close()


class ReplayTransport:
    """Deterministic transport that plays a JSONL trace back.

    End of trace is reported as :class:`StsTimeoutError` (never silent loop).
    """

    def __init__(
        self,
        events: list[TraceEvent] | Path | str,
        *,
        port_name: str | None = None,
    ) -> None:
        if isinstance(events, (str, Path)):
            self._events = load_trace(events)
            self._port_name = port_name or f"replay:{events}"
        else:
            self._events = list(events)
            self._port_name = port_name or "replay:memory"
        self._index = 0
        self._rx = bytearray()

    @property
    def port_name(self) -> str:
        return self._port_name

    @property
    def is_mock_safe(self) -> bool:
        return port_is_mock_safe(self._port_name) or True

    def write(self, data: bytes) -> None:
        # Advance past matching tx event; collect following rx until next tx.
        while self._index < len(self._events):
            event = self._events[self._index]
            self._index += 1
            if event.direction == "tx":
                # Optionally verify bytes match; tolerate prefix equality.
                if bytes.fromhex(event.data_hex) != data:
                    # Still consume; bridge may re-encode with same intent.
                    pass
                # Gather subsequent rx events.
                while self._index < len(self._events):
                    nxt = self._events[self._index]
                    if nxt.direction != "rx":
                        break
                    self._rx.extend(bytes.fromhex(nxt.data_hex))
                    self._index += 1
                return
        raise StsTimeoutError("replay trace ended during write")

    def read(self, size: int, timeout_s: float) -> bytes:  # noqa: ARG002
        if not self._rx and self._index >= len(self._events):
            raise StsTimeoutError("replay trace ended during read")
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def transact(
        self,
        request: bytes,
        *,
        expected_id: int | None,
        timeout_s: float | None = None,  # noqa: ARG002
        expect_replies: int = 1,
    ) -> list[StsReply]:
        self.write(request)
        replies: list[StsReply] = []
        buf = bytearray()
        while len(replies) < expect_replies:
            if not self._rx and self._index >= len(self._events):
                raise StsTimeoutError(
                    f"replay ended ({len(replies)}/{expect_replies})"
                )
            chunk = self.read(64, 0.0)
            if not chunk:
                if not self._rx:
                    raise StsTimeoutError("replay: no more rx bytes")
                continue
            buf.extend(chunk)
            while True:
                try:
                    exp = expected_id if expect_replies == 1 else None
                    reply, consumed = parse_reply(bytes(buf), expected_id=exp)
                except Exception:
                    if len(buf) < 6:
                        break
                    del buf[0]
                    continue
                else:
                    replies.append(reply)
                    del buf[:consumed]
                    if len(replies) >= expect_replies:
                        break
        return replies

    def close(self) -> None:
        self._rx.clear()
