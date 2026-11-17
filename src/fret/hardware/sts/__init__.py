"""Feetech STS3215 protocol, mock bus, and transports."""

from fret.hardware.sts.errors import (
    StsChecksumError,
    StsError,
    StsIdMismatchError,
    StsLengthError,
    StsPortError,
    StsProtocolError,
    StsTimeoutError,
    TorqueEnableRefusedError,
)
from fret.hardware.sts.fake_device import FakeStsDevice
from fret.hardware.sts.mock_bus import (
    FaultConfig,
    FaultKind,
    MockStsBus,
    make_so101_mock,
)
from fret.hardware.sts.protocol import (
    build_ping,
    build_read,
    build_sync_read,
    build_sync_write,
    build_write,
    decode_sign_magnitude,
    encode_sign_magnitude,
    parse_reply,
)
from fret.hardware.sts.replay import (
    RecordingTransport,
    ReplayTransport,
    TraceRecorder,
    load_trace,
)
from fret.hardware.sts.transport import (
    InMemoryTransport,
    SerialTransport,
    port_is_mock_safe,
)

__all__ = [
    "FakeStsDevice",
    "FaultConfig",
    "FaultKind",
    "InMemoryTransport",
    "MockStsBus",
    "RecordingTransport",
    "ReplayTransport",
    "SerialTransport",
    "StsChecksumError",
    "StsError",
    "StsIdMismatchError",
    "StsLengthError",
    "StsPortError",
    "StsProtocolError",
    "StsTimeoutError",
    "TorqueEnableRefusedError",
    "TraceRecorder",
    "build_ping",
    "build_read",
    "build_sync_read",
    "build_sync_write",
    "build_write",
    "decode_sign_magnitude",
    "encode_sign_magnitude",
    "load_trace",
    "make_so101_mock",
    "parse_reply",
    "port_is_mock_safe",
]
