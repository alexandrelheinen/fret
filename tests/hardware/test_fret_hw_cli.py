"""FR-HW-26 / FR-HW-27: fret-hw CLI against the mock bus."""

from __future__ import annotations

import json
from pathlib import Path

from fret.hardware.cli import main


def test_scan_lists_mock_servos() -> None:
    assert (
        main(["--port", "mock:so101", "scan", "--start", "1", "--end", "6"])
        == 0
    )


def test_ping_ok() -> None:
    assert main(["--port", "mock:so101", "ping", "--id", "1"]) == 0


def test_set_id_refuses_with_two_servos_fr_hw_27() -> None:
    # Default mock has six servos → refuse.
    rc = main(["--port", "mock:so101", "set-id", "--new-id", "10"])
    assert rc == 2


def test_set_id_succeeds_with_single_servo_fr_hw_27() -> None:
    from fret.hardware.cli import _Client, _open_bus
    from fret.hardware.sts.mock_bus import MockStsBus
    from fret.hardware.sts.transport import InMemoryTransport

    bus = MockStsBus([7])
    client = _Client(InMemoryTransport(bus, port_name="mock:one"), bus=bus)
    assert client.scan(range(0, 253)) == [7]
    client.write(7, 5, bytes((8,)))
    assert client.ping(8)
    client.close()


def test_calibrate_writes_yaml_fr_hw_26(tmp_path: Path) -> None:
    out = tmp_path / "cal.yaml"
    rc = main(
        [
            "--port",
            "mock:so101",
            "calibrate",
            "--names",
            "shoulder_pan",
            "--ids",
            "1",
            "--samples",
            "3",
            "--period",
            "0.0",
            "--output",
            str(out),
        ]
    )
    assert rc == 0
    text = out.read_text(encoding="utf-8")
    assert "shoulder_pan" in text
    assert "range_min" in text


def test_import_lerobot_calibration_fr_hw_26(tmp_path: Path) -> None:
    src = tmp_path / "lerobot.json"
    src.write_text(
        json.dumps(
            {
                "motors": {
                    "shoulder_pan": {
                        "id": 1,
                        "range_min": 100,
                        "range_max": 4000,
                        "homing_offset": 2048,
                        "drive_sign": 1,
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    out = tmp_path / "out.yaml"
    rc = main(
        [
            "import-lerobot-calibration",
            "--input",
            str(src),
            "--output",
            str(out),
        ]
    )
    assert rc == 0
    assert "shoulder_pan" in out.read_text(encoding="utf-8")


def test_torque_off() -> None:
    assert main(["--port", "mock:so101", "torque", "off", "--ids", "1"]) == 0


def test_record_and_replay(tmp_path: Path) -> None:
    trace = tmp_path / "t.jsonl"
    assert (
        main(
            [
                "--port",
                "mock:so101",
                "record",
                "--output",
                str(trace),
                "--seconds",
                "0.05",
            ]
        )
        == 0
    )
    assert trace.is_file()
    assert main(["replay", "--input", str(trace)]) == 0


def test_fake_bus_short_run() -> None:
    assert (
        main(
            [
                "fake-bus",
                "--servos",
                "1,2",
                "--seconds",
                "0.05",
            ]
        )
        == 0
    )
