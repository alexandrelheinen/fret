"""Unit tests for TelemetrySession CSV / agent registration."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from fret.telemetry.session import (
    TelemetrySession,
    register_se2_mobile_agent,
    se2_pose_values,
)


def test_disabled_session_is_noop(tmp_path: Path) -> None:
    session = TelemetrySession(
        run_id="r1",
        scenario_id="dubins_race",
        enabled=False,
        output_dir=tmp_path,
    )
    agent = session.register_agent("tb3_sst")
    agent.register("position", frame="enu", components=("x", "y"), unit="m")
    session.record(0.0, {"tb3_sst.position_enu.x": 1.0})
    assert session.close() is None
    assert list(tmp_path.iterdir()) == []


def test_agent_registers_fields_and_writes_plotjuggler_csv(
    tmp_path: Path,
) -> None:
    session = TelemetrySession(
        run_id="r1",
        scenario_id="dubins_race",
        enabled=True,
        output_dir=tmp_path,
        dt_nominal_s=0.05,
        csv_basename="dubins_race_overview",
    )
    sst = register_se2_mobile_agent(
        session, "tb3_sst", with_commands=False, with_rates=False
    )
    assert "tb3_sst.position_enu.x" in sst.series_ids()
    sim = session.register_simulator(
        "mujoco", metadata={"physics_mode": False}
    )
    sim.register("step", frame="ctrl", components=("val",), unit="1")

    session.record(
        0.0,
        {
            **se2_pose_values("tb3_sst", (1.0, 2.0, 0.3)),
            "mujoco.step_ctrl.val": 0.0,
        },
        tick=0,
    )
    session.record(
        0.05,
        {
            **se2_pose_values("tb3_sst", (1.1, 2.1, 0.31)),
            "mujoco.step_ctrl.val": 1.0,
        },
        tick=1,
    )
    csv_path = session.close()
    assert csv_path is not None
    assert csv_path.name == "dubins_race_overview.csv"
    assert session.manifest_path.name == "dubins_race_overview.json"

    with open(csv_path, encoding="utf-8", newline="") as fh:
        rows = list(csv.reader(fh))
    header = rows[0]
    assert header[0] == "t"
    assert "t_wall" in header
    assert header.count("t") == 1
    assert "tb3_sst.position_enu.x" in header
    assert len(rows) == 3  # header + 2 samples

    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 1
    assert manifest["plotjuggler"]["recommended_time_axis"] == "t"
    assert "tb3_sst" in manifest["agents"]
    assert manifest["agents_meta"]["mujoco"]["kind"] == "simulator"


def test_strict_record_rejects_unknown_series(tmp_path: Path) -> None:
    session = TelemetrySession(
        run_id="r1",
        scenario_id="dubins_race",
        enabled=True,
        output_dir=tmp_path,
        strict=True,
    )
    register_se2_mobile_agent(
        session, "tb3_sst", with_commands=False, with_rates=False
    )
    with pytest.raises(KeyError):
        session.record(0.0, {"tb3_sst.position_enu.z": 1.0})


def test_close_idempotent(tmp_path: Path) -> None:
    session = TelemetrySession(
        run_id="r1",
        scenario_id="dubins_race",
        enabled=True,
        output_dir=tmp_path,
    )
    register_se2_mobile_agent(
        session, "tb3_a", with_commands=False, with_rates=False
    )
    session.record(0.0, se2_pose_values("tb3_a", (0.0, 0.0, 0.0)))
    p1 = session.close()
    p2 = session.close()
    assert p1 == p2
