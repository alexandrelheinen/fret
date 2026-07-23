"""Unit tests for telemetry series-id grammar."""

from __future__ import annotations

import pytest

from fret.telemetry.naming import (
    build_series_id,
    parse_series_id,
    validate_agent_name,
    validate_series_id,
)


@pytest.mark.parametrize(
    "series_id",
    [
        "tb3_sst.position_enu.x",
        "tb3_rrt.orientation_enu.yaw",
        "tb3_sst.velocity_body.x",
        "tb3_sst.cmd_omega_ctrl.val",
        "omy.position_joint.arm_1",
        "omy.ee_position_enu.z",
        "lidar.range_sensor_front.val",
    ],
)
def test_valid_series_ids(series_id: str) -> None:
    validate_series_id(series_id)
    parsed = parse_series_id(series_id)
    assert str(parsed) == series_id


@pytest.mark.parametrize(
    "series_id",
    [
        "TB3.position_enu.x",
        "tb3.position.x",
        "tb3/position_enu/x",
        "tb3.position_enu.x,y",
        "tb3.position_world.x",
        "tb3.position_enu.X",
        "",
        "no_dots",
    ],
)
def test_invalid_series_ids(series_id: str) -> None:
    with pytest.raises(ValueError):
        validate_series_id(series_id)


def test_build_series_id_round_trip() -> None:
    sid = build_series_id("tb3_sst", "position", "enu", "x")
    assert sid == "tb3_sst.position_enu.x"


def test_validate_agent_name() -> None:
    validate_agent_name("tb3_sst")
    with pytest.raises(ValueError):
        validate_agent_name("TB3")
