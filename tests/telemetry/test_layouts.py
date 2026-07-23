"""PlotJuggler layout assets stay aligned with scenario ids / series grammar."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from fret.telemetry.layout_paths import (
    layout_path_for_scenario,
    load_layout_index,
)
from fret.telemetry.naming import validate_series_id

_LAYOUT_DIR = (
    Path(__file__).resolve().parents[2]
    / "src"
    / "fret"
    / "telemetry"
    / "layouts"
)


def test_index_maps_known_scenarios() -> None:
    index = load_layout_index()
    assert "dubins_race" in index
    assert index["dubins_race"] == "dubins_race.xml"
    assert index["omx_wall_maze"] == "omx_arm.xml"
    assert index["omy_clutter"] == "omy_arm.xml"


def test_layout_path_resolves() -> None:
    path = layout_path_for_scenario("dubins_race")
    assert path.is_file()
    assert path.name == "dubins_race.xml"


def test_unknown_scenario_raises() -> None:
    with pytest.raises(KeyError, match="no PlotJuggler layout"):
        layout_path_for_scenario("not_a_real_scenario")


@pytest.mark.parametrize(
    "xml_name",
    ["dubins_race.xml", "omx_arm.xml", "omy_arm.xml"],
)
def test_layout_xml_well_formed_and_curves_named(xml_name: str) -> None:
    path = _LAYOUT_DIR / xml_name
    root = ET.parse(path).getroot()
    assert root.tag == "root"
    tw = root.find("tabbed_widget")
    assert tw is not None
    assert tw.get("name") == "Main Window"

    series_names: list[str] = []
    for curve in root.iter("curve"):
        # TimeSeries: name is the series id.
        # XYPlot: curve_x / curve_y are the series ids.
        if curve.get("curve_x") and curve.get("curve_y"):
            series_names.append(curve.get("curve_x", ""))
            series_names.append(curve.get("curve_y", ""))
        else:
            series_names.append(curve.get("name", ""))

    assert series_names, f"no curves in {xml_name}"
    for name in series_names:
        validate_series_id(name)


def test_index_files_exist() -> None:
    index = load_layout_index()
    for basename in set(index.values()):
        assert (_LAYOUT_DIR / basename).is_file()
