#!/usr/bin/env python3
"""Generate checked-in PlotJuggler layout XML under src/fret/telemetry/layouts/.

Run after changing the FR-SIM-12 series set for a scenario family:

  python3 scripts/gen_plotjuggler_layouts.py
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

_OUT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "fret"
    / "telemetry"
    / "layouts"
)

# Match scripts/plot_telemetry.py agent colors.
_COLORS = {
    "tb3_rrt": "#1f77b4",
    "tb3_sst": "#2ca02c",
    "tb3_dummy": "#737373",
    "omx": "#1f77b4",
    "omy": "#ff7f0e",
    "joint": "#17becf",
}


def _indent(elem: ET.Element, level: int = 0) -> None:
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        for child in elem:
            _indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = pad + "  "
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = pad
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def _range() -> ET.Element:
    el = ET.Element(
        "range",
        {
            "bottom": "0.000000",
            "top": "1.000000",
            "left": "0.000000",
            "right": "1.000000",
        },
    )
    return el


def _plot_timeseries(
    curves: list[tuple[str, str]],
    *,
    style: str = "Lines",
    line_width: str = "1.5",
) -> ET.Element:
    plot = ET.Element(
        "plot",
        {
            "mode": "TimeSeries",
            "style": style,
            "flip_x": "false",
            "flip_y": "false",
            "line_width": line_width,
        },
    )
    plot.append(_range())
    plot.append(ET.Element("limitY"))
    for name, color in curves:
        plot.append(ET.Element("curve", {"name": name, "color": color}))
    return plot


def _plot_xy(
    curves: list[tuple[str, str, str, str]],
    *,
    style: str = "Lines",
    line_width: str = "2.0",
) -> ET.Element:
    """curves: (display_name, color, series_x, series_y)."""
    plot = ET.Element(
        "plot",
        {
            "mode": "XYPlot",
            "style": style,
            "flip_x": "false",
            "flip_y": "false",
            "line_width": line_width,
        },
    )
    plot.append(_range())
    plot.append(ET.Element("limitY"))
    for display, color, sx, sy in curves:
        plot.append(
            ET.Element(
                "curve",
                {
                    "name": display,
                    "color": color,
                    "curve_x": sx,
                    "curve_y": sy,
                },
            )
        )
    return plot


def _dock_area(name: str, plot: ET.Element) -> ET.Element:
    area = ET.Element("DockArea", {"name": name})
    area.append(plot)
    return area


def _splitter(
    orientation: str,
    sizes: list[float],
    children: list[ET.Element],
) -> ET.Element:
    assert orientation in {"|", "-"}
    assert len(sizes) == len(children)
    el = ET.Element(
        "DockSplitter",
        {
            "orientation": orientation,
            "count": str(len(children)),
            "sizes": ";".join(f"{s:.6f}" for s in sizes),
        },
    )
    for child in children:
        el.append(child)
    return el


def _layout_doc(tab_name: str, splitter: ET.Element) -> ET.Element:
    root = ET.Element("root")
    tw = ET.SubElement(
        root,
        "tabbed_widget",
        {"name": "Main Window", "parent": "main_window"},
    )
    tab = ET.SubElement(tw, "Tab", {"containers": "1", "tab_name": tab_name})
    container = ET.SubElement(tab, "Container")
    container.append(splitter)
    ET.SubElement(tw, "currentTabIndex", {"index": "0"})
    ET.SubElement(root, "use_relative_time_offset", {"enabled": "0"})
    return root


def _write(path: Path, root: ET.Element) -> None:
    _indent(root)
    tree = ET.ElementTree(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    # ElementTree does not emit the XML declaration PlotJuggler expects.
    xml_body = ET.tostring(root, encoding="unicode")
    path.write_text(
        "<?xml version='1.0' encoding='UTF-8'?>\n" + xml_body + "\n",
        encoding="utf-8",
    )
    print(f"wrote {path.relative_to(_OUT.parent.parent.parent.parent)}")


def gen_dubins_race() -> None:
    agents = ("tb3_rrt", "tb3_sst", "tb3_dummy")
    xy = _dock_area(
        "ENU paths",
        _plot_xy(
            [
                (
                    f"{a} path",
                    _COLORS[a],
                    f"{a}.position_enu.x",
                    f"{a}.position_enu.y",
                )
                for a in agents
            ]
        ),
    )
    yaw = _dock_area(
        "yaw",
        _plot_timeseries(
            [(f"{a}.orientation_enu.yaw", _COLORS[a]) for a in agents]
        ),
    )
    speed = _dock_area(
        "body speed",
        _plot_timeseries(
            [(f"{a}.velocity_body.x", _COLORS[a]) for a in agents]
        ),
    )
    path_err = _dock_area(
        "path error",
        _plot_timeseries(
            [
                ("tb3_rrt.cross_track_map.val", _COLORS["tb3_rrt"]),
                ("tb3_sst.cross_track_map.val", _COLORS["tb3_sst"]),
                ("tb3_rrt.progress_map.val", "#aec7e8"),
                ("tb3_sst.progress_map.val", "#98df8a"),
            ]
        ),
    )
    cmds = _dock_area(
        "commands",
        _plot_timeseries(
            [
                ("tb3_rrt.cmd_velocity_ctrl.val", _COLORS["tb3_rrt"]),
                ("tb3_sst.cmd_velocity_ctrl.val", _COLORS["tb3_sst"]),
                ("tb3_dummy.cmd_velocity_ctrl.val", _COLORS["tb3_dummy"]),
                ("tb3_rrt.cmd_omega_ctrl.val", "#9edae5"),
                ("tb3_sst.cmd_omega_ctrl.val", "#c5b0d5"),
            ]
        ),
    )
    right = _splitter(
        "-",
        [0.25, 0.25, 0.25, 0.25],
        [yaw, speed, path_err, cmds],
    )
    root_split = _splitter("|", [0.45, 0.55], [xy, right])
    _write(_OUT / "dubins_race.xml", _layout_doc("dubins_race", root_split))


def gen_arm(agent: str, n_joints: int, filename: str, tab: str) -> None:
    color = _COLORS[agent]
    joint_colors = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
    ]
    ee = _dock_area(
        "EE ENU",
        _plot_timeseries(
            [
                (f"{agent}.ee_position_enu.x", "#1f77b4"),
                (f"{agent}.ee_position_enu.y", "#ff7f0e"),
                (f"{agent}.ee_position_enu.z", "#2ca02c"),
            ]
        ),
    )
    ee_xy = _dock_area(
        "EE XY",
        _plot_xy(
            [
                (
                    f"{agent} ee",
                    color,
                    f"{agent}.ee_position_enu.x",
                    f"{agent}.ee_position_enu.y",
                )
            ]
        ),
    )
    joints = _dock_area(
        "joints",
        _plot_timeseries(
            [
                (
                    f"{agent}.position_joint.joint{i}",
                    joint_colors[(i - 1) % len(joint_colors)],
                )
                for i in range(1, n_joints + 1)
            ]
        ),
    )
    top = _splitter("|", [0.55, 0.45], [ee, ee_xy])
    root_split = _splitter("-", [0.45, 0.55], [top, joints])
    _write(_OUT / filename, _layout_doc(tab, root_split))


def main() -> None:
    gen_dubins_race()
    gen_arm("omx", 4, "omx_arm.xml", "omx_arm")
    gen_arm("omy", 6, "omy_arm.xml", "omy_arm")


if __name__ == "__main__":
    main()
