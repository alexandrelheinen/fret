"""FR-HW-63: ros.env and CycloneDDS peers rendering."""

from __future__ import annotations

from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]


def test_ros_env_recommends_zenoh() -> None:
    text = (_REPO / "deploy" / "ros.env.in").read_text(encoding="utf-8")
    assert "RMW_IMPLEMENTATION=rmw_zenoh_cpp" in text
    assert "ROS_DOMAIN_ID=" in text
    assert "Wi-Fi loss only cuts PC monitoring" in text or "monitoring" in text


def test_cyclonedds_disables_multicast() -> None:
    text = (_REPO / "deploy" / "dds" / "cyclonedds.xml.in").read_text(
        encoding="utf-8"
    )
    assert "AllowMulticast>false" in text
    assert "<Peers>" in text


def test_zenoh_unit_exists() -> None:
    assert (_REPO / "deploy" / "systemd" / "fret-zenohd.service").is_file()
