"""Docs acceptance for FR-HW-62 / FR-HW-63 / FR-HW-74."""

from __future__ import annotations

from pathlib import Path

_DOC = (
    Path(__file__).resolve().parents[1]
    / "docs"
    / "hardware"
    / "deployment-rpi5.md"
)


def test_deployment_doc_required_sections() -> None:
    text = _DOC.read_text(encoding="utf-8")
    required = [
        "Headless first boot",
        "5 V / 5 A",
        "12 V",
        "Active Cooler",
        "A2",
        "zram",
        "rmw_zenoh_cpp",
        "CycloneDDS",
        "What runs where",
        "NVMe",
        "cloud-init",
        "Imager",
    ]
    missing = [item for item in required if item not in text]
    assert missing == [], f"missing sections/phrases: {missing}"
