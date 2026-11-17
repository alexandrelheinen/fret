"""Hardware bridge configuration loading."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml

from fret.hardware.safety import (
    SafetyConfig,
    default_safety_config_from_mapping,
)


@dataclass(frozen=True, slots=True)
class JointCalibration:
    name: str
    servo_id: int
    drive_sign: float
    range_min: int
    range_max: int
    zero: int


@dataclass(frozen=True, slots=True)
class BridgeConfig:
    robot: str = "so101"
    baud_rate: int = 1_000_000
    update_rate_hz: float = 50.0
    cycle_budget_ms: float = 20.0
    overrun_warn_consecutive: int = 3
    connect_range_margin_rad: float = 0.15
    reply_timeout_s: float = 0.05
    reconnect_backoff_s: float = 0.5
    max_joint_velocity: float = 2.0
    command_timeout_s: float = 0.2
    max_dt_s: float = 0.05
    velocity_source: str = "present_speed"
    joint_names: tuple[str, ...] = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )
    servo_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
    expected_model_number: int = 777
    calibration: tuple[JointCalibration, ...] = field(default_factory=tuple)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    @property
    def dof(self) -> int:
        return len(self.servo_ids)

    def joint_limits_rad(self) -> npt.NDArray[np.float64]:
        """Return (DOF, 2) limits from calibration ticks (or ±π default)."""
        import math

        from fret.hardware.sts.protocol import ticks_to_radians

        limits = np.zeros((self.dof, 2), dtype=np.float64)
        cal_by_id = {c.servo_id: c for c in self.calibration}
        for i, sid in enumerate(self.servo_ids):
            cal = cal_by_id.get(sid)
            if cal is None:
                limits[i] = (-math.pi, math.pi)
                continue
            lo = ticks_to_radians(
                cal.range_min, mid=cal.zero, sign=cal.drive_sign
            )
            hi = ticks_to_radians(
                cal.range_max, mid=cal.zero, sign=cal.drive_sign
            )
            limits[i, 0] = min(lo, hi)
            limits[i, 1] = max(lo, hi)
        return limits


def load_calibration(path: Path | str) -> tuple[JointCalibration, ...]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    joints = raw.get("joints", [])
    out: list[JointCalibration] = []
    for entry in joints:
        out.append(
            JointCalibration(
                name=str(entry["name"]),
                servo_id=int(entry["id"]),
                drive_sign=float(entry.get("drive_sign", 1)),
                range_min=int(entry["range_min"]),
                range_max=int(entry["range_max"]),
                zero=int(entry.get("zero", 2048)),
            )
        )
    return tuple(out)


def load_bridge_config(
    path: Path | str | None = None,
    *,
    calibration_path: Path | str | None = None,
) -> BridgeConfig:
    data: dict[str, Any] = {}
    if path is not None:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    cal: tuple[JointCalibration, ...] = ()
    if calibration_path is not None:
        cal = load_calibration(calibration_path)
    return BridgeConfig(
        robot=str(data.get("robot", "so101")),
        baud_rate=int(data.get("baud_rate", 1_000_000)),
        update_rate_hz=float(data.get("update_rate_hz", 50.0)),
        cycle_budget_ms=float(data.get("cycle_budget_ms", 20.0)),
        overrun_warn_consecutive=int(data.get("overrun_warn_consecutive", 3)),
        connect_range_margin_rad=float(
            data.get("connect_range_margin_rad", 0.15)
        ),
        reply_timeout_s=float(data.get("reply_timeout_s", 0.05)),
        reconnect_backoff_s=float(data.get("reconnect_backoff_s", 0.5)),
        max_joint_velocity=float(data.get("max_joint_velocity", 2.0)),
        command_timeout_s=float(data.get("command_timeout_s", 0.2)),
        max_dt_s=float(data.get("max_dt_s", 0.05)),
        velocity_source=str(data.get("velocity_source", "present_speed")),
        joint_names=tuple(
            data.get(
                "joint_names",
                [
                    "shoulder_pan",
                    "shoulder_lift",
                    "elbow_flex",
                    "wrist_flex",
                    "wrist_roll",
                    "gripper",
                ],
            )
        ),
        servo_ids=tuple(data.get("servo_ids", [1, 2, 3, 4, 5, 6])),
        expected_model_number=int(data.get("expected_model_number", 777)),
        calibration=cal,
        safety=default_safety_config_from_mapping(
            data.get("safety")
            if isinstance(data.get("safety"), dict)
            else None
        ),
    )


def default_example_calibration_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "hardware"
        / "so101_example_calibration.yaml"
    )


def default_bridge_config_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "hardware"
        / "so101.yml"
    )
