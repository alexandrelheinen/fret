#!/usr/bin/env python3
"""Load the canonical release showcase matrix from config/release/showcase.yml."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_MANIFEST = _REPO_ROOT / "src/fret/config/release/showcase.yml"

RobotClass = Literal["mobile", "static"]


@dataclass(frozen=True)
class ShowcaseScenario:
    """One release showcase scenario entry."""

    id: str
    model: str
    robot_class: RobotClass
    cameras: tuple[str, ...]
    physics_mode: bool
    planner_algorithm: str
    collision_backend: str
    timing_file: str
    primary_video: str
    fps: int | None = None
    clip_duration_s: float | None = None

    @property
    def requires_follow(self) -> bool:
        """True when this entry lists a follow camera (optional for mobile)."""
        return "follow" in self.cameras

    def effective_fps(self, default_fps: int) -> int:
        """Return per-scenario fps override or the manifest default."""
        return int(self.fps) if self.fps is not None else int(default_fps)


@dataclass(frozen=True)
class ShowcaseManifest:
    """Full release showcase matrix."""

    fps: int
    width: int
    height: int
    scenarios: tuple[ShowcaseScenario, ...]

    def by_id(self, scenario_id: str) -> ShowcaseScenario:
        for item in self.scenarios:
            if item.id == scenario_id:
                return item
        raise KeyError(f"Unknown showcase scenario: {scenario_id!r}")


def _parse_scenario(raw: dict[str, Any]) -> ShowcaseScenario:
    robot_class = str(raw["robot_class"])
    if robot_class not in {"mobile", "static"}:
        raise ValueError(
            f"robot_class must be 'mobile' or 'static', got {robot_class!r}"
        )
    cameras = tuple(str(c) for c in raw["cameras"])
    if "overview" not in cameras:
        raise ValueError(
            f"scenario {raw.get('id')!r} must include overview camera"
        )
    if robot_class == "static" and "follow" in cameras:
        raise ValueError(
            f"static scenario {raw.get('id')!r} must not export follow"
        )
    fps_raw = raw.get("fps")
    clip_raw = raw.get("clip_duration_s")
    return ShowcaseScenario(
        id=str(raw["id"]),
        model=str(raw["model"]),
        robot_class=robot_class,  # type: ignore[arg-type]
        cameras=cameras,
        physics_mode=bool(raw.get("physics_mode", False)),
        planner_algorithm=str(raw.get("planner_algorithm", "rrt_star")),
        collision_backend=str(raw.get("collision_backend", "mujoco")),
        timing_file=str(raw["timing_file"]),
        primary_video=str(raw["primary_video"]),
        fps=int(fps_raw) if fps_raw is not None else None,
        clip_duration_s=(
            float(clip_raw) if clip_raw is not None else None
        ),
    )


def load_showcase_manifest(
    path: Path | None = None,
) -> ShowcaseManifest:
    """Parse ``showcase.yml`` into a typed manifest."""
    manifest_path = path if path is not None else _DEFAULT_MANIFEST
    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid showcase manifest: {manifest_path}")
    scenarios_raw = data.get("scenarios")
    if not isinstance(scenarios_raw, list) or not scenarios_raw:
        raise ValueError("showcase.yml must list at least one scenario")
    scenarios = tuple(_parse_scenario(item) for item in scenarios_raw)
    return ShowcaseManifest(
        fps=int(data.get("fps", 30)),
        width=int(data.get("width", 1280)),
        height=int(data.get("height", 720)),
        scenarios=scenarios,
    )


def release_cameras_for_robot_class(
    robot_class: RobotClass,
) -> tuple[str, ...]:
    """Return the CI-required release camera set for a robot class.

    Follow is optional for mobile (local / optional workflow only) so the
    Dubins encode budget stays under ~10 minutes.
    """
    _ = robot_class
    return ("overview",)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=_DEFAULT_MANIFEST,
        help="Path to showcase.yml",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print the manifest as JSON (for CI matrix generation)",
    )
    parser.add_argument(
        "--ids",
        action="store_true",
        help="Print scenario ids one per line",
    )
    args = parser.parse_args(argv)
    manifest = load_showcase_manifest(args.manifest)
    if args.ids:
        for item in manifest.scenarios:
            print(item.id)
        return 0
    if args.json:
        payload = {
            "fps": manifest.fps,
            "width": manifest.width,
            "height": manifest.height,
            "scenarios": [
                {
                    "id": s.id,
                    "model": s.model,
                    "robot_class": s.robot_class,
                    "cameras": list(s.cameras),
                    "physics_mode": s.physics_mode,
                    "planner_algorithm": s.planner_algorithm,
                    "collision_backend": s.collision_backend,
                    "timing_file": s.timing_file,
                    "primary_video": s.primary_video,
                    "fps": s.fps,
                    "clip_duration_s": s.clip_duration_s,
                }
                for s in manifest.scenarios
            ],
        }
        print(json.dumps(payload, indent=2))
        return 0
    for item in manifest.scenarios:
        cams = ",".join(item.cameras)
        print(f"{item.id}\t{item.model}\t{item.robot_class}\t{cams}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
