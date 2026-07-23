#!/usr/bin/env python3
"""Render one (or all) release showcase scenario(s) from showcase.yml."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "scripts" / "release"))

from showcase_manifest import (  # noqa: E402
    ShowcaseManifest,
    ShowcaseScenario,
    load_showcase_manifest,
)


def _video_argv(
    scenario: ShowcaseScenario,
    *,
    manifest: ShowcaseManifest,
    output_dir: Path,
) -> list[str]:
    argv = [
        str(_REPO_ROOT / "scripts" / "video.sh"),
        "--model",
        scenario.model,
        "--scenario",
        scenario.id,
        "--output-dir",
        str(output_dir),
        "--timing-json",
        str(output_dir / scenario.timing_file),
        "--fps",
        str(scenario.effective_fps(manifest.fps)),
        "--width",
        str(scenario.effective_width(manifest.width)),
        "--height",
        str(scenario.effective_height(manifest.height)),
        "--collision-backend",
        scenario.collision_backend,
        "--planner-algorithm",
        scenario.planner_algorithm,
    ]
    for camera in scenario.cameras:
        argv.extend(["--camera", camera])
    if scenario.clip_duration_s is not None:
        argv.extend(["--duration", str(scenario.clip_duration_s)])
    else:
        argv.append("--full-duration")
        if scenario.clip_scale is not None:
            argv.extend(["--clip-scale", str(scenario.clip_scale)])
    if scenario.physics_mode:
        argv.append("--physics-mode")
    return argv


def render_scenario(
    scenario: ShowcaseScenario,
    *,
    manifest: ShowcaseManifest,
    output_dir: Path,
) -> None:
    """Invoke ``video.sh`` for one manifest entry."""
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = _video_argv(scenario, manifest=manifest, output_dir=output_dir)
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, cwd=_REPO_ROOT)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        action="append",
        dest="scenarios",
        metavar="ID",
        help="Scenario id to render (repeatable; default: all)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_REPO_ROOT / "showcase_renders",
        help="Directory for MP4s + timing JSON",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Override path to showcase.yml",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List scenario ids and exit",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    manifest = load_showcase_manifest(args.manifest)
    if args.list:
        for item in manifest.scenarios:
            print(item.id)
        return 0

    selected = (
        [manifest.by_id(sid) for sid in args.scenarios]
        if args.scenarios
        else list(manifest.scenarios)
    )
    for scenario in selected:
        render_scenario(
            scenario,
            manifest=manifest,
            output_dir=args.output_dir,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
