#!/usr/bin/env python3
"""Write release showcase metadata from rendered MP4 and timing JSON files."""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

_EXPECTED = {
    "ppp_warehouse": {
        "model": "ppp",
        "timing_file": "ppp_timing.json",
        "primary_video": "ppp_warehouse_overview.mp4",
    },
    "dubins_race": {
        "model": "dubins",
        "timing_file": "dubins_timing.json",
        "primary_video": "dubins_race_overview.mp4",
    },
}


def write_showcase_meta(
    *,
    renders_dir: Path,
    tag: str,
    repo: str,
    git_sha: str,
    workflow_run: str,
    output_path: Path,
) -> dict[str, object]:
    """Build and write ``meta.json`` for a release showcase bundle."""
    renders = sorted(renders_dir.glob("*.mp4"))
    if not renders:
        raise SystemExit("No showcase MP4 files were produced")

    showcases: list[dict[str, object]] = []
    for scenario, cfg in _EXPECTED.items():
        prefix = f"{scenario}_"
        scenario_files = [path for path in renders if path.name.startswith(prefix)]
        if not scenario_files:
            raise SystemExit(f"Missing showcase clips for {scenario}")

        timing_path = renders_dir / str(cfg["timing_file"])
        if not timing_path.is_file():
            raise SystemExit(f"Missing timing metadata: {timing_path}")
        timing_data = json.loads(timing_path.read_text(encoding="utf-8"))
        clip_timing = {item["file"]: item for item in timing_data.get("clips", [])}

        videos: list[dict[str, object]] = []
        duration_s = 0.0
        for path in sorted(scenario_files):
            camera = path.stem.removeprefix(prefix)
            timing = clip_timing.get(path.name, {})
            sim_time_s = float(timing.get("sim_time_s", 0.0))
            duration_s = max(duration_s, sim_time_s)
            videos.append(
                {
                    "camera": camera,
                    "file": path.name,
                    "bytes": path.stat().st_size,
                    "sim_time_s": sim_time_s,
                    "real_time_factor": float(
                        timing.get("real_time_factor", 1.0)
                    ),
                }
            )

        showcases.append(
            {
                "scenario": scenario,
                "model": cfg["model"],
                "duration_s": duration_s,
                "realtime_playback": True,
                "cameras": [video["camera"] for video in videos],
                "videos": videos,
                "primary_video": cfg["primary_video"],
            }
        )

    meta: dict[str, object] = {
        "repo": repo,
        "git_sha": git_sha,
        "git_ref": tag,
        "release_cameras": ["overview", "follow"],
        "realtime_playback": True,
        "fps": 30,
        "width": 1280,
        "height": 720,
        "showcases": showcases,
        "primary_videos": {
            item["scenario"]: item["primary_video"] for item in showcases
        },
        "rendered_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "workflow_run": workflow_run,
    }
    output_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    return meta


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--renders-dir",
        type=Path,
        default=Path("showcase_renders"),
        help="Directory containing MP4 clips and timing JSON files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("meta.json"),
        help="Destination metadata JSON path",
    )
    parser.add_argument(
        "--tag",
        default=os.environ.get("TAG", ""),
        help="Release tag label (default: TAG env var)",
    )
    parser.add_argument(
        "--repo",
        default=os.environ.get("GITHUB_REPOSITORY", ""),
        help="GitHub repository slug",
    )
    parser.add_argument(
        "--git-sha",
        default=os.environ.get("GITHUB_SHA", ""),
        help="Git commit SHA",
    )
    parser.add_argument(
        "--workflow-run",
        default=os.environ.get("WORKFLOW_RUN", ""),
        help="GitHub Actions workflow run URL",
    )
    args = parser.parse_args(argv)

    if not args.tag:
        parser.error("--tag or TAG env var is required")

    meta = write_showcase_meta(
        renders_dir=args.renders_dir,
        tag=args.tag,
        repo=args.repo,
        git_sha=args.git_sha,
        workflow_run=args.workflow_run,
        output_path=args.output,
    )
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
