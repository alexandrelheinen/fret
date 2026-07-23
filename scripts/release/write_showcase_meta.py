#!/usr/bin/env python3
"""Write release showcase metadata from rendered MP4 and timing JSON files."""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from showcase_manifest import load_showcase_manifest  # noqa: E402


def write_showcase_meta(
    *,
    renders_dir: Path,
    tag: str,
    repo: str,
    git_sha: str,
    workflow_run: str,
    output_path: Path,
    manifest_path: Path | None = None,
) -> dict[str, object]:
    """Build and write ``meta.json`` for a release showcase bundle."""
    manifest = load_showcase_manifest(manifest_path)
    renders = sorted(renders_dir.glob("*.mp4"))
    if not renders:
        raise SystemExit("No showcase MP4 files were produced")

    showcases: list[dict[str, object]] = []
    for scenario in manifest.scenarios:
        prefix = f"{scenario.id}_"
        scenario_files = [
            path for path in renders if path.name.startswith(prefix)
        ]
        if not scenario_files:
            continue

        timing_path = renders_dir / scenario.timing_file
        if not timing_path.is_file():
            raise SystemExit(f"Missing timing metadata: {timing_path}")
        timing_data = json.loads(timing_path.read_text(encoding="utf-8"))
        clip_timing = {
            item["file"]: item for item in timing_data.get("clips", [])
        }

        videos: list[dict[str, object]] = []
        logs: list[dict[str, object]] = []
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
                    "r2_key": f"{scenario.id}/{path.name}",
                    "bytes": path.stat().st_size,
                    "sim_time_s": sim_time_s,
                    "real_time_factor": float(
                        timing.get("real_time_factor", 1.0)
                    ),
                }
            )
            csv_path = renders_dir / f"{path.stem}.csv"
            manifest_path = renders_dir / f"{path.stem}.json"
            if csv_path.is_file():
                logs.append(
                    {
                        "kind": "telemetry_csv",
                        "file": csv_path.name,
                        "r2_key": f"{scenario.id}/{csv_path.name}",
                        "bytes": csv_path.stat().st_size,
                        "pairs_with": path.name,
                    }
                )
            if manifest_path.is_file():
                logs.append(
                    {
                        "kind": "telemetry_manifest",
                        "file": manifest_path.name,
                        "r2_key": f"{scenario.id}/{manifest_path.name}",
                        "bytes": manifest_path.stat().st_size,
                        "pairs_with": path.name,
                    }
                )

        cameras_found = [video["camera"] for video in videos]
        for required in scenario.cameras:
            if required not in cameras_found:
                raise SystemExit(
                    f"Scenario {scenario.id} missing required camera "
                    f"{required!r} (found {cameras_found})"
                )

        showcases.append(
            {
                "scenario": scenario.id,
                "model": scenario.model,
                "robot_class": scenario.robot_class,
                "duration_s": duration_s,
                "realtime_playback": True,
                "cameras": cameras_found,
                "videos": videos,
                "logs": logs,
                "r2_prefix": f"{scenario.id}/",
                "primary_video": scenario.primary_video,
            }
        )

    if not showcases:
        raise SystemExit("No showcase clips found for any scenario")

    mobile_cameras = ["overview", "follow"]
    static_cameras = ["overview"]
    meta: dict[str, object] = {
        "repo": repo,
        "git_sha": git_sha,
        "git_ref": tag,
        "release_cameras": {
            "mobile": mobile_cameras,
            "static": static_cameras,
        },
        "realtime_playback": True,
        "partial": len(showcases) < len(manifest.scenarios),
        "fps": manifest.fps,
        "width": manifest.width,
        "height": manifest.height,
        "showcases": showcases,
        "primary_videos": {
            item["scenario"]: item["primary_video"] for item in showcases
        },
        "rendered_at": datetime.now(timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"
        ),
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
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Optional override for showcase.yml",
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
        manifest_path=args.manifest,
    )
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
