#!/usr/bin/env python3
"""Audit local showcase_renders/ against v1.4.x release video specs.

Checks (per scenario in showcase.yml):
  * primary overview MP4 exists and is non-trivial size
  * timing JSON present with real_time_factor (RTF postprocess evidence)
  * static arms: gate_cam side clips present
  * OM-X maze / OMY clutter: telemetry wall-contact rate == 0 when logged
  * OM-X scenarios: ball_radius_m == 0.020 in scenario YAML

Exit 0 if all checks pass.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

_REPO = Path(__file__).resolve().parents[2]
_DEFAULT_MANIFEST = _REPO / "src/fret/config/release/showcase.yml"
_SCENARIO_DIR = _REPO / "src/fret/config/scenarios"


def _load_manifest(path: Path) -> list[dict]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return list(data.get("scenarios") or [])


def _scenario_yaml(scenario_id: str) -> Path | None:
    # Planner variants share base YAML for ball radius (rrt/sst overlays).
    base = scenario_id
    for suffix in ("_rrt", "_sst"):
        if scenario_id.endswith(suffix):
            cand = _SCENARIO_DIR / f"{scenario_id}.yml"
            if cand.is_file():
                return cand
            base = scenario_id[: -len(suffix)]
            break
    path = _SCENARIO_DIR / f"{base}.yml"
    return path if path.is_file() else None


def _ball_radius(scenario_id: str) -> float | None:
    path = _scenario_yaml(scenario_id)
    if path is None:
        return None
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = (raw or {}).get("/**") or raw or {}
    if isinstance(params, dict) and "ros__parameters" in params:
        params = params["ros__parameters"]
    if not isinstance(params, dict):
        return None
    if "ball_radius_m" in params:
        return float(params["ball_radius_m"])
    return None


def _wall_contact_rate(tele_csv: Path) -> float | None:
    """Return fraction of rows with wall_contact==1 if column exists."""
    import csv

    with tele_csv.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            return None
        cols = {c.lower(): c for c in reader.fieldnames}
        key = None
        for cand in (
            "wall_contact",
            "arm_wall_contact",
            "transfer_wall_contact",
        ):
            if cand in cols:
                key = cols[cand]
                break
        if key is None:
            return None
        rows = list(reader)
    if not rows:
        return 0.0
    hits = sum(1 for r in rows if float(r.get(key) or 0) > 0.5)
    return hits / len(rows)


def audit(output_dir: Path, manifest: Path) -> int:
    scenarios = _load_manifest(manifest)
    errors: list[str] = []
    reports: list[dict] = []

    for sc in scenarios:
        sid = str(sc["id"])
        primary = str(sc.get("primary_video") or f"{sid}_overview.mp4")
        timing_name = str(sc.get("timing_file") or f"{sid}_timing.json")
        overview = output_dir / primary
        timing = output_dir / timing_name
        row: dict = {"id": sid, "ok": True, "notes": []}

        if not overview.is_file() or overview.stat().st_size < 10_000:
            errors.append(f"{sid}: missing/small overview {overview}")
            row["ok"] = False
        else:
            row["overview_bytes"] = overview.stat().st_size

        if not timing.is_file():
            errors.append(f"{sid}: missing timing JSON {timing}")
            row["ok"] = False
        else:
            meta = json.loads(timing.read_text(encoding="utf-8"))
            rtf = meta.get("real_time_factor")
            if rtf is None and isinstance(meta.get("clips"), list) and meta["clips"]:
                rtf = meta["clips"][0].get("real_time_factor")
            if rtf is None and isinstance(meta.get("cameras"), dict):
                cams = meta["cameras"]
                first = next(iter(cams.values()), {})
                rtf = first.get("real_time_factor")
            row["real_time_factor"] = rtf
            if rtf is None:
                errors.append(f"{sid}: timing JSON missing real_time_factor")
                row["ok"] = False

        if sc.get("robot_class") == "static":
            for cam in ("gate_cam_left", "gate_cam_right"):
                side = output_dir / f"{sid}_{cam}.mp4"
                if not side.is_file() or side.stat().st_size < 1_000:
                    # Gate cams are required for v1.4+ arm cells per showcase.yml.
                    errors.append(f"{sid}: missing gate clip {side.name}")
                    row["ok"] = False
                else:
                    row.setdefault("gate_cams", []).append(side.name)

        if sid.startswith("omx_"):
            r = _ball_radius(sid)
            row["ball_radius_m"] = r
            if r is None or abs(r - 0.020) > 1e-9:
                errors.append(
                    f"{sid}: expected ball_radius_m=0.020, got {r}"
                )
                row["ok"] = False

        tele = output_dir / f"{sid}_overview.csv"
        if tele.is_file() and (
            "wall_maze" in sid or "clutter" in sid or "desk" in sid
        ):
            rate = _wall_contact_rate(tele)
            row["wall_contact_rate"] = rate
            if rate is not None and rate > 0.0:
                errors.append(
                    f"{sid}: wall_contact_rate={rate:.3%} (must be 0)"
                )
                row["ok"] = False

        reports.append(row)

    out_json = output_dir / "audit_report.json"
    out_json.write_text(json.dumps(reports, indent=2), encoding="utf-8")
    print(json.dumps(reports, indent=2))
    if errors:
        print("\nAUDIT FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1
    print(f"\nAUDIT PASSED ({len(scenarios)} scenarios) → {out_json}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--output-dir",
        type=Path,
        default=Path("showcase_renders"),
    )
    p.add_argument("--manifest", type=Path, default=_DEFAULT_MANIFEST)
    args = p.parse_args()
    return audit(args.output_dir, args.manifest)


if __name__ == "__main__":
    raise SystemExit(main())
