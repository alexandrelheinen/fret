#!/usr/bin/env python3
"""Plot FRET telemetry CSV (PlotJuggler companion / CI evidence).

Examples:
  python3 scripts/plot_telemetry.py \\
    --csv /tmp/fret_telemetry/.../dubins_race_overview.csv \\
    --output-dir docs/images/telemetry
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def _load_csv(path: Path) -> tuple[list[str], dict[str, list[float]]]:
    with open(path, encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        columns = list(reader.fieldnames)
        series: dict[str, list[float]] = {c: [] for c in columns}
        for row in reader:
            for col in columns:
                raw = row.get(col, "")
                if raw is None or raw == "":
                    series[col].append(float("nan"))
                else:
                    series[col].append(float(raw))
    return columns, series


def _agents_from_columns(columns: list[str]) -> list[str]:
    agents: set[str] = set()
    for col in columns:
        if col in {"t", "t_wall", "tick"}:
            continue
        if "." in col:
            agents.add(col.split(".", 1)[0])
    return sorted(agents)


def plot_telemetry(
    csv_path: Path,
    output_dir: Path,
    *,
    title: str | None = None,
) -> list[Path]:
    """Render XY path + yaw/speed time series; return written PNG paths."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    columns, series = _load_csv(csv_path)
    if "t" not in series:
        raise ValueError("telemetry CSV must include column 't'")
    t = np.asarray(series["t"], dtype=np.float64)
    agents = [
        a for a in _agents_from_columns(columns) if a not in {"mujoco", "sim"}
    ]
    if not agents:
        raise ValueError("no agent series found in CSV")

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = csv_path.stem
    written: list[Path] = []

    # --- XY paths ---
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    colors = {"tb3_rrt": "C0", "tb3_sst": "C2", "tb3_dummy": "0.45"}
    for agent in agents:
        x_key = f"{agent}.position_enu.x"
        y_key = f"{agent}.position_enu.y"
        if x_key not in series or y_key not in series:
            continue
        ax.plot(
            series[x_key],
            series[y_key],
            color=colors.get(agent, None),
            lw=1.6,
            label=agent,
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("east x [m]")
    ax.set_ylabel("north y [m]")
    ax.set_title(title or f"{stem} — ENU paths")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    path_xy = output_dir / f"{stem}_xy.png"
    fig.savefig(path_xy, dpi=140)
    plt.close(fig)
    written.append(path_xy)

    # --- yaw + speed ---
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for agent in agents:
        yaw_key = f"{agent}.orientation_enu.yaw"
        spd_key = f"{agent}.velocity_body.x"
        if yaw_key in series:
            axes[0].plot(
                t,
                series[yaw_key],
                color=colors.get(agent, None),
                lw=1.2,
                label=agent,
            )
        if spd_key in series:
            axes[1].plot(
                t,
                series[spd_key],
                color=colors.get(agent, None),
                lw=1.2,
                label=agent,
            )
    axes[0].set_ylabel("yaw [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8, loc="best")
    axes[0].set_title(title or f"{stem} — yaw / body speed")
    axes[1].set_ylabel("v_body.x [m/s]")
    axes[1].set_xlabel("t [s]")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    path_ts = output_dir / f"{stem}_timeseries.png"
    fig.savefig(path_ts, dpi=140)
    plt.close(fig)
    written.append(path_ts)

    # Sidecar summary for README / CI.
    summary = {
        "csv": str(csv_path),
        "agents": agents,
        "rows": int(len(t)),
        "t_final_s": float(t[-1]) if len(t) else 0.0,
        "plots": [str(p) for p in written],
    }
    summary_path = output_dir / f"{stem}_plot_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    written.append(summary_path)
    return written


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path, required=True, help="telemetry.csv"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory for PNG plots",
    )
    parser.add_argument("--title", default=None, help="Optional plot title")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = plot_telemetry(args.csv, args.output_dir, title=args.title)
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
