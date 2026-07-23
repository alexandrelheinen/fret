# PlotJuggler layouts

Saved PlotJuggler (≥3.x) layout XML for FRET telemetry CSVs (FR-SIM-12).

## Why here (not `docs/`)

| Candidate | Verdict |
|---|---|
| `docs/` | **No** — docs are prose/images; layouts are operational UI assets loaded by a tool. |
| `config/` / `src/fret/config/` | Possible, but layouts are tied to the **series schema**, not scenario YAML params. |
| `scripts/plotjuggler/` | OK for a launcher only; XML would sit far from the producer. |
| **`src/fret/telemetry/layouts/`** | **Yes** — co-located with `fret.telemetry`, shippable as package data, one place for naming + viewers. |

## Files

| Layout | Scenarios (see `index.yaml`) | Main panes |
|---|---|---|
| `dubins_race.xml` | `dubins_race` | ENU XY paths; yaw; body speed; cross-track / progress |
| `omx_arm.xml` | `omx_pick_place`, `omx_wall_maze`, `omx_desk_clutter`, `omx_reach` | EE XYZ; joint angles |
| `omy_arm.xml` | `omy_pick_place`, `omy_clutter`, `omy_reach` | EE XYZ; joint angles |

## Usage

```bash
# After a telemetry run (FRET_TELEMETRY_ENABLED=1):
bash scripts/plotjuggler.sh \
  --csv /tmp/fret_telemetry/<run_id>/telemetry.csv \
  --scenario dubins_race

# Equivalent:
plotjuggler -d /path/to/telemetry.csv \
  -l src/fret/telemetry/layouts/dubins_race.xml
```

In PlotJuggler, confirm the CSV time axis is column **`t`**.

If curves are missing (older CSV / different agent set), PlotJuggler creates
placeholders; drag any extra series from the timeseries list as needed.

## Regenerating layouts

Prefer editing in PlotJuggler (**File → Save Layout**) against a real CSV, then
overwrite the matching file here so sizes/colors stay authentic. Hand-authored
XML in this folder matches PlotJuggler’s `root` / `tabbed_widget` / `Tab` /
`DockSplitter` / `plot` / `curve` schema and is safe to load with `-l`.
