# Calibration

Tools: `fret-calib`. Outputs are machine-local under
`/var/lib/fret/calibration/` and must never be committed.

## Boards

```bash
fret-calib boards --print
```

Print the checkerboard (intrinsics) and ChArUco (extrinsics) specs in mm.

## Intrinsics (FR-HW-33)

```bash
# On the Pi with a live camera (operator session):
# fret-calib intrinsics --output /var/lib/fret/calibration/cam0_intrinsics.yml
# Offline / CI synthetic recovery:
fret-calib intrinsics --synthetic --output /tmp/cam0_intrinsics.yml
```

Pass: RMS reprojection ≤ 0.5 px (warn ≤ 1.0 px). Needs ≥ 15 accepted views.

## Extrinsics (FR-HW-33)

Place the ChArUco board at the known table pose. Write `t_world_cam` into a
VisionConfig snippet the existing loader accepts.

```bash
fret-calib extrinsics --synthetic --output /tmp/cam0_extrinsics.yml
```

## Base check (FR-HW-34)

Tip-touch at least four known board points; pass if max error ≤ 10 mm
(provisional, GAPS G10).

```bash
fret-calib base-check --tips tips.yml --board-points board.yml
```
