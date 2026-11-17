# Diagnostics (`fret-diag`)

Every command exits 0 (pass), 1 (fail), or 2 (skipped / needs operator),
prints a human summary, and writes JSON under `/var/log/fret/diag/`
(FR-HW-65).

## Commands

| Command | Role |
|---|---|
| `pi` | Host / OS / ROS / ARCO / storage / thermal / timer (FR-HW-59) |
| `usb` | udev symlinks present |
| `bus` | Feetech bus ping |
| `servos` | Present-position read |
| `calibration` | Local calibration files |
| `estop` | Mock power-loss → FAULT, no auto re-enable |
| `joints` | Guarded motion (needs `--allow-motion`) |
| `gripper` | Guarded motion (needs `--allow-motion`) |
| `camera` | Negotiated format / resolution / fps |
| `vision` | Ball vs printed mark |
| `loop` | Bridge cycle stats |
| `load` | Full-stack budget (FR-HW-73) |
| `e2e` | Guarded supervised-pick dry-run |
| `all` | Read-only chain; stops at first failure |

```bash
fret-diag pi --timer-duration 60
fret-diag all
fret-diag joints --allow-motion --yes   # motion only when intentional
fret-diag load
```

Motion commands are never part of `all` (FR-HW-66).
