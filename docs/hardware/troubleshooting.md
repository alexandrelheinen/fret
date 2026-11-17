# Troubleshooting (Pi + SO-101)

| Symptom | Likely cause | Check |
|---|---|---|
| `pip install --user` refused | PEP 668 | Use the install script venv (`--system-site-packages`) |
| ARCO install tries to compile | Missing aarch64 wheel | Confirm OPS-05 URL/SHA; script must not fall back to sdist |
| No `/dev/fret-servo-follower` | udev not captured / wrong port | `udev_capture.sh --follower`; check `ID_PATH` for clone adapters |
| ModemManager grabs tty | Rules missing ignore | Ensure `ID_MM_DEVICE_IGNORE=1` |
| Thermal throttle in `fret-diag load` | No Active Cooler / undervoltage | Cooler + 5 V / 5 A supply |
| PC loses topics over Wi-Fi | Multicast blocked | Prefer Zenoh unicast; or CycloneDDS peers XML |
| microSD full / slow | Logs on disk / swap file | tmpfs logs, zram, A2 card; `fret-diag pi` free-disk check |
| Motion diag refused | Missing flags | `--allow-motion` + confirmation; bridge disabled; speed ≤ 0.3 |
| Vision fails, ball invisible | Lighting / HSV | Fail with mask stats; do not invent a pose |
| Service restart enables torque | Bug | Must never happen (FR-HW-57); file an issue |
