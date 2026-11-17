# Servo setup and calibration

Feetech STS3215 tools for the SO-101 follower. Prefer
`fret-hw fake-bus` or `port:=mock:so101` until OPS-08. Real-port torque
enable requires an active `SafetySupervisor` (`FR-SYS-19`). See
[safety.md](safety.md).

## Install

`pyserial` is required for the real serial path and the pty fake device.
The in-process mock does not need a port.

## Commands (`fret-hw`)

| Command | Purpose |
|---|---|
| `scan` | List servo ids that answer PING |
| `ping --id N` | PING one servo |
| `read --id N --address A [--size 2]` | READ register(s) |
| `watch --id N` | Stream present position |
| `set-id --new-id N` | Change id (refuses if ≠1 servo on bus; verifies with PING) |
| `set-baud --id N --baud-value 1000000` | Write baud-rate code |
| `torque off [--ids …]` | Torque disable |
| `calibrate --output PATH` | Record mid-range calibration YAML |
| `import-lerobot-calibration --input J --output Y` | Convert LeRobot JSON |
| `record --output TRACE.jsonl` | Capture mock bus traffic |
| `replay --input TRACE.jsonl` | Load a recorded trace |
| `fake-bus --servos 1,2,3,4,5,6` | Pty mock for dry runs |

Machine-local calibration path:
`/var/lib/fret/calibration/so101_follower.yaml`. An example ships as
`config/hardware/so101_example_calibration.yaml`.

## Calibration file fields

Per joint: `name`, `id`, `drive_sign`, `range_min`, `range_max`, `zero`
(mid-range ticks). Tick↔radian mapping:

`rad = drive_sign * (ticks - zero) * 2π / 4096`

## Launch

```bash
fret-hw fake-bus --servos 1,2,3,4,5,6
ros2 launch fret hardware.py robot:=so101 port:=/dev/pts/N
```
