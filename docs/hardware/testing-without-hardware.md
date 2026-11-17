# Testing without hardware

FRET `v2.1.0` ships a byte-level Feetech STS3215 mock so the SO-101 bridge
and `fret-hw` tools can be developed and regression-tested before the arm
arrives. **Nothing in this release has been validated on real hardware.**

## Three stand-ins

1. **In-process `MockStsBus`** — register-level model of N STS3215 servos
   with first-order motion toward goal position. Unit tests inject faults
   (latency, dropped reply, bad checksum, wrong id, missing servo, low
   voltage, overload, overheat, cable pull).
2. **Pty fake device** — `fret-hw fake-bus` (or
   `fret.hardware.sts.fake_device.FakeStsDevice`) opens a pty pair, runs
   the mock on the master, and prints the slave path. Point
   `port:=/dev/pts/N` at it to exercise the real pyserial path and
   `ros2 launch fret hardware.py`.
3. **Recorded replay** — `fret-hw record` writes timestamped JSONL bus
   traffic; `fret-hw replay` / `ReplayTransport` plays it back
   bit-for-bit (`FR-HW-68`). Fixtures under
   `tests/fixtures/hardware/traces/` were recorded from the mock.

## Mock-only torque interlock (`FR-SYS-18`)

Until PR4, the bridge refuses to enable torque unless the port is a
mock (`mock:`), a replay (`replay:`), or a pty (`/dev/pts/…`). On a real
`/dev/ttyUSB*` path it stays in `CONNECTED_DISABLED` and logs that the
safety layer is required.

## Quick dry run

```bash
# Terminal A — mock bus on a pty
fret-hw fake-bus --servos 1,2,3,4,5,6

# Terminal B — bridge (replace /dev/pts/N with the path printed above)
ros2 launch fret hardware.py robot:=so101 port:=/dev/pts/N \
  calibration:=$(ros2 pkg prefix fret)/share/fret/config/hardware/so101_example_calibration.yaml
```

Expect `CONNECTED_DISABLED` and `/joint_states` at 50 Hz. Enabling
torque on the pty is allowed for dry runs; do not point `port` at a
real USB adapter with this release.
