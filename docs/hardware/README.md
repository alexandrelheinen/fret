# Hardware (v2.x)

This directory, together with `docs/robots/`, is where FRET documents
specific hardware targets. The rest of `docs/` is hardware-neutral
(`FR-SYS-13`): it does not assume a particular robot, microcontroller, bus,
or protocol.

## Status as of v2.2.0

The v2.x hardware target is the **Seeed SO-ARM101 Pro** (SO-101 follower: 5
revolute joints + a friction gripper, Feetech STS3215 USB smart servos, no
onboard firmware), driven by a Raspberry Pi 5. See
[`arm-selection.md`](arm-selection.md) for the decision record and
[`docs/robots/so101.md`](../robots/so101.md) for the model page.

PR4 (`v2.2.0`) ships the **safety layer** (watchdog, limits, soft start,
hold on fault, enable lease, e-stop / power-loss detection) on top of the
PR3 mock bridge. Validated on the mock bus and the MuJoCo twin —
**not validated on real hardware**.

| Doc | Contents |
|---|---|
| [deployment-rpi5.md](deployment-rpi5.md) | Pi install, update, rollback, Wi-Fi/RMW, storage |
| [bring-up.md](bring-up.md) | Hardware-arrival checklist (OPS-08) |
| [calibration.md](calibration.md) | `fret-calib` intrinsics / extrinsics / base-check |
| [diagnostics.md](diagnostics.md) | `fret-diag` subsystem commands |
| [troubleshooting.md](troubleshooting.md) | Common Pi / bus / vision failures |
| [safety.md](safety.md) | Stop categories, e-stop wiring, supervisor behavior |
| [testing-without-hardware.md](testing-without-hardware.md) | Mock bus, pty fake, record/replay |
| [servo-setup.md](servo-setup.md) | `fret-hw` commands and calibration |
| [bridge.md](bridge.md) | Bridge module overview |
| [arm-selection.md](arm-selection.md) | Hardware decision record |
| [../modules/hardware.md](../modules/hardware.md) | Module map and FR checklist |

## Status as of v2.3.0-rc.1

PR5 ships the Raspberry Pi 5 deployment kit and bring-up diagnostics.
Validated in an arm64 container and on the mock bus — **not validated on
real Pi hardware or the physical arm** until OPS-06/07/08.
